/*
 * ESP32 Geiger Counter - SBM-20 tube
 * ----------------------------------
 * - Pulse counting via falling-edge GPIO interrupt (GPIO 27)
 * - CPM over a rolling 60-second window (ring buffer of per-second counts)
 * - uSv/h conversion using SBM20_USV_PER_CPM
 * - SSD1306 128x64 OLED (U8g2, HW I2C), 3 screens selected with a button
 * - Passive piezo "click" via LEDC PWM, long-press the button to mute
 * - Status LED blinks per pulse, solid while muted
 * - CSV on Serial @115200: uptime_ms,cpm,usv_h,total_counts
 *
 * Architecture notes:
 *  - The ISR only increments a counter (inside a portMUX critical section) and
 *    sends a task notification. It never blocks.
 *  - Click + LED timing is handled by a small FreeRTOS task on core 0 that
 *    sleeps until notified. This keeps the click length accurate even while
 *    loop() is busy pushing the display buffer over I2C (~20-25 ms).
 *  - loop() is entirely millis()-based and non-blocking (no delay()).
 */

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <stdio.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// ---- Pin assignment --------------------------------------------------------
constexpr uint8_t PIN_GEIGER = 27;   // Geiger pulse input (falling edge, pull-up)
constexpr uint8_t PIN_SDA    = 21;   // OLED SDA
constexpr uint8_t PIN_SCL    = 22;   // OLED SCL
constexpr uint8_t PIN_BUZZER = 25;   // Passive piezo (LEDC channel 0)
constexpr uint8_t PIN_BUTTON = 0;    // Mode button, active low, pull-up
constexpr uint8_t PIN_LED    = 2;    // Onboard status LED

// ---- Measurement -----------------------------------------------------------
constexpr uint8_t  WINDOW_SECONDS    = 60;      // Rolling window / ring buffer slots
constexpr uint32_t TICK_INTERVAL_MS  = 1000;    // Ring buffer update period

// SBM-20 conversion factor: micro-Sieverts per hour per count-per-minute.
// Adjust this for your own calibration / reference source.
constexpr float SBM20_USV_PER_CPM = 0.0057f;

// ---- Buzzer ----------------------------------------------------------------
constexpr uint8_t  BUZZER_LEDC_CHANNEL  = 0;
constexpr uint8_t  BUZZER_LEDC_RES_BITS = 8;
constexpr uint32_t BUZZER_FREQ_HZ       = 4000;  // Click tone frequency
constexpr uint32_t BUZZER_CLICK_US      = 2000;  // Click length (~2 ms)
constexpr uint32_t BUZZER_DUTY          = 128;   // 50% of 8-bit range

// ---- Status LED ------------------------------------------------------------
constexpr uint32_t LED_PULSE_MS = 15;            // LED on-time per pulse
                                                 // (also the minimum gap between clicks)

// ---- Button ----------------------------------------------------------------
constexpr uint32_t BUTTON_DEBOUNCE_MS   = 30;
constexpr uint32_t BUTTON_LONG_PRESS_MS = 1000;  // Long press toggles mute

// ---- Display ---------------------------------------------------------------
constexpr uint8_t  OLED_I2C_ADDR       = 0x3C;   // 7-bit address
constexpr uint32_t OLED_I2C_CLOCK_HZ   = 400000;
constexpr uint32_t DISPLAY_REFRESH_MS  = 250;
constexpr uint32_t TOAST_DURATION_MS   = 1000;
constexpr uint8_t  NUM_SCREENS         = 3;

// Bar graph geometry (screen 2)
constexpr int GRAPH_X = 4;     // First bar x (inner area)
constexpr int GRAPH_Y = 13;    // Top row of inner area
constexpr int GRAPH_H = 50;    // Inner height in pixels (rows 13..62)
constexpr int GRAPH_PITCH = 2; // 60 bars * 2 px = 120 px
constexpr int GRAPH_BAR_W = 1; // Bar width in px

// ---- Serial ----------------------------------------------------------------
constexpr uint32_t SERIAL_BAUD = 115200;

// ============================================================================
// LEDC compatibility (Arduino-ESP32 core 2.x vs 3.x)
// ============================================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void buzzerInit() {
  ledcAttachChannel(PIN_BUZZER, BUZZER_FREQ_HZ, BUZZER_LEDC_RES_BITS, BUZZER_LEDC_CHANNEL);
  ledcWrite(PIN_BUZZER, 0);
}
static inline void buzzerWrite(uint32_t duty) { ledcWrite(PIN_BUZZER, duty); }
#else
static void buzzerInit() {
  ledcSetup(BUZZER_LEDC_CHANNEL, BUZZER_FREQ_HZ, BUZZER_LEDC_RES_BITS);
  ledcAttachPin(PIN_BUZZER, BUZZER_LEDC_CHANNEL);
  ledcWrite(BUZZER_LEDC_CHANNEL, 0);
}
static inline void buzzerWrite(uint32_t duty) { ledcWrite(BUZZER_LEDC_CHANNEL, duty); }
#endif

// ============================================================================
// GLOBAL STATE
// ============================================================================

// ---- Display (full-buffer mode, SSD1306 128x64, hardware I2C) ---------------
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset= */ U8X8_PIN_NONE);

// ---- ISR-shared state ------------------------------------------------------
static portMUX_TYPE g_counterMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t g_pulseCount = 0;          // Total pulses since boot
static TaskHandle_t g_feedbackTask = nullptr;       // Click/LED task handle
static volatile bool g_muted = false;               // Read by feedback task

// ---- Rolling window --------------------------------------------------------
static uint32_t g_ring[WINDOW_SECONDS];             // Per-second counts
static uint8_t  g_head = 0;                         // Next slot to write (= oldest)
static uint8_t  g_filled = 0;                       // Valid slots (0..WINDOW_SECONDS)
static uint32_t g_windowSum = 0;                    // Sum of all slots
static uint32_t g_lastTotal = 0;                    // Total at previous tick
static uint32_t g_lastTickMs = 0;

static uint32_t g_cpm = 0;
static float    g_usvh = 0.0f;

// ---- UI state --------------------------------------------------------------
static uint8_t  g_screen = 0;
static bool     g_forceRedraw = true;
static uint32_t g_lastDrawMs = 0;
static const char* g_toastText = nullptr;
static uint32_t g_toastUntilMs = 0;

// ---- Button state ----------------------------------------------------------
static bool     g_btnRaw = false;       // Last raw reading (true = pressed)
static bool     g_btnStable = false;    // Debounced state
static uint32_t g_btnChangeMs = 0;
static uint32_t g_btnPressMs = 0;
static bool     g_btnLongFired = false;

// ============================================================================
// ISR + FEEDBACK TASK
// ============================================================================

// Geiger pulse ISR: count + notify. No blocking code.
void IRAM_ATTR geigerISR() {
  BaseType_t woken = pdFALSE;

  portENTER_CRITICAL_ISR(&g_counterMux);
  g_pulseCount++;
  portEXIT_CRITICAL_ISR(&g_counterMux);

  if (g_feedbackTask != nullptr) {
    vTaskNotifyGiveFromISR(g_feedbackTask, &woken);
    if (woken) {
      portYIELD_FROM_ISR();
    }
  }
}

static uint32_t readTotalCounts() {
  portENTER_CRITICAL(&g_counterMux);
  uint32_t v = g_pulseCount;
  portEXIT_CRITICAL(&g_counterMux);
  return v;
}

// Sleeps until a pulse notification arrives, then emits a click and a short
// LED blink. Pulses arriving during a click/blink are coalesced, which limits
// audible clicks to roughly 1000 / (LED_PULSE_MS + click) per second.
static void feedbackTask(void* /*arg*/) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Clears the notification count

    if (g_muted) {
      continue;  // LED is held solid by the mute logic
    }

    digitalWrite(PIN_LED, HIGH);
    buzzerWrite(BUZZER_DUTY);
    delayMicroseconds(BUZZER_CLICK_US);
    buzzerWrite(0);

    vTaskDelay(pdMS_TO_TICKS(LED_PULSE_MS));
    digitalWrite(PIN_LED, g_muted ? HIGH : LOW);
  }
}

// ============================================================================
// MEASUREMENT
// ============================================================================

// Called once per TICK_INTERVAL_MS from loop().
static void secondTick() {
  uint32_t total = readTotalCounts();
  uint32_t delta = total - g_lastTotal;  // Wrap-safe
  g_lastTotal = total;

  // Replace the oldest slot with the newest per-second count.
  g_windowSum -= g_ring[g_head];
  g_ring[g_head] = delta;
  g_windowSum += delta;
  g_head = (g_head + 1) % WINDOW_SECONDS;
  if (g_filled < WINDOW_SECONDS) {
    g_filled++;
  }

  // During the first minute, scale the partial sum up to a per-minute rate
  // so the reading is a usable estimate right away (noisy until 60 s).
  if (g_filled >= WINDOW_SECONDS) {
    g_cpm = g_windowSum;
  } else {
    g_cpm = (g_windowSum * (uint32_t)WINDOW_SECONDS + g_filled / 2) / g_filled;
  }
  g_usvh = (float)g_cpm * SBM20_USV_PER_CPM;

  Serial.printf("%lu,%lu,%.4f,%lu\n",
                (unsigned long)millis(),
                (unsigned long)g_cpm,
                (double)g_usvh,
                (unsigned long)total);

  g_forceRedraw = true;
}

// ============================================================================
// UI HELPERS
// ============================================================================

static void showToast(const char* text) {
  g_toastText = text;
  g_toastUntilMs = millis() + TOAST_DURATION_MS;
  g_forceRedraw = true;
}

static void toggleMute() {
  g_muted = !g_muted;
  digitalWrite(PIN_LED, g_muted ? HIGH : LOW);  // Solid while muted
  showToast(g_muted ? "MUTED" : "SOUND ON");
}

static void nextScreen() {
  g_screen = (g_screen + 1) % NUM_SCREENS;
  g_forceRedraw = true;
}

// Non-blocking debounce + short/long press detection.
static void handleButton(uint32_t now) {
  bool pressed = (digitalRead(PIN_BUTTON) == LOW);

  if (pressed != g_btnRaw) {
    g_btnRaw = pressed;
    g_btnChangeMs = now;
  }

  if ((now - g_btnChangeMs) >= BUTTON_DEBOUNCE_MS && g_btnRaw != g_btnStable) {
    g_btnStable = g_btnRaw;
    if (g_btnStable) {
      g_btnPressMs = now;
      g_btnLongFired = false;
    } else if (!g_btnLongFired) {
      nextScreen();  // Short press, acted on release
    }
  }

  if (g_btnStable && !g_btnLongFired && (now - g_btnPressMs) >= BUTTON_LONG_PRESS_MS) {
    g_btnLongFired = true;
    toggleMute();
  }
}

// ============================================================================
// DRAWING
// ============================================================================

static void drawCentered(int y, const char* s) {
  int w = u8g2.getStrWidth(s);
  u8g2.drawStr((128 - w) / 2, y, s);
}

// Screen 1: big CPM number + uSv/h
static void drawScreenCpm() {
  char buf[32];

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "CPM");

  if (g_filled < WINDOW_SECONDS) {
    snprintf(buf, sizeof(buf), "WARMUP %us", (unsigned)g_filled);
    u8g2.drawStr(128 - u8g2.getStrWidth(buf), 10, buf);
  }

  unsigned long shown = (g_cpm > 99999UL) ? 99999UL : (unsigned long)g_cpm;
  snprintf(buf, sizeof(buf), "%lu", shown);
  u8g2.setFont(u8g2_font_logisoso32_tn);
  drawCentered(50, buf);

  u8g2.setFont(u8g2_font_6x10_tf);
  snprintf(buf, sizeof(buf), "%.3f uSv/h", (double)g_usvh);
  u8g2.drawStr(0, 63, buf);

  if (g_muted) {
    u8g2.drawStr(128 - u8g2.getStrWidth("MUTE"), 63, "MUTE");
  }
}

// Screen 2: bar graph of the last 60 seconds (oldest left, newest right)
static void drawScreenGraph() {
  char buf[32];

  uint32_t maxVal = 1;
  for (uint8_t i = 0; i < WINDOW_SECONDS; i++) {
    if (g_ring[i] > maxVal) {
      maxVal = g_ring[i];
    }
  }

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "LAST 60s");
  snprintf(buf, sizeof(buf), "peak %lu/s", (unsigned long)((maxVal == 1 && g_windowSum == 0) ? 0 : maxVal));
  u8g2.drawStr(128 - u8g2.getStrWidth(buf), 10, buf);

  u8g2.drawFrame(GRAPH_X - 1, GRAPH_Y - 1, WINDOW_SECONDS * GRAPH_PITCH + 2, GRAPH_H + 2);

  for (uint8_t i = 0; i < WINDOW_SECONDS; i++) {
    uint32_t v = g_ring[(g_head + i) % WINDOW_SECONDS];
    if (v == 0) {
      continue;
    }
    int h = (int)((v * (uint32_t)GRAPH_H) / maxVal);
    if (h < 1) {
      h = 1;
    }
    int x = GRAPH_X + i * GRAPH_PITCH;
    int y = GRAPH_Y + GRAPH_H - h;  // Bottom row = GRAPH_Y + GRAPH_H - 1
    u8g2.drawBox(x, y, GRAPH_BAR_W, h);
  }
}

// Screen 3: total counts + uptime
static void drawScreenTotals() {
  char buf[32];

  u8g2.setFont(u8g2_font_6x10_tf);
  drawCentered(10, "TOTAL COUNTS");

  snprintf(buf, sizeof(buf), "%lu", (unsigned long)readTotalCounts());
  u8g2.setFont(u8g2_font_ncenB14_tr);
  drawCentered(30, buf);

  u8g2.setFont(u8g2_font_6x10_tf);
  drawCentered(44, "UPTIME");

  unsigned long s = millis() / 1000UL;
  snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu", s / 3600UL, (s / 60UL) % 60UL, s % 60UL);
  u8g2.setFont(u8g2_font_ncenB14_tr);
  drawCentered(62, buf);
}

static void drawToast(const char* text) {
  u8g2.setFont(u8g2_font_helvB12_tr);
  int w = u8g2.getStrWidth(text);
  int bw = w + 16;
  int bx = (128 - bw) / 2;
  const int by = 22;
  const int bh = 22;

  u8g2.setDrawColor(0);
  u8g2.drawBox(bx, by, bw, bh);
  u8g2.setDrawColor(1);
  u8g2.drawFrame(bx, by, bw, bh);
  u8g2.drawStr(bx + 8, by + 16, text);
}

static void drawDisplay(uint32_t now) {
  u8g2.clearBuffer();

  switch (g_screen) {
    case 0:  drawScreenCpm();    break;
    case 1:  drawScreenGraph();  break;
    default: drawScreenTotals(); break;
  }

  if (g_toastText != nullptr) {
    if ((int32_t)(g_toastUntilMs - now) > 0) {
      drawToast(g_toastText);
    } else {
      g_toastText = nullptr;
    }
  }

  u8g2.sendBuffer();
}

// ============================================================================
// SETUP / LOOP
// ============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_GEIGER, INPUT_PULLUP);

  buzzerInit();

  // Click/LED task on core 0 (Arduino loop() runs on core 1).
  xTaskCreatePinnedToCore(feedbackTask, "feedback", 2048, nullptr, 5, &g_feedbackTask, 0);

  // OLED
  Wire.begin(PIN_SDA, PIN_SCL);
  u8g2.setI2CAddress(OLED_I2C_ADDR << 1);  // U8g2 expects the 8-bit address
  u8g2.begin();
  u8g2.setBusClock(OLED_I2C_CLOCK_HZ);
  drawDisplay(millis());

  Serial.println("uptime_ms,cpm,usv_h,total_counts");

  g_lastTickMs = millis();

  // Attach the interrupt last, after the feedback task exists.
  attachInterrupt(digitalPinToInterrupt(PIN_GEIGER), geigerISR, FALLING);
}

void loop() {
  uint32_t now = millis();

  handleButton(now);

  // 1-second tick (millis-based, drift-free, catches up if ever delayed)
  while ((uint32_t)(now - g_lastTickMs) >= TICK_INTERVAL_MS) {
    g_lastTickMs += TICK_INTERVAL_MS;
    secondTick();
  }

  if (g_forceRedraw || (uint32_t)(now - g_lastDrawMs) >= DISPLAY_REFRESH_MS) {
    g_forceRedraw = false;
    g_lastDrawMs = now;
    drawDisplay(now);
  }
}