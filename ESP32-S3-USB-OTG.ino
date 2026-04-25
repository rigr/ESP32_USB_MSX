// =======================================================================
// ESP32-S3 USB Mouse → Roland Sampler
// ROBUST AUTO-LEARNING VERSION
// MSX PART UNCHANged - should support Roland ans MSX
//
// MSX part based on code by NYYRIKKY and Peter Ullrich
// and USB-host part based on https://github.com/tanakamasayuki/EspUsbHost
// Richard, 25.4.2026
// =======================================================================

#include "EspUsbHost.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include <Preferences.h>
#include <esp_system.h>

#define MAX_SAMPLES 50
#define MAX_LEN 16

// ========================= PIN DEFINITION =========================
#define PIN_MX0 14
#define PIN_MX1 13
#define PIN_MX2 12
#define PIN_MX3 11
#define PIN_BTN_L 8
#define PIN_BTN_R 3
#define PIN_STROBE 46

#define B0 (1UL << PIN_MX0)
#define B1 (1UL << PIN_MX1)
#define B2 (1UL << PIN_MX2)
#define B3 (1UL << PIN_MX3)

// ========================= GLOBAL =========================
volatile int16_t lastX = 0, lastY = 0;
volatile bool leftBtn = false, rightBtn = false;

int scale = 15;
Preferences prefs;

// ========================= MAPPING =========================
struct MouseMapping {
  uint8_t mainLen;
  uint8_t btnByte;
  uint8_t wheelByte;
  uint8_t xByte;
  uint8_t yByte;
  bool constantMask[MAX_LEN];
  bool valid;
} mouseMap;

// ========================= BUFFER =========================
struct Buffer {
  uint8_t data[MAX_SAMPLES][MAX_LEN];
  int count = 0;
};

Buffer buffers[17];

// ========================= STATE =========================
enum State {
  IDLE,
  BUTTON_DETECT,
  WHEEL,
  XY,
  DONE
};

// ========================= USB HOST =========================
class MyEspUsbHost : public EspUsbHost {
public:

  State state = IDLE;
  bool collecting = false;
  bool ready = false;

  // ================= RECEIVE =================
  void onReceive(const usb_transfer_t* t) override {
    if (!t || !t->data_buffer) return;

    uint8_t* d = t->data_buffer;
    int len = t->actual_num_bytes;
    if (len > MAX_LEN) return;

    // ===== NORMAL MODE =====
    if (!collecting && mouseMap.valid) {
      printRaw(d, len);
      runMouse(d, len);
      return;
    }

    // ===== CALIBRATION =====
    if (!collecting) return;

    if (buffers[len].count >= MAX_SAMPLES) return;

    memcpy(buffers[len].data[buffers[len].count], d, len);
    buffers[len].count++;

    Serial.printf("Collecting (%d/%d)\n", buffers[len].count, MAX_SAMPLES);

    if (buffers[len].count >= MAX_SAMPLES) {
      collecting = false;
      ready = true;
      Serial.println("\n=== Enough data collected ===");
      analyze();
      Serial.println(">>> Press BOOT to continue <<<\n");
    }
  }

  // ================= RAW =================
  void printRaw(uint8_t* d, int len) {
    Serial.printf("RAW len=%d | ", len);
    for (int i = 0; i < len; i++) Serial.printf("%02X ", d[i]);
  }

  // ================= MAIN LEN =================
  int getMainLen() {
    int bestLen = 0, bestScore = 0;

    for (int len = 0; len < 17; len++) {
      if (buffers[len].count < 10) continue;

      int var = 0;
      for (int b = 0; b < len; b++) {
        uint8_t f = buffers[len].data[0][b];
        for (int i = 1; i < buffers[len].count; i++) {
          if (buffers[len].data[i][b] != f) {
            var++;
            break;
          }
        }
      }

      if (var > bestScore) {
        bestScore = var;
        bestLen = len;
      }
    }

    Serial.printf("Selected main len=%d\n", bestLen);
    return bestLen;
  }

  // ================= CONSTANT BYTES =================
  void detectConstants(int len) {
    for (int b = 0; b < len; b++) {
      bool same = true;
      uint8_t v = buffers[len].data[0][b];

      for (int i = 1; i < buffers[len].count; i++) {
        if (buffers[len].data[i][b] != v) {
          same = false;
          break;
        }
      }
      mouseMap.constantMask[b] = same;
    }
  }

  // ================= BUTTON DETECTION =================
  void detectButtons(int len) {

    for (int b = 0; b < len; b++) {

      bool seen[8] = { 0 };

      for (int i = 0; i < buffers[len].count; i++) {
        uint8_t v = buffers[len].data[i][b] & 0x07;
        seen[v] = true;
      }

      if (seen[0] && (seen[1] || seen[2] || seen[3] || seen[4])) {
        mouseMap.btnByte = b;
        Serial.printf("Button byte detected at %d\n", b);
        return;
      }
    }

    Serial.println("Button byte NOT found!");
  }

  // ================= SCROLL DETECTION =================
  void detectWheel(int len) {

    int bestScore = 0;

    for (int b = 0; b < len; b++) {

      if (mouseMap.constantMask[b]) continue;
      if (b == mouseMap.btnByte) continue;

      int signChange = 0, activity = 0;
      int8_t last = 0;

      for (int i = 0; i < buffers[len].count; i++) {
        int8_t v = (int8_t)buffers[len].data[i][b];

        if (v != 0) {
          activity++;

          if (last != 0 && ((v > 0 && last < 0) || (v < 0 && last > 0))) {
            signChange++;
          }

          last = v;
        }
      }

      int score = signChange * 3 + activity;

      if (score > bestScore) {
        bestScore = score;
        mouseMap.wheelByte = b;
      }
    }

    Serial.printf("Scrollwheel byte detected at %d\n", mouseMap.wheelByte);
  }

  // ================= XY DETECTION =================
  void detectXY(int len) {
    struct Candidate {
      int index;
      int activity;
    };

    Candidate candidates[MAX_LEN];
    int count = 0;

    // =========================
    // 1. collect samples
    // =========================
    for (int b = 0; b < len; b++) {

      if (mouseMap.constantMask[b]) continue;
      if (b == mouseMap.btnByte) continue;
      if (b == mouseMap.wheelByte) continue;

      int activity = 0;

      for (int i = 0; i < buffers[len].count; i++) {
        int8_t v = (int8_t)buffers[len].data[i][b];
        activity += abs(v);
      }

      // choose relevant bytes only
      if (activity > 10) {
        candidates[count].index = b;
        candidates[count].activity = activity;
        count++;
      }
    }

    Serial.printf("XY candidates found: %d\n", count);

    if (count < 2) {
      Serial.println("ERROR: Not enough XY candidates!");
      return;
    }

    // =========================
    // 2. sort by position, not activity
    // =========================
    for (int i = 0; i < count - 1; i++) {
      for (int j = i + 1; j < count; j++) {
        if (candidates[j].index < candidates[i].index) {
          Candidate tmp = candidates[i];
          candidates[i] = candidates[j];
          candidates[j] = tmp;
        }
      }
    }

    // =========================
    // 3. devide → X / Y
    // =========================
    int half = count / 2;

    // fallback if uneven (3 Bytes)
    if (count == 3) {
      half = 1;  // 1 Byte X, 2 Byte Y
    }

    // =========================
    // 4. set mapping
    // =========================
    mouseMap.xByte = candidates[0].index;
    mouseMap.yByte = candidates[half].index;

    Serial.print("X bytes: ");
    for (int i = 0; i < half; i++) {
      Serial.printf("%d ", candidates[i].index);
    }

    Serial.print(" | Y bytes: ");
    for (int i = half; i < count; i++) {
      Serial.printf("%d ", candidates[i].index);
    }

    Serial.println();

    // =========================
    // 5. INFO for Debug
    // =========================
    Serial.printf("Primary X=%d, Primary Y=%d\n",
                  mouseMap.xByte,
                  mouseMap.yByte);
  }

  // ================= ANALYZE =================
  void analyze() {

    int len = getMainLen();
    mouseMap.mainLen = len;
    detectConstants(len);
    switch (state) {

      case BUTTON_DETECT:
        detectButtons(len);
        break;

      case WHEEL:
        detectWheel(len);
        break;

      case XY:
        detectXY(len);
        break;

      default: break;
    }
  }

  // ================= FLOW =================
  void nextStep() {

    ready = false;

    for (int i = 0; i < 17; i++) buffers[i].count = 0;

    switch (state) {

      case IDLE:
        Serial.println("Lift mouse and click whatever mouse buttons randomly.");
        state = BUTTON_DETECT;
        break;

      case BUTTON_DETECT:
        Serial.println("Now repeatedly move scrollwheel UP and DOWN.");
        state = WHEEL;
        break;

      case WHEEL:
        Serial.println("Now please place mouse on the table and move mouse forward and left - do NOT press any buttons.");
        state = XY;
        break;

      case XY:
        Serial.println("Great, now we finalize and safe settings for that mouse.");
        finalize();
        return;

      default: break;
    }

    Serial.println(">>> Collecting data now... <<<\n");
    collecting = true;
  }

  // ================= FINAL =================
  void finalize() {

    mouseMap.valid = true;

    prefs.begin("mouse", false);
    prefs.putBytes("map", &mouseMap, sizeof(mouseMap));
    prefs.end();

    Serial.println("\n===== FINAL RESULT =====");
    Serial.printf("len=%d btn=%d wheel=%d x=%d y=%d\n",
                  mouseMap.mainLen,
                  mouseMap.btnByte,
                  mouseMap.wheelByte,
                  mouseMap.xByte,
                  mouseMap.yByte);

    Serial.println("Restarting...");
    delay(1500);
    ESP.restart();
  }

  // ================= NORMAL RUN =================
  void runMouse(uint8_t* d, int len) {

    if (len != mouseMap.mainLen) return;

    uint8_t btn = d[mouseMap.btnByte] & 0x07;

    bool L = (btn == 1 || btn == 3);
    bool R = (btn == 2 || btn == 3);

    int8_t x = (int8_t)d[mouseMap.xByte];
    int8_t y = (int8_t)d[mouseMap.yByte];
    int8_t w = (int8_t)d[mouseMap.wheelByte];

    if (w != 0) {
      scale -= w;
      if (scale < 4) scale = 4;
      if (scale > 40) scale = 40;
    }

    Serial.printf(" - Mouse %c%c - x=%d y=%d Zoom=%d%%\n",
                  L ? 'L' : ' ',
                  R ? 'R' : ' ',
                  x, y,
                  (int)(20.0 / scale * 100));

    lastX = x;
    lastY = y;
    leftBtn = L;
    rightBtn = R;
  }

} usbHost;

// ========================= MSX PROTOKOLL =========================
inline void lineLow(uint32_t mask) {
  GPIO.out_w1tc = mask;
  GPIO.enable_w1ts = mask;
}
inline void lineRelease(uint32_t mask) {
  GPIO.enable_w1tc = mask;
}

void sendNibble(uint8_t n) {
  uint32_t lowMask = 0, relMask = 0;
  if (n & 8) relMask |= B3;
  else lowMask |= B3;
  if (n & 4) relMask |= B2;
  else lowMask |= B2;
  if (n & 2) relMask |= B1;
  else lowMask |= B1;
  if (n & 1) relMask |= B0;
  else lowMask |= B0;
  GPIO.out_w1tc = lowMask;
  GPIO.enable_w1ts = lowMask;
  GPIO.enable_w1tc = relMask;
}

void sendMSX(int8_t c) {
  while (digitalRead(PIN_STROBE) == LOW)
    ;
  sendNibble((c >> 4) & 0xF);
  while (digitalRead(PIN_STROBE) == HIGH)
    ;
  sendNibble(c & 0xF);
}

// ========================= SETUP =========================
unsigned long t0;
bool pressed = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Please wait..");
  delay(500);
  Serial.println("OK..");


  pinMode(0, INPUT_PULLUP);

  prefs.begin("mouse", true);
  if (prefs.getBytesLength("map") == sizeof(mouseMap)) {
    prefs.getBytes("map", &mouseMap, sizeof(mouseMap));
    mouseMap.valid = true;
    Serial.println("Mapping loaded.");
  }
  prefs.end();

  usbHost.begin();

  Serial.println("To start calibrating the mouse attached via USB pick it up from the table, then press BOOT more than 3 seconds");
}

// ========================= LOOP =========================
void loop() {

  usbHost.task();

  bool now = digitalRead(0) == LOW;

  if (now && !pressed) {
    t0 = millis();
    pressed = true;
  }

  if (!now && pressed) {
    pressed = false;

    if (usbHost.state == IDLE && millis() - t0 > 3000) {
      Serial.println("\n=== CALIBRATION START ===");
      usbHost.nextStep();
      return;
    }

    if (usbHost.ready) {
      usbHost.nextStep();
    }
  }

  delay(5);
}
