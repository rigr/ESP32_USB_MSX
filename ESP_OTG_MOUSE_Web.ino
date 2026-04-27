// =======================================================================
// ESP32-S3 USB Mouse → Roland Sampler
// ROBUST AUTO-LEARNING VERSION + WEB INTERFACE (on demand)
// Web interface - accesspoint USB-MOUSE, PW: USB-MOUSE
// 
//
// Richard, 27.4.2026
// =======================================================================
#include "EspUsbHost.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include <Preferences.h>
#include <esp_system.h>
#include <WiFi.h>
#include <WebServer.h>

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

// ========================= WEB INTERFACE =========================
const char* ap_ssid = "USB-MOUSE";
const char* ap_password = "USB-MOUSE";
WebServer webServer(80);

bool webActive = false;
String currentInstruction = "";
String statusMessage = "Bereit zum Starten...";
bool collectingData = false;

// ========================= USB HOST CLASS =========================
class MyEspUsbHost : public EspUsbHost {
public:
  State state = IDLE;
  bool collecting = false;
  bool ready = false;

  void onReceive(const usb_transfer_t* t) override {
    if (!t || !t->data_buffer) return;
    uint8_t* d = t->data_buffer;
    int len = t->actual_num_bytes;
    if (len > MAX_LEN) return;

    if (!collecting && mouseMap.valid) {
      printRaw(d, len);
      runMouse(d, len);
      return;
    }

    if (!collecting) return;
    if (buffers[len].count >= MAX_SAMPLES) return;

    memcpy(buffers[len].data[buffers[len].count], d, len);
    buffers[len].count++;

    Serial.printf("Collecting (%d/%d)\n", buffers[len].count, MAX_SAMPLES);

    if (buffers[len].count >= MAX_SAMPLES) {
      collecting = false;
      ready = true;
      collectingData = false;
      Serial.println("\n=== Enough data collected ===");
      analyze();
      updateWebInstruction();
      Serial.println(">>> Press BOOT or click OK to continue <<<\n");
    }
  }

  void printRaw(uint8_t* d, int len) {
    Serial.printf("RAW len=%d | ", len);
    for (int i = 0; i < len; i++) Serial.printf("%02X ", d[i]);
    Serial.println();
  }

  int getMainLen() {
    int bestLen = 0, bestScore = 0;
    for (int len = 0; len < 17; len++) {
      if (buffers[len].count < 10) continue;
      int var = 0;
      for (int b = 0; b < len; b++) {
        uint8_t f = buffers[len].data[0][b];
        for (int i = 1; i < buffers[len].count; i++) {
          if (buffers[len].data[i][b] != f) { var++; break; }
        }
      }
      if (var > bestScore) { bestScore = var; bestLen = len; }
    }
    Serial.printf("Selected main len=%d\n", bestLen);
    return bestLen;
  }

  void detectConstants(int len) {
    for (int b = 0; b < len; b++) {
      bool same = true;
      uint8_t v = buffers[len].data[0][b];
      for (int i = 1; i < buffers[len].count; i++) {
        if (buffers[len].data[i][b] != v) { same = false; break; }
      }
      mouseMap.constantMask[b] = same;
    }
  }

  void detectButtons(int len) {
    for (int b = 0; b < len; b++) {
      bool seen[8] = { false };
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

  void detectWheel(int len) {
    int bestScore = 0;
    for (int b = 0; b < len; b++) {
      if (mouseMap.constantMask[b] || b == mouseMap.btnByte) continue;
      int signChange = 0, activity = 0;
      int8_t last = 0;
      for (int i = 0; i < buffers[len].count; i++) {
        int8_t v = (int8_t)buffers[len].data[i][b];
        if (v != 0) {
          activity++;
          if (last != 0 && ((v > 0 && last < 0) || (v < 0 && last > 0))) signChange++;
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

  void detectXY(int len) {
    struct Candidate { int index; int activity; };
    Candidate candidates[MAX_LEN];
    int count = 0;

    for (int b = 0; b < len; b++) {
      if (mouseMap.constantMask[b] || b == mouseMap.btnByte || b == mouseMap.wheelByte) continue;
      int activity = 0;
      for (int i = 0; i < buffers[len].count; i++) {
        int8_t v = (int8_t)buffers[len].data[i][b];
        activity += abs(v);
      }
      if (activity > 10) {
        candidates[count].index = b;
        candidates[count].activity = activity;
        count++;
      }
    }

    Serial.printf("XY candidates found: %d\n", count);
    if (count < 2) { Serial.println("ERROR: Not enough XY candidates!"); return; }

    for (int i = 0; i < count - 1; i++) {
      for (int j = i + 1; j < count; j++) {
        if (candidates[j].index < candidates[i].index) {
          Candidate tmp = candidates[i];
          candidates[i] = candidates[j];
          candidates[j] = tmp;
        }
      }
    }

    int half = count / 2;
    if (count == 3) half = 1;

    mouseMap.xByte = candidates[0].index;
    mouseMap.yByte = candidates[half].index;
    Serial.printf("Primary X=%d, Primary Y=%d\n", mouseMap.xByte, mouseMap.yByte);
  }

  void analyze() {
    int len = getMainLen();
    mouseMap.mainLen = len;
    detectConstants(len);

    switch (state) {
      case BUTTON_DETECT: detectButtons(len); break;
      case WHEEL:         detectWheel(len);   break;
      case XY:            detectXY(len);      break;
      default: break;
    }
  }

  void nextStep();

  void finalize() {
    mouseMap.valid = true;
    prefs.begin("mouse", false);
    prefs.putBytes("map", &mouseMap, sizeof(mouseMap));
    prefs.end();

    Serial.println("\n===== FINAL RESULT =====");
    Serial.printf("len=%d btn=%d wheel=%d x=%d y=%d\n",
                  mouseMap.mainLen, mouseMap.btnByte, mouseMap.wheelByte,
                  mouseMap.xByte, mouseMap.yByte);

    currentInstruction = "Kalibrierung erfolgreich abgeschlossen!<br>Das Gerät startet neu...";
    statusMessage = "Fertig";
    collectingData = false;
    delay(1500);
    ESP.restart();
  }

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
                  L ? 'L' : ' ', R ? 'R' : ' ', x, y, (int)(20.0 / scale * 100));

    lastX = x; lastY = y; leftBtn = L; rightBtn = R;
  }

  void updateWebInstruction() {
    if (collecting) {
      int currentLen = (mouseMap.mainLen > 0) ? mouseMap.mainLen : 8;
      statusMessage = "Daten werden gesammelt... (" + String(buffers[currentLen].count) + "/" + String(MAX_SAMPLES) + ")";
    } else {
      statusMessage = "Bereit – drücken Sie OK / Weiter";
    }
  }
} usbHost;

// ========================= WEB HANDLER =========================
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>USB Mouse Calibration</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <meta http-equiv="refresh" content="3">
  <style>
    body { font-family: Arial, sans-serif; margin: 20px; background: #f4f4f4; text-align: center; }
    .box { background: white; padding: 25px; border-radius: 12px; box-shadow: 0 4px 15px rgba(0,0,0,0.1); max-width: 680px; margin: 0 auto; }
    h1 { color: #333; margin-bottom: 10px; }
    .instruction { font-size: 18px; line-height: 1.6; margin: 20px 0; text-align: left; display: inline-block; }
    .status { font-size: 17px; font-weight: bold; color: #0066cc; margin: 15px 0; }
    button { 
      background: #0066cc; color: white; font-size: 20px; padding: 18px 50px; 
      border: none; border-radius: 10px; cursor: pointer; margin-top: 25px; width: 90%; max-width: 400px;
    }
    button:hover { background: #0055aa; }
    button:disabled { background: #888; cursor: not-allowed; }
    .footer { margin-top: 40px; font-size: 14px; color: #666; }
  </style>
</head>
<body>
  <div class="box">
    <h1>Welcome.</h1>
    <div class="instruction">
)rawliteral";

  html += currentInstruction;
  html += R"rawliteral(
    </div>
    <div class="status">)rawliteral";
  html += statusMessage;
  html += R"rawliteral(</div>
    <form action="/next" method="POST">
      <button type="submit" )rawliteral";
  if (collectingData) html += "disabled";
  html += R"rawliteral(>OK / Weiter</button>
    </form>
    <div class="footer">
      Richard - 27.4.2026<br>
      <a href="https://github.com/rigr/ESP32_USB_MSX" target="_blank">https://github.com/rigr/ESP32_USB_MSX</a>
    </div>
  </div>
</body>
</html>
)rawliteral";

  webServer.send(200, "text/html", html);
}

void handleNext() {
  if (webActive && !collectingData && usbHost.ready) {
    usbHost.nextStep();
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

// ========================= nextStep =========================
void MyEspUsbHost::nextStep() {
  ready = false;
  collectingData = true;
  for (int i = 0; i < 17; i++) buffers[i].count = 0;

  switch (state) {
    case IDLE:
      // Erster Schritt: Nur Anweisung, KEIN Sammeln, aber ready = true damit OK funktioniert
      Serial.println("Step 1: Lift the mouse from the desk.");
      currentInstruction = "1) Lift the mouse from the desk, so no movement is recognized.<br>Then press OK";
      statusMessage = "Maus angehoben? → Drücken Sie OK / Weiter";
      state = BUTTON_DETECT;
      collecting = false;      // WICHTIG: Noch nicht sammeln
      ready = true;            // Button/BOOT kann weitergehen
      collectingData = false;
      break;

    case BUTTON_DETECT:
      Serial.println("Step 2: Click left and right mouse buttons several times.");
      currentInstruction = "2) Click left and right mouse buttons several times.<br>Then confirm pressing OK";
      statusMessage = "Warte auf OK...";
      state = WHEEL;
      collecting = true;       // Ab hier sammeln
      break;

    case WHEEL:
      Serial.println("Step 3: Move scrollwheel UP and DOWN several times.");
      currentInstruction = "3) Move scrollwheel UP and DOWN several times.<br>Then confirm pressing OK";
      statusMessage = "Warte auf OK...";
      state = XY;
      collecting = true;
      break;

    case XY:
      Serial.println("Step 4: Move mouse forward/back and left/right.");
      currentInstruction = "4) Set the mouse on the table and move it forward and to the left and in the opposite direction.<br>Confirm pressing OK again.<br> After this step the mouse will restart.";
      statusMessage = "Warte auf OK...";
      state = DONE;
      collecting = true;
      break;

    case DONE:
      Serial.println("Finalizing calibration...");
      finalize();
      return;
  }

  if (collecting) {
    Serial.println(">>> Collecting data now... <<<\n");
  }
  updateWebInstruction();
}

// ========================= MSX PROTOKOLL =========================
inline void lineLow(uint32_t mask) { GPIO.out_w1tc = mask; GPIO.enable_w1ts = mask; }
inline void lineRelease(uint32_t mask) { GPIO.enable_w1tc = mask; }

void sendNibble(uint8_t n) {
  uint32_t lowMask = 0, relMask = 0;
  if (n & 8) relMask |= B3; else lowMask |= B3;
  if (n & 4) relMask |= B2; else lowMask |= B2;
  if (n & 2) relMask |= B1; else lowMask |= B1;
  if (n & 1) relMask |= B0; else lowMask |= B0;
  GPIO.out_w1tc = lowMask; GPIO.enable_w1ts = lowMask; GPIO.enable_w1tc = relMask;
}

void sendMSX(int8_t c) {
  while (digitalRead(PIN_STROBE) == LOW);
  sendNibble((c >> 4) & 0xF);
  while (digitalRead(PIN_STROBE) == HIGH);
  sendNibble(c & 0xF);
}

// ========================= SETUP & LOOP =========================
unsigned long t0 = 0;
bool pressed = false;

void startWebInterface() {
  if (webActive) return;
  WiFi.softAP(ap_ssid, ap_password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("Access Point gestartet → SSID: ");
  Serial.println(ap_ssid);
  Serial.print("IP: ");
  Serial.println(myIP);

  webServer.on("/", handleRoot);
  webServer.on("/next", HTTP_POST, handleNext);
  webServer.begin();

  webActive = true;
  Serial.println("Webinterface ist jetzt aktiv.");
}

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

  currentInstruction = "Halte die BOOT-Taste am Board länger als 3 Sekunden,<br>um die Kalibrierung und das Webinterface zu starten.";

  Serial.println("Halte die BOOT-Taste länger als 3 Sekunden zum Starten der Kalibrierung.");
}

void loop() {
  usbHost.task();

  if (webActive) {
    webServer.handleClient();
  }

  bool now = digitalRead(0) == LOW;
  if (now && !pressed) {
    t0 = millis();
    pressed = true;
  }

  if (!now && pressed) {
    pressed = false;
    unsigned long duration = millis() - t0;

    if (duration > 3000) {
      if (!webActive) {
        Serial.println("\n=== STARTING CALIBRATION + WEB INTERFACE ===");
        startWebInterface();
        usbHost.state = IDLE;
        usbHost.nextStep();        // Zeigt Schritt 1 an
      } else if (usbHost.ready && !usbHost.collecting) {
        usbHost.nextStep();
      }
    } else if (duration > 50 && webActive && usbHost.ready && !usbHost.collecting) {
      usbHost.nextStep();
    }
  }

  if (usbHost.collecting) {
    usbHost.updateWebInstruction();
  }

  delay(5);
}
