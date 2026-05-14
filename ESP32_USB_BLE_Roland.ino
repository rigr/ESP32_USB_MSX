// =================================================================
// ESP32 USB BLE MSX MOUSE - VERSION 01 - WITH CALIBRATION
// BLE + USB Auto-Learning Calibration via Web Interface
// https://github.com/rigr/ESP32_USB_MSX
//
// IMPORTANT: Include order matters! EspUsbHost must be included BEFORE NimBLE
// to avoid HID type conflicts. See: https://github.com/h2zero/NimBLE-Arduino/issues/715
//
// Board: ESP32-WROOM-32D (30 pins).  Board defintion 3.00 von espressif
// NimBLE Version: 2.1.0 by h2zero
//
// rigr 2026-05-13
//
// Still needs some more tests to check, if Roland Sampler will really accept the mouse
// Feel free to inform me if it works or if it does'nt. Use "issues" on github
//
// =================================================================

// USB Host includes FIRST (before NimBLE to avoid HID type conflicts)
#include "EspUsbHost.h"
#include <Preferences.h>
#include <esp_system.h>

// Then BLE
#include <NimBLEDevice.h>

// Then everything else
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <HTTPUpdateServer.h>
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"
#include <vector>
#include "freertos/semphr.h"

#define MAX_SAMPLES 50
#define MAX_LEN 16

// =================================================================
// BLE Configuration
// =================================================================
NimBLEClient* client = nullptr;
bool connected = false;
bool isScanning = false;
unsigned long lastAlive = 0;
int currentRSSI = -999;
std::string mouseName = "";
std::string mouseAddr = "";

struct DeviceInfo {
  std::string name;
  NimBLEAddress addr;
  int rssi;
};
std::vector<DeviceInfo> deviceList;
int selectedDevice = -1;
bool selectingDevice = false;
unsigned long selectionStart = 0;

// =================================================================
// HID Parser Structures (Fallback when no calibration)
// =================================================================
struct HIDField {
  uint16_t usage;
  uint8_t reportID;
  uint16_t bitOffset;
  uint8_t bitSize;
  bool isSigned;
  int32_t rawValue;
  bool valueChanged;
};

struct HIDMouseFormat {
  uint8_t reportID = 0;
  HIDField x, y, wheel;
  HIDField leftButton;
  HIDField rightButton;
  bool valid = false;
  bool hasExplicitButtons = false;
  bool hasLeftButton = false;
  bool hasRightButton = false;
  bool usesStandardMouseFormat = false;
};

HIDMouseFormat hidFmt;

// =================================================================
// CALIBRATION - Auto Learning 
// =================================================================
struct MouseMapping {
  uint8_t mainLen;
  uint8_t btnByte;
  uint8_t wheelByte;
  uint8_t xByte;
  uint8_t yByte;
  bool constantMask[MAX_LEN];
  bool valid;
} mouseMap;

struct Buffer {
  uint8_t data[MAX_SAMPLES][MAX_LEN];
  int count = 0;
};
Buffer buffers[17];

enum CalState {
  CAL_IDLE,
  CAL_LIFT,
  CAL_BUTTON_DETECT,
  CAL_WHEEL,
  CAL_XY,
  CAL_DONE
};

enum MouseType {
  MOUSE_NONE,
  MOUSE_BLE,
  MOUSE_USB
};

MouseType currentMouseType = MOUSE_NONE;
MouseType selectedMouseType = MOUSE_NONE;
CalState calState = CAL_IDLE;
bool collectingData = false;
bool calReady = false;
String calInstruction = "";
String calStatus = "";
int sampleCounter = 0;

// =================================================================
// ESP32 Serial / MAC
// =================================================================
uint64_t chipid;
char ssid[23];

// =================================================================
// WiFi/Web Configuration
// =================================================================
const char* ap_ssid = "ESP32_MOUSE";
const char* ap_password = "1234567890";
WebServer server(80);
HTTPUpdateServer httpUpdater;
bool webServerActive = false;
bool calibrationMode = false;

// =================================================================
// Pin Definitions
// =================================================================
const uint8_t MX0_PIN = 14;
const uint8_t MX1_PIN = 13;
const uint8_t MX2_PIN = 12;
const uint8_t MX3_PIN = 11;
const uint8_t MX4_PIN =  8;
const uint8_t MX5_PIN =  3;
const uint8_t CS_PIN =  46;
const uint8_t MAN_SCAN = 15;
const uint8_t LED_PIN =  2;
const uint8_t BOOT_PIN = 0;

#define B0 (1UL << MX0_PIN)
#define B1 (1UL << MX1_PIN)
#define B2 (1UL << MX2_PIN)
#define B3 (1UL << MX3_PIN)

// =================================================================
// Global Variables
// =================================================================
TaskHandle_t msxTaskHandle = nullptr;
volatile bool msxEnabled = true;

volatile int16_t lastX = 0, lastY = 0;
volatile bool leftBtn = false, rightBtn = false;
volatile bool lastLeftBtn = false, lastRightBtn = false;
unsigned long lastMouseUpdate = 0;

volatile int8_t wheelDelta = 0;
volatile char currentScale = 15;
const char minScale = 4;
const char maxScale = 40;
bool scaleChanged = false;

unsigned long leftButtonPressTime = 0;
unsigned long rightButtonPressTime = 0;
unsigned long bootButtonPressTime = 0;
bool leftButtonPressed = false;
bool rightButtonPressed = false;
bool bootButtonPressed = false;
bool lastBootButtonState = false;

const unsigned long WEB_START_BOOT_THRESHOLD = 3000;
const unsigned long WEB_STOP_BOOT_THRESHOLD = 6000;
const unsigned long DEBOUNCE_DELAY = 300;

SemaphoreHandle_t scaleMutex;
Preferences prefs;
const char* NVS_NAMESPACE = "msxmous";

// =================================================================
// USB HOST CLASS 
// =================================================================
class MyEspUsbHost : public EspUsbHost {
public:
  bool collecting = false;
  bool ready = false;

  void onReceive(const usb_transfer_t* t) override {
    if (!t || !t->data_buffer) return;
    uint8_t* d = t->data_buffer;
    int len = t->actual_num_bytes;
    if (len > MAX_LEN) return;

    if (!collecting && mouseMap.valid && currentMouseType == MOUSE_USB) {
      printRaw(d, len);
      runMouse(d, len);
      return;
    }

    if (!collecting) {
      printRaw(d, len);
      return;
    }

    if (buffers[len].count >= MAX_SAMPLES) return;

    memcpy(buffers[len].data[buffers[len].count], d, len);
    buffers[len].count++;
    sampleCounter = buffers[len].count;

    Serial.printf("Collecting (%d/%d)\n", buffers[len].count, MAX_SAMPLES);

    if (buffers[len].count >= MAX_SAMPLES) {
      collecting = false;
      ready = true;
      collectingData = false;
      Serial.println("\n=== Enough data collected ===");
      analyze();
      calStatus = "READY - press OK";
      Serial.println(">>> Press BOOT or click OK to continue <<<");
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
    struct Candidate {
      int index;
      int activity;
    };
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
    if (count < 2) {
      Serial.println("ERROR: Not enough XY candidates!");
      return;
    }

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

    switch (calState) {
      case CAL_BUTTON_DETECT: detectButtons(len); break;
      case CAL_WHEEL: detectWheel(len); break;
      case CAL_XY: detectXY(len); break;
      default: break;
    }
  }

  void finalize() {
    mouseMap.valid = true;
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putInt("mouseType", (int)currentMouseType);
    if (currentMouseType == MOUSE_BLE) {
      prefs.putBytes("bleMap", &mouseMap, sizeof(mouseMap));
      prefs.putString("lastBLEAddr", String(mouseAddr.c_str()));
    } else if (currentMouseType == MOUSE_USB) {
      prefs.putBytes("usbMap", &mouseMap, sizeof(mouseMap));
    }
    prefs.end();

    Serial.println("\n===== FINAL RESULT =====");
    Serial.printf("len=%d btn=%d wheel=%d x=%d y=%d\n",
                  mouseMap.mainLen, mouseMap.btnByte, mouseMap.wheelByte,
                  mouseMap.xByte, mouseMap.yByte);
    Serial.println("\n===== CALIBRATION SAVED =====");

    calInstruction = "Calibration successfully finished!<br>Mouse is ready to use.";
    calStatus = "Done";
    collectingData = false;
    calState = CAL_DONE;

    if (currentMouseType == MOUSE_USB) {
      Serial.println("\n===== RESTARTING NOW (USB mode) =====");
      delay(1500);
      ESP.restart();
    } else {
      Serial.println("\n===== BLE CALIBRATION COMPLETE - NO RESTART NEEDED =====");
      calInstruction += "<br><br><a href='/'><button style='width:100%;padding:12px;font-size:16px;'>Back to Main</button></a>";
    }
  }

  void runMouse(uint8_t* d, int len) {
    if (len != mouseMap.mainLen) return;
    uint8_t btn = d[mouseMap.btnByte] & 0x07;
    bool L = (btn == 1 || btn == 3 || btn == 5 || btn == 7);
    bool R = (btn == 2 || btn == 3 || btn == 6 || btn == 7);
    int8_t x = (int8_t)d[mouseMap.xByte];
    int8_t y = (int8_t)d[mouseMap.yByte];
    int8_t w = (int8_t)d[mouseMap.wheelByte];

    if (w != 0) {
      if (scaleMutex != NULL) xSemaphoreTake(scaleMutex, portMAX_DELAY);
      currentScale -= w;
      if (currentScale < minScale) currentScale = minScale;
      if (currentScale > maxScale) currentScale = maxScale;
      scaleChanged = true;
      if (scaleMutex != NULL) xSemaphoreGive(scaleMutex);
    }

    Serial.printf(" - Mouse %c%c - x=%d y=%d Zoom=%d%%\n",
                  L ? 'L' : ' ', R ? 'R' : ' ', x, y, (int)(20.0 / currentScale * 100));

    lastX = x;
    lastY = y;
    leftBtn = L;
    rightBtn = R;
    lastMouseUpdate = millis();
  }

  void resetBuffers() {
    for (int i = 0; i < 17; i++) buffers[i].count = 0;
  }
} usbHost;

// =================================================================
// Function Prototypes
// =================================================================
void notifyCB(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool isNotify);
bool connectTo(const NimBLEAdvertisedDevice* dev);
void scanAndConnect();
void scanDevices();
void printDeviceList();
void connectSelectedDevice();
void disconnectMouse();
void setupWebServer();
void startWebServer();
void startCalibrationMode();
void msxProtocolTask(void* parameter);
void sendMSX(int8_t c);
void bootButtonISR();
void updateLED();
void checkLongButtonPresses();
void handleWebCommand(String cmd);
void handleSerialCommand(String cmd);

void calibrationNextStep();
void resetCalibration();
int32_t extractBits(uint8_t* data, int bitOffset, int bitSize, bool isSigned);
void parseHIDReport(uint8_t* data, size_t len);
void interpretMouseData(uint8_t* data, size_t len, int16_t& x, int16_t& y, int8_t& wheel, bool& leftButton, bool& rightButton);

// =================================================================
// GPIO FUNCTIONS - Direct Register Access
// =================================================================
inline void lineLow(uint32_t mask) {
  GPIO.out_w1tc = mask;
  GPIO.enable_w1ts = mask;
}
inline void lineRelease(uint32_t mask) {
  GPIO.enable_w1tc = mask;
}
inline void sendBit(uint32_t mask, bool bit) {
  if (bit)
    lineRelease(mask);
  else
    lineLow(mask);
}
void sendNibble(uint8_t n) {
  uint32_t lowMask = 0;
  uint32_t relMask = 0;

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

// =================================================================
// BOOT BUTTON ISR
// =================================================================
void IRAM_ATTR bootButtonISR() {
  bool currentButtonState = !digitalRead(BOOT_PIN);

  if (currentButtonState != lastBootButtonState) {
    lastBootButtonState = currentButtonState;

    if (currentButtonState) {
      bootButtonPressTime = millis();
      bootButtonPressed = true;
    } else {
      unsigned long pressDuration = millis() - bootButtonPressTime;
      bootButtonPressed = false;

      if (pressDuration < DEBOUNCE_DELAY && !webServerActive && !calibrationMode && !isScanning && !connected) {
        scanDevices();
      }
    }
  }
}

// =================================================================
// Detect long press of boot
// =================================================================
void checkLongButtonPresses() {
  unsigned long currentTime = millis();

  if (bootButtonPressed) {
    unsigned long pressDuration = currentTime - bootButtonPressTime;

    if (pressDuration >= WEB_START_BOOT_THRESHOLD && pressDuration < WEB_STOP_BOOT_THRESHOLD) {
      if (!webServerActive && !calibrationMode) {
        startCalibrationMode();
      }
      bootButtonPressed = false;
    } else if (pressDuration >= WEB_STOP_BOOT_THRESHOLD) {
      if (webServerActive || calibrationMode) {
        server.stop();
        WiFi.softAPdisconnect(true);
        webServerActive = false;
        calibrationMode = false;
        for (int i = 0; i < 2; i++) {
          digitalWrite(LED_PIN, HIGH);
          delay(500);
          digitalWrite(LED_PIN, LOW);
          delay(500);
        }
        Serial.println("Web server stopped");
      }
      bootButtonPressed = false;
    }
  }
}

// =================================================================
// LED Update Function
// =================================================================
void updateLED() {
  static unsigned long lastBlink = 0;
  static bool state = false;
  int interval = 0;

  if (calibrationMode) {
    interval = 150;
  } else if (webServerActive) {
    interval = 200;
  } else if (isScanning) {
    interval = 100;
  } else if (selectingDevice) {
    interval = 250;
  } else if (connected) {
    digitalWrite(LED_PIN, HIGH);
    return;
  } else {
    digitalWrite(LED_PIN, LOW);
    return;
  }

  if (millis() - lastBlink > interval) {
    lastBlink = millis();
    state = !state;
    digitalWrite(LED_PIN, state);
  }
}

// =================================================================
// Calibration Mode Start
// =================================================================
void startCalibrationMode() {
  if (calibrationMode) return;

  WiFi.softAP(ap_ssid, ap_password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("MSX Calibration AP ready: ");
  Serial.println(IP);

  calibrationMode = true;
  webServerActive = true;

  setupWebServer();
  httpUpdater.setup(&server);
  Serial.println("OTA Update ready");

  server.begin();

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }

  Serial.println("Calibration web interface started");
  Serial.println("Connect to WiFi: ESP32_MOUSE / ESP32_MOUSE");
  Serial.println("Open: http://" + IP.toString());
}

// =================================================================
// SCAN FUNCTIONS - Device List Management
// =================================================================

void scanDevices() {
  if (isScanning || connected) return;

  Serial.println("Starting 20s BLE scan...");
  deviceList.clear();
  selectedDevice = -1;
  selectingDevice = false;

  isScanning = true;
  updateLED();

  NimBLEScan* scan = NimBLEDevice::getScan();
  bool scanSuccess = scan->start(20, true);
  NimBLEScanResults results = scan->getResults();

  Serial.print("Scan completed. ");
  Serial.print(results.getCount());
  Serial.println(" devices found.");

  for (int i = 0; i < results.getCount(); i++) {
    const NimBLEAdvertisedDevice* dev = results.getDevice(i);
    DeviceInfo info;
    info.name = dev->getName();
    info.addr = dev->getAddress();
    info.rssi = dev->getRSSI();
    deviceList.push_back(info);
  }

  std::sort(deviceList.begin(), deviceList.end(),
            [](const DeviceInfo& a, const DeviceInfo& b) {
              return a.rssi > b.rssi;
            });

  if (!deviceList.empty()) {
    selectedDevice = 0;
    selectingDevice = true;
    selectionStart = millis();
    printDeviceList();
  } else {
    Serial.println("No BLE devices found.");
    selectingDevice = false;
  }

  isScanning = false;
  updateLED();
}

void printDeviceList() {
  Serial.println("Available BLE devices:");
  for (int i = 0; i < deviceList.size(); i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(deviceList[i].name.c_str());
    Serial.print("  RSSI:");
    Serial.println(deviceList[i].rssi);
  }
  Serial.println("Use command: select <nr> or Web Interface");
}

void connectSelectedDevice() {
  if (selectedDevice < 0 || selectedDevice >= (int)deviceList.size()) {
    Serial.println("No valid device selected");
    return;
  }

  const NimBLEAddress& addr = deviceList[selectedDevice].addr;
  Serial.print("Connecting to device: ");
  Serial.print(deviceList[selectedDevice].name.c_str());
  Serial.print(" (");
  Serial.print(addr.toString().c_str());
  Serial.println(")");

  NimBLEScan* scan = NimBLEDevice::getScan();
  const NimBLEAdvertisedDevice* dev = scan->getResults().getDevice(addr);

  if (dev != nullptr) {
    connectTo(dev);
  } else {
    Serial.println("Device not in current scan results - starting short scan...");
    isScanning = true;
    updateLED();
    scan->start(5, false);
    delay(5000);
    const NimBLEAdvertisedDevice* dev2 = scan->getResults().getDevice(addr);
    if (dev2 != nullptr) {
      connectTo(dev2);
    } else {
      Serial.println("Could not find device by address");
      Serial.println("Please run 'scan' command and try again");
    }
    isScanning = false;
    updateLED();
  }

  selectingDevice = false;
}

// =====================================================================
// MSX PROTOCOL - DUAL CORE TASK (Core 1)
// =====================================================================

void msxProtocolTask(void* parameter) {
  pinMode(MX0_PIN, INPUT);
  pinMode(MX1_PIN, INPUT);
  pinMode(MX2_PIN, INPUT);
  pinMode(MX3_PIN, INPUT);
  pinMode(CS_PIN, INPUT_PULLUP);

  digitalWrite(MX0_PIN, LOW);
  digitalWrite(MX1_PIN, LOW);
  digitalWrite(MX2_PIN, LOW);
  digitalWrite(MX3_PIN, LOW);

  pinMode(MX4_PIN, OUTPUT);
  pinMode(MX5_PIN, OUTPUT);
  digitalWrite(MX4_PIN, HIGH);
  digitalWrite(MX5_PIN, HIGH);

  static int16_t mx = 0, my = 0;

  Serial.println("MSX Protocol Task started on Core 1");
  Serial.println("Pin Configuration:");
  Serial.println("MX0=14, MX1=13, MX2=12, MX3=11, MX4=8(L), MX5=3(R), CS=46, SCAN=15, LED=2");
  Serial.println(" ");
  Serial.print("Initial Zoom Factor: ");
  Serial.print((int)currentScale);
  Serial.print(" (");
  Serial.print((int)(20.0 / currentScale * 100));
  Serial.println("%)");
  Serial.println("Zoom Range: 4-40 (20%-200%)");

  while (1) {
    if (!msxEnabled) {
      delay(10);
      continue;
    }

    if (scaleMutex != NULL) {
      xSemaphoreTake(scaleMutex, portMAX_DELAY);
    }
    int16_t currentX = lastX;
    int16_t currentY = lastY;
    bool currentLeftBtn = leftBtn;
    bool currentRightBtn = rightBtn;
    lastX = 0;
    lastY = 0;
    if (scaleMutex != NULL) {
      xSemaphoreGive(scaleMutex);
    }

    mx += (currentX * -1);
    my += (currentY * -1);

    while (digitalRead(CS_PIN) == LOW) {
      delayMicroseconds(20);
      yield();
    }

    char x = (char)(mx * currentScale / 15);
    char y = (char)(my * currentScale / 15);

    sendMSX(x);

    unsigned long timeout = millis() + 20;
    while (digitalRead(CS_PIN) == HIGH && millis() < timeout) {
      delayMicroseconds(20);
    }
    if (millis() >= timeout) continue;

    sendMSX(y);

    mx = 0;
    my = 0;

    if (currentLeftBtn) {
      digitalWrite(MX4_PIN, LOW);
    } else {
      digitalWrite(MX4_PIN, HIGH);
    }

    if (currentRightBtn) {
      digitalWrite(MX5_PIN, LOW);
    } else {
      digitalWrite(MX5_PIN, HIGH);
    }

    delay(1);
  }
}

void sendMSX(int8_t c) {
  while (digitalRead(CS_PIN) == LOW)
    ;
  sendNibble((c >> 4) & 0xF);
  while (digitalRead(CS_PIN) == HIGH)
    ;
  sendNibble(c & 0xF);
}

// =================================================================
// HID Parser Functions (Fallback)
// =================================================================

int32_t extractBits(uint8_t* data, int bitOffset, int bitSize, bool isSigned) {
  int32_t value = 0;
  for (int i = 0; i < bitSize; i++) {
    int byteIndex = (bitOffset + i) / 8;
    int bitIndex = (bitOffset + i) % 8;
    if (data[byteIndex] & (1 << bitIndex)) {
      value |= (1 << i);
    }
  }
  if (isSigned && (value & (1 << (bitSize - 1)))) {
    value |= (-1 << bitSize);
  }
  return value;
}

void parseHIDReport(uint8_t* data, size_t len) {
  uint8_t reportSize = 0;
  uint8_t reportCount = 0;
  uint8_t reportID = 0;
  uint16_t bitOffset = 0;
  uint8_t currentUsagePage = 0;
  uint16_t usageList[16];
  int usageCount = 0;

  hidFmt = {};

  Serial.println("Parsing HID Report Map:");

  for (int i = 0; i < len; i++) {
    uint8_t b = data[i];

    if (b == 0x85) {
      reportID = data[++i];
      Serial.printf("  Found Report ID: %d", reportID);
      bitOffset = 0;
    }
    else if (b == 0x05) {
      currentUsagePage = data[++i];
      Serial.printf("  Usage Page: 0x%02X", currentUsagePage);
    }
    else if (b == 0x09) {
      if (usageCount < 16) {
        uint8_t usage = data[++i];
        usageList[usageCount++] = usage;
        Serial.printf("  Usage: 0x%02X", usage);
      }
    }
    else if (b == 0x75) {
      reportSize = data[++i];
      Serial.printf("  Report Size: %d", reportSize);
    }
    else if (b == 0x95) {
      reportCount = data[++i];
      Serial.printf("  Report Count: %d", reportCount);
    }
    else if (b == 0x81) {
      uint8_t input = data[++i];
      Serial.printf("  Input found with properties: 0x%02X", input);

      for (int j = 0; j < reportCount; j++) {
        uint16_t u = (j < usageCount) ? usageList[j] : 0;

        if (currentUsagePage == 0x01) {
          if (u == 0x30) {
            hidFmt.x = { u, reportID, bitOffset, reportSize, true };
            Serial.printf("  X-Axis @bitOffset=%d, bitSize=%d", hidFmt.x.bitOffset, hidFmt.x.bitSize);
          }
          else if (u == 0x31) {
            hidFmt.y = { u, reportID, bitOffset, reportSize, true };
            Serial.printf("  Y-Axis @bitOffset=%d, bitSize=%d", hidFmt.y.bitOffset, hidFmt.y.bitSize);
          }
          else if (u == 0x38) {
            hidFmt.wheel = { u, reportID, bitOffset, reportSize, true };
            Serial.printf("  Wheel @bitOffset=%d, bitSize=%d", hidFmt.wheel.bitOffset, hidFmt.wheel.bitSize);
          }
        }
        else if (currentUsagePage == 0x09) {
          hidFmt.hasExplicitButtons = true;
          if (u >= 0x01 && u <= 0x08) {
            if (u == 0x01) {
              hidFmt.leftButton = { u, reportID, bitOffset, reportSize, false };
              hidFmt.hasLeftButton = true;
              Serial.printf("  Left Button (0x01) @bitOffset=%d, bitSize=%d", bitOffset, reportSize);
            } else if (u == 0x02) {
              hidFmt.rightButton = { u, reportID, bitOffset, reportSize, false };
              hidFmt.hasRightButton = true;
              Serial.printf("  Right Button (0x02) @bitOffset=%d, bitSize=%d", bitOffset, reportSize);
            }
          }
        }

        bitOffset += reportSize;
      }

      usageCount = 0;
    }
  }

  hidFmt.valid = true;
  hidFmt.reportID = reportID;

  if (!hidFmt.hasExplicitButtons) {
    hidFmt.usesStandardMouseFormat = true;
    Serial.println("  No explicit button definitions found - assuming standard mouse format (first byte for buttons)");
  }

  Serial.println("HID Mouse detected:");
  Serial.printf("  Report ID: %d", hidFmt.reportID);
  Serial.printf("  X @bitOffset=%d, bitSize=%d", hidFmt.x.bitOffset, hidFmt.x.bitSize);
  Serial.printf("  Y @bitOffset=%d, bitSize=%d", hidFmt.y.bitOffset, hidFmt.y.bitSize);
  Serial.printf("  Wheel @bitOffset=%d, bitSize=%d", hidFmt.wheel.bitOffset, hidFmt.wheel.bitSize);
  if (hidFmt.hasExplicitButtons) {
    if (hidFmt.hasLeftButton) {
      Serial.printf("  Left Button defined @bitOffset=%d, bitSize=%d", hidFmt.leftButton.bitOffset, hidFmt.leftButton.bitSize);
    }
    if (hidFmt.hasRightButton) {
      Serial.printf("  Right Button defined @bitOffset=%d, bitSize=%d", hidFmt.rightButton.bitOffset, hidFmt.rightButton.bitSize);
    }
  } else {
    Serial.println("  Using standard mouse format: first byte contains button status");
  }
}

void interpretMouseData(uint8_t* data, size_t len, int16_t& x, int16_t& y, int8_t& wheel, bool& leftButton, bool& rightButton) {
  x = 0; y = 0; wheel = 0;
  leftButton = false; rightButton = false;

  if (len == 0) return;

  uint8_t offset = 0;

  if (len >= 2 && hidFmt.valid && hidFmt.reportID != 0 && data[0] == hidFmt.reportID && hidFmt.reportID >= 0x10) {
    Serial.printf("Using Report ID: %d ", data[0]);
    offset = 1;
  }

  if (len <= offset) {
    Serial.println("Invalid length!");
    return;
  }

  if (hidFmt.x.bitSize > 0) {
    x = extractBits(data + offset, hidFmt.x.bitOffset, hidFmt.x.bitSize, hidFmt.x.isSigned);
  }
  if (hidFmt.y.bitSize > 0) {
    y = extractBits(data + offset, hidFmt.y.bitOffset, hidFmt.y.bitSize, hidFmt.y.isSigned);
  }
  if (hidFmt.wheel.bitSize > 0) {
    wheel = extractBits(data + offset, hidFmt.wheel.bitOffset, hidFmt.wheel.bitSize, hidFmt.wheel.isSigned);
  }

  uint8_t buttonByte = data[offset];
  leftButton = (buttonByte & 0x01) != 0;
  rightButton = (buttonByte & 0x02) != 0;

  Serial.printf("Buttons: 0x%02X -> ", buttonByte);
  uint8_t state = buttonByte & 0x07;
  switch (state) {
    case 0x00: Serial.print("0x00(no button)"); break;
    case 0x01: Serial.print("0x01(left button)"); break;
    case 0x02: Serial.print("0x02(right button)"); break;
    case 0x03: Serial.print("0x03(both buttons)"); break;
    case 0x04: Serial.print("0x04(middle button)"); break;
    case 0x05: Serial.print("0x05(left + middle)"); break;
    case 0x06: Serial.print("0x06(right + middle)"); break;
    case 0x07: Serial.print("0x07(all three buttons)"); break;
    default: Serial.printf("0x%02X(?)", state); break;
  }
  Serial.printf(" L:%d R:%d", leftButton, rightButton);
}

// =================================================================
// BLE CALLBACK - Handles incoming mouse data
// =================================================================

void notifyCB(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool isNotify) {
  Serial.print("LEN=");
  Serial.print(len);
  Serial.print(" DATA: ");
  for (int i = 0; i < len; i++) {
    Serial.printf("%02X ", data[i]);
  }
  Serial.print(" -  ");

  // =========================
  // CALIBRATION MODE: Collect data
  // =========================
  if (collectingData && currentMouseType == MOUSE_BLE) {
    if (len < MAX_LEN && buffers[len].count < MAX_SAMPLES) {
      memcpy(buffers[len].data[buffers[len].count], data, len);
      buffers[len].count++;
      sampleCounter = buffers[len].count;
      Serial.printf("Collecting (%d/%d)", buffers[len].count, MAX_SAMPLES);

      if (buffers[len].count >= MAX_SAMPLES) {
        collectingData = false;
        usbHost.ready = true;
        Serial.println("=== Enough data collected ===");
        usbHost.analyze();
        calStatus = "READY - press OK";
        Serial.println(">>> Press BOOT or click OK to continue <<<");
      }
    }
    return;
  }

  // =========================
  // NORMAL MODE: Process mouse data
  // =========================
  int16_t x = 0, y = 0;
  int8_t wheel = 0;
  bool leftButton = false, rightButton = false;

  // Use calibrated mapping if available and matches length
  if (mouseMap.valid && len == mouseMap.mainLen) {
    uint8_t btn = data[mouseMap.btnByte] & 0x07;
    leftButton = (btn == 1 || btn == 3 || btn == 5 || btn == 7);
    rightButton = (btn == 2 || btn == 3 || btn == 6 || btn == 7);
    x = (int8_t)data[mouseMap.xByte];
    y = (int8_t)data[mouseMap.yByte];
    wheel = (int8_t)data[mouseMap.wheelByte];

    Serial.printf("CALIBRATED: btn=0x%02X x=%d y=%d w=%d L:%d R:%d", btn, x, y, wheel, leftButton, rightButton);
  }
  // Fallback to HID parser
  else if (hidFmt.valid) {
    interpretMouseData(data, len, x, y, wheel, leftButton, rightButton);
    Serial.print("HID PARSER: ");
  }
  else {
    Serial.println("No valid format! Use calibration.");
    return;
  }

  // =========================
  // THREAD-SAFE UPDATE
  // =========================
  bool prevLeftBtn = leftBtn;
  bool prevRightBtn = rightBtn;

  if (scaleMutex != NULL) {
    xSemaphoreTake(scaleMutex, portMAX_DELAY);
  }

  lastX = x;
  lastY = y;
  leftBtn = leftButton;
  rightBtn = rightButton;
  lastMouseUpdate = millis();

  if (scaleMutex != NULL) {
    xSemaphoreGive(scaleMutex);
  }

  // =========================
  // SCROLL -> ZOOM
  // =========================
  if (wheel != 0) {
    if (scaleMutex != NULL) {
      xSemaphoreTake(scaleMutex, portMAX_DELAY);
    }

    char oldScale = currentScale;
    currentScale -= wheel;

    if (currentScale < minScale) currentScale = minScale;
    if (currentScale > maxScale) currentScale = maxScale;

    if (oldScale != currentScale) {
      scaleChanged = true;
      Serial.print("ZOOM: Factor = ");
      Serial.print((int)currentScale);
      Serial.print(" (");
      Serial.print((int)(20.0 / currentScale * 100));
      Serial.print("%)");
    }

    if (scaleMutex != NULL) {
      xSemaphoreGive(scaleMutex);
    }
  }

  // Debug output
  bool buttonChanged = (prevLeftBtn != leftBtn) || (prevRightBtn != rightBtn);

  if (x != 0 || y != 0 || buttonChanged || wheel != 0) {
    Serial.print("BLE: x");
    Serial.print(x >= 0 ? "+" : "");
    Serial.print(x);
    Serial.print(" y");
    Serial.print(y >= 0 ? "+" : "");
    Serial.print(y);
    Serial.print(" w");
    Serial.print(wheel >= 0 ? "+" : "");
    Serial.print(wheel);
    Serial.print(" L:");
    Serial.print(leftBtn ? "1" : "0");
    if (prevLeftBtn && !leftBtn) Serial.print("^");
    Serial.print(" R:");
    Serial.print(rightBtn ? "1" : "0");
    if (prevRightBtn && !rightBtn) Serial.print("^");
    Serial.print(" Z:");
    Serial.print((int)currentScale);
    Serial.print("%");
  }

  Serial.println();
}

// =================================================================
// BLE FUNCTIONS
// =================================================================

bool connectTo(const NimBLEAdvertisedDevice* dev) {
  Serial.println("MSX: Connecting...");

  client = NimBLEDevice::createClient();
  client->setConnectTimeout(10);

  if (!client->connect(dev)) {
    Serial.println("MSX: Connection failed");
    NimBLEDevice::deleteClient(client);
    Serial.println("NimBLE: Client deleted...");
    client = nullptr;
    return false;
  }

  connected = true;
  Serial.println("MSX: Connected!");

  NimBLERemoteService* hidSvc = client->getService("1812");
  if (hidSvc) {
    NimBLERemoteCharacteristic* reportMap = hidSvc->getCharacteristic("2A4B");
    if (reportMap) {
      std::string map = reportMap->readValue();
      Serial.println("HID Report Map:");
      for (int i = 0; i < map.length(); i++) {
        printf("%02X ", (uint8_t)map[i]);
      }
      Serial.println();
      parseHIDReport((uint8_t*)map.data(), map.length());
    } else {
      Serial.println("No Report Map found!");
    }
  } else {
    Serial.println("No HID Service found!");
  }

  mouseAddr = dev->getAddress().toString();
  mouseName = dev->getName().c_str();
  if (mouseName.empty()) mouseName = "Unknown Mouse";

  auto services = client->getServices(true);
  for (auto svc : services) {
    auto chrs = svc->getCharacteristics(true);
    for (auto chr : chrs) {
      if (chr->canNotify()) {
        chr->subscribe(true, notifyCB);
      }
    }
  }

  return true;
}

void scanAndConnect() {
  Serial.println("MSX: Scanning for HID mouse...");
  isScanning = true;

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setDuplicateFilter(false);
  scan->start(15, true);

  isScanning = false;
  auto res = scan->getResults();

  for (int i = 0; i < res.getCount(); i++) {
    const NimBLEAdvertisedDevice* dev = res.getDevice(i);
    if (dev->haveServiceUUID() && dev->isAdvertisingService(NimBLEUUID((uint16_t)0x1812))) {
      Serial.println("MSX: HID mouse found");
      connectTo(dev);
      return;
    }
  }

  Serial.println("MSX: No HID devices found");
}

void disconnectMouse() {
  if (client && connected) {
    client->disconnect();
    connected = false;
    mouseName = "";
    mouseAddr = "";
    Serial.println("MSX: Disconnected");
  }
}

// =================================================================
// CALIBRATION FUNCTIONS
// =================================================================

void resetCalibration() {
  for (int i = 0; i < 17; i++) buffers[i].count = 0;
  usbHost.resetBuffers();
  sampleCounter = 0;
  collectingData = false;
  usbHost.collecting = false;
  usbHost.ready = false;
  memset(&mouseMap, 0, sizeof(mouseMap));
}

void calibrationNextStep() {
  usbHost.ready = false;
  collectingData = false;
  usbHost.collecting = false;

  for (int i = 0; i < 17; i++) buffers[i].count = 0;
  usbHost.resetBuffers();

  switch (calState) {
    case CAL_IDLE:
      Serial.println("Step 0: Lift mouse (NO DATA COLLECTION), then press BOOT or OK");
      calInstruction = 
        "<h2>Calibration - Step 1 of 4</h2>"
        "<b>1) Lift the mouse from the desk so it is in the air.</b><br>"
        "Do NOT move it.<br><br>"
        "Then press OK.";
      calStatus = "Waiting for confirmation...";
      calState = CAL_LIFT;
      return;

    case CAL_LIFT:
      Serial.println("Step 1: Detect buttons - click left and right mouse buttons several times.");
      calInstruction = 
        "<h2>Calibration - Step 2 of 4</h2>"
        "<b>2) Now click LEFT and RIGHT mouse buttons several times.</b><br>"
        "Keep clicking until counter reaches 50.<br><br>"
        "Collected: <span id='counter'>0</span>/50";
      calStatus = "Collecting...";
      calState = CAL_BUTTON_DETECT;
      collectingData = true;
      usbHost.collecting = true;
      break;

    case CAL_BUTTON_DETECT:
      Serial.println("Step 2: Detect scroll wheel: move wheel up and down several times.");
      calInstruction = 
        "<h2>Calibration - Step 3 of 4</h2>"
        "<b>3) Move scrollwheel UP and DOWN several times.</b><br>"
        "Keep scrolling until counter reaches 50.<br><br>"
        "Collected: <span id='counter'>0</span>/50";
      calStatus = "Collecting...";
      calState = CAL_WHEEL;
      collectingData = true;
      usbHost.collecting = true;
      break;

    case CAL_WHEEL:
      Serial.println("Step 3: Detect XY movement: place mouse on desk and move to upper left and lower right corner.");
      calInstruction = 
        "<h2>Calibration - Step 4 of 4</h2>"
        "<b>4) Move mouse forward/back and left/right.</b><br>"
        "Keep moving until counter reaches 50.<br><br>"
        "Collected: <span id='counter'>0</span>/50";
      calStatus = "Collecting...";
      calState = CAL_XY;
      collectingData = true;
      usbHost.collecting = true;
      break;

    case CAL_XY:
      Serial.println("Finalizing calibration...");
      usbHost.finalize();
      return;

    case CAL_DONE:
      return;
  }

  Serial.println(">>> Collecting data now... <<<");
}

// =================================================================
// WEB SERVER INTERFACE
// =================================================================

void setupWebServer() {

  // =========================
  // MAIN PAGE - Mode Selection
  // =========================
  server.on("/", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta http-equiv='Content-Type' content='text/html; charset=utf-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<title>ESP32 USB BLE MSX MOUSE - VERSION 01</title>";
    html += "<style>";
    html += "body{font-family:Arial;margin:20px;background:#f0f0f0;}";
    html += "h1{text-align:center;color:#333;}";
    html += ".section{margin:15px 0;padding:15px;background:white;border-radius:5px;box-shadow:0 2px 5px rgba(0,0,0,0.1);}";
    html += ".status{padding:10px;margin:10px 0;border-radius:5px;}";
    html += ".connected{background:#d4edda;color:#155724;}";
    html += ".disconnected{background:#f8d7da;color:#721c24;}";
    html += ".web-active{background:#fff3cd;color:#856404;}";
    html += "button{width:100%;padding:15px;margin:8px 0;font-size:16px;border:none;border-radius:8px;background:#007bff;color:#fff;cursor:pointer;}";
    html += "button:hover{background:#0056b3;}";
    html += "button:active{background:#004494;}";
    html += ".mode-btn{font-size:20px;padding:20px;background:#28a745;}";
    html += ".mode-btn:hover{background:#218838;}";
    html += ".cal-btn{background:#fd7e14;}";
    html += ".cal-btn:hover{background:#e56b0a;}";
    html += ".data-row{display:flex;align-items:center;margin:8px 0;}";
    html += ".data-label{width:120px;font-weight:bold;color:#555;}";
    html += ".data-value{color:#333;font-family:monospace;}";
    html += "input[type=number]{width:80px;padding:5px;margin:0 5px;}";
    html += "form{display:inline;}";
    html += "a{display:block;padding:10px;margin:6px 0;text-decoration:none;color:#fff;background:#6c757d;border-radius:5px;text-align:center;}";
    html += "a:hover{background:#5a6268;}";
    html += ".big-btn{font-size:18px;padding:18px;}";
    html += "</style></head><body>";

    html += "<h1>ESP32 MOUSE v014</h1>";

    // Mode Selection (FIRST ITEM)
    html += "<div class='section'>";
    html += "<h2>PLEASE CHOOSE MODE:</h2>";
    html += "<div style='display:flex;gap:10px;'>";
    html += "<form action='/select_mode' method='get' style='flex:1;'><input type='hidden' name='type' value='usb'><button type='submit' class='mode-btn' style='width:100%;height:80px;'>USB MOUSE</button></form>";
    html += "<form action='/select_mode' method='get' style='flex:1;'><input type='hidden' name='type' value='ble'><button type='submit' class='mode-btn' style='width:100%;height:80px;'>BLE MOUSE</button></form>";
    html += "</div>";
    html += "<div style='margin-top:10px;text-align:center;font-size:14px;color:#666;'>";
    html += "Current: ";
    if (currentMouseType == MOUSE_USB) html += "<b>USB Mouse</b>";
    else if (currentMouseType == MOUSE_BLE) html += "<b>BLE Mouse</b>";
    else html += "<b>Not selected</b>";
    html += "</div>";
    html += "</div>";

    // Calibration Section
    html += "<div class='section'>";
    html += "<h3>Calibration</h3>";
    html += "<p>Mouse calibration data: ";
    if (mouseMap.valid) {
      html += "<span style='color:green;font-weight:bold;'>VALID</span>";
      html += "<br>Length: " + String(mouseMap.mainLen) + " bytes";
      html += "<br>Buttons@" + String(mouseMap.btnByte) + " Wheel@" + String(mouseMap.wheelByte);
      html += "<br>X@" + String(mouseMap.xByte) + " Y@" + String(mouseMap.yByte);
    } else {
      html += "<span style='color:red;font-weight:bold;'>NOT CALIBRATED</span>";
    }
    html += "</p>";
    html += R"(<button onclick="location.href='/calibrate'" class='cal-btn'>Start Calibration</button>)";
    // html += "<button onclick="location.href='/calibrate'" class='cal-btn'>Start Calibration</button>";
    html += "</div>";


    // Connection Status
    html += "<div class='section'>";
    html += "<h3>BLE - Status</h3>";
    html += "<div class='status ";
    html += connected ? "connected" : "disconnected";
    html += "'>";
    if (connected) {
      html += "BLE: " + String(mouseName.c_str()) + "<br>";
      html += "RSSI: " + String(currentRSSI) + " dBm<br>";
      html += "Data: " + String((millis() - lastMouseUpdate < 1000) ? "Receiving" : "No action transmitted");
    } else {
      html += "Not connected to BLE mouse. SCAN & Connect or choose a device from the list";
    }
    html += "</div>";
    html += "</div>";

    // BLE Device List
    html += "<div class='section'>";
    html += "<h3>BLE DEVICES</h3>";
    if (deviceList.empty()) {
      html += "<em>No devices scanned. Use SCAN button to find devices.</em>";
    } else {
      for (int i = 0; i < deviceList.size(); i++) {
        html += "<a href='/select?id=" + String(i) + "'>";
        html += String(i) + ": " + String(deviceList[i].name.c_str());
        html += " (RSSI: " + String(deviceList[i].rssi) + " dBm)";
        html += "</a>";
      }
    }
    html += "</div>";

    // Mouse Data
    html += "<div class='section'>";
    html += "<h3>MOUSE DATA</h3>";
    html += "<div class='data-row'><span class='data-label'>X:</span><span class='data-value'>";
    html += String(lastX >= 0 ? "+" : "") + String(lastX) + "</span></div>";
    html += "<div class='data-row'><span class='data-label'>Y:</span><span class='data-value'>";
    html += String(lastY >= 0 ? "+" : "") + String(lastY) + "</span></div>";
    html += "<div class='data-row'><span class='data-label'>Left:</span><span class='data-value'>";
    html += leftBtn ? "PRESSED" : "RELEASED";
    if (lastLeftBtn && !leftBtn) html += " (RELEASED)";
    html += "</span></div>";
    html += "<div class='data-row'><span class='data-label'>Right:</span><span class='data-value'>";
    html += rightBtn ? "PRESSED" : "RELEASED";
    if (lastRightBtn && !rightBtn) html += " (RELEASED)";
    html += "</span></div>";
    html += "</div>";

    // Zoom Control
    html += "<div class='section'>";
    html += "<h3>ZOOM CONTROL (20%-200%)</h3>";
    html += R"(<button onclick="location.href='/zoom_out'">Zoom OUT (slower)</button>)";
    html += R"(<button onclick="location.href='/zoom_in'">Zoom IN (faster)</button>)";
    html += R"(<button onclick="location.href='/zoom_reset'">Reset Zoom (100%)</button>)";
    html += "<form action='/set_zoom' method='get' style='margin-top:10px;'>";
    html += "Factor (4-40): <input type='number' min='4' max='40' name='value' value='" + String((int)currentScale) + "'>";
    html += "<input type='submit' value='Set' style='padding:5px 10px;margin-left:5px;'>";
    html += "</form>";
    html += "<div style='margin-top:10px;font-size:12px;color:#666;'>";
    html += "Scroll Wheel: OUT = Zoom (slower), IN = faster<br>";
    html += "Button Release: displayed with ^ symbol<br>";
    html += "Commands: scale X | scale (show current)</div>";
    html += "</div>";

    // Controls
    html += "<div class='section'>";
    html += "<h3>CONTROLS</h3>";
    html += R"(<button onclick="location.href='/scan'">Scan & Connect Mouse</button>)";
    html += R"(<button onclick="location.href='/scanlist'">Scan Device List</button>)";
    html += R"(<button onclick="location.href='/disconnect'">Disconnect Mouse</button>)";
    html += R"(<button onclick="location.href='/update'">Firmware Update</button>)";
    html += R"(<button onclick="location.href='/reset'">Reset ESP32</button>)";
    html += "</div>";

    // Info
    html += "<div class='section'>";
    html += "<h3>DEVICE INFO</h3>";
    html += "<div class='data-row'><span class='data-label'>Board:</span><span class='data-value'>" + String(ssid) + "</span></div>";
    html += "<div class='data-row'><span class='data-label'>Pins:</span><span class='data-value'>Data: 14,13,12,11, Buttons: 8,3, Strobe: 46, Scan: 15, LED: 2</span></div>";
    html += "<div class='data-row'><span class='data-label'>GPIO Mode:</span><span class='data-value'>Direct Register Access synchronized to Strobe</span></div>";
    html += "<div class='data-row'><span class='data-label'>Uptime:</span><span class='data-value'>" + String(millis() / 1000) + "s</span></div>";
    html += "<h4>https://github.com/rigr/ESP32_USB_MSX";
    html += "</div>";

    html += "<script>setTimeout(() => location.reload(), 2000);</script>";
    html += "</body></html>";

    server.send(200, "text/html; charset=utf-8", html);
  });

  // =========================
  // SELECT MODE Endpoint
  // =========================
  server.on("/select_mode", HTTP_GET, []() {
    String type = server.arg("type");
    if (type == "usb") {
      selectedMouseType = MOUSE_USB;
      currentMouseType = MOUSE_USB;
      Serial.println("Mode selected: USB");
      usbHost.begin();
    } else if (type == "ble") {
      selectedMouseType = MOUSE_BLE;
      currentMouseType = MOUSE_BLE;
      Serial.println("Mode selected: BLE");
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Redirecting...");
  });

  // =========================
  // CALIBRATION PAGE
  // =========================
  server.on("/calibrate", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta http-equiv='Content-Type' content='text/html; charset=utf-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<meta http-equiv='refresh' content='2'>";
    html += "<title>Mouse Calibration</title>";
    html += "<style>";
    html += "body{font-family:Arial;margin:20px;background:#f4f4f4;text-align:center;}";
    html += ".box{background:white;padding:25px;border-radius:12px;box-shadow:0 4px 15px rgba(0,0,0,0.1);max-width:680px;margin:0 auto;}";
    html += "h1{color:#333;margin-bottom:10px;}";
    html += ".instruction{font-size:18px;line-height:1.6;margin:20px 0;text-align:left;}";
    html += ".status{font-size:17px;font-weight:bold;color:#0066cc;margin:15px 0;}";
    html += ".counter{font-size:24px;color:#fd7e14;font-weight:bold;margin:10px 0;}";
    html += "button{background:#0066cc;color:white;font-size:20px;padding:18px 50px;border:none;border-radius:10px;cursor:pointer;margin-top:25px;width:90%;max-width:400px;}";
    html += "button:hover{background:#0055aa;}";
    html += "button:disabled{background:#888;cursor:not-allowed;}";
    html += ".done-btn{background:#28a745;}";
    html += ".done-btn:hover{background:#218838;}";
    html += ".footer{margin-top:40px;font-size:14px;color:#666;}";
    html += "</style></head><body>";
    html += "<div class='box'>";
    html += "<h1>Mouse Calibration</h1>";
    html += "<div class='instruction'>";
    html += calInstruction;
    html += "</div>";

    if (collectingData) {
      html += "<div class='counter'>Collected: " + String(sampleCounter) + "/50</div>";
      html += "<div class='status'>" + calStatus + "</div>";
      html += "<button disabled>Collecting data...</button>";
    } else if (calState == CAL_DONE) {
      html += "<div class='status' style='color:green;'>Calibration complete!</div>";
      html += R"(<button onclick="location.href='/'" class='done-btn'>Back to Main</button>)";
    } else if (calState == CAL_IDLE || calState == CAL_LIFT || usbHost.ready) {
      html += "<div class='status'>" + calStatus + "</div>";
      html += "<form action='/cal_next' method='POST'><button type='submit'>OK</button></form>";
    } else {
      html += "<div class='status'>" + calStatus + "</div>";
      html += "<button disabled>Waiting for data...</button>";
    }

    html += "<div class='footer'>";
    html += "<a href='/'>Cancel and return to main</a>";
    html += "</div>";
    html += "</div>";
    html += "</body></html>";

    server.send(200, "text/html; charset=utf-8", html);
  });

  // =========================
  // CALIBRATION NEXT STEP
  // =========================
  server.on("/cal_next", HTTP_POST, []() {
    if (!collectingData && (usbHost.ready || calState == CAL_IDLE || calState == CAL_LIFT)) {
      calibrationNextStep();
    }
    server.sendHeader("Location", "/calibrate");
    server.send(303, "text/plain", "Redirecting...");
  });

  // =========================
  // LEGACY ENDPOINTS
  // =========================
  server.on("/scan", HTTP_GET, []() {
    if (!connected) {
      scanAndConnect();
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Redirecting...");
  });

  server.on("/scanlist", HTTP_GET, []() {
    if (!connected) {
      scanDevices();
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Redirecting...");
  });

  server.on("/select", HTTP_GET, []() {
    int id = server.arg("id").toInt();
    if (id >= 0 && id < deviceList.size()) {
      selectedDevice = id;
      Serial.print("Selected Device: ");
      Serial.println(deviceList[id].name.c_str());
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
  });

  server.on("/disconnect", HTTP_GET, []() {
    disconnectMouse();
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Redirecting...");
  });

  // Zoom Controls
  server.on("/zoom_out", HTTP_GET, []() {
    if (scaleMutex != NULL) {
      xSemaphoreTake(scaleMutex, portMAX_DELAY);
    }
    if (currentScale > minScale) {
      currentScale--;
      scaleChanged = true;
      Serial.print("ZOOM OUT: Factor = ");
      Serial.print((int)currentScale);
      Serial.print(" (");
      Serial.print((int)(20.0 / currentScale * 100));
      Serial.print("%)");
    }
    if (scaleMutex != NULL) {
      xSemaphoreGive(scaleMutex);
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Redirecting...");
  });

  server.on("/zoom_in", HTTP_GET, []() {
    if (scaleMutex != NULL) {
      xSemaphoreTake(scaleMutex, portMAX_DELAY);
    }
    if (currentScale < maxScale) {
      currentScale++;
      scaleChanged = true;
      Serial.print("ZOOM IN: Factor = ");
      Serial.print((int)currentScale);
      Serial.print(" (");
      Serial.print((int)(20.0 / currentScale * 100));
      Serial.print("%)");
    }
    if (scaleMutex != NULL) {
      xSemaphoreGive(scaleMutex);
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Redirecting...");
  });

  server.on("/zoom_reset", HTTP_GET, []() {
    if (scaleMutex != NULL) {
      xSemaphoreTake(scaleMutex, portMAX_DELAY);
    }
    currentScale = 15;
    scaleChanged = true;
    Serial.print("ZOOM RESET: Factor = ");
    Serial.print((int)currentScale);
    Serial.print(" (");
    Serial.print((int)(20.0 / currentScale * 100));
    Serial.print("%)");
    if (scaleMutex != NULL) {
      xSemaphoreGive(scaleMutex);
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Redirecting...");
  });

  server.on("/set_zoom", HTTP_GET, []() {
    if (server.hasArg("value")) {
      int newScale = server.arg("value").toInt();
      if (newScale >= minScale && newScale <= maxScale) {
        if (scaleMutex != NULL) {
          xSemaphoreTake(scaleMutex, portMAX_DELAY);
        }
        currentScale = newScale;
        scaleChanged = true;
        Serial.print("Web: Factor set to ");
        Serial.print((int)currentScale);
        Serial.print(" (");
        Serial.print((int)(20.0 / currentScale * 100));
        Serial.print("%)");
        if (scaleMutex != NULL) {
          xSemaphoreGive(scaleMutex);
        }
      }
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Redirecting...");
  });

  // Firmware Update
  server.on("/update", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta http-equiv='Content-Type' content='text/html; charset=utf-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<title>Firmware Update</title>";
    html += "<style>body{font-family:Arial;margin:20px;}</style></head><body>";
    html += "<h2>Firmware Update</h2>";
    html += "<p><strong>Current Firmware Version: 01</strong></p>";
    html += "<p><strong>Current Status: " + String(webServerActive ? "ACTIVE" : "INACTIVE") + "</strong></p>";
    html += "<form method='POST' action='/update' enctype='multipart/form-data'>";
    html += "<input type='file' name='update' accept='.bin'>";
    html += "<input type='submit' value='Firmware Update'>";
    html += "</form>";
    html += "<p><a href='/'>Back to Main Page</a></p>";
    html += "</body></html>";
    server.send(200, "text/html; charset=utf-8", html);
  });

  server.on(
    "/update", HTTP_POST, []() {
      server.send(200, "text/plain; charset=utf-8", "OTA Update started");
    },
    []() {
      HTTPUpload& upload = server.upload();
      if (upload.status == UPLOAD_FILE_START) {
        Serial.setDebugOutput(true);
        String filename = upload.filename;
        if (!filename.startsWith("/")) filename = "/" + filename;
        Serial.print("Update: ");
        Serial.println(filename);
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if (!Update.begin(maxSketchSpace)) {
          Serial.println("Update begin failed");
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
          Serial.println("Update successful");
          Serial.println("ESP32 will restart now...");
          delay(1000);
          ESP.restart();
        } else {
          Update.printError(Serial);
        }
        Serial.setDebugOutput(false);
      }
    });

  server.on("/reset", HTTP_GET, []() {
    ESP.restart();
  });

  server.begin();
}

// =================================================================
// Serial Commands Handler
// =================================================================

void handleWebCommand(String cmd) {
  if (cmd == "web" || cmd == "webinterface" || cmd == "web toggle") {
    if (webServerActive) {
      server.stop();
      WiFi.softAPdisconnect(true);
      webServerActive = false;
      calibrationMode = false;
    } else {
      startCalibrationMode();
    }
  } else if (cmd.startsWith("web ")) {
    String subCmd = cmd.substring(4);
    if (subCmd == "on" || subCmd == "start") {
      if (!webServerActive) startCalibrationMode();
    } else if (subCmd == "off" || subCmd == "stop") {
      if (webServerActive) {
        server.stop();
        WiFi.softAPdisconnect(true);
        webServerActive = false;
        calibrationMode = false;
      }
    }
  } else if (cmd == "web status" || cmd == "web state") {
    Serial.println("Web Server: " + String(webServerActive ? "ACTIVE" : "INACTIVE"));
  }
}

void handleSerialCommand(String cmd) {
  if (cmd.equals("s")) {
    if (!connected) scanAndConnect();
    else Serial.println("MSX: Already connected");
  } else if (cmd.equals("d")) {
    disconnectMouse();
  } else if (cmd.equals("r")) {
    Serial.println("MSX: Resetting...");
    ESP.restart();
  } else if (cmd.equals("scale")) {
    if (scaleMutex != NULL) {
      xSemaphoreTake(scaleMutex, portMAX_DELAY);
    }
    Serial.print("Current Factor: ");
    Serial.print((int)currentScale);
    Serial.print(" (");
    Serial.print((int)(20.0 / currentScale * 100));
    Serial.print("%)");
    if (scaleMutex != NULL) {
      xSemaphoreGive(scaleMutex);
    }
  } else if (cmd.startsWith("scale ")) {
    int newScale = cmd.substring(6).toInt();
    if (newScale >= minScale && newScale <= maxScale) {
      if (scaleMutex != NULL) {
        xSemaphoreTake(scaleMutex, portMAX_DELAY);
      }
      currentScale = newScale;
      scaleChanged = true;
      Serial.print("Factor set to: ");
      Serial.print((int)currentScale);
      Serial.print(" (");
      Serial.print((int)(20.0 / currentScale * 100));
      Serial.print("%)");
      if (scaleMutex != NULL) {
        xSemaphoreGive(scaleMutex);
      }
    } else {
      Serial.print("Factor must be between ");
      Serial.print((int)minScale);
      Serial.print(" and ");
      Serial.print((int)maxScale);
      Serial.println(" (20%-200%)");
    }
  } else if (cmd.equals("scan")) {
    scanDevices();
  } else if (cmd.equals("list")) {
    printDeviceList();
  } else if (cmd.startsWith("select ")) {
    int id = cmd.substring(7).toInt();
    if (id >= 0 && id < deviceList.size()) {
      selectedDevice = id;
      Serial.print("Selected Device: ");
      Serial.println(deviceList[id].name.c_str());
    } else {
      Serial.println("Invalid device number");
    }
  } else if (cmd.equals("cal")) {
    Serial.println("Starting calibration mode...");
    startCalibrationMode();
  } else if (cmd.equals("help") || cmd.equals("h")) {
    Serial.println("=== MSX MOUSE COMMANDS ===");
    Serial.println("s - Scan & Connect first HID mouse");
    Serial.println("scan - Scan device list (20s)");
    Serial.println("list - Show device list");
    Serial.println("select <nr> - Select device from list");
    Serial.println("d - Disconnect mouse");
    Serial.println("scale - Show current zoom");
    Serial.println("scale X - Set zoom (4-40, 20%-200%)");
    Serial.println("cal - Start calibration web interface");
    Serial.println("web - Toggle web interface");
    Serial.println("web on - Start web interface");
    Serial.println("web off - Stop web interface");
    Serial.println("web status - Show web interface status");
    Serial.println("r - Reset ESP32");
    Serial.println("h or help - Show this help");
    Serial.println("^ in output indicates button release");
    Serial.println("GPIO Operations: Direct Register Access + Strobe Sync");
    Serial.println("");
    Serial.println("=== ACTIVATION METHODS ===");
    Serial.println("BOOT Button: Hold 3s to start calibration, 6s to stop");
    Serial.println("Serial Command: 'cal' or 'web' to toggle");
    Serial.println("");
    Serial.println("Current Web Server Status: " + String(webServerActive ? "ACTIVE" : "INACTIVE"));
    Serial.println("Current Mouse Type: " + String(currentMouseType == MOUSE_USB ? "USB" : (currentMouseType == MOUSE_BLE ? "BLE" : "NONE")));
    Serial.println("Calibration Valid: " + String(mouseMap.valid ? "YES" : "NO"));
  } else {
    if (cmd.startsWith("web")) {
      handleWebCommand(cmd);
    }
  }
}

// =================================================================
// SETUP AND LOOP (Core 0)
// =================================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  // LED Blink Sequence
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  uint64_t chipid = ESP.getEfuseMac();

  Serial.println("=====================================");
  Serial.println("ESP32 USB BLE MSX MOUSE - VERSION 01");
  Serial.println("https://github.com/rigr/ESP32_USB_MSX");
  Serial.println("NimBLE Version: 2.1.0 by h2zero");
  Serial.print("ESP32 Chip ID: ");
  Serial.printf("%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  Serial.println("=====================================");

  // Load NVS data
  prefs.begin(NVS_NAMESPACE, true);
  int savedType = prefs.getInt("mouseType", 0);

  if (savedType == 1) {  // BLE
    currentMouseType = MOUSE_BLE;
    if (prefs.getBytesLength("bleMap") == sizeof(mouseMap)) {
      prefs.getBytes("bleMap", &mouseMap, sizeof(mouseMap));
      Serial.println("BLE Mapping loaded from NVS");
    }
    String lastAddr = prefs.getString("lastBLEAddr", "");
    if (lastAddr != "") {
      Serial.print("Last BLE device: ");
      Serial.println(lastAddr);
    }
  } else if (savedType == 2) {  // USB
    currentMouseType = MOUSE_USB;
    if (prefs.getBytesLength("usbMap") == sizeof(mouseMap)) {
      prefs.getBytes("usbMap", &mouseMap, sizeof(mouseMap));
      Serial.println("USB Mapping loaded from NVS");
    }
  }
  prefs.end();

  Serial.println("Web: START via BOOT button (3s), STOP via BOOT button (6s)");
  Serial.println("Commands: scale X | scale (show) | cal (calibration) | web (toggle) | s/d/scale/scan/list/select X | help");

  snprintf(ssid, 23, "ESP32-%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  Serial.println(ssid);

  // Create mutex for thread safety
  scaleMutex = xSemaphoreCreateMutex();
  if (scaleMutex == NULL) {
    Serial.println("Error creating mutex");
  }

  // Initialize pins
  pinMode(BOOT_PIN, INPUT_PULLUP);
  pinMode(MAN_SCAN, INPUT);
  pinMode(CS_PIN, INPUT);

  // Attach interrupt to BOOT button
  attachInterrupt(digitalPinToInterrupt(BOOT_PIN), bootButtonISR, CHANGE);

  // Initialize BLE
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  // Configure scanner
  NimBLEDevice::getScan()->setActiveScan(true);
  NimBLEDevice::getScan()->setDuplicateFilter(false);

  // Start MSX Protocol Task on Core 1
  xTaskCreatePinnedToCore(
    msxProtocolTask,
    "MSXProtocolTask",
    4096,
    NULL,
    1,
    &msxTaskHandle,
    1);

  // Start USB host if USB mode
  if (currentMouseType == MOUSE_USB) {
    usbHost.begin();
    Serial.println("USB Host started");
  }

  Serial.println("Pull D35 to GND to scan and connect to devices");
  Serial.println("BOOT Button (D0) - Press 3s to start calibration, 6s to stop");
  Serial.println("ROLAND / MSX Mouse Emulation Ready!");
  Serial.println("Serial: 'cal' to start calibration, 'web' to toggle web interface");
  Serial.println("Type 'help' or 'h' for all commands");

  if (!mouseMap.valid) {
    Serial.println("*** NO CALIBRATION FOUND ***");
    Serial.println("Please calibrate your mouse via web interface (hold BOOT 3s)");
  }
}

void loop() {
  // Handle web requests
  if (webServerActive) {
    server.handleClient();
  }

  // Handle USB tasks if in USB mode
  if (currentMouseType == MOUSE_USB) {
    usbHost.task();
  }

  // Heartbeat every 5 seconds
  if (millis() - lastAlive >= 5000) {
    lastAlive = millis();
    Serial.print("Heartbeat | Uptime: ");
    Serial.print(millis() / 1000);
    Serial.print("s | BLE: ");

    if (connected && client && client->isConnected()) {
      currentRSSI = client->getRssi();
      Serial.print("CONNECTED | ");
      Serial.print(mouseName.c_str());
      Serial.print(" | RSSI: ");
      Serial.print(currentRSSI);
      Serial.print(" | Web: ");
      Serial.print(webServerActive ? "ACTIVE" : "INACTIVE");
      Serial.print(" | Zoom: ");
      Serial.print((int)currentScale);
      Serial.print(" (");
      Serial.print((int)(20.0 / currentScale * 100));
      Serial.print("%)");
    } else {
      Serial.print("DISCONNECTED");
    }
    Serial.println();
  }

  // Handle device selection timeout
  if (selectingDevice) {
    if (millis() - selectionStart > 30000) {
      Serial.println("Auto-connecting to selected device");
      selectingDevice = false;
      connectSelectedDevice();
    }
  }

  // Update LED
  updateLED();

  // BOOT button long press?
  checkLongButtonPresses();

  // Handle serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('/n');
    cmd.trim();
    cmd.toLowerCase();
    handleSerialCommand(cmd);
  }

  // Manual scan trigger (D15 LOW)
  if (!digitalRead(MAN_SCAN) && !(currentMouseType == MOUSE_USB) && !connected && !isScanning) {
    Serial.println("MSX: Manual scan triggered via D15...");
    scanAndConnect();
    delay(1000);
  }

  delay(10);
}
