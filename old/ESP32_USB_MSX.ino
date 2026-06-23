// =================================================================
// ESP32-S3 USB Maus → Roland Sampler / MSX-Protokoll
// Ri April 2026
// USB-Host + MSX-Protokoll auf Core 1
// https://github.com/rigr/ESP32_USB_MSX
//
// thanks to https://github.com/tanakamasayuki/EspUsbHost for the USB-host part!
// thanks to NYYRIKKY and Peter Ullrich for their basic work with MSX/Roland protocoll.
// =================================================================

#include "EspUsbHost.h"
#include "driver/gpio.h"       
#include "soc/gpio_struct.h"   

// ========================= PIN DEFINITION =========================
#define PIN_MX0     14   // Data bit 0
#define PIN_MX1     13   // Data bit 1
#define PIN_MX2     12   // Data bit 2
#define PIN_MX3     11   // Data bit 3
#define PIN_BTN_L    8   // Left button output
#define PIN_BTN_R    3   // Right button output
#define PIN_STROBE  46   // Strobe input (CS)

// Bitmasken für direkten Register-Zugriff
#define B0 (1UL << PIN_MX0)
#define B1 (1UL << PIN_MX1)
#define B2 (1UL << PIN_MX2)
#define B3 (1UL << PIN_MX3)

// ========================= GLOBALE VARIABLEN =========================
volatile int16_t lastX = 0, lastY = 0;
volatile bool leftBtn = false, rightBtn = false;

volatile char currentScale = 15;        // Start: 100%  (15 entspricht 100%)
const char minScale = 4;               // 20%
const char maxScale = 40;              // 200%

SemaphoreHandle_t scaleMutex = nullptr;

// ========================= USB HOST KLASSE =========================
class MyEspUsbHost : public EspUsbHost {
public:
  uint8_t lastButtons = 0;

  void onMouseButtons(hid_mouse_report_t report, uint8_t last_buttons) {
    processMouse(report, last_buttons);
  }

  void onMouseMove(hid_mouse_report_t report) {
    processMouse(report, lastButtons);
  }

private:
  void processMouse(hid_mouse_report_t report, uint8_t last_buttons) {
    if (scaleMutex != nullptr) xSemaphoreTake(scaleMutex, portMAX_DELAY);
    
    lastX = report.x;
    lastY = report.y;
    leftBtn  = (report.buttons & MOUSE_BUTTON_LEFT)   != 0;
    rightBtn = (report.buttons & MOUSE_BUTTON_RIGHT)  != 0;

    if (report.wheel != 0) {
      char oldScale = currentScale;
      currentScale -= report.wheel;
      if (currentScale < minScale) currentScale = minScale;
      if (currentScale > maxScale) currentScale = maxScale;
      
      if (oldScale != currentScale) {
        Serial.print("ZOOM geändert → ");
      }
    }

    if (scaleMutex != nullptr) xSemaphoreGive(scaleMutex);

    // serielle Ausgabe
    Serial.print("USB: buttons=0x");
    Serial.print(report.buttons, HEX);
    Serial.print("(");
    Serial.print((report.buttons & MOUSE_BUTTON_LEFT)   ? 'L' : ' ');
    Serial.print((report.buttons & MOUSE_BUTTON_RIGHT)  ? 'R' : ' ');
    Serial.print((report.buttons & MOUSE_BUTTON_MIDDLE) ? 'M' : ' ');
    Serial.print("), x=");
    Serial.print(report.x);
    Serial.print(", y=");
    Serial.print(report.y);
    Serial.print(", wheel=");
    Serial.print(report.wheel);
    Serial.print(" Z:");
    Serial.print((int)(20.0 / currentScale * 100));
    Serial.println("%");

    lastButtons = report.buttons;
  }
};

MyEspUsbHost usbHost;

// ========================= MSX PROTOKOLL TASK (Core 1) =========================
TaskHandle_t msxTaskHandle = nullptr;
volatile bool msxEnabled = true;

inline void lineLow(uint32_t mask) {
  GPIO.out_w1tc = mask;
  GPIO.enable_w1ts = mask;
}

inline void lineRelease(uint32_t mask) {
  GPIO.enable_w1tc = mask;
}

void sendNibble(uint8_t n) {
  uint32_t lowMask = 0;
  uint32_t relMask = 0;

  if (n & 8) relMask |= B3; else lowMask |= B3;
  if (n & 4) relMask |= B2; else lowMask |= B2;
  if (n & 2) relMask |= B1; else lowMask |= B1;
  if (n & 1) relMask |= B0; else lowMask |= B0;

  GPIO.out_w1tc = lowMask;
  GPIO.enable_w1ts = lowMask;
  GPIO.enable_w1tc = relMask;
}

void sendMSX(int8_t c) {
  while (digitalRead(PIN_STROBE) == LOW);
  sendNibble((c >> 4) & 0xF);

  while (digitalRead(PIN_STROBE) == HIGH);
  sendNibble(c & 0xF);
}

void msxProtocolTask(void* parameter) {
  pinMode(PIN_MX0, INPUT);
  pinMode(PIN_MX1, INPUT);
  pinMode(PIN_MX2, INPUT);
  pinMode(PIN_MX3, INPUT);
  pinMode(PIN_STROBE, INPUT);

  digitalWrite(PIN_MX0, LOW);
  digitalWrite(PIN_MX1, LOW);
  digitalWrite(PIN_MX2, LOW);
  digitalWrite(PIN_MX3, LOW);

  pinMode(PIN_BTN_L, OUTPUT);
  pinMode(PIN_BTN_R, OUTPUT);
  digitalWrite(PIN_BTN_L, HIGH);
  digitalWrite(PIN_BTN_R, HIGH);

  int16_t mx = 0, my = 0;

  Serial.println("MSX-Protokoll Task gestartet auf Core 1");
  Serial.println("Pins: MX0=14, MX1=13, MX2=12, MX3=11, BTN_L=8, BTN_R=3, STROBE=46");

  while (true) {
    if (!msxEnabled) {
      delay(10);
      continue;
    }

    if (scaleMutex != nullptr) xSemaphoreTake(scaleMutex, portMAX_DELAY);
    int16_t currentX = lastX;
    int16_t currentY = lastY;
    bool curLeft  = leftBtn;
    bool curRight = rightBtn;
    lastX = 0;
    lastY = 0;
    if (scaleMutex != nullptr) xSemaphoreGive(scaleMutex);

    mx += (currentX * -1);
    my += (currentY * -1);

    while (digitalRead(PIN_STROBE) == LOW) {
      delayMicroseconds(10);
      yield();
    }

    char x = (char)(mx * currentScale / 15);
    char y = (char)(my * currentScale / 15);

    sendMSX(x);

    unsigned long timeout = millis() + 20;
    while (digitalRead(PIN_STROBE) == HIGH && millis() < timeout) {
      delayMicroseconds(10);
    }
    if (millis() >= timeout) {
      mx = 0; my = 0;
      continue;
    }

    sendMSX(y);

    mx = 0;
    my = 0;

    digitalWrite(PIN_BTN_L, curLeft  ? LOW : HIGH);
    digitalWrite(PIN_BTN_R, curRight ? LOW : HIGH);

    delay(1);
  }
}

// ========================= SETUP =========================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("====================================================================================");
  Serial.println("ESP32-S3 USB Maus → Roland Sampler");
  Serial.println("MSX-Protokoll + EspUsbHost ");
  Serial.println("Thanks to https://github.com/tanakamasayuki/EspUsbHost for the USB-host part!");
  Serial.println("Thanks to NYYRIKKY and Peter Ullrich for their basic work with MSX/Roland protocoll.");
  Serial.println("RG 18.4.26 - https://github.com/rigr/ESP32_USB_MSX");
  Serial.println("====================================================================================");

  scaleMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    msxProtocolTask,
    "MSXProtocol",
    4096,
    NULL,
    1,
    &msxTaskHandle,
    1
  );

  usbHost.begin();

  Serial.println("USB-Host gestartet. Verbinde eine USB-Maus...");
}

// ========================= LOOP =========================
void loop() {
  usbHost.task();
  delay(1);
}
