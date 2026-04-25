# ESP32-S3 USB Maus → Roland Sampler / MSX-Protokoll

**Ri April 2026**

**Achtung - das ist noch nicht fertig, es werden erst zwei verschiedene USB-Mäuse unterstützt...**


**UPDATE: ESP32-S3-USB-OTG.ino sollte jetzt jede USB Maus unterstützen.**
..
..
..




Dieses Projekt ermöglicht es, eine normale USB-Maus an einen **ESP32-S3** (N16R8 Board) anzuschließen und die Maus-Signale in das klassische MSX-Maus-Protokoll umzuwandeln.  
Der Roland Sampler verwendet exakt dasselbe Protokoll wie alte MSX-Computer.

### Features

- Direkte USB-Host-Unterstützung für USB-Mäuse
- Stabiles MSX-Maus-Protokoll mit **direktem GPIO-Register-Zugriff** 
- Mausbewegung wird auf Core 1 in einem dedizierten Task verarbeitet → minimal Jitter
- Zoom-Funktion mit dem Mausrad (20 % – 200 %)
- Serielle Ausgabe zur Fehlersuche

### Hardware

- **ESP32-S3 N16R8** (Dual USB-Port Version empfohlen)
- USB-Maus (beliebige Standard-USB-Maus) über OTG-Adapter
- Roland Sampler (oder MSX-Computer) mit Maus-Eingang (9pin)

**Wichtiger Hinweis zur Verkabelung:**
- Der ESP32-S3 verwendet den **nativen USB-OTG-Port** (meist GPIO19 = D-, GPIO20 = D+).
- Auf vielen N16R8-Boards muss ein **OTG-Jumper** gesetzt oder eine Lötbrücke geschlossen werden, damit der Port als Host mit 5 V funktioniert.

### Pin-Belegung (ESP32-S3 → Sampler)

| ESP32-S3 Pin | Funktion              | Sampler subd9f
|--------------|-----------------------|---------------| 
| GPIO 14      | MX0  – Data Bit 0     | Pin 1         |
| GPIO 13      | MX1  – Data Bit 1     | Pin 2         |
| GPIO 12      | MX2  – Data Bit 2     | Pin 3         |
| GPIO 11      | MX3  – Data Bit 3     | Pin 4         |
| Vin          | 5V                    | Pin 5         |
| GPIO 8       | BTN_L – Linke Maustaste | Pin 6       |
| GPIO 3       | BTN_R – Rechte Maustaste | Pin 7      |
| GPIO 46      | STROBE / CS (Eingang) | Pin 8         |
| GND          | Masse                 | Pin 9         |

### Serielle Ausgabe

Beispiel:
USB: buttons=0x01(L  ), x=-3, y=1, wheel=0 Z:100%
USB: buttons=0x00(   ), x=0, y=0, wheel=1 Z:120%
text- `L` = Linke Taste gedrückt  
- `R` = Rechte Taste gedrückt  
- `M` = Mittlere Taste gedrückt  
- `Z:` zeigt den aktuellen Zoom-Faktor in Prozent

### Zoom-Funktion

- Mit dem **Mausrad** kannst du den Maus-Zoom zwischen **20 % und 200 %** einstellen.
- Startwert ist **100 %**.

### Danksagungen

- **tanakamasayuki** – für die hervorragende [EspUsbHost](https://github.com/tanakamasayuki/EspUsbHost) Bibliothek
- **NYYRIKKY** und **Peter Ullrich** – für die grundlegende Arbeit am MSX/Roland Maus-Protokoll
- Original-MSX-Maus-Projekt, das BLE verwendet: https://github.com/rigr/ESP32_USB_MSX

### Kompilieren & Flashen

1. Installiere die Bibliothek **EspUsbHost** über den Arduino Bibliotheksverwalter oder direkt von GitHub.
2. Wähle im Arduino IDE folgendes Board:
   - **ESP32S3 Dev Module** (oder deine exakte Board-Definition)
   - USB Mode: **Hardware CDC and JTAG**
   - Flash Size: **16MB** (falls verfügbar)
3. Lade den Sketch hoch.

Der Sketch startet automatisch. Sobald eine USB-Maus erkannt wird, erscheinen Bewegungen und Klicks in der seriellen Konsole.

---

Viel Erfolg und guten Sound mit deinem Roland Sampler!

## License

Dieses Projekt folgt der MIT License

## Contribution

Erste Unterstützung kam von NYYRIKKY und Peter Ullrich - danke euch beiden!

