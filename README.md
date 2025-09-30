# Medical Line-Follower Robot with Vital Signs (SpO₂ + HR)

**Goal:** A robot follows a black line, stops at a cross-line, checks for a “patient,” deploys a sensor arm, reads **SpO₂** and **heart rate** via **MAX30100**, and sends the results over **Bluetooth (HC-05)** to a phone app (MIT App Inventor). The app timestamps the values, shows them to the user, and optionally pushes them to a cloud database.

## Features
- Differential drive with **L298N** (2 DC motors)
- **Two IR sensors** for black line tracking
- Stop at perpendicular line → **HC-SR04** obstacle check
- Servo lever deploys finger sensor (**MAX30100**, I²C)
- **Bluetooth (HC-05)** → phone app (MIT App Inventor)
- Optional cloud logging via **Google Apps Script** + **Google Sheets**

## Repo Layout
firmware/arduino/src/main.ino # Complete Arduino sketch
app/mit-app-inventor/ # Place your .aia here + optional .apk
cloud/apps-script/Code.gs # Paste into Google Apps Script (Sheets endpoint)
docs/ # Wiring diagrams, screenshots, notes
hardware/ # BoM, pinouts, physical build notes


## Hardware (minimum)
- Arduino Uno/Nano (or similar 5V Arduino)
- L298N dual H-bridge motor driver + 2 DC gear motors + wheels + chassis + battery
- 2× IR line sensors (digital)
- HC-SR04 ultrasonic sensor
- SG90 (or similar) servo for the sensor lever
- MAX30100 pulse-ox sensor (I²C)
- HC-05 Bluetooth module
- Wires, breadboard/PCB, mounts

> **Voltage note:** HC-05 RX expects **3.3V** logic. Use a simple divider from Arduino TX → HC-05 RX.

## Wiring (quick map)
- **L298N**: ENA→D5, IN1→D6, IN2→D7; ENB→D9, IN3→D8, IN4→D4  
- **IR sensors**: Left→D2, Right→D3 (digital). If your modules output LOW on black, set `IR_ACTIVE_LOW = true`.
- **Ultrasonic**: TRIG→D12, ECHO→A0  
- **Servo**: signal→A1  
- **HC-05**: HC-05 TX→D10, HC-05 RX→D11 (with divider), GND shared  
- **MAX30100**: SDA/SCL → (Uno A4/A5), **3.3V** supply (check your breakout), common GND

## Build & Flash
1. Arduino IDE → **Library Manager** → install **MAX30100 PulseOximeter** (oxullo).
2. Open `firmware/arduino/src/main.ino`, select board/port, **Upload**.
3. Pair your phone with **HC-05** (PIN 1234/0000 default). Open the MIT App.

## App (MIT App Inventor)
- Connect to HC-05, read text lines.
- When a line starts with `"CSV,"`, split by commas and then key/value (e.g., `HR_bpm=…`).
- Timestamp locally and **POST** JSON to your web endpoint (see `cloud/`).

### Export / Download your MIT App Inventor project
- In App Inventor: **Projects → Export selected project (.aia) to my computer**  
  Put the file into `app/mit-app-inventor/project.aia`.
- To get an installable app: **Build → App (save .apk to my computer)**  
  Optionally store it under `app/mit-app-inventor/apk/app-release.apk`.
- **Do NOT commit your keystore** (used for signing). Back it up privately:
  **Projects → Export Keystore** (keep it safe, not in git).

## Cloud Logging (optional)
- Use `cloud/apps-script/Code.gs` with a Google Sheet.
- Deploy as **Web app**, copy the URL into your App Inventor “Web” block (POST JSON).

## Data format (Bluetooth → App → Cloud)
CSV,HR_bpm=78.4,SpO2_pct=97.1
Your app timestamps on receipt and forwards JSON like:
```json
{"hr":78.4,"spo2":97.1,"deviceId":"linebot-01","ts":"<phone-local-ISO8601>"}
