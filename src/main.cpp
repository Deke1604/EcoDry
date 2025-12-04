/* Mode 1 - RFID required to start
   Complete integrated ESP32 Smart Dryer sketch
   - OLED SH110X, AHT20 & BMP280 via TCA multiplexer (2 sensors)
   - RTC DS3231, MFRC522 (SPI), fans (PWM), solenoid lock, heater (relay), buzzer, LEDs
   - Buttons: NEXT (navigate) + SELECT (enter/select)
   - PID-like control, heater safety, OLED progress, web UI, webhook POST
*/

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SH110X.h>
#include <RTClib.h>
#include <WebServer.h>
#include <SPI.h>
#include <MFRC522.h>

// ========== DISPLAY ==========
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ========== PINS ==========
#define BTN_NEXT 32
#define BTN_SELECT 33

#define POWER_LED_PIN 13    // Power LED (system ON)
#define HEATER_LED_PIN 12   // Heater indicator LED (mirrors heater on/off)
#define BUZZER_PIN 15       // Buzzer

#define HEATER_PIN 14       // Heater relay (must be via relay/SSR)
#define FAN1_PIN 25
#define FAN2_PIN 26
#define SOLENOID_PIN 27

// RFID SPI pins (SPI.begin(SCK, MISO, MOSI, SS))
#define RST_PIN 4
#define SS_PIN 5

// ========== NETWORK ==========
const char* AP_SSID = "Deke";
const char* AP_PASS = "12345678";
const char* WEBHOOK_URL = "http://agrisun.com/webhook"; // set empty "" to disable webhook

// ========== HARDWARE & THRESHOLDS ==========
#define TCA_ADDR 0x70
#define TEMP_THRESHOLD 35.0f   // fallback normal mode (unused in Mode 1 drying control)
#define HUM_THRESHOLD 60.0f

// PWM
const int freq = 5000;
const int fanChannel1 = 0;
const int fanChannel2 = 1;
const int pwmResolution = 8;

// Buttons / debounce / override
unsigned long lastPress = 0;
const int debounceDelay = 200;
const unsigned long OVERRIDE_HOLD_MS = 3000UL; // mech override open lock
const unsigned long CANCEL_HOLD_MS = 3000UL;   // cancel drying by holding SELECT

// ========== UI & MENU ==========
bool inSubMenu = false;
int currentPage = 0;
String pages[] = {"Environment", "Grains", "Legumes", "Fruits", "Root Tubers", "Stem Tubers"};
int totalPages = sizeof(pages) / sizeof(pages[0]);

// ========== PERIPHERALS ==========
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
RTC_DS3231 rtc;
WebServer server(80);
MFRC522 rfid(SS_PIN, RST_PIN);
byte registeredUID[4] = {0x3B, 0x07, 0x92, 0x5F}; // replace with your card UID (Mode1 auth)

// ========== SENSOR DATA ==========
struct SensorData { float temp; float hum; float press; String timeStr; };
SensorData sensors[2]; // sensors[0] = inlet (collector), sensors[1] = chamber

// ========== STATES ==========
bool fan1On=false, fan2On=false, lockOpen=false;
unsigned long fan1Start=0, fan2Start=0, lockStart=0;

// ========== DRYING / CROPS ==========
bool dryingMode = false;
bool dryingCompleted = false;
unsigned long dryingStart = 0;
unsigned long dryingDuration = 0; // ms
float targetTemp = 0.0f, targetHum = 0.0f;
float finalHumidityThreshold = 0.0f; // chamber humidity at which drying can stop
String alertMessage = "";

// Crop profiles defined by user ranges.
// We will use midpoint of the temp range as targetTemp,
// and the LOWER bound of final humidity range as final threshold (conservative).
struct CropProfile {
  String name;
  float tempMin, tempMax;
  float finalHumMin, finalHumMax;
  unsigned long timeMs; // fallback max time
};
CropProfile cropProfiles[] = {
  {"Grains", 40.0f, 55.0f, 60.0f, 70.0f, 6UL*60UL*60UL*1000UL},       // 6 hrs default
  {"Legumes",45.0f,55.0f,50.0f,60.0f,7UL*60UL*60UL*1000UL},         // 7 hrs
  {"Fruits",50.0f,70.0f,55.0f,70.0f,8UL*60UL*60UL*1000UL},          // 8 hrs
  {"Root Tubers",60.0f,80.0f,65.0f,75.0f,10UL*60UL*60UL*1000UL},    // 10 hrs
  {"Stem Tubers",60.0f,80.0f,65.0f,75.0f,10UL*60UL*60UL*1000UL},    // 10 hrs
  {"Vegetables",50.0f,60.0f,50.0f,65.0f,4UL*60UL*60UL*1000UL}       // 4 hrs
};
int cropCount = sizeof(cropProfiles)/sizeof(cropProfiles[0]);

// ========== PID-like controllers ==========
struct PIDController {
  float kp; float ki; float kd;
  float integral; float lastError;
  unsigned long lastMillis;
  float outMin, outMax;
  PIDController(float p=1,float i=0.1,float d=0.0,float omin=0,float omax=255) {
    kp=p; ki=i; kd=d; integral=0; lastError=0; lastMillis=millis();
    outMin=omin; outMax=omax;
  }
  int update(float setpoint, float measured) {
    unsigned long now = millis();
    float dt = (now - lastMillis)/1000.0f;
    if (dt <= 0) dt = 0.001f;
    float error = setpoint - measured;
    integral += error * dt;
    float derivative = (error - lastError) / dt;
    float out = kp*error + ki*integral + kd*derivative;
    lastError = error;
    lastMillis = now;
    if (out < outMin) out = outMin;
    if (out > outMax) out = outMax;
    return (int)out;
  }
  void reset() { integral=0; lastError=0; lastMillis=millis(); }
};
PIDController pidTemp(6.0f, 0.08f, 0.03f, 0, 255); // tune carefully
PIDController pidHum(5.0f, 0.06f, 0.02f, 0, 255);

// ========== HEATER SAFETY ==========
unsigned long heaterOnStart = 0;
bool heaterForcedOff = false;
unsigned long heaterCooldownStart = 0;
#define MAX_HEATER_ON_TIME 30000UL   // 30 seconds
#define HEATER_COOLDOWN_TIME 10000UL // 10 seconds
#define MIN_FAN_SPEED_DURING_DRYING 128 // ensure fans always on during drying (50%)

// ========== UTIL ==========
void safePrint(const String &s) { Serial.println(s); }
void tcaSelect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

// Read both sensors via TCA (0=inlet,1=chamber)
void readSensors() {
  for (uint8_t i=0;i<2;i++) {
    tcaSelect(i);
    sensors_event_t humEvent, tempEvent;
    aht.getEvent(&humEvent, &tempEvent);
    float pressure = bmp.readPressure() / 100.0F;
    DateTime now = rtc.now();
    sensors[i].temp = tempEvent.temperature;
    sensors[i].hum = humEvent.relative_humidity;
    sensors[i].press = pressure;
    sensors[i].timeStr = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
  }
}

// ========== FAN NORMAL MODE (unused while drying) ==========
void controlFansNormal() {
  // fan1 from inlet temp threshold (simple on/off)
  if (sensors[0].temp > TEMP_THRESHOLD) {
    if (!fan1On) { fan1On = true; fan1Start = millis(); }
    ledcWrite(fanChannel1, 255);
  } else {
    if (fan1On) fan1On = false;
    ledcWrite(fanChannel1, 0);
  }
  // fan2 from chamber humidity threshold (simple on/off)
  if (sensors[1].hum > HUM_THRESHOLD) {
    if (!fan2On) { fan2On = true; fan2Start = millis(); }
    ledcWrite(fanChannel2, 255);
  } else {
    if (fan2On) fan2On = false;
    ledcWrite(fanChannel2, 0);
  }
}

// ========== HEATER SAFETY CONTROLLER ==========
void heaterSafetyControl(bool requestHeat, int fan1Speed) {
  // If in forced cooldown, keep heater off until cooldown time passes
  if (heaterForcedOff) {
    if (millis() - heaterCooldownStart < HEATER_COOLDOWN_TIME) {
      digitalWrite(HEATER_PIN, LOW);
      digitalWrite(HEATER_LED_PIN, LOW);
      return;
    } else {
      heaterForcedOff = false;
    }
  }

  if (!requestHeat) {
    // ensure off
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(HEATER_LED_PIN, LOW);
    heaterOnStart = 0;
    return;
  }

  // If fan1 very high, disable heater to avoid over-extraction
  if (fan1Speed > 220) { // ~86% PWM
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(HEATER_LED_PIN, LOW);
    return;
  }

  // If fan1 high, reduce heater duty (pulsing strategy)
  bool reducedDuty = false;
  if (fan1Speed > 180) reducedDuty = true; // >70% reduce heater

  // Turn heater on if it's off
  if (digitalRead(HEATER_PIN) == LOW) {
    digitalWrite(HEATER_PIN, HIGH);
    digitalWrite(HEATER_LED_PIN, HIGH);
    heaterOnStart = millis();
  }

  // If pulsing due to high fan speed, pulse heater 1s on / 1s off
  if (reducedDuty) {
    if (((millis() / 1000) % 2) == 0) {
      // allow on (already on)
    } else {
      digitalWrite(HEATER_PIN, LOW);
      digitalWrite(HEATER_LED_PIN, LOW);
    }
  }

  // Safety: check max continuous ON time
  if (heaterOnStart != 0 && (millis() - heaterOnStart > MAX_HEATER_ON_TIME)) {
    // force cooldown
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(HEATER_LED_PIN, LOW);
    heaterForcedOff = true;
    heaterCooldownStart = millis();
    safePrint("Heater forced off for cooldown");
    heaterOnStart = 0;
  }
}

// ========== STOP DRYING ==========
void stopDrying(bool canceled=false) {
  dryingMode = false;
  digitalWrite(HEATER_PIN, LOW);
  digitalWrite(HEATER_LED_PIN, LOW);
  ledcWrite(fanChannel1, 0);
  ledcWrite(fanChannel2, 0);
  fan1On = fan2On = false;
  pidTemp.reset(); pidHum.reset();
  if (canceled) {
    alertMessage = "Drying canceled";
    // cancel short pattern
    for (int i=0;i<2;i++) {
      digitalWrite(BUZZER_PIN,HIGH); digitalWrite(POWER_LED_PIN,HIGH);
      delay(120);
      digitalWrite(BUZZER_PIN,LOW); digitalWrite(POWER_LED_PIN,LOW);
      delay(80);
    }
  } else {
    alertMessage = "Drying complete";
  }
  dryingCompleted = true;
  safePrint("Drying stopped: " + alertMessage);

  // webhook (best-effort simple POST) - only host-only naive implementation
  if (strlen(WEBHOOK_URL) > 3) {
    WiFiClient client;
    // naive parse: expect "http://host/path" or "http://host"
    String url = String(WEBHOOK_URL);
    String host = url;
    String path = "/";
    if (url.startsWith("http://")) host = url.substring(7);
    int slash = host.indexOf('/');
    if (slash >= 0) { path = host.substring(slash); host = host.substring(0, slash); }
    if (client.connect(host.c_str(), 80)) {
      String payload = "{\"event\":\"drying_complete\",\"message\":\"" + alertMessage + "\"}";
      String req = String("POST ") + path + " HTTP/1.1\r\n";
      req += "Host: " + host + "\r\n";
      req += "Content-Type: application/json\r\n";
      req += "Content-Length: " + String(payload.length()) + "\r\n\r\n";
      req += payload;
      client.print(req);
      delay(200);
      client.stop();
      safePrint("Webhook POST attempted");
    } else {
      safePrint("Webhook connect failed");
    }
  }

  // final beep + LED pattern
  for (int i=0;i<4;i++) {
    digitalWrite(BUZZER_PIN,HIGH); digitalWrite(POWER_LED_PIN,HIGH);
    delay(140);
    digitalWrite(BUZZER_PIN,LOW); digitalWrite(POWER_LED_PIN,LOW);
    delay(120);
  }
}

// ========== DRYING CONTROL (PID-like) ==========
void handleDrying() {
  if (!dryingMode) return;

  unsigned long elapsed = millis() - dryingStart;
  if (elapsed >= dryingDuration) {
    stopDrying(false);
    return;
  }

  // First: PID outputs
  int tempOut = pidTemp.update(targetTemp, sensors[1].temp); // use chamber sensor for temp control
  int humOut = pidHum.update(targetHum, sensors[1].hum);

  // Ensure fans always run during drying at least at MIN_FAN_SPEED_DURING_DRYING
  int fan1Speed = tempOut;
  if (fan1Speed < MIN_FAN_SPEED_DURING_DRYING) fan1Speed = MIN_FAN_SPEED_DURING_DRYING;
  if (fan1Speed > 255) fan1Speed = 255;
  ledcWrite(fanChannel1, fan1Speed);
  fan1On = (fan1Speed > 0);

  int fan2Speed = humOut;
  if (fan2Speed < MIN_FAN_SPEED_DURING_DRYING) fan2Speed = MIN_FAN_SPEED_DURING_DRYING;
  if (fan2Speed > 255) fan2Speed = 255;
  ledcWrite(fanChannel2, fan2Speed);
  fan2On = (fan2Speed > 0);

  // Heater request is based on chamber temperature (sensor[1])
  bool heatNeeded = (sensors[1].temp < targetTemp - 0.5f);

  // Heater safety control considers fan1Speed (higher fan speeds can disable/reduce heater)
  heaterSafetyControl(heatNeeded, fan1Speed);

  // Check final humidity threshold: if chamber humidity <= finalHumidityThreshold, stop drying.
  if (sensors[1].hum <= finalHumidityThreshold) {
    safePrint("Final humidity reached: " + String(sensors[1].hum));
    stopDrying(false);
    return;
  }

  // Optionally show dynamic info on OLED in showEnvironmentPage()
}

// ========== RFID (Mode 1) ==========
void checkRFID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;
  Serial.print("Tag UID: ");
  bool match = true;
  for (byte i=0;i<4;i++){
    Serial.print(rfid.uid.uidByte[i], HEX);
    if (rfid.uid.uidByte[i] != registeredUID[i]) match = false;
  }
  Serial.println();
  if (match) {
    digitalWrite(SOLENOID_PIN, HIGH);
    lockOpen = true; lockStart = millis();
    safePrint("Access Granted: Solenoid OPEN");
    // short beep
    digitalWrite(BUZZER_PIN,HIGH); delay(120); digitalWrite(BUZZER_PIN,LOW);
    // also, if we were awaiting auth to start drying, start now (we track awaitingAuth via global)
  } else {
    safePrint("Access Denied");
    // denial beep
    digitalWrite(BUZZER_PIN,HIGH); delay(60); digitalWrite(BUZZER_PIN,LOW); delay(80);
    digitalWrite(BUZZER_PIN,HIGH); delay(60); digitalWrite(BUZZER_PIN,LOW);
  }
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// ========== WEB UI ==========
void handleRoot() {
  String html = "<html><head><title>Smart Dryer</title><meta name='viewport' content='width=device-width'>";
  html += "<style>body{text-align:center;font-family:Arial;} .sensor{border:1px solid #333;padding:10px;margin:10px;display:inline-block;}</style></head><body>";
  html += "<h2>ESP32 Smart Dryer</h2>";
  html += "<b>Mode:</b> Mode 1 (RFID required)<br>";
  html += "<b>Fan1:</b> " + String(fan1On ? "ON" : "OFF") + "<br>";
  html += "<b>Fan2:</b> " + String(fan2On ? "ON" : "OFF") + "<br>";
  html += "<b>Heater:</b> " + String(digitalRead(HEATER_PIN) ? "ON" : "OFF") + "<br>";
  html += "<b>Lock:</b> " + String(lockOpen ? "OPEN" : "LOCKED") + "<br>";
  html += "<b>Drying Mode:</b> " + String(dryingMode ? "RUNNING" : dryingCompleted ? "COMPLETE" : "IDLE") + "<br>";
  if (dryingMode) {
    unsigned long elapsed = millis() - dryingStart;
    unsigned long remain = dryingDuration > elapsed ? (dryingDuration - elapsed) : 0;
    html += "<b>Time left:</b> " + String(remain/1000) + " s<br>";
  }
  if (alertMessage.length()) html += "<hr><b>Alert:</b> " + alertMessage + "<br>";
  for (int i=0;i<2;i++){
    html += "<div class='sensor'><h3>Sensor " + String(i+1) + "</h3>";
    html += "Temp: " + String(sensors[i].temp) + " °C<br>";
    html += "Hum: " + String(sensors[i].hum) + " %<br>";
    html += "Press: " + String(sensors[i].press) + " hPa<br>";
    html += "Time: " + sensors[i].timeStr + "</div>";
  }
  html += "<script>setTimeout(()=>location.reload(),1500)</script></body></html>";
  server.send(200, "text/html", html);
}

// ========== OLED UI ==========
void showMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("== Main Menu ==");
  display.drawLine(0,10,128,10,SH110X_WHITE);
  display.setTextSize(2);
  display.setCursor(0,25);
  display.println(pages[currentPage]);
  display.setTextSize(1);
  display.setCursor(0,55);
  display.println("Next=Move  Select=Enter");
  display.display();
}

void showEnvironmentPage() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("Environment");
  display.drawLine(0,10,128,10,SH110X_WHITE);

  char buf[64];
  // Show inlet (sensor0) and chamber (sensor1)
  snprintf(buf,sizeof(buf),"In.T: %.1fC In.H: %.1f%%", sensors[0].temp, sensors[0].hum);
  display.setCursor(0,12); display.println(buf);
  snprintf(buf,sizeof(buf),"Ch.T: %.1fC Ch.H: %.1f%%", sensors[1].temp, sensors[1].hum);
  display.setCursor(0,26); display.println(buf);

  snprintf(buf,sizeof(buf),"F1:%s F2:%s", fan1On?"ON":"OFF", fan2On?"ON":"OFF");
  display.setCursor(0,40); display.println(buf);

  snprintf(buf,sizeof(buf),"Hea:%s", digitalRead(HEATER_PIN) ? "ON" : "OFF");
  display.setCursor(0,50); display.println(buf);

  if (dryingMode) {
    unsigned long elapsed = millis() - dryingStart;
    float pct = (float)elapsed / (float)dryingDuration;
    if (pct > 1.0f) pct = 1.0f;
    int barW = 100;
    int filled = (int)(pct * barW);
    display.drawRect(14,56,barW,6,SH110X_WHITE);
    for (int x=0; x<filled; x+=2) display.drawFastVLine(14 + x, 56, 6, SH110X_WHITE);
    unsigned long remain = dryingDuration > elapsed ? (dryingDuration - elapsed) : 0;
    display.setCursor(0,58);
    char tbuf[20]; snprintf(tbuf,sizeof(tbuf)," %lus", remain/1000); display.print(tbuf);
  }
  display.display();
}

void showSubMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("Category: ");
  display.println(pages[currentPage]);
  display.drawLine(0,10,128,10,SH110X_WHITE);

  if (currentPage == 0) {
    showEnvironmentPage();
  } else {
    int idx = currentPage - 1;
    if (idx >= 0 && idx < cropCount) {
      display.setCursor(0,22); display.println(cropProfiles[idx].name);
      display.setCursor(0,34);
      char buf[40];
      float tmid = (cropProfiles[idx].tempMin + cropProfiles[idx].tempMax) / 2.0f;
      snprintf(buf,sizeof(buf),"T: %.0f-%.0fC (Tgt %.0fC)", cropProfiles[idx].tempMin, cropProfiles[idx].tempMax, tmid);
      display.println(buf);
      display.setCursor(0,46);
      char t2[40]; snprintf(t2,sizeof(t2),"Final H: %.0f-%.0f%%", cropProfiles[idx].finalHumMin, cropProfiles[idx].finalHumMax);
      display.println(t2);
      display.setCursor(0,56); display.println("Select -> Tap RFID to start");
    } else {
      display.setCursor(0,25); display.println("Info loading...");
    }
    display.display();
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  safePrint("Booting...");

  Wire.begin(21,22); // SDA, SCL
  SPI.begin(18,19,23,5); // SCK, MISO, MOSI, SS

  // pins
  pinMode(BTN_NEXT, INPUT);
  pinMode(BTN_SELECT, INPUT);
  pinMode(POWER_LED_PIN, OUTPUT);
  pinMode(HEATER_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);

  digitalWrite(POWER_LED_PIN, HIGH); // system on
  digitalWrite(HEATER_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(HEATER_PIN, LOW);
  digitalWrite(SOLENOID_PIN, LOW);

  // display
  if (!display.begin(OLED_ADDR, true)) safePrint("OLED not found!");
  display.clearDisplay(); display.setTextColor(SH110X_WHITE);

  // sensors & rtc & rfid
  if (!rtc.begin()) safePrint("RTC not found!");
  if (!aht.begin()) safePrint("AHT not found!");
  if (!bmp.begin(0x76)) safePrint("BMP280 not found!");
  rfid.PCD_Init(); safePrint("RFID initialized");

  // PWM fans
  ledcSetup(fanChannel1, freq, pwmResolution);
  ledcSetup(fanChannel2, freq, pwmResolution);
  ledcAttachPin(FAN1_PIN, fanChannel1);
  ledcAttachPin(FAN2_PIN, fanChannel2);

  // WiFi AP + server
  WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/", handleRoot);
  server.begin();
  safePrint("Web server started, AP IP: " + WiFi.softAPIP().toString());

  // initial sensor read
  readSensors();

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("System Ready (Mode 1)");
  display.display();
  delay(800);
  showMenu();
}

// ========== LOOP ==========
bool awaitingAuth = false;
unsigned long authWaitStart = 0;
const unsigned long AUTH_TIMEOUT_MS = 30000UL; // 30s to tap RFID after selecting crop

void loop() {
  server.handleClient();
  readSensors();
  checkRFID(); // this also handles access unlocking

  // Buttons & override
  bool nextPressed = digitalRead(BTN_NEXT) == HIGH;
  bool selectPressed = digitalRead(BTN_SELECT) == HIGH;

  // Mechanical override: both buttons held => open solenoid (3s)
  static unsigned long overrideStart = 0;
  if (nextPressed && selectPressed) {
    if (overrideStart == 0) overrideStart = millis();
    else if (millis() - overrideStart >= OVERRIDE_HOLD_MS && !lockOpen) {
      digitalWrite(SOLENOID_PIN, HIGH);
      lockOpen = true; lockStart = millis();
      safePrint("Mechanical override: Lock opened");
      // confirm beep
      digitalWrite(BUZZER_PIN,HIGH); delay(200); digitalWrite(BUZZER_PIN,LOW);
      overrideStart = 0;
    }
  } else overrideStart = 0;

  // SELECT long press to cancel drying when dryingMode active
  static unsigned long selectHoldStart = 0;
  if (selectPressed) {
    if (selectHoldStart == 0) selectHoldStart = millis();
    else if (dryingMode && millis() - selectHoldStart >= CANCEL_HOLD_MS) {
      stopDrying(true);
      selectHoldStart = 0;
    }
  } else selectHoldStart = 0;

  // Normal navigation (when not awaitingAuth)
  if (!awaitingAuth) {
    if (nextPressed && millis() - lastPress > debounceDelay) {
      lastPress = millis();
      if (!inSubMenu) {
        currentPage++;
        if (currentPage >= totalPages) currentPage = 0;
        showMenu();
      } else {
        // ignore or expand
      }
    }

    if (selectPressed && millis() - lastPress > debounceDelay) {
      lastPress = millis();
      if (!inSubMenu) {
        inSubMenu = true; showSubMenu();
      } else {
        // inside submenu
        if (currentPage == 0) {
          inSubMenu = false; showMenu();
        } else {
          int idx = currentPage - 1;
          if (idx >=0 && idx < cropCount) {
            // User selected crop: REQUIRE RFID auth (Mode 1)
            awaitingAuth = true;
            authWaitStart = millis();
            display.clearDisplay();
            display.setCursor(0,0);
            display.println("Tap RFID to start");
            display.drawLine(0,10,128,10,SH110X_WHITE);
            display.setCursor(0,22);
            display.println(cropProfiles[idx].name);
            display.setCursor(0,40);
            display.println("Waiting for card...");
            display.display();
            // store intended profile in globals
            float tmid = (cropProfiles[idx].tempMin + cropProfiles[idx].tempMax)/2.0f;
            targetTemp = tmid;
            // target humidity maybe pick lower bound of final range
            finalHumidityThreshold = cropProfiles[idx].finalHumMin;
            targetHum = cropProfiles[idx].finalHumMax; // target humidity for PID (we keep a target range)
            dryingDuration = cropProfiles[idx].timeMs; // fallback max
          } else {
            inSubMenu = false; showMenu();
          }
        }
      }
    }
  } else {
    // awaiting RFID auth to start drying
    // Check if card was presented by reading rfid in checkRFID() — we need to detect last tag match.
    // To implement this we will poll for a tag here and require it to match registeredUID.
    // We'll do a simplified polling: if a valid tag is presented, start drying.
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      bool match = true;
      for (byte i=0;i<4;i++) {
        if (rfid.uid.uidByte[i] != registeredUID[i]) match = false;
      }
      if (match) {
        // authorized -> start drying
        dryingStart = millis();
        dryingMode = true;
        dryingCompleted = false;
        awaitingAuth = false;
        pidTemp.reset(); pidHum.reset();
        alertMessage = "";
        safePrint("RFID authorized. Drying started. TargetT: " + String(targetTemp) + "C FinalH<= " + String(finalHumidityThreshold) );
        // start fans at minimal speed immediately
        ledcWrite(fanChannel1, MIN_FAN_SPEED_DURING_DRYING);
        ledcWrite(fanChannel2, MIN_FAN_SPEED_DURING_DRYING);
        fan1On = fan2On = true;
        // short ack beep
        digitalWrite(BUZZER_PIN,HIGH); delay(120); digitalWrite(BUZZER_PIN,LOW);
        rfid.PICC_HaltA();
        rfid.PCD_StopCrypto1();
      } else {
        // Not authorized
        safePrint("RFID presented but not authorized");
        // quick deny beep
        digitalWrite(BUZZER_PIN,HIGH); delay(60); digitalWrite(BUZZER_PIN,LOW);
        rfid.PICC_HaltA();
        rfid.PCD_StopCrypto1();
      }
    }
    // timeout waiting for RFID
    if (millis() - authWaitStart > AUTH_TIMEOUT_MS) {
      awaitingAuth = false;
      inSubMenu = false;
      safePrint("Auth wait timed out");
      showMenu();
    }
  }

  // Control loop
  if (!dryingMode) controlFansNormal();
  else handleDrying();

  // Update OLED
  if (inSubMenu) {
    if (currentPage == 0) showEnvironmentPage();
    else showSubMenu();
  } else {
    showMenu();
  }

  // auto-lock solenoid after 5s
  if (lockOpen && millis() - lockStart > 5000) {
    digitalWrite(SOLENOID_PIN, LOW);
    lockOpen = false;
    safePrint("Solenoid locked again.");
  }

  // one-time drying complete handling
  if (dryingCompleted) {
    // keep message on web UI; reset dryingCompleted flag so we don't re-run patterns
    dryingCompleted = false;
  }

  delay(300); // main loop pacing
}