/*****************************************
 ESP32 + KY-026 (flame) + KY-012 (active buzzer)
 Blynk Legacy ready-to-flash sketch

 Widgets in Blynk app:
 - Gauge -> V1 (0..100)
 - Label -> V2 (text status / raw)
 - LED   -> V3 (alarm indicator)
 - Mute Button (SWITCH) -> V4
 - Optional Notification Label -> V5 (receives notification text)

 IMPORTANT:
 - Replace auth/ssid/pass with your values.
 - If you want continuous Gauge use useAnalog = true and connect AO -> ADC1 pin (32..39).
 - If you prefer binary Gauge use useAnalog = false and connect DO -> digital pin.
 - Power KY-026 at 3.3V when connecting AO/DO directly to ESP32 pins.
*****************************************/

/* Template defines (put before Blynk includes to satisfy newer libs) */
#define BLYNK_TEMPLATE_ID "TMPL6L4yXvtDG"
#define BLYNK_TEMPLATE_NAME "ESP32 Flame Alarm"
#define BLYNK_AUTH_TOKEN "FQetwFeu_pQeOiRrRGbPG5o3snGQZqES"
#define BLYNK_PRINT Serial

/* Print Blynk debug to Serial */
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

/* --- FILL THESE --- */
char auth[] = "FQetwFeu_pQeOiRrRGbPG5o3snGQZqES";
char ssid[] = "SISWA_SMK_TELKOM_BANDUNG";
char pass[] = "#SmkTelkomBDG25";
/* ------------------- */

/* Configuration */
const bool useAnalog = true;          // true = AO (continuous gauge), false = DO (binary)
const int flameAnalogPin = 35;        // AO -> ADC1 pin (32..39). Used if useAnalog == true
const int flameDigitalPin = 19;       // DO -> digital input
const int buzzerPin = 23;             // KY-012 SIG -> GPIO23

const bool digitalActiveLow = true;   // If DO is LOW when flame present
int analogThreshold = 1200;           // 0..4095 threshold for AO detection (tune)
const int SAMPLE_INTERVAL_MS = 100;   // sampling interval
const int CONSECUTIVE_TO_TRIGGER = 2; // number of consecutive detections to trigger alarm
const unsigned long muteDurationMs = 60UL * 1000UL; // mute time when pressing Mute (60s)

/* AO smoothing */
const int AO_SMOOTH_SAMPLES = 3;

BlynkTimer timer;

int consecutiveDetected = 0;
bool alarmActive = false;
bool muted = false;
unsigned long muteUntil = 0;

/* Helper: moving average analog read */
int readAnalogSmoothed(int pin) {
  long sum = 0;
  for (int i = 0; i < AO_SMOOTH_SAMPLES; ++i) {
    sum += analogRead(pin);
    delay(5);
  }
  return (int)(sum / AO_SMOOTH_SAMPLES);
}

/* Helper: send "notification" to app in a safe way (avoids calling Blynk.notify()
   which may not exist in all library versions). We update a virtual pin V5
   and also print to Serial. If your library supports Blynk.notify(), you may
   uncomment the Blynk.notify() line below. */
void sendNotification(const String &msg) {
  // If your installed Blynk legacy library supports Blynk.notify,
  // you can uncomment the next line:
  // Blynk.notify(msg);
  // Fallback: write message to virtual pin V5 (Label widget) and Serial
  Blynk.virtualWrite(V5, msg);
  Serial.println(String("NOTIFY: ") + msg);
}

/* Trigger and stop alarm */
void triggerAlarm() {
  alarmActive = true;
  digitalWrite(buzzerPin, HIGH);
  Blynk.virtualWrite(V3, 255);            // LED ON
  Blynk.virtualWrite(V2, "ALARM: FLAME!"); // Label update
  Blynk.virtualWrite(V1, 100);            // Gauge -> full
  sendNotification("ALARM: Flame detected!");
  Serial.println("ALARM: Flame detected!");
}

void stopAlarm() {
  alarmActive = false;
  digitalWrite(buzzerPin, LOW);
  Blynk.virtualWrite(V3, 0); // LED OFF
  Serial.println("Alarm stopped");
  consecutiveDetected = 0;
}

/* Main sensor read + UI update */
void readSensorAndUpdateUI() {
  bool detected = false;

  if (useAnalog) {
    int raw = readAnalogSmoothed(flameAnalogPin); // 0..4095
    int pct = (raw * 100) / 4095;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    // Update Gauge (V1) and Label (V2)
    Blynk.virtualWrite(V1, pct);
    Blynk.virtualWrite(V2, String("RAW: ") + raw + " | " + String(pct) + "%");

    detected = (raw >= analogThreshold);
    Serial.printf("AO=%d pct=%d -> %s\n", raw, pct, detected ? "DETECTED" : "none");
  } else {
    int val = digitalRead(flameDigitalPin);
    detected = (val == (digitalActiveLow ? LOW : HIGH));
    Blynk.virtualWrite(V1, detected ? 100 : 0);
    Blynk.virtualWrite(V2, detected ? "FLAME" : "NO FLAME");
    Serial.printf("DO=%d -> %s\n", val, detected ? "DETECTED" : "none");
  }

  // Mute expiration
  if (muted && millis() > muteUntil) {
    muted = false;
    Serial.println("Mute expired");
    // Optionally update Label to remove mute note
  }

  // Alarm logic
  if (!muted && detected) consecutiveDetected++;
  else consecutiveDetected = 0;

  if (!alarmActive && consecutiveDetected >= CONSECUTIVE_TO_TRIGGER) {
    triggerAlarm();
  }
  if (alarmActive && !detected) {
    stopAlarm();
  }
}

/* Mute button handler (V4 expected to be SWITCH mode) */
BLYNK_WRITE(V4) {
  int v = param.asInt();
  if (v) {
    muted = true;
    muteUntil = millis() + muteDurationMs;
    stopAlarm();
    sendNotification("Alarm muted for 60 seconds");
    Serial.println("Muted by user");
  } else {
    // SWITCH released — no action needed
  }
}

/* Optional: allow remote force alarm via V6 button (uncomment to enable)
BLYNK_WRITE(V6) {
  if (param.asInt()) triggerAlarm();
  else stopAlarm();
}
*/

void setup() {
  Serial.begin(115200);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  if (useAnalog) {
    // Configure ADC
    analogReadResolution(12); // 0..4095
    #if defined(ARDUINO_ARCH_ESP32)
      // Set attenuation for the analog pin to read up to ~3.3V
      analogSetPinAttenuation(flameAnalogPin, ADC_11db);
    #endif
  } else {
    pinMode(flameDigitalPin, INPUT);
  }

  Serial.println("Connecting to WiFi/Blynk...");
  Blynk.begin(auth, ssid, pass);

  Serial.println("Ready. Mode:");
  Serial.println(useAnalog ? "ANALOG (AO -> Gauge continuous)" : "DIGITAL (DO -> Gauge binary)");

  timer.setInterval(SAMPLE_INTERVAL_MS, readSensorAndUpdateUI);
}

void loop() {
  Blynk.run();
  timer.run();
}
