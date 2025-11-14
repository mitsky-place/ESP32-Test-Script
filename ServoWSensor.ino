#include <ESP32Servo.h>
#include <WiFi.h>
#include <UniversalTelegramBot.h>
#include <WiFiClientSecure.h>
#include <NewPing.h>

// WiFi credentials
const char* ssid = "SISWA_SMK_TELKOM_BANDUNG";
const char* password = "#SmkTelkomBDG25";

// Telegram Bot Token (didapatkan dari @BotFather di Telegram)
const char* botToken = "8510605983:AAEjWi8IYK7nMQwu7xqsTCaFRWKPeR7pwvI";
const char* chat_id = "5403041412";

WiFiClientSecure client;
UniversalTelegramBot bot(botToken, client);

Servo myservo; // inisialisasi servo
#define TRIGGER_PIN 27 // Pin Trigger
#define ECHO_PIN 14 // Pin Echo
#define MAX_DISTANCE 100 // Jarak maksimum pengukuran ultrasonic
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  // Koneksi ke WiFi
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.println("Menghubungkan ke WiFi...");
  }
  Serial.println("Terhubung ke WiFi");
  client.setInsecure(); // Menonaktifkan verifikasi sertifikat SSL

  myservo.attach(2); // Menghubungkan servo ke pin digital 2
  bot.sendMessage(chat_id, "ESP32 telah siap dan terhubung ke WiFi.", "");
}

void loop() {
  delay(50);
  int jarak = sonar.ping_cm();
  Serial.println(jarak);

  // Jika jarak terdeteksi di bawah 10 cm
  if ((jarak > 0) && (jarak <= 10)) {
    // Kirim pesan bahwa tong sampah terbuka
    bot.sendMessage(chat_id, "Tong sampah terbuka.", "");

    // Membuka servo
    myservo.write(10);
    delay(1000);

    // Kirim pesan bahwa tong sampah tertutup setelah waktu tertentu
    myservo.write(100);
    bot.sendMessage(chat_id, "Tong sampah tertutup.", "");
    delay(1000);
  }
}
