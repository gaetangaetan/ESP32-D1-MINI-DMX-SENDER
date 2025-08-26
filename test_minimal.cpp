#include <Arduino.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  Serial.println("=== TEST MINIMAL ESP32 ===");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println("ESP32 fonctionne !");
}

void loop() {
  Serial.println("Heartbeat...");
  delay(2000);
}
