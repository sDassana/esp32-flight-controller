#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

const int escPins[4] = {12, 13, 14, 27};
Servo escs[4];
int escValues[4] = {1000, 1000, 1000, 1000};  // Initial throttle

const char* ssid = "DroneController";
const char* password = "12345678";

WebServer server(80);

void armESCs() {
  Serial.println("Arming ESCs (MAX → MIN throttle)");

  for (int i = 0; i < 4; i++) {
    escs[i].attach(escPins[i], 1000, 2000);
    escs[i].writeMicroseconds(2000);  // MAX throttle
  }
  delay(2000);
  for (int i = 0; i < 4; i++) {
    escs[i].writeMicroseconds(1000);  // MIN throttle
  }
  Serial.println("ESCs armed.");
}

void updateESC(int motorIndex, int value) {
  value = constrain(value, 1000, 2000);
  escValues[motorIndex] = value;
  escs[motorIndex].writeMicroseconds(value);
  Serial.printf("Motor %d set to %d µs\n", motorIndex + 1, value);
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>"
                "<style>body{font-family:sans-serif;text-align:center;}input{width:80%;}</style>"
                "<title>Drone ESC Control</title></head><body><h2>ESP32 Drone ESC Control</h2>";

  for (int i = 0; i < 4; i++) {
    html += "<p>Motor " + String(i + 1) + ": <span id='val" + String(i) + "'>" + String(escValues[i]) + "</span> µs<br>"
            "<input type='range' min='1000' max='2000' value='" + String(escValues[i]) + "' "
            "oninput='setVal(" + String(i) + ", this.value)'/></p>";
  }

  html += "<script>"
          "function setVal(motor, val){"
          "document.getElementById('val'+motor).innerText = val;"
          "fetch('/set?motor='+motor+'&val='+val);"
          "}"
          "</script></body></html>";

  server.send(200, "text/html", html);
}

void handleSet() {
  if (server.hasArg("motor") && server.hasArg("val")) {
    int m = server.arg("motor").toInt();
    int v = server.arg("val").toInt();
    if (m >= 0 && m < 4) updateESC(m, v);
  }
  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  Serial.print("WiFi AP started. Connect to: ");
  Serial.println(WiFi.softAPIP());

  armESCs();

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.begin();
}

void loop() {
  server.handleClient();
}
