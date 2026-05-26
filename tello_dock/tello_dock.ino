// Tello dock firmware (ESP32).
//
// A micro servo physically taps the Tello's power button so loki can cold-boot
// the drone over WiFi. The drone must be seated in a cradle with its power
// button parked under the servo arm.
//
// HARDWARE
//   ESP32 dev board (38-pin WROOM-32), SG90 micro servo.
//   Servo brown -> GND, red -> 5V (VIN), orange/signal -> GPIO13.
//   If the servo twitches the board on movement, add a 470uF cap across the
//   servo's 5V/GND. Power the ESP32 from any always-on USB source.
//
// LIBRARIES (Arduino Library Manager):
//   - WiFiManager (tzapu)  -> captive-portal WiFi setup; no credentials in firmware
//   - ESP32Servo           -> stock Servo lib can't drive the ESP32's LEDC PWM
//
// FLASH (from caesar, board plugged in via USB):
//   arduino-cli core install esp32:esp32
//   arduino-cli lib install WiFiManager ESP32Servo
//   arduino-cli compile --fqbn esp32:esp32:esp32 tello_dock
//   arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/cu.usbserial-XXXX tello_dock
//
// FIRST RUN
//   The board comes up as an open AP "TelloDock-Setup". Join it from a phone,
//   pick your home WiFi, enter the password (stored in the ESP32's NVS, never in
//   this file). After that it auto-joins and is reachable at http://tello-dock.local
//
// CALIBRATE (live, no reflash)
//   curl "http://tello-dock.local/set?idle=20&press=70&ms=1500"
//   Nudge idle until the arm sits just clear of the button, press until it fully
//   depresses it. Values persist across reboots.
//
// ENDPOINTS
//   GET /            human status page
//   GET /status      JSON state
//   GET /press?ms=N  one button tap held N ms (default = calibrated pressMs)
//   GET /on /off     aliases of /press (the button is a toggle; hardware can't
//                    tell on from off, the distinction lives in drone.py)
//   GET /set?idle=&press=&ms=   live-calibrate the two angles + hold duration

#include <WiFi.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ESP32Servo.h>
#include <Preferences.h>

static const int SERVO_PIN = 13;

Servo arm;
WebServer server(80);
Preferences prefs;

int idleAngle  = 20;    // arm parked clear of the button
int pressAngle = 70;    // arm pushing the button down
int pressMs    = 1500;  // how long to hold the press

void loadCal() {
  prefs.begin("dock", true);
  idleAngle  = prefs.getInt("idle", idleAngle);
  pressAngle = prefs.getInt("press", pressAngle);
  pressMs    = prefs.getInt("ms", pressMs);
  prefs.end();
}

void saveCal() {
  prefs.begin("dock", false);
  prefs.putInt("idle", idleAngle);
  prefs.putInt("press", pressAngle);
  prefs.putInt("ms", pressMs);
  prefs.end();
}

void doPress(int ms) {
  arm.write(pressAngle);
  delay(ms);
  arm.write(idleAngle);
}

String statusJson() {
  return String("{\"ok\":true,\"ip\":\"") + WiFi.localIP().toString() +
         "\",\"idle\":" + idleAngle +
         ",\"press\":" + pressAngle +
         ",\"ms\":" + pressMs + "}";
}

void handleRoot() {
  server.send(200, "text/html",
    "<h2>Tello dock</h2><pre>" + statusJson() + "</pre>"
    "<p>/press?ms=1500 &middot; /on &middot; /off &middot; "
    "/set?idle=20&press=70&ms=1500 &middot; /status</p>");
}

void handleStatus() { server.send(200, "application/json", statusJson()); }

void handlePress() {
  int ms = server.hasArg("ms") ? server.arg("ms").toInt() : pressMs;
  ms = constrain(ms, 100, 5000);
  doPress(ms);
  server.send(200, "application/json", String("{\"ok\":true,\"pressed_ms\":") + ms + "}");
}

void handleSet() {
  if (server.hasArg("idle"))  idleAngle  = constrain(server.arg("idle").toInt(), 0, 180);
  if (server.hasArg("press")) pressAngle = constrain(server.arg("press").toInt(), 0, 180);
  if (server.hasArg("ms"))    pressMs    = constrain(server.arg("ms").toInt(), 100, 5000);
  saveCal();
  arm.write(idleAngle);  // move to the new idle so you can eyeball the parked position
  server.send(200, "application/json", statusJson());
}

void setup() {
  Serial.begin(115200);
  loadCal();

  ESP32PWM::allocateTimer(0);
  arm.setPeriodHertz(50);
  arm.attach(SERVO_PIN, 500, 2400);
  arm.write(idleAngle);

  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  if (!wm.autoConnect("TelloDock-Setup")) {
    ESP.restart();
  }

  if (MDNS.begin("tello-dock")) {
    MDNS.addService("http", "tcp", 80);
  }

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/press", handlePress);
  server.on("/on", handlePress);
  server.on("/off", handlePress);
  server.on("/set", handleSet);
  server.begin();

  Serial.print("Dock ready at http://tello-dock.local  IP=");
  Serial.println(WiFi.localIP());
}

void loop() {
  server.handleClient();
}
