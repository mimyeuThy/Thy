#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include "SHTTask.h"

const char* ssid = "ESP32_CAR";
const char* password = "12345678";

WebServer server(80);

// ============================================================================
// 1. SƠ ĐỒ CHÂN (theo bảng đấu nối mới)
// ============================================================================
// --- Hai kenh IN1/IN2 va IN11/IN22 (GIU NGUYEN CHIEU XE DA CHAY DUNG) ---
#define IN1  26
#define IN2  27      // PWM kênh 4
#define IN11 32
#define IN22 33      // PWM kênh 5
// --- Hai kenh IN3/IN4 va IN33/IN44 (GIU NGUYEN CHIEU XE DA CHAY DUNG) ---
#define IN3  14      // PWM kênh 6
#define IN4  12
#define IN33 25      // PWM kênh 7
#define IN44 13

#define CH_IN2  4
#define CH_IN22 5
#define CH_IN3  6
#define CH_IN33 7


// Order used by HTTP and MQTT telemetry: front, rear, left, right.
const uint8_t trigPins[] = {5, 23, 15, 18};
const uint8_t echoPins[] = {34, 19, 35, 17};
const char* sensorNames[] = {"trước","sau","trái","phải"};
long lastDist[4] = {-1,-1,-1,-1};
unsigned long lastDistAt[4] = {};
Preferences prefs;
String statusMessage = "Sẵn sàng";

// BỘ TƯƠNG THÍCH PWM CHO ESP32 CORE V2.X VÀ V3.X
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define IS_ESP32_V3 1
#else
  #define IS_ESP32_V3 0
#endif

void setPwm(uint8_t pin, uint8_t channel, uint32_t val) {
#if IS_ESP32_V3
  ledcWrite(pin, val);
#else
  ledcWrite(channel, val);
#endif
}

// 3. BIẾN TRẠNG THÁI XE & IOT
char trangThaiHienTai = 'S';
char lastAppliedState = ' ';
int currentSpeed = 80;
int lastAppliedSpeed = -1;
constexpr int PWM_FREQ = 5000;
constexpr int PWM_RES = 8;
bool cmdFromMqtt = false;
unsigned long lastMqttCmd = 0;
void motor_Phai_Dung() {
  digitalWrite(IN1, LOW);  setPwm(IN2, CH_IN2, 0);
  digitalWrite(IN11, LOW); setPwm(IN22, CH_IN22, 0);
}
void motor_Phai_Tien(int speed) {
  digitalWrite(IN1, HIGH);  setPwm(IN2, CH_IN2, 255 - speed);
  digitalWrite(IN11, HIGH); setPwm(IN22, CH_IN22, 255 - speed);
}
void motor_Phai_Lui(int speed) {
  digitalWrite(IN1, LOW);  setPwm(IN2, CH_IN2, speed);
  digitalWrite(IN11, LOW); setPwm(IN22, CH_IN22, speed);
}

void motor_Trai_Dung() {
  setPwm(IN3, CH_IN3, 0);   digitalWrite(IN4, LOW);
  setPwm(IN33, CH_IN33, 0); digitalWrite(IN44, LOW);
}
void motor_Trai_Tien(int speed) {
  digitalWrite(IN4, LOW);  setPwm(IN3, CH_IN3, speed);
  digitalWrite(IN44, LOW); setPwm(IN33, CH_IN33, speed);
}
void motor_Trai_Lui(int speed) {
  digitalWrite(IN4, HIGH);  setPwm(IN3, CH_IN3, 255 - speed);
  digitalWrite(IN44, HIGH); setPwm(IN33, CH_IN33, 255 - speed);
}

// Motor output and autonomous lease are serialized with the independent watchdog.
SemaphoreHandle_t motorMutex=nullptr;
// The watchdog owns physical stopping independently of HTTP, MQTT and sensors.
const unsigned long MANUAL_LEASE_MS=450;
bool manualLeaseActive=false;
volatile bool manualLeaseTimedOut=false;
unsigned long manualLeaseAt=0;
String httpDriveSession="";
uint32_t httpLastSeq=0, httpLastGesture=0, httpClosedGesture=0;
unsigned long lastLocalHttpAt=0;
bool localHttpSeen=false;
void driveOutput(char command,int l,int r){
  if(motorMutex)xSemaphoreTake(motorMutex,portMAX_DELAY);
  if(manualLeaseTimedOut&&command!='S')command='S';
  if(command=='S'||command=='K')manualLeaseActive=false;
  switch(command){
    case 'F':motor_Trai_Tien(l);motor_Phai_Lui(r);break;
    case 'B':motor_Trai_Lui(l);motor_Phai_Tien(r);break;
    case 'R':motor_Trai_Tien(l);motor_Phai_Tien(r);break;
    case 'L':motor_Trai_Lui(l);motor_Phai_Lui(r);break;
    case 'K': // PHANH CHU DONG: hai dau moi H-bridge cung muc cao
      digitalWrite(IN1,HIGH);  setPwm(IN2,CH_IN2,255);
      digitalWrite(IN11,HIGH); setPwm(IN22,CH_IN22,255);
      digitalWrite(IN4,HIGH);  setPwm(IN3,CH_IN3,255);
      digitalWrite(IN44,HIGH); setPwm(IN33,CH_IN33,255);
      break;
    default:motor_Phai_Dung();motor_Trai_Dung();break;
  }

  if(motorMutex)xSemaphoreGive(motorMutex);
}
void xeTien(int l,int r){driveOutput('F',l,r);}
void xeLui(int l,int r){driveOutput('B',l,r);}
void xoayPhai(int s){driveOutput('R',s,s);}
void xoayTrai(int s){driveOutput('L',s,s);}
void dungXe(){driveOutput('S',0,0);}
void xePhanh(){driveOutput('K',0,0);}
void renewManualLease(unsigned long at){
  if(motorMutex)xSemaphoreTake(motorMutex,portMAX_DELAY);
  manualLeaseAt=at;manualLeaseActive=true;manualLeaseTimedOut=false;
  if(motorMutex)xSemaphoreGive(motorMutex);
}
void checkManualLease(){
  if(motorMutex)xSemaphoreTake(motorMutex,portMAX_DELAY);
  if(manualLeaseActive&&millis()-manualLeaseAt>=MANUAL_LEASE_MS){
    motor_Phai_Dung();motor_Trai_Dung();manualLeaseActive=false;manualLeaseTimedOut=true;
  }
  if(motorMutex)xSemaphoreGive(motorMutex);
}
void motorWatchdogTask(void*){
  for(;;){checkManualLease();vTaskDelay(pdMS_TO_TICKS(10));}
}

// ============================================================================
// IoT CONFIG & TELEMETRY
// ============================================================================
#define FW_VERSION "1.13.1-user-sht-task"
#define MQTT_TLS 0
#define OTA_PASSWORD "xe-ota-2026"
#define HAS_BATTERY_SENSE 0 // Enable only when GPIO36 has a voltage divider fitted.
#define BAT_PIN 36
const float BAT_DIV = 5.0f;      // tỉ lệ module đo áp 0-25V
#define BAT_FULL_V 12.6f         // Pin 3S
#define BAT_EMPTY_V 9.0f
#define DEADMAN_MS MANUAL_LEASE_MS
#define BRAKE_CM 5

volatile uint16_t wifiDisconnectReason=0;
bool wifiJoinPending=false,wifiJoining=false,wifiAttemptFailed=false,wifiHadLink=false;
unsigned long wifiJoinDue=0,wifiJoinAt=0;
void scheduleWifiJoin(){
  wifiJoinPending=true;wifiJoining=false;wifiAttemptFailed=false;
  wifiDisconnectReason=0;wifiJoinDue=millis()+500;
}
String staSsid = "", staPass = "", mqttHost = "broker.hivemq.com", mqttUser = "", mqttPass = "";
String deviceId = "", topicBase = "";
int mqttPort = 1883;
bool remoteEnabled = false;
const bool autoBrake = true;
bool otaStarted = false;
int brakeCnt = 0;
char brakeBlockedCmd = ' '; // Khóa hướng vừa bị phanh đến khi nhả nút/đổi hướng.
unsigned long lastBrakeCheck = 0;
unsigned long lastDistanceRefresh = 0;
// Khoa quyen lai tay: 1=HTTP/AP, 2=MQTT. Hai giao dien khong tranh lenh.
uint8_t driveOwner = 0;
float batCal = 1.0f, batV = -1;
unsigned long lastMqttTry = 0, lastTele = 0, lastBrakeEvt = 0, lastBatRead = 0, mqttRetryMs = 5000;

#if MQTT_TLS
  WiFiClientSecure netClient;
#else
  WiFiClient netClient;
#endif
PubSubClient mqtt(netClient);

String jsonEscape(const String &s) {
  String o;
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else if ((unsigned char)c < 0x20) o += ' ';
    else o += c;
  }
  return o;
}
#define EVQ 8
String evQ[EVQ];
int evHead = 0, evCount = 0;
String evLog[10];
int evLogN = 0;

void queueEvent(const String &type, const String &msg) {
  String j = "{\"type\":\"" + type + "\",\"msg\":\"" + jsonEscape(msg) + "\",\"up\":" + String(millis() / 1000) + "}";
  evLog[evLogN % 10] = j;
  evLogN++;
  if (evCount < EVQ) { evQ[(evHead + evCount) % EVQ] = j; evCount++; }
}
float readBatteryV() {
  if (!HAS_BATTERY_SENSE) return -1.0f;

  // Chỉ đo khi xe THỰC SỰ đứng yên (chế độ Follow/Avoid có thể di chuyển dù state='S').
  if (trangThaiHienTai != 'S') {
    return batV;
  }

  const int NUM_SAMPLES = 30;
  float samples[NUM_SAMPLES];

  for (int i = 0; i < NUM_SAMPLES; i++) {
    samples[i] = analogReadMilliVolts(BAT_PIN);
    delayMicroseconds(200);
  }

  for (int i = 0; i < NUM_SAMPLES - 1; i++) {
    for (int j = i + 1; j < NUM_SAMPLES; j++) {
      if (samples[i] > samples[j]) {
        float temp = samples[i];
        samples[i] = samples[j];
        samples[j] = temp;
      }
    }
  }

  float sum = 0.0f;
  for (int i = 10; i < 20; i++) sum += samples[i];

  float adcV = (sum / 10.0f) / 1000.0f;
  float voltage = adcV * BAT_DIV * batCal;

  if (voltage < 2.0f || voltage > 15.0f) {
    return (batV > 2.0f) ? batV : -1.0f;
  }

  batV = voltage;
  return batV;
}

int batteryPercent(float v) {
  if (v < 2.0f) return -1;
  float pct = ((v - BAT_EMPTY_V) / (BAT_FULL_V - BAT_EMPTY_V)) * 100.0f;
  return constrain((int)round(pct), 0, 100);
}

bool carIdle(){return trangThaiHienTai=='S';}
void sampleSensor(){
  static uint8_t i=0;
  static unsigned long lastPing=0;
  if(millis()-lastPing<60)return;
  lastPing=millis();
  digitalWrite(trigPins[i],HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[i],LOW);
  unsigned long pulse=pulseIn(echoPins[i],HIGH,9000);
  lastDist[i]=pulse ? max(1L,(long)((pulse+29)/58)) : -1;
  lastDistAt[i]=millis();
  i=(i+1)%4;
}
void emergencyStop() {
  if(httpLastGesture>httpClosedGesture)httpClosedGesture=httpLastGesture;
  dungXe();
  trangThaiHienTai = 'S';
  lastAppliedState = 'S';
  brakeBlockedCmd = ' ';
  driveOwner = 0;
  cmdFromMqtt = false;
  statusMessage="Dừng khẩn cấp";
  queueEvent("estop",statusMessage);
}
bool controllerBusy(uint8_t source) {
  return driveOwner != 0 && driveOwner != source && cmdFromMqtt &&
         trangThaiHienTai != 'S' && millis() - lastMqttCmd <= DEADMAN_MS;
}

// Mọi nguồn lệnh (HTTP/MQTT) dùng chung logic dừng / khóa phanh.
void acceptDriveCommand(char cmd, uint8_t source) {
  if (cmd != 'F' && cmd != 'B' && cmd != 'L' && cmd != 'R' && cmd != 'S') return;
  if (cmd == 'S') {
      trangThaiHienTai = 'S';
      cmdFromMqtt = false;
      if (trangThaiHienTai == 'S' && brakeBlockedCmd == ' ') statusMessage = "Đã dừng";
      brakeBlockedCmd = ' ';
      driveOwner = 0;
      dungXe();
      lastAppliedState = 'S';
    return;
  }
  if (controllerBusy(source)) return;
  if (cmd == brakeBlockedCmd) return;
  brakeBlockedCmd = ' ';
  if(manualLeaseTimedOut)lastAppliedState='S';
  if(trangThaiHienTai != cmd) {
    statusMessage = String("Lái tay · ") + cmd;
  }
  trangThaiHienTai = cmd;
  driveOwner = source;
  cmdFromMqtt = true; // Bộ ngắt độc lập 450 ms cho cả HTTP và MQTT.
  lastMqttCmd = millis();
  renewManualLease(lastMqttCmd);
}

void applyManualDrive() {
  if (manualLeaseTimedOut || (cmdFromMqtt && trangThaiHienTai != 'S' && millis() - lastMqttCmd > DEADMAN_MS)) {
    trangThaiHienTai = 'S';
    cmdFromMqtt = false;
    driveOwner = 0;
  }

  const char cmd=trangThaiHienTai;
  if(cmd!='S'){
    const int i=cmd=='F'?0:cmd=='B'?1:cmd=='L'?2:3;
    const bool fresh=lastDistAt[i] && millis()-lastDistAt[i]<500;
    if(!fresh || (lastDist[i]>0 && lastDist[i]<=BRAKE_CM)){
      brakeBlockedCmd=cmd;
      trangThaiHienTai='S';
      cmdFromMqtt=false;
      driveOwner=0;
      dungXe();
      lastAppliedState='S';
      statusMessage=String("Dừng: ")+sensorNames[i]+" "+(fresh ? String(lastDist[i])+" cm" : String("chưa có số đo mới"));
      queueEvent("brake",statusMessage);
      return;
    }
  }

  if (trangThaiHienTai == lastAppliedState && currentSpeed == lastAppliedSpeed) return;
  lastAppliedState = trangThaiHienTai;
  lastAppliedSpeed = currentSpeed;

  int tocDo = currentSpeed;
  switch (trangThaiHienTai) {
    case 'F': xeTien(tocDo, tocDo); break;
    case 'B': xeLui(tocDo, tocDo); break;
    case 'L': xoayTrai(180); break;
    case 'R': xoayPhai(180); break;
    case 'S': default: dungXe(); break;
  }
}

String distArrayJson() {
  String j = "[";
  for (int i = 0; i < 4; i++) { j += String(lastDist[i]); if (i < 3) j += ","; }
  j += "]";
  return j;
}
String telemetryJson() {
  String j = String("{\"fw\":\"") + FW_VERSION + "\",";
  j += SHTRoom::json();
  j += "\"up\":" + String(millis() / 1000) + ",";
  j += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  j += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  j += "\"mode\":\"" + String("manual") + "\",";
  j += "\"msg\":\"" + jsonEscape(statusMessage) + "\",";
  // d = [trước, sau, trái, phải]
  j += "\"d\":" + distArrayJson() + ",";
  j += "\"age\":[";
  for (int i = 0; i < 4; i++) {
    j += String(lastDistAt[i] == 0 ? -1L : (long)(millis() - lastDistAt[i]));
    if (i < 3) j += ",";
  }
  j += "],";
  j += "\"bat\":" + String(batV, 2) + ",\"batp\":" + String(batteryPercent(batV)) + ",";
  j += "\"spd\":" + String(currentSpeed) + ",";
  j += "\"manualMotor\":\"" + String(manualLeaseTimedOut?'S':trangThaiHienTai) + "\",";
  j += "\"manualPwm\":" + String(manualLeaseTimedOut||trangThaiHienTai=='S'?0:((trangThaiHienTai=='L'||trangThaiHienTai=='R')?180:currentSpeed)) + ",";
  j += "\"manualTimeout\":" + String(manualLeaseTimedOut?1:0) + ",";
  j += "\"owner\":" + String(driveOwner) + ",";
  j += "\"brake\":" + String(autoBrake ? 1 : 0) + ",";
  j += "\"remote\":" + String(remoteEnabled ? 1 : 0) + ",";
  j += "\"heap\":" + String(ESP.getFreeHeap()) + "}";
  return j;
}
void updateLocalTelemetry() {
  // Chỉ làm mới cảm biến khi xe đứng yên, không giành thời gian xử lý điều khiển.
  // HTTP telemetry returns cached samples; never read six sensors in its handler.
  // A separate idle tick below refreshes at most one sensor at a time.
  if (HAS_BATTERY_SENSE && millis() - lastBatRead > 2000) {
    lastBatRead = millis();
    batV = readBatteryV();
  }
}
void publishTelemetry() {
  updateLocalTelemetry();
  mqtt.publish((topicBase + "/telemetry").c_str(), telemetryJson().c_str());
}
#include "CarAudio.h"

void onMqtt(char* topic, byte* payload, unsigned int len) {
  if (CarAudio::downstream == topic) { CarAudio::receive(payload,len); return; }
  String t(topic), p;
  for (unsigned int i = 0; i < len && i < 64; i++) p += (char)payload[i];
  p.trim();
  String pre = topicBase + "/cmd/";
  if (!t.startsWith(pre)) return;
  String c = t.substring(pre.length());
  if (c == "estop") { emergencyStop(); return; }
  if (!remoteEnabled) { queueEvent("remote_blocked", "lệnh '" + c + "' bị chặn"); return; }
  if (c == "drive" && p.length() == 1) {
    acceptDriveCommand(p[0], 2);
  } else if (controllerBusy(2)) {
    queueEvent("controller_busy", "HTTP dang dieu khien; bo qua MQTT " + c);
    return;
  } else if (c == "speed") {
    currentSpeed = constrain((int)p.toInt(), 0, 255);
    lastAppliedSpeed = -1;
  }
}
void mqttConnect() {
  mqtt.setServer(mqttHost.c_str(), mqttPort);
  String clientId = "xe-" + deviceId + "-" + String((unsigned long)(esp_random() & 0xFFFF));
  String willTopic = topicBase + "/status";
  bool ok;
  if (mqttUser.length() > 0) ok = mqtt.connect(clientId.c_str(), mqttUser.c_str(), mqttPass.c_str(), willTopic.c_str(), 1, true, "offline");
  else                       ok = mqtt.connect(clientId.c_str(), NULL, NULL, willTopic.c_str(), 1, true, "offline");
  if (ok) {
    mqtt.publish(willTopic.c_str(), "online", true);
    CarAudio::connected();
    mqtt.subscribe((topicBase + "/cmd/#").c_str());
    queueEvent("mqtt", "đã kết nối broker " + mqttHost);
    lastTele = 0;
    mqttRetryMs = 5000;
  } else {
    Serial.printf("MQTT connect failed: state=%d host=%s port=%d retry=%lu ms\n",
                  mqtt.state(), mqttHost.c_str(), mqttPort, (unsigned long)mqttRetryMs);
    mqttRetryMs = mqttRetryMs * 2 > 30000 ? 30000 : mqttRetryMs * 2;
  }
}

void netLoop() {
  // Keep an established broker session alive while the local dashboard is open.
  // Only defer a NEW blocking DNS/TCP connection attempt during active local HTTP.
  if (otaStarted && carIdle()) ArduinoOTA.handle();
  unsigned long now = millis();
  if (WiFi.status() != WL_CONNECTED) return;
  bool localBusy = WiFi.softAPgetStationNum() > 0 ||
                   (localHttpSeen && now - lastLocalHttpAt < 3000) || driveOwner == 1;
  if (!mqtt.connected()) {
    if (!localBusy && now - lastMqttTry > mqttRetryMs && carIdle()) {
      lastMqttTry = now;
      mqttConnect();
    }
    return;
  }
  mqtt.loop();
  int n = 0;
  while (evCount > 0 && n < 2) {
    mqtt.publish((topicBase + "/event").c_str(), evQ[evHead].c_str());
    evHead = (evHead + 1) % EVQ; evCount--; n++;
  }
  if (now - lastTele >= 1000) { lastTele = now; publishTelemetry(); }
}

String wifiReasonText(){
  if(WiFi.status()==WL_CONNECTED)return "Đã kết nối và nhận IP";
  if(wifiJoinPending)return "Đã lưu, chuẩn bị kết nối";
  if(wifiJoining)return "Đang kết nối, chờ tối đa 25 giây";
  switch(wifiDisconnectReason){
    case 201:return "Không tìm thấy Wi-Fi: kiểm tra tên mạng, băng tần 2.4 GHz và tín hiệu";
    case 202:case 15:case 204:return "Lỗi xác thực: kiểm tra mật khẩu và bảo mật router";
    case 203:return "Router từ chối kết nối";
    case 200:return "Mất tín hiệu từ router";
    case 2:return "Xác thực hết hạn: kiểm tra tín hiệu và router";
    default:return wifiAttemptFailed?"Hết thời gian chờ IP; kiểm tra DHCP, router và tín hiệu":"Chưa kết nối Wi-Fi nhà";
  }
}
void wifiConnectionTick(){
  if(wifiJoinPending&&(long)(millis()-wifiJoinDue)>=0&&carIdle()){
    wifiJoinPending=false;wifiHadLink=false;wifiDisconnectReason=0;
    if(staSsid.length()){
      WiFi.begin(staSsid.c_str(),staPass.c_str());wifiJoining=true;wifiJoinAt=millis();
    }else{WiFi.disconnect(false,false);}
  }
  if(WiFi.status()==WL_CONNECTED){wifiJoining=false;wifiAttemptFailed=false;wifiDisconnectReason=0;wifiHadLink=true;return;}
  if(wifiHadLink&&!wifiAttemptFailed&&!wifiJoinPending&&!wifiJoining&&carIdle()){wifiHadLink=false;scheduleWifiJoin();}
  if(wifiJoining&&millis()-wifiJoinAt>=25000){
    wifiJoining=false;wifiAttemptFailed=true;WiFi.disconnect(false,false);
  }
}

void loadIotConfig() {
  uint64_t chip = ESP.getEfuseMac();
  char idbuf[16];
  snprintf(idbuf, sizeof(idbuf), "xe%06X", (unsigned int)(chip & 0xFFFFFF));
  staSsid = prefs.getString("ssid", "");
  staPass = prefs.getString("wpass", "");
  mqttHost = prefs.getString("host", mqttHost);
  mqttPort = prefs.getInt("port", mqttPort);
  mqttUser = prefs.getString("muser", "");
  mqttPass = prefs.getString("mpass", "");
  deviceId = prefs.getString("did", String(idbuf));
  topicBase = "pbl3/" + deviceId;
  remoteEnabled = prefs.getInt("remote", 0) != 0;
  batCal = prefs.getFloat("bcal", 1.0f);
}
void handleRoot() {
  localHttpSeen=true;lastLocalHttpAt=millis();
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <meta charset="UTF-8">
      <title>BT-Rover Gear Control</title>
      <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
      <style>
        body { text-align: center; font-family: 'Segoe UI', sans-serif; background: #121212; color: #fff; margin-top: 20px; user-select: none; -webkit-user-select: none; }
        h2 { color: #00d2ff; margin-bottom: 15px; }
        .control-panel { background: #1e1e1e; display: inline-block; padding: 20px; border-radius: 20px; box-shadow: 0 8px 16px rgba(0,0,0,0.6); max-width: 340px; width: 90%; touch-action: pan-y; }
        .btn { width: 100px; height: 55px; font-size: 18px; font-weight: bold; margin: 6px; border-radius: 12px; background: #007BFF; color: white; border: none; cursor: pointer; touch-action: manipulation; }
        .btn:active { background: #0056b3; transform: scale(0.95); }
        .btn-autopark { width: 224px; height: 48px; font-size: 15px; font-weight: bold; margin: 6px; border-radius: 12px; background: #ff4757; color: white; border: none; cursor: pointer; }
        .gear-container { margin-top: 15px; background: #2a2a2a; padding: 15px; border-radius: 12px; }
        .btn-gear { width: 65px; height: 40px; font-size: 15px; font-weight: bold; margin: 4px; border-radius: 8px; background: #444; color: white; border: none; cursor: pointer; }
        .btn-gear.active { background: #00d2ff; color: #121212; box-shadow: 0 0 10px #00d2ff; }
        .gear-text { font-size: 15px; color: #00d2ff; font-weight: bold; margin-top: 8px; display: block; }
        .wifi-container { margin-top: 15px; background: #2a2a2a; padding: 15px; border-radius: 12px; border: 1px solid #333; }
        .wifi-input { width: 85%; padding: 8px; margin-top: 8px; border-radius: 6px; border: 1px solid #555; background: #333; color: #fff; font-size: 14px; }
        .btn-wifi { width: 90%; padding: 10px; margin-top: 10px; background: #28a745; color: white; font-weight: bold; border: none; border-radius: 8px; cursor: pointer; }
        .btn-check { width: 90%; padding: 8px; margin-top: 6px; background: #17a2b8; color: white; font-weight: bold; border: none; border-radius: 8px; cursor: pointer; }
        .wifi-status { margin-top: 10px; font-size: 13px; font-weight: bold; color: #ffca28; min-height: 18px; }
      </style>
      <script>
async function refreshConnection(){
 const el=document.getElementById('connectionStatus');
 el.textContent='Đang kiểm tra ESP32 tại '+location.origin+'…';
 try{
  const d=JSON.parse(await localApi('/netinfo?nocache='+Date.now()));
  el.textContent='Firmware '+d.fw+' | Mã xe '+d.deviceId+' | Trang đang mở: '+location.origin+' | Wi-Fi nhà: '+(d.sta?d.staSsid+' — IP '+d.ip:'chưa kết nối');
  const a=document.getElementById('homeCarLink');
  a.hidden=!d.sta;a.href='http://'+d.ip+'/?build=196';a.textContent='Mở trang xe qua Wi-Fi nhà: '+d.ip;
  document.getElementById('cameraDriveStatus').textContent='Bật camera và chờ có hình trước khi dùng các nút dưới camera. Có thể lái không hình bằng bảng lái tay bên dưới.';
 }catch(e){el.textContent='Chưa xác nhận được ESP32 tại '+location.origin+': '+e.message+'. Trang đang hiển thị chưa chứng minh xe còn kết nối.';}
}

let localAction=0,localTuneLoaded=false;
async function localApi(path){
  const ctrl=new AbortController();const timer=setTimeout(()=>ctrl.abort(),1800);
  try{const r=await fetch(path,{cache:'no-store',signal:ctrl.signal});if(!r.ok)throw new Error(await r.text());return await r.text();}finally{clearTimeout(timer);}
}
function cancelLocalManual(){return stopDrive()||stopPromise||Promise.resolve();}
async function stopAutonomy(){
  ++localAction;await cancelLocalManual();
  try{await localApi('/estop');if(remoteCamOn)toggleRemoteCam();if(!driveSession)await initDriveSession();document.getElementById('localRequest').textContent='Đã nhận lệnh dừng; hình camera đã tắt.';}
  catch(e){document.getElementById('localRequest').textContent='CHƯA XÁC NHẬN DỪNG: '+e.message+'. Dùng công tắc nguồn nếu cần.';}
}

async function pollSensors(){
  const el=document.getElementById('sensorText');
  try{
    const d=JSON.parse(await localApi('/telemetry'));
    el.textContent=['Trước','Sau','Trái','Phải'].map((name,i)=>name+' '+(d.age[i]<0||d.age[i]>500?'cũ':d.d[i]>0?d.d[i]+' cm':'N/A')).join(' | ')+' · '+d.msg;
  }catch(e){el.textContent='Mất kết nối xe';}
  setTimeout(pollSensors,650);
}
        function setGear(speed, gearName, btnElement) {
          fetch("/speed?val=" + speed);
          document.getElementById("gearVal").innerText = gearName + " (" + speed + ")";
          let buttons = document.getElementsByClassName("btn-gear");
          for (let btn of buttons) btn.classList.remove("active");
          btnElement.classList.add("active");
        }

        let wifiPollTimer=null;
        async function checkWifiStatus(){
          const st=document.getElementById('wifiStatusText');
          try{
            const d=JSON.parse(await localApi('/netinfo'));
            if(!document.getElementById('wifiSsid').value&&d.staSsid)document.getElementById('wifiSsid').value=d.staSsid;
            if(d.sta){
              st.style.color='#2ecc71';st.replaceChildren(document.createTextNode('Đã kết nối '+d.staSsid+'. Mở trên cùng Wi-Fi: '));
              const link=document.createElement('a');link.href='http://'+d.ip+'/';link.textContent=link.href;link.style.color='#00d2ff';st.appendChild(link);
            }else{
              st.style.color=d.wifiJoining?'#ffca28':'#ff4757';
              st.textContent=(d.wifiMessage||'Chưa kết nối')+' (mã '+d.wifiReason+'). ESP32 chỉ dùng Wi-Fi 2.4 GHz.';
            }
            return d;
          }catch(e){st.textContent='Tạm mất liên lạc. Nếu vừa lưu Wi-Fi, nối lại ESP32_CAR rồi mở 192.168.4.1.';return null;}
        }
        async function saveWifi(){
          await cancelLocalManual();await localApi('/estop').catch(()=>{});
          const s=document.getElementById('wifiSsid').value,p=document.getElementById('wifiPass').value;
          if(!s){alert('Nhập tên Wi-Fi 2.4 GHz.');return;}
          const st=document.getElementById('wifiStatusText');st.style.color='#ffca28';
          st.textContent='Đang lưu. Khi đổi kênh Wi-Fi, điện thoại có thể cần nối lại ESP32_CAR.';
          if(wifiPollTimer!==null)clearTimeout(wifiPollTimer);
          try{
            const r=await fetch('/iot',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
              body:'ssid='+encodeURIComponent(s)+'&wpass='+encodeURIComponent(p),cache:'no-store'});
            const text=await r.text();if(!r.ok)throw new Error(text);
          }catch(e){st.textContent='Chưa xác nhận lưu: '+e.message+'. Nối lại ESP32_CAR và bấm Kiểm tra.';return;}
          let count=0;
          const poll=async()=>{
            const d=await checkWifiStatus();++count;
            if(d&&d.sta)return;
            if(count<30&&(d===null||d.wifiJoining))wifiPollTimer=setTimeout(poll,1000);
          };
          wifiPollTimer=setTimeout(poll,1000);
        }

        // ============================================================
// ĐIỀU KHIỂN GIỮ NÚT: CHẠY LIÊN TỤC CHO ĐẾN KHI NHẢ NÚT
// ============================================================

let activeDrivePointerId=null,activeDriveCommand=null,driveHeartbeat=null;
let driveSession='',driveSequence=0,driveGesture=0,driveFlight=null,stopPending=false;
let serverClock=0,clientClock=0,driveEpoch=0,stopPromise=null;
function driveStatus(text){document.getElementById('driveStatus').textContent=text;}
async function initDriveSession(){
  try{
    const d=JSON.parse(await localApi('/drive-session'));
    driveSession=d.sid;serverClock=Number(d.ms);clientClock=performance.now();
    driveSequence=0;driveGesture=0;driveStatus('Sẵn sàng: giữ nút để chạy, nhả nút để dừng.');
    return true;
  }catch(e){driveStatus('Chưa sẵn sàng lái tay: '+e.message);return false;}
}
function driveUrl(cmd,g){
  const at=(serverClock+Math.floor(performance.now()-clientClock))>>>0;
  return '/cmd?val='+cmd+'&sid='+encodeURIComponent(driveSession)+'&n='+(++driveSequence)+'&g='+g+'&at='+at;
}
async function pumpDrive(){
  if(!activeDriveCommand||driveFlight||stopPending||!driveSession)return;
  const flight={ctrl:new AbortController(),epoch:driveEpoch};driveFlight=flight;
  const cmd=activeDriveCommand,started=performance.now();
  const timer=setTimeout(()=>flight.ctrl.abort(),350);
  try{
    const r=await fetch(driveUrl(cmd,driveGesture),{cache:'no-store',signal:flight.ctrl.signal});
    const text=await r.text();if(!r.ok)throw new Error(text);
    const d=JSON.parse(text);
    if(flight.epoch===driveEpoch&&activeDriveCommand){
      if(d.cmd!==cmd)throw new Error('ESP32 đang dừng/chặn lệnh '+cmd);
      driveStatus('Đã nhận '+cmd+' · '+Math.round(performance.now()-started)+' ms');
    }
  }catch(e){
    if(flight.epoch===driveEpoch){stopDrive();driveSession='';driveStatus('Đã ngừng gửi lệnh: mạng trễ/lỗi. Nhả nút và bấm lại. '+e.message);}
  }finally{clearTimeout(timer);if(driveFlight===flight)driveFlight=null;}
}
async function transmitStop(g){
  if(!driveSession)return;
  stopPending=true;
  try{
    for(let attempt=0;attempt<3;attempt++){
      const ctrl=new AbortController(),timer=setTimeout(()=>ctrl.abort(),350);
      try{
        const r=await fetch(driveUrl('S',g),{cache:'no-store',signal:ctrl.signal,keepalive:true});
        const text=await r.text();if(!r.ok)throw new Error(text);
        driveStatus('ESP32 đã xác nhận DỪNG.');return;
      }catch(e){if(attempt===2)driveStatus('Chưa nhận được xác nhận DỪNG. Bộ ngắt mất lệnh 450 ms đang bảo vệ motor.');}
      finally{clearTimeout(timer);}
    }
  }finally{stopPending=false;}
}
function stopDrive(e){
  if(e&&e.pointerId!==undefined&&activeDrivePointerId!==null&&e.pointerId!==activeDrivePointerId)return;
  if(activeDrivePointerId===null&&!activeDriveCommand&&!driveFlight)return;
  const gesture=driveGesture;++driveEpoch;
  activeDrivePointerId=null;activeDriveCommand=null;
  if(driveHeartbeat!==null){clearInterval(driveHeartbeat);driveHeartbeat=null;}
  if(driveFlight){driveFlight.ctrl.abort();driveFlight=null;}
  document.querySelectorAll('.btn').forEach(b=>b.classList.remove('driving'));
  stopPromise=transmitStop(gesture);return stopPromise;
}
function setupButton(id,command){
  const btn=document.getElementById(id);if(!btn)return;
  btn.style.touchAction='none';btn.style.userSelect='none';
  btn.addEventListener('contextmenu',e=>e.preventDefault());
  btn.addEventListener('pointerdown',async e=>{
    if(e.button!==undefined&&e.button!==0)return;
    e.preventDefault();
    if(id.startsWith('cam-')&&(!remoteCamOn||Date.now()-remoteCamAt>2000)){document.getElementById('cameraDriveStatus').textContent='Chưa có hình mới: nhập IP camera, bấm LƯU rồi BẬT CAMERA. Lái không hình bằng bảng phía dưới.';return;}
    if(activeDrivePointerId!==null||stopPending)return;
    if(!driveSession){await initDriveSession();driveStatus('Đã thử tạo lại phiên. Nhả nút rồi bấm lại; nếu xe đang tự hành hãy bấm DỪNG.');return;}
    activeDrivePointerId=e.pointerId;activeDriveCommand=command;++driveGesture;++driveEpoch;
    btn.setPointerCapture(e.pointerId);btn.classList.add('driving');
    driveStatus('Đang gửi '+command+'...');pumpDrive();
    driveHeartbeat=setInterval(pumpDrive,100);
  },{passive:false});
  btn.addEventListener('pointerup',stopDrive);
  btn.addEventListener('pointercancel',stopDrive);
  btn.addEventListener('lostpointercapture',stopDrive);
}
window.addEventListener('pointerup',stopDrive);
window.addEventListener('pointercancel',stopDrive);
window.addEventListener('blur',()=>stopDrive());
window.addEventListener('pagehide',()=>stopDrive());
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopDrive();});
let remoteCamBase='http://192.168.1.51';
function cameraAddress(value){
 const host=value.trim().replace(/^http:\/\//i,'').replace(/\/$/,'');
 if(!/^\d{1,3}(\.\d{1,3}){3}$/.test(host))throw Error('Nhập IP camera, ví dụ 192.168.1.51');
 const n=host.split('.').map(Number);
 if(n.some(x=>x>255)||n[3]===0||n[3]===255||!(n[0]===10||(n[0]===192&&n[1]===168)||(n[0]===172&&n[1]>=16&&n[1]<=31)))throw Error('Cần IP camera trong mạng nội bộ');
 return 'http://'+n.join('.');
}
function saveCameraAddress(){
 try{
   const next=cameraAddress(document.getElementById('remoteCamAddress').value);
   if(remoteCamOn)toggleRemoteCam();
   stopDrive();remoteCamBase=next;
   try{localStorage.setItem('carCameraAddress',next);}catch(e){}
   document.getElementById('remoteCamStatus').textContent='Đã chọn '+next+'. Bấm bật camera.';
 }catch(e){document.getElementById('remoteCamStatus').textContent=e.message;}
}
let remoteCamOn=false,remoteCamAt=0,remoteCamTimer=null,remoteCamGeneration=0;
function toggleRemoteCam(){
 remoteCamOn=!remoteCamOn;++remoteCamGeneration;clearTimeout(remoteCamTimer);stopDrive();
 const im=document.getElementById('remoteCam');im.onload=im.onerror=null;
 remoteCamAt=0;
 if(remoteCamOn)remoteCamNext(remoteCamGeneration);
 else {im.removeAttribute('src');document.getElementById('remoteCamStatus').textContent='Camera đã tắt';}
}
function remoteCamNext(g){
 if(!remoteCamOn||g!==remoteCamGeneration)return;
 const im=document.getElementById('remoteCam'),status=document.getElementById('remoteCamStatus');
 im.onload=()=>{if(g!==remoteCamGeneration)return;remoteCamAt=Date.now();status.textContent='Đang xem — giữ nút bên dưới để lái';clearTimeout(remoteCamTimer);remoteCamTimer=setTimeout(()=>remoteCamNext(g),300);};
 im.onerror=()=>{if(g!==remoteCamGeneration)return;status.textContent='Mất hình — kiểm tra nguồn ESP32-CAM';stopDrive();clearTimeout(remoteCamTimer);remoteCamTimer=setTimeout(()=>remoteCamNext(g),1000);};
 im.src=remoteCamBase+'/jpg?n='+Date.now();
 remoteCamTimer=setTimeout(()=>{if(g!==remoteCamGeneration||!remoteCamOn)return;stopDrive();status.textContent='Camera không phản hồi';remoteCamNext(g);},2500);
}
setInterval(()=>{if(remoteCamOn&&Date.now()-remoteCamAt>2000)stopDrive();},200);
document.addEventListener('visibilitychange',()=>{if(document.hidden&&remoteCamOn)toggleRemoteCam();});
document.addEventListener('DOMContentLoaded',()=>{
  try{remoteCamBase=cameraAddress(localStorage.getItem('carCameraAddress')||remoteCamBase);}catch(e){}
  document.getElementById('remoteCamAddress').value=remoteCamBase; 
  setupButton('btn-forward','F');setupButton('btn-backward','B');
  setupButton('btn-left','L');setupButton('btn-right','R');
  setupButton('cam-forward','F');setupButton('cam-backward','B');setupButton('cam-left','L');setupButton('cam-right','R');
  initDriveSession().finally(pollSensors);checkWifiStatus();refreshConnection();
});
      </script>
    </head>
    <body>
      <h2>ESP32 Smart Car · IoT 4 hướng</h2>
<section style="max-width:560px;margin:auto;padding:12px;background:#202631;border-radius:12px">
<p id="connectionStatus" role="status">Đang xác nhận kết nối xe…</p>
<a id="homeCarLink" hidden style="color:#00d2ff"></a><br>
<button onclick="refreshConnection()">KIỂM TRA KẾT NỐI XE</button>
<p>Nếu đang nối ESP32_CAR: lưu Wi-Fi nhà bên dưới, ghi lại IP xe, sau đó chuyển điện thoại sang Wi-Fi nhà và mở IP đó.</p>
</section>
<section style="max-width:560px;margin:12px auto;padding:12px;background:#202631;border-radius:12px;text-align:center">
<h3>📷 GIÁM SÁT VÀ LÁI XE</h3>
<img id="remoteCam" style="width:100%;max-height:300px;object-fit:contain;background:#111" alt="Bấm bật camera để xem">
<p id="remoteCamStatus">Xe, camera và điện thoại cùng Wi-Fi nhà. Nhập IP camera rồi bật hình.</p>
<label>IP camera <input id="remoteCamAddress" type="text" inputmode="url" placeholder="192.168.1.51" style="max-width:220px"></label>
<button onclick="saveCameraAddress()">LƯU IP CAMERA</button>
<button onclick="toggleRemoteCam()">BẬT / TẮT CAMERA</button>
<button onclick="stopAutonomy()">DỪNG / VỀ LÁI TAY</button>
<p id="cameraDriveStatus" role="status">Bật camera và chờ có hình để lái ở bảng này.</p>
<div style="margin:12px"><button class="btn" id="cam-forward" style="padding:12px;font-size:18px">TIẾN</button><br>
<button class="btn" id="cam-left" style="padding:12px;font-size:18px">TRÁI</button>
<button class="btn" id="cam-right" style="padding:12px;font-size:18px">PHẢI</button><br>
<button class="btn" id="cam-backward" style="padding:12px;font-size:18px">LÙI</button></div>
<p>Giữ nút để chạy, thả để dừng. Mất ảnh quá 2 giây sẽ ngắt lệnh giữ nút. Hoạt động trong cùng mạng Wi-Fi nhà hoặc mạng ESP32_CAR; chưa truy cập qua Internet.</p>
</section>
<p id="localRequest"></p>
      <div class="control-panel">
        <p id="driveStatus" role="status">Đang tạo phiên lái tay...</p>
        <div>
          <button id="btn-forward" class="btn">TIẾN</button><br>
          <button id="btn-left" class="btn">TRÁI</button>
          <button id="btn-right" class="btn">PHẢI</button><br>
          <button id="btn-backward" class="btn">LÙI</button>
        </div>
        <div class="gear-container">
          <label><b>Tốc độ tiến/lùi · Xoay D2 (180)</b></label><br><br>
          <button class="btn-gear active" onclick="setGear(80, 'D1 (Chậm)', this)">D1</button>
          <button class="btn-gear" onclick="setGear(180, 'D2 (Vừa)', this)">D2</button>
          <button class="btn-gear" onclick="setGear(255, 'D3 (Nhanh)', this)">D3</button>
          <span class="gear-text">Đang chạy: <span id="gearVal">D1 (80)</span></span>
        </div>
        <div class="wifi-container">
          <label><b>📡 Cảm biến (4 hướng cố định):</b></label><br>
                    <div id="sensorText" class="wifi-status" style="white-space:pre-wrap">Đang đọc cảm biến</div>
        </div>
        <div class="wifi-container">
          <label><b>📶 Kết Nối Wi-Fi Nhà / Hotspot:</b></label><br>
          <input type="text" id="wifiSsid" class="wifi-input" placeholder="Tên Wi-Fi 2.4 GHz (SSID)" autocomplete="off" autocapitalize="none" spellcheck="false"><br>
          <input type="password" id="wifiPass" class="wifi-input" placeholder="Mật khẩu Wi-Fi"><br>
          <button class="btn-wifi" onclick="saveWifi()">💾 Lưu & Kết Nối Wi-Fi</button>
          <button class="btn-check" onclick="checkWifiStatus()">🔍 Kiểm Tra Trạng Thái Wi-Fi</button>
          <div id="wifiStatusText" class="wifi-status">Wi-Fi 2.4 GHz. Kết nối xong, dùng IP nhà hiển thị bên trên để lái trực tiếp.</div>
        </div>
      </div>
    </body>
    </html>
  )rawliteral";
  server.sendHeader("Cache-Control","no-store, no-cache, must-revalidate");
  server.send(200, "text/html", html);
}

// ============================================================================
// REST API / WEBSERVER
// ============================================================================
bool readDriveNumber(const String &s,uint32_t &value){
  if(!s.length()||s.length()>10)return false;
  uint64_t n=0;
  for(unsigned int i=0;i<s.length();++i){if(s[i]<'0'||s[i]>'9')return false;n=n*10+(s[i]-'0');if(n>UINT32_MAX)return false;}
  value=(uint32_t)n;return true;
}
void handleDriveSession(){
  localHttpSeen=true;lastLocalHttpAt=millis();
  if(!carIdle()){server.send(409,"text/plain","Dừng xe để tạo phiên lái tay");return;}
  httpDriveSession=String(esp_random(),HEX)+String(esp_random(),HEX);
  httpLastSeq=httpLastGesture=httpClosedGesture=0;
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json",String("{\"sid\":\"")+httpDriveSession+"\",\"ms\":"+String(millis())+"}");
}
void handleCommand() {
  localHttpSeen=true;lastLocalHttpAt=millis();
  if (!server.hasArg("val") || server.arg("val").length() != 1) {
    server.send(400, "text/plain", "Lệnh không hợp lệ");return;
  }
  char cmd=server.arg("val")[0];
  if(cmd!='F'&&cmd!='B'&&cmd!='L'&&cmd!='R'&&cmd!='S'){server.send(400,"text/plain","Lệnh không hợp lệ");return;}
  bool ordered=server.hasArg("sid");uint32_t seq=0,gesture=0,stamp=0;
  if(ordered){
    if(server.arg("sid")!=httpDriveSession||!httpDriveSession.length()){
      server.send(409,"text/plain","Phiên lái hết hạn: tải lại trang");return;
    }
    if(!readDriveNumber(server.arg("n"),seq)||!seq||!readDriveNumber(server.arg("g"),gesture)||!gesture||
       (int32_t)(seq-httpLastSeq)<=0){server.send(409,"text/plain","Bỏ lệnh cũ/sai thứ tự");return;}
    httpLastSeq=seq;
    if(cmd!='S'){
      if(!readDriveNumber(server.arg("at"),stamp)||(int32_t)(millis()-stamp)<-100||
         (int32_t)(millis()-stamp)>=(int32_t)MANUAL_LEASE_MS){server.send(408,"text/plain","Lệnh đến quá chậm; nhả nút và thử lại");return;}
      if(gesture<httpLastGesture || gesture<=httpClosedGesture || (manualLeaseTimedOut&&gesture==httpLastGesture)){
        server.send(409,"text/plain","Đã dừng: nhả nút rồi bấm lại");return;
      }
      httpLastGesture=gesture;
    }else{
      if(gesture>httpClosedGesture)httpClosedGesture=gesture;
    }
  }else if(cmd!='S'&&httpDriveSession.length()){
    server.send(409,"text/plain","Mở trang điều khiển mới trên ESP32");return;
  }
  acceptDriveCommand(cmd,1);
  if(ordered&&cmd!='S'&&cmdFromMqtt&&driveOwner==1&&trangThaiHienTai==cmd)renewManualLease(stamp);
  applyManualDrive();
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json",String("{\"cmd\":\"")+String(trangThaiHienTai)+"\",\"seq\":"+String(seq)+"}");
}

void handleEmergencyStop() {
  emergencyStop();
  server.send(200, "text/plain", "DA DUNG KHAN CAP");
}

void handleTelemetry() {
  localHttpSeen=true;lastLocalHttpAt=millis();
  // Dashboard Direct IP sử dụng cùng định dạng JSON với MQTT.
  updateLocalTelemetry();
  server.send(200, "application/json", telemetryJson());
}
void handleSpeed() {
  if (controllerBusy(1)) { server.send(409, "text/plain", "MQTT dang lai xe"); return; }
  if (server.hasArg("val")) {
    currentSpeed = constrain((int)server.arg("val").toInt(), 0, 255);
    lastAppliedSpeed = -1;
  }
  server.send(200, "text/plain", "OK");
}
void handleNetInfo() {
  server.sendHeader("Cache-Control","no-store, no-cache, must-revalidate");
  localHttpSeen=true;lastLocalHttpAt=millis();
  String j = String("{\"deviceId\":\"") + deviceId + "\",\"topicBase\":\"" + topicBase +
             "\",\"staSsid\":\"" + jsonEscape(staSsid) + "\",\"sta\":" + (WiFi.status() == WL_CONNECTED ? "true" : "false") +
             ",\"ip\":\"" + WiFi.localIP().toString() + "\",\"rssi\":" + String(WiFi.RSSI()) +
             ",\"broker\":\"" + mqttHost + ":" + String(mqttPort) + "\",\"mqtt\":" + (mqtt.connected() ? "true" : "false") +
             ",\"mqttState\":" + String(mqtt.state()) + ",\"remote\":" + String(remoteEnabled ? 1 : 0) +
             ",\"autoBrake\":" + String(autoBrake ? 1 : 0) + ",\"laser\":" + String(1) +
             ",\"apSsid\":\"" + String(ssid) + "\",\"apIp\":\"" + WiFi.softAPIP().toString() +
             "\",\"apClients\":" + String(WiFi.softAPgetStationNum()) +
             ",\"wifiChannel\":" + String(WiFi.channel()) +
             ",\"wifiJoining\":" + String((wifiJoining||wifiJoinPending)?1:0) +
             ",\"wifiReason\":" + String(wifiDisconnectReason) +
             ",\"wifiMessage\":\"" + jsonEscape(wifiReasonText()) + "\"" +
             ",\"fw\":\"" + FW_VERSION + "\"}";
  server.send(200, "application/json", j);
}

void handleEvents() {
  String t;
  int start = evLogN > 10 ? evLogN - 10 : 0;
  for (int i = start; i < evLogN; i++) t += evLog[i % 10] + "\n";
  server.send(200, "text/plain", t.length() ? t : "chưa có sự kiện");
}

void handleIot() {
  if(!carIdle()){server.send(409,"text/plain","Dừng xe trước khi đổi Wi-Fi");return;}
  if(server.hasArg("ssid")&&server.arg("ssid").length()>32){server.send(400,"text/plain","SSID quá 32 byte");return;}
  if(server.hasArg("wpass")){
    unsigned int n=server.arg("wpass").length();
    if(n>64||(n>0&&n<8)){server.send(400,"text/plain","Mật khẩu Wi-Fi cần 8-63 ký tự hoặc 64 ký tự hex; để trống cho mạng mở");return;}
  }
  bool wifiChanged = false, mqttChanged = false;
  if (server.hasArg("ssid"))  { staSsid = server.arg("ssid"); prefs.putString("ssid", staSsid); wifiChanged = true; }
  if (server.hasArg("wpass")) { staPass = server.arg("wpass"); prefs.putString("wpass", staPass); wifiChanged = true; }
  if (server.hasArg("broker")){ mqttHost = server.arg("broker"); mqttHost.trim(); prefs.putString("host", mqttHost); mqttChanged = true; }
  if (server.hasArg("port"))  { mqttPort = constrain((int)server.arg("port").toInt(), 1, 65535); prefs.putInt("port", mqttPort); mqttChanged = true; }
  if (server.hasArg("muser")) { mqttUser = server.arg("muser"); prefs.putString("muser", mqttUser); mqttChanged = true; }
  if (server.hasArg("mpass")) { mqttPass = server.arg("mpass"); prefs.putString("mpass", mqttPass); mqttChanged = true; }
  if (server.hasArg("id")) {
    String id = server.arg("id"), clean;
    for (unsigned int i = 0; i < id.length(); i++) { char c = id[i]; if (isalnum(c) || c == '-' || c == '_') clean += c; }
    if (clean.length() > 0) { deviceId = clean; prefs.putString("did", deviceId); topicBase = "pbl3/" + deviceId; mqttChanged = true; }
  }
  if (server.hasArg("remote")) { remoteEnabled = server.arg("remote").toInt() != 0; prefs.putInt("remote", remoteEnabled ? 1 : 0); }
  if (server.hasArg("bcal"))   { batCal = constrain(server.arg("bcal").toFloat(), 0.5f, 2.0f); prefs.putFloat("bcal", batCal); }
  if (wifiChanged) scheduleWifiJoin();
  if (mqttChanged && mqtt.connected()) mqtt.disconnect();
  mqttRetryMs = 5000; lastMqttTry = 0;
  handleNetInfo();
}
void setup() {
  Serial.begin(115200);
  analogSetPinAttenuation(BAT_PIN, ADC_11db);
  analogReadResolution(12);
  pinMode(BAT_PIN, INPUT);

  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);
  pinMode(IN11, OUTPUT); digitalWrite(IN11, LOW);
  pinMode(IN44, OUTPUT); digitalWrite(IN44, LOW);


  for(int i=0;i<4;i++){
    pinMode(trigPins[i],OUTPUT);
    digitalWrite(trigPins[i],LOW);
    pinMode(echoPins[i],INPUT);
  }
  // PWM động cơ: kênh 4,5,6,7 (kênh 0 dành cho servo)
#if IS_ESP32_V3
  ledcAttachChannel(IN2, PWM_FREQ, PWM_RES, CH_IN2);
  ledcAttachChannel(IN22, PWM_FREQ, PWM_RES, CH_IN22);
  ledcAttachChannel(IN3, PWM_FREQ, PWM_RES, CH_IN3);
  ledcAttachChannel(IN33, PWM_FREQ, PWM_RES, CH_IN33);
#else
  ledcSetup(CH_IN2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_IN22, PWM_FREQ, PWM_RES);
  ledcSetup(CH_IN3, PWM_FREQ, PWM_RES);
  ledcSetup(CH_IN33, PWM_FREQ, PWM_RES);

  ledcAttachPin(IN2, CH_IN2);
  ledcAttachPin(IN22, CH_IN22);
  ledcAttachPin(IN3, CH_IN3);
  ledcAttachPin(IN33, CH_IN33);
#endif

  motorMutex=xSemaphoreCreateMutex();
  if(!motorMutex){motor_Phai_Dung();motor_Trai_Dung();while(true)delay(1000);}
  dungXe();
  if(xTaskCreatePinnedToCore(motorWatchdogTask,"motorGuard",3072,nullptr,3,nullptr,1)!=pdPASS){while(true)delay(1000);}
  prefs.begin("park", false);
  loadIotConfig();

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoReconnect(false);
  WiFi.onEvent([](WiFiEvent_t event,WiFiEventInfo_t info){
    if(event==ARDUINO_EVENT_WIFI_STA_DISCONNECTED&&info.wifi_sta_disconnected.reason!=8)wifiDisconnectReason=info.wifi_sta_disconnected.reason;
    if(event==ARDUINO_EVENT_WIFI_STA_GOT_IP)wifiDisconnectReason=0;
  });
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  WiFi.softAP(ssid, password, 6);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  if (staSsid.length() > 0) {
    scheduleWifiJoin();
  }

  CarAudio::begin();
  SHTRoom::begin();
  mqtt.setCallback(onMqtt);
  // Telemetry UTF-8 (loi/ly do dung co dau) co the vuot 1024 byte.
  // Tranh broker bo ban tin dung luc Avoid HALT / servo vua quet.
  mqtt.setBufferSize(4096);
  mqtt.setKeepAlive(15);
  mqtt.setSocketTimeout(2);

#if MQTT_TLS
  netClient.setInsecure();
#endif

  ArduinoOTA.setHostname(("xe-" + deviceId).c_str());
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() { emergencyStop(); });
  ArduinoOTA.begin();
  otaStarted = true;

  server.on("/", handleRoot);
  server.on("/cmd", handleCommand);
  server.on("/drive-session",handleDriveSession);
  server.on("/estop", handleEmergencyStop);
  server.on("/telemetry", handleTelemetry);
  server.on("/speed", handleSpeed);
  server.on("/iot", HTTP_GET, handleIot); // Giữ tương thích với API cũ.
  server.on("/iot", HTTP_POST, handleIot); // Gửi mật khẩu qua body, không đưa vào URL.
  server.on("/netinfo", handleNetInfo);
  server.on("/status", handleNetInfo);
  server.on("/version", [](){server.sendHeader("Cache-Control","no-store");server.send(200,"text/plain",FW_VERSION);});
  server.on("/events", handleEvents);

  server.begin();

  Serial.println("ESP32 IoT - 4 sonar");
}
void loop(){
  server.handleClient();
  sampleSensor();
  applyManualDrive();
  wifiConnectionTick();
  netLoop();
  CarAudio::tick();
  applyManualDrive();
}
