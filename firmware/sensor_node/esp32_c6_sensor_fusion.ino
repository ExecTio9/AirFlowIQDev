#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MFRC522.h>
#include <RH_RF95.h>
#include <esp32-hal-rgb-led.h>
#include <driver/usb_serial_jtag.h>
#include <esp_sleep.h>
extern "C" {
  #include "esp_wifi.h"
}
// ---------- Feature flags ----------
#define USE_BME 1   // << set to 1 if you want BME active (uses original software SPI)
// ---------- Pins ----------
#define BATTERY_PIN       0
#define DIAG_PIN          2
#define WIND_PIN          3
#define SCK               4
#define MOSI              5
#define REGULATOR_EN_PIN  7
#define RGB_BUILTIN       8
#define MISO              15
#define BME_CS            23
#define RFID_CS           22
#define LORA_CS           21
#define LORA_RST          20
#define RFID_RST          18

#define us_S              1000000ULL
#define sleep_Time        60
#define TIMEOUT_MS        30000
#define RFID_TIMEOUT_MS   15000   // 15 s; set to 0 to wait forever

// ---------- Objects ----------
#if USE_BME
Adafruit_BME280 bme(BME_CS, MOSI, MISO, SCK);   // original software-SPI form
#endif
MFRC522 rfid(RFID_CS, RFID_RST);
RH_RF95  radio(LORA_CS);
RTC_DATA_ATTR int bootCount = 0;

// ---------- State ----------
String ID = "blowerRead";
String Web_App_Path = "/macros/s/AKfycby0Y0FUDXrrgpBRS0fGOVgzpTs0XwQK9daSMbiqvNdLIBRZrIXhGrmlCXVB0VWVd-vP/exec";
String DataURL, Status_Read_Sensor, rfidUID;
float Temp = -1, Humd = -1, Prs = -1, Vref = 3.3, dividerGain = 2.0, kMvPerRPM = .9, kVPerRPM = kMvPerRPM/1000;
float lambda_TSR = 2.5, D = .1, ema_rpm = 0, alpha_ema = 0.2, RPM_TO_MPS = .002094;
float batteryVoltage=0, batteryPercent=0, windSpeed=0;
bool isEncodedHost=false, tryEncodedHostFallback=true;
int ADCMax = 4095, WIND_OVERSAMPLES = 16;

inline void deselectAllSPI(){ digitalWrite(BME_CS,HIGH); digitalWrite(RFID_CS,HIGH); digitalWrite(LORA_CS,HIGH); }
inline float pctFromBatt(float v){ v=constrain(v,2.0,3.0); return (v-2.0f)*100.0f; }

String urlEncode(const String& s){
  String out; const char* H="0123456789ABCDEF";
  for (uint16_t i=0;i<s.length();++i){ unsigned char c=s[i];
    if (isalnum(c)||c=='-'||c=='_'||c=='.'||c=='~') out+=char(c);
    else { out+='%'; out+=H[(c>>4)&0xF]; out+=H[c&0xF]; } }
  return out;
}

void buildURL(){
  String qp="?sts=write";
  qp+="&id="+ID;
  qp+="&bc="+String(bootCount);
  qp+="&bat="+String(batteryPercent,2);
  qp+="&srs="+Status_Read_Sensor;
  qp+="&temp="+String(Temp);
  qp+="&humd="+String(Humd);
  qp+="&Prs="+String(Prs);
  qp+="&wind="+String(windSpeed,2);
  qp+="&rfid="+rfidUID;

  if (isEncodedHost) DataURL = "http://192.168.4.1/data?url=" + urlEncode(Web_App_Path + qp);
  else               DataURL = "https://script.google.com" + Web_App_Path + qp;
}

// ===== RFID wait loop (re-init each pass; detects stationary tags) =====
bool waitForRFID_WithTimeout(String &uidHex, uint32_t timeout_ms) {
  uidHex = "";
  Serial.println(F("📡 Waiting for RFID card..."));
  byte ver = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.print(F("🔧 VersionReg: 0x")); Serial.println(ver, HEX);

  const unsigned long t0 = millis();
  while (uidHex.isEmpty()) {
    deselectAllSPI();
    rfid.PCD_StopCrypto1();
    rfid.PCD_Init();
    rfid.PCD_AntennaOn();
    rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
    delay(5);

    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      for (byte i = 0; i < rfid.uid.size; i++) {
        if (rfid.uid.uidByte[i] < 0x10) uidHex += "0";
        uidHex += String(rfid.uid.uidByte[i], HEX);
      }
      uidHex.toUpperCase();
      Serial.print(F("📟 UID HEX: ")); Serial.println(uidHex);
      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
      return true;                     // got one
    }

    if (timeout_ms && (millis() - t0 >= timeout_ms)) {
      Serial.println(F("⏱️ RFID timeout — continuing without card."));
      rfid.PCD_StopCrypto1();
      return false;
    }
    delay(60);
  }
  return true;
}

// ---------- Sensors ----------
void readSensors() {
  if (!waitForRFID_WithTimeout(rfidUID, RFID_TIMEOUT_MS)) {
    rfidUID = "";
  }

#if USE_BME
  deselectAllSPI();
  digitalWrite(BME_CS, LOW);
  Temp = bme.readTemperature();
  Humd = bme.readHumidity();
  Prs  = bme.readPressure() / 100.0F;
  digitalWrite(BME_CS, HIGH);
  Status_Read_Sensor = (isnan(Temp)||isnan(Humd)||isnan(Prs)) ? "Failed" : "Success";
#else
  Status_Read_Sensor = "Bypass";
  Temp = Humd = Prs = -1;
#endif

// Wind (measured via generator voltage -> RPM -> m/s)
{
  float rpm_now = 0.0f, volts_now = 0.0f; uint16_t raw_now = 0;
  windSpeed = readWindSpeed_mps(rpm_now, volts_now, raw_now);

  // Helpful debug
  Serial.printf("rpm: %.2f rpm\n", rpm_now);
  Serial.printf("Wind speed: %.3f m/s\n", windSpeed);
  Serial.printf("V: %.3f V\n", volts_now);
  Serial.printf("raw: %u\n", raw_now);
}

}

float readWindSpeed_mps(float &outRPM, float &outVolts, uint16_t &outRaw) {
  // Oversample ADC
  uint32_t acc = 0;
  for (int i = 0; i < WIND_OVERSAMPLES; ++i) {
    acc += analogRead(WIND_PIN);
    delayMicroseconds(150); // tiny spacing to decorrelate noise
  }
  float adc = acc / float(WIND_OVERSAMPLES);
  outRaw    = uint16_t(adc);

  // Convert to terminal voltage (undo divider)
  outVolts  = (adc / ADCMax) * Vref * dividerGain;

  // Voltage -> RPM using motor constant
  float rpm = (kVPerRPM > 0.0f) ? (outVolts / kVPerRPM) : 0.0f;

  // EMA smoothing
  ema_rpm   = alpha_ema * rpm + (1.0f - alpha_ema) * ema_rpm;
  outRPM    = ema_rpm;

  // RPM -> wind speed
  if (RPM_TO_MPS > 0.0f) {
    return RPM_TO_MPS * ema_rpm;  // calibrated relationship (recommended)
  } else {
    // TSR fallback: v = tip_speed / lambda = (pi*D*rpm)/60 / lambda
    return (PI * D * ema_rpm) / (60.0f * lambda_TSR);
  }
}

// ---------- HTTP ----------
bool HTTP_send(){
  const int tries=3;
  for(int a=1;a<=tries;++a){
    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.setTimeout(20000);
    Serial.printf("🌐 Attempt %d: %s\n", a, DataURL.c_str());
    http.begin(DataURL);
    int code=http.GET();
    if(code>0){
      Serial.printf("📡 HTTP %d\n", code);
      String payload=http.getString();
      Serial.print("📄 "); Serial.println(payload);
      http.end();
      if(code==HTTP_CODE_OK && payload.indexOf("OK")>=0) return true;
      Serial.println("⚠️ Server responded but not as expected.");
    } else {
      Serial.printf("❌ HTTP error: %s\n", http.errorToString(code).c_str());
      http.end();
    }
    delay(300);
  }
  return false;
}

// ---------- LoRa ----------
bool LoRa_sendAndVerify(){
  digitalWrite(LORA_RST, LOW); delay(10);
  digitalWrite(LORA_RST, HIGH); delay(10);
  if(!radio.init()) return false;
  radio.setFrequency(915.0);
  radio.setTxPower(2,false);
  radio.send((uint8_t*)DataURL.c_str(), DataURL.length());
  radio.waitPacketSent();
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; uint8_t len=sizeof(buf);
  if(radio.waitAvailableTimeout(1000) && radio.recv(buf,&len)) return true;
  return false;
}

void prepareRegulatorPin(){
  pinMode(REGULATOR_EN_PIN, OUTPUT);
  digitalWrite(REGULATOR_EN_PIN, HIGH);
  gpio_hold_dis((gpio_num_t)REGULATOR_EN_PIN);
  gpio_hold_en((gpio_num_t)REGULATOR_EN_PIN);
}

// ---------- NEW: Wi-Fi RF tuning applied before any connect ----------
static void applyWiFiRFHints() {
  // Disable power save
  WiFi.setSleep(false);

  // Max TX power (units: 0.25 dBm, so 78 = 19.5 dBm)
  esp_wifi_set_max_tx_power(60);

  // Force 20 MHz bandwidth
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);

  // Set regulatory domain/country (optional but useful)
  wifi_country_t country = {
    .cc = "US",        // country code
    .schan = 1,        // start channel
    .nchan = 11,       // number of channels
    .max_tx_power = 78,// again in 0.25 dBm
    .policy = WIFI_COUNTRY_POLICY_MANUAL
  };
  esp_wifi_set_country(&country);

  // If you want legacy rates enabled:
  // esp_wifi_set_protocol(WIFI_IF_STA,
  //   WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
}


// ---------- WiFi (saved creds or portal on DIAG) ----------
bool connectWiFiWithInputs(bool forcePortal=false){
  WiFi.mode(WIFI_STA);
  applyWiFiRFHints();                 // <<—— apply RF tweaks up front

  if (!forcePortal) {
    WiFi.begin();
    Serial.println("📶 Attempting WiFi connection...");
    unsigned long t0=millis();
    while(WiFi.status()!=WL_CONNECTED && millis()-t0<8000){ delay(250); Serial.print("."); }
    Serial.println();
    if (WiFi.status()==WL_CONNECTED) { Serial.println("✅ Connected to saved WiFi"); return true; }
  }

  // If we fall through, use WiFiManager AP portal (RF hints still help STA join later)
  WiFiManager wm;
  String apName = "FiltSure-Setup-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  wm.setConfigPortalTimeout(180);
  bool ok = wm.autoConnect(apName.c_str());
  if (ok) { Serial.println("✅ Connected via WiFiManager"); return true; }
  Serial.println("❌ WiFiManager timeout/failed — continuing without WiFi");
  return false;
}

// ========================= setup / loop =========================
void setup(){
  Serial.begin(115200);
  analogReadResolution(12);
  delay(200);
  ++bootCount;
  Serial.printf("🔄 Boot #%d\n", bootCount);

  // CS defaults HIGH
  pinMode(BME_CS,  OUTPUT); digitalWrite(BME_CS,  HIGH);
  pinMode(RFID_CS, OUTPUT); digitalWrite(RFID_CS, HIGH);
  pinMode(LORA_CS, OUTPUT); digitalWrite(LORA_CS, HIGH);
  pinMode(LORA_RST, OUTPUT); digitalWrite(LORA_RST, HIGH);
  pinMode(DIAG_PIN, INPUT_PULLUP);

  // Keep regulator during sleep
  pinMode(REGULATOR_EN_PIN, OUTPUT); digitalWrite(REGULATOR_EN_PIN, HIGH);
  gpio_hold_dis((gpio_num_t)REGULATOR_EN_PIN);
  gpio_hold_en((gpio_num_t)REGULATOR_EN_PIN);

  // SPI bus (MFRC522 is sensitive—keep 4 MHz)
  SPI.begin(SCK, MISO, MOSI);
  SPI.setFrequency(4000000);

#if USE_BME
  bool bmeOK = bme.begin(BME_CS);
  Serial.println(bmeOK ? "✅ BME280 ready" : "❌ BME280 init failed");
#else
  Serial.println("⏭️ BME280 disabled (USE_BME=0)");
#endif

  // MFRC522 init
  rfid.PCD_Init();
  rfid.PCD_AntennaOn();
  rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
  Serial.println(F("✅ MFRC522 initialised."));

  // WiFi (saved → portal if DIAG held)
  bool forcePortal = (digitalRead(DIAG_PIN)==LOW);
  if (forcePortal) { delay(2000); forcePortal = (digitalRead(DIAG_PIN)==LOW); }
  bool wifiOK = connectWiFiWithInputs(forcePortal);

  if (wifiOK) {
    String ssid = WiFi.SSID();
    isEncodedHost = ssid.startsWith("ESP32_HOST") || ssid.startsWith("FiltSure");
  } else {
    isEncodedHost = true;
  }

  // Battery & wind (pre-read)
  batteryVoltage = analogRead(BATTERY_PIN) * (3.0 / 4095.0) * 2.0;
  batteryPercent = pctFromBatt(batteryVoltage);
  windSpeed      = analogRead(WIND_PIN) * (3.0 / 4095.0);

  // ---- Block until tag, then finish cycle ----
  readSensors();
  buildURL();

  // Uploads
  unsigned long startMillis = millis();
  bool uploadSuccess = false;
  if (wifiOK) {
    uploadSuccess = HTTP_send();
    if(!uploadSuccess && tryEncodedHostFallback && millis()-startMillis<TIMEOUT_MS){
      Serial.println("⚠️ Sheets failed, trying encoded host...");
      isEncodedHost = true; buildURL();
      uploadSuccess = HTTP_send();
    }
  }
  if(!uploadSuccess && millis()-startMillis<TIMEOUT_MS){
    Serial.println("📡 Trying LoRa fallback...");
    uploadSuccess = LoRa_sendAndVerify();
    Serial.println(uploadSuccess ? "✅ LoRa upload success" : "❌ LoRa upload failed");
  } else if(!uploadSuccess){
    Serial.println("🚫 Upload attempts exceeded timeout — aborting.");
  }

  // Sleep
  esp_sleep_enable_timer_wakeup(sleep_Time * us_S);
  prepareRegulatorPin();
  Serial.println("💤 Going to deep sleep...");
  Serial.flush();
  esp_deep_sleep_start();
}

void loop(){}
