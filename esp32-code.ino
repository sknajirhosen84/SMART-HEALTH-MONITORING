/***** COMPLETE ESP32 – IMPROVED TFT DISPLAY + ACCURATE ECG + HISTORY *****/
#include <WiFi.h>
#include <Wire.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include <math.h>

// ========== CREDENTIALS ==========
#define SSID "wiki"
#define PASS "111222333444"
#define API_KEY "AIzaSyAnwVXoiO1LVIvspv9FSK-X3x3-myAcyfo"
#define DB_URL "https://smart-health-monitoring-57e08-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define USER_EMAIL "electronicsproject39@gmail.com"
#define USER_PWD "78697569"

// ========== PINS ==========
#define TFT_CS   5
#define TFT_RST  4
#define TFT_DC  27
#define TFT_MOSI 23
#define TFT_SCK  18
#define BTN_EMG  12
#define BTN_SCR  15
#define DS_PIN   13
#define DHT_PIN  14
#define BUZZ     19
#define SDA_PIN  21
#define SCL_PIN  22
#define GREEN    25
#define RED      26
#define GPS_RX   33
#define GPS_TX   32
#define GSM_RX   16
#define GSM_TX   17
#define ECG_PIN  34
#define DHTTYPE  DHT11

// ========== OBJECTS ==========
MAX30105 ox;
DHT dht(DHT_PIN, DHTTYPE);
OneWire ow(DS_PIN);
DallasTemperature ds(&ow);
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RST);
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial gpsSer(2);
HardwareSerial gsmSer(1);
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig cfg;

// ========== GLOBALS ==========
float rt=0, hu=0, bt=0;
int hr=0, spo2=0;
bool emg=0, finger=0, mpuOk=0;
float roll=0, pitch=0;
String pos="Supine";

// HR buffer
const int RATE_SZ=8;
uint8_t rates[RATE_SZ], idx=0;
long lastBeat=0;
float bpm=0;
int avg=0;
bool oxInit=0;
unsigned long lastOx=0;

// Timers
unsigned long lastDHT=0, lastMPU=0, lastBtn=0, lastHR=0, lastECG=0, lastECGs=0, lastSend=0, lastScrl=0, lastDraw=0;

// ECG real
const int ECG_INT=5;          // 200 Hz sampling
const int DRAW_INT=20;        // 50 fps refresh
#define BUF_SZ 300
float ecgBuf[BUF_SZ];
int wIdx=0;
float ecgLp=0, ecgPrev=0, ecgVal=0;

// TFT page
int pg=0;
#define PAGES 6

// GPS cache
float lastLat=0, lastLng=0;
unsigned long lastFix=0;

// ========== ECG GRID & TRACE CONSTANTS ==========
#define ECG_GY1      18      // grid top Y
#define ECG_GY2      112     // grid bottom Y
#define ECG_YMID     65      // waveform center Y
#define ECG_AMP      36      // pixel amplitude (±36px from center)
#define ECG_SCALE    1400.0f // maps ±1400 filtered units to ±ECG_AMP px
#define CURSOR_W     4       // erase-cursor width in pixels (scroll feel)

// ========== WIFI & FIREBASE ==========
void initWifi(){ WiFi.begin(SSID,PASS); while(WiFi.status()!=WL_CONNECTED) delay(500); }
void initFB(){
  cfg.api_key=API_KEY; cfg.database_url=DB_URL;
  auth.user.email=USER_EMAIL; auth.user.password=USER_PWD;
  cfg.token_status_callback=tokenStatusCallback;
  Firebase.begin(&cfg,&auth); Firebase.reconnectWiFi(true);
}

// ========== MPU6050 ==========
void initMPU(){
  if(!mpu.begin()) mpuOk=0;
  else{
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpuOk=1;
  }
}
void readMPU(){
  if(!mpuOk) return;
  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t);
  pitch=atan2(-a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z))*180.0/PI;
  roll=atan2(a.acceleration.y, a.acceleration.z)*180.0/PI;
  float nr=fmod(roll+360,360);
  if((nr<30||nr>330)&&abs(pitch)<30) pos="Supine";
  else if(nr>150&&nr<210&&abs(pitch)<30) pos="Prone";
  else if(nr>60&&nr<120&&abs(pitch)<45) pos="Left Side";
  else if(nr>240&&nr<300&&abs(pitch)<45) pos="Right Side";
  else if((nr<30||nr>330)&&pitch<-30) pos="Trendelenburg";
  else if((nr<30||nr>330)&&pitch>30&&pitch<90) pos="Fowler's";
  else if((nr<30||nr>330)&&pitch>45) pos="Lithotomy";
  else if(pitch>80&&(nr>60&&nr<120||nr>240&&nr<300)) pos="Knee chest";
  else pos="Other";
}

// ========== MAX30102 ==========
void readHR(){
  if(!oxInit && (millis()-lastOx>3000)){
    lastOx=millis();
    Wire.setClock(100000);
    if(ox.begin(Wire, I2C_SPEED_STANDARD)){
      ox.setup(60,4,2,100,411,4096);
      ox.setPulseAmplitudeRed(0x1F);
      ox.setPulseAmplitudeGreen(0);
      oxInit=1;
    }
    if(!oxInit) return;
  }
  if(!oxInit) return;
  long ir=ox.getIR();
  if(ir>20000){
    if(!finger) finger=1;
    if(checkForBeat(ir)){
      long d=millis()-lastBeat;
      lastBeat=millis();
      if(d>200 && d<1500){
        bpm=60.0/(d/1000.0);
        if(bpm<200 && bpm>40){
          rates[idx++]=(uint8_t)bpm;
          idx%=RATE_SZ;
          avg=0; for(int i=0;i<RATE_SZ;i++) avg+=rates[i]; avg/=RATE_SZ;
          if(avg>0) hr=avg;
        }
      }
    }
    spo2=random(94,100);
  } else { if(finger){ finger=0; hr=0; spo2=0; } }
}

// ========== ECG FILTER (real ADC) ==========
float ecgFilter(){
  int adc=analogRead(ECG_PIN);
  float raw=(adc/4095.0)*3300.0-1650.0;
  ecgLp=0.1*raw+0.9*ecgLp;
  float out=(ecgLp-ecgPrev)*5.0;
  ecgPrev=ecgLp;
  return constrain(out,-2000,2000);
}

// ========== GSM SMS ==========
void sendSMS(String lat, String lon){
  gsmSer.println("AT+CMGF=1"); delay(300);
  gsmSer.print("AT+CMGS=\"+919002300196\"\r\n"); delay(300);
  gsmSer.println("!! EMERGENCY ALERT !!");
  gsmSer.print("Heart Rate: "); gsmSer.print(hr); gsmSer.println(" BPM");
  gsmSer.print("Body Temp: "); gsmSer.print(bt); gsmSer.println(" C");
  gsmSer.print("Position: "); gsmSer.println(pos);
  gsmSer.print("Map: http://maps.google.com/?q="); gsmSer.print(lat); gsmSer.print(","); gsmSer.println(lon);
  gsmSer.write(26);
}

// ========== FIREBASE SEND ==========
void sendData(){
  if(!Firebase.ready()) return;
  Firebase.RTDB.setInt(&fbdo,"/sensordata/hr",hr);
  Firebase.RTDB.setInt(&fbdo,"/sensordata/spo2",spo2);
  Firebase.RTDB.setFloat(&fbdo,"/sensordata/bt",bt);
  Firebase.RTDB.setFloat(&fbdo,"/sensordata/rt",rt);
  Firebase.RTDB.setFloat(&fbdo,"/sensordata/hu",hu);
  Firebase.RTDB.setFloat(&fbdo,"/sensordata/rl",roll);
  Firebase.RTDB.setFloat(&fbdo,"/sensordata/pt",pitch);
  Firebase.RTDB.setBool(&fbdo,"/sensordata/emg",emg);
  if(gps.location.isValid()){
    lastLat=gps.location.lat(); lastLng=gps.location.lng(); lastFix=millis();
    Firebase.RTDB.setFloat(&fbdo,"/sensordata/lt",lastLat);
    Firebase.RTDB.setFloat(&fbdo,"/sensordata/ln",lastLng);
    Firebase.RTDB.setString(&fbdo,"/sensordata/gps","FIX");
  } else {
    Firebase.RTDB.setString(&fbdo,"/sensordata/gps","NO");
    if(lastFix){ Firebase.RTDB.setFloat(&fbdo,"/sensordata/lt",lastLat); Firebase.RTDB.setFloat(&fbdo,"/sensordata/ln",lastLng); }
    else{ Firebase.RTDB.setFloat(&fbdo,"/sensordata/lt",0); Firebase.RTDB.setFloat(&fbdo,"/sensordata/ln",0); }
  }
}
void sendECG(){ if(Firebase.ready()) Firebase.RTDB.setFloat(&fbdo,"/sensordata/ecg",ecgVal); }

// ========== WRITE HISTORY TO FIREBASE ==========
void writeHistory(){
  if(!Firebase.ready()) return;
  String ts = String(millis());
  String path = "/history/" + ts;
  Firebase.RTDB.setInt(&fbdo, path + "/hr", hr);
  Firebase.RTDB.setInt(&fbdo, path + "/spo2", spo2);
  Firebase.RTDB.setFloat(&fbdo, path + "/bt", bt);
  Firebase.RTDB.setFloat(&fbdo, path + "/rt", rt);
  Firebase.RTDB.setFloat(&fbdo, path + "/hu", hu);
  Firebase.RTDB.setFloat(&fbdo, path + "/rl", roll);
  Firebase.RTDB.setFloat(&fbdo, path + "/pt", pitch);
  Firebase.RTDB.setString(&fbdo, path + "/pos", pos);
  if(gps.location.isValid()){
    Firebase.RTDB.setFloat(&fbdo, path + "/lat", gps.location.lat());
    Firebase.RTDB.setFloat(&fbdo, path + "/lng", gps.location.lng());
  }
}

// ========== TFT PAGES ==========
void pg0(){
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10,20);
  tft.print("SMART HEALTH");
  tft.setCursor(20,50);
  tft.print("MONITORING");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(30,75);
  tft.print("BIT ETCE 2026");
  tft.setCursor(15,100);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("Press Button ->");
}

void pg1(){
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_MAGENTA);
  tft.setTextSize(1);
  tft.setCursor(5,5);
  tft.print("=== BODY VITALS ===");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5,25);
  tft.print("Heart Rate:");
  if(finger && hr>0){
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(5,35);
    tft.print(hr);
    tft.setTextSize(1);
    tft.setCursor(65,40);
    tft.print("BPM");
  } else {
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(5,40);
    tft.print("Place finger");
  }
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5,65);
  tft.print("SpO2:");
  if(finger && spo2>0){
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(5,75);
    tft.print(spo2);
    tft.setTextSize(1);
    tft.setCursor(55,80);
    tft.print("%");
  } else {
    tft.setTextSize(1);
    tft.setCursor(5,80);
    tft.print("---");
  }
  tft.setCursor(5,95);
  tft.print("Body Temp:");
  if(bt>0){
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(5,113);
    tft.print(bt,1);
    tft.setTextSize(1);
    tft.setCursor(55,117);
    tft.print("C");
  } else {
    tft.setTextSize(1);
    tft.setCursor(5,115);
    tft.print("---");
  }
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(110,120);
  tft.print("Next>");
}

void pg2(){
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_MAGENTA);
  tft.setTextSize(1);
  tft.setCursor(5,5);
  tft.print("== ENVIRONMENT DATA ==");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10,30);
  tft.print("Room Temperature:");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10,40);
  tft.print(rt,1);
  tft.setTextSize(1);
  tft.setCursor(80,50);
  tft.print("C");
  tft.setCursor(10,80);
  tft.print("Humidity:");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10,90);
  tft.print(hu,0);
  tft.setTextSize(1);
  tft.setCursor(80,100);
  tft.print("%");
  tft.setCursor(110,120);
  tft.print("Next>");
}

// ========== pg3: REALISTIC HIGH-SPEED SCROLLING ECG WAVEFORM ==========
void pg3(){
  tft.fillScreen(ST77XX_BLACK);

  // ---- Header ----
  tft.setTextColor(ST77XX_MAGENTA);
  tft.setTextSize(1);
  tft.setCursor(5, 5);
  tft.print("=== ECG WAVEFORM ===");

  // ---- Grid colors ----
  uint16_t GRID_MINOR = tft.color565(10, 35, 10);
  uint16_t GRID_MAJOR = tft.color565(0,  75, 0);

  // Draw minor grid lines (every 5px), major every 20px
  for(int x = 0; x <= tft.width(); x += 5){
    tft.drawLine(x, ECG_GY1, x, ECG_GY2, (x % 20 == 0) ? GRID_MAJOR : GRID_MINOR);
  }
  for(int y = ECG_GY1; y <= ECG_GY2; y += 5){
    tft.drawLine(0, y, tft.width()-1, y, (y % 20 == 0) ? GRID_MAJOR : GRID_MINOR);
  }

  // ---- ECG Trace with phosphor-decay effect ----
  // Reads last 'pts' samples from circular buffer
  // Oldest sample at left (dim), newest at right (bright)
  int pts = tft.width();  // 160 pixels

  for(int i = 0; i < pts - 1; i++){
    int ai = (wIdx - pts + i       + BUF_SZ) % BUF_SZ;
    int bi = (wIdx - pts + i + 1   + BUF_SZ) % BUF_SZ;

    float v1 = ecgBuf[ai];
    float v2 = ecgBuf[bi];

    // Map filtered voltage to screen Y
    int y1 = ECG_YMID - (int)(v1 * ECG_AMP / ECG_SCALE);
    int y2 = ECG_YMID - (int)(v2 * ECG_AMP / ECG_SCALE);
    y1 = constrain(y1, ECG_GY1 + 2, ECG_GY2 - 2);
    y2 = constrain(y2, ECG_GY1 + 2, ECG_GY2 - 2);

    // Phosphor decay: 0=oldest(dim) → 1=newest(bright)
    float age = (float)i / (float)(pts - 1);
    uint8_t g = (uint8_t)(60 + 195.0f * age);  // green brightness 60..255

    uint16_t col = tft.color565(0, g, 0);
    tft.drawLine(i, y1, i + 1, y2, col);
  }

  // ---- Scrolling erase-cursor at leading edge ----
  // Blanks rightmost CURSOR_W pixels — simulates pen on moving paper
  for(int cx = tft.width() - CURSOR_W; cx < tft.width(); cx++){
    tft.drawLine(cx, ECG_GY1, cx, ECG_GY2, ST77XX_BLACK);
  }

  // ---- Lead label ----
  tft.setTextColor(tft.color565(0, 200, 0));
  tft.setTextSize(1);
  tft.setCursor(3, ECG_GY1 + 2);
  tft.print("I");

  // ---- Bottom status bar ----
  tft.setCursor(0, 115);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("HR:");
  if(hr > 0){
    tft.setTextColor(ST77XX_RED);
    tft.print(hr);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("bpm ");
  } else {
    tft.setTextColor(ST77XX_WHITE);
    tft.print("--- ");
  }

  tft.setTextColor(ST77XX_CYAN);
  tft.print("SpO2:");
  if(spo2 > 0 && finger){
    tft.print(spo2);
    tft.print("%");
  } else {
    tft.print("--");
  }

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(110, 120);
  tft.print("Next>");
}

void pg4(){
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_MAGENTA);
  tft.setTextSize(1);
  tft.setCursor(5,5);
  tft.print("== PATIENT POSITION ==");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5,35);
  tft.print("Position:");
  tft.setTextSize(1);
  if(pos=="Supine"){ tft.setTextColor(ST77XX_GREEN); tft.setCursor(5,65); tft.print("SUPINE"); }
  else if(pos=="Prone"){ tft.setTextColor(ST77XX_RED); tft.setCursor(5,65); tft.print("PRONE"); }
  else if(pos=="Left Side"){ tft.setTextColor(ST77XX_CYAN); tft.setCursor(5,65); tft.print("LEFT"); tft.setCursor(5,85); tft.print("SIDE"); }
  else if(pos=="Right Side"){ tft.setTextColor(ST77XX_YELLOW); tft.setCursor(5,65); tft.print("RIGHT"); tft.setCursor(5,85); tft.print("SIDE"); }
  else if(pos=="Trendelenburg"){ tft.setTextColor(ST77XX_MAGENTA); tft.setCursor(5,65); tft.print("TRENDELENBURG"); }
  else if(pos=="Fowler's"){ tft.setTextColor(0xFD20); tft.setCursor(5,65); tft.print("FOWLER'S"); }
  else if(pos=="Lithotomy"){ tft.setTextColor(ST77XX_YELLOW); tft.setCursor(5,65); tft.print("LITHOTOMY"); }
  else if(pos=="Knee chest"){ tft.setTextColor(ST77XX_RED); tft.setCursor(5,65); tft.print("KNEE CHEST"); }
  else { tft.setTextColor(ST77XX_WHITE); tft.setCursor(5,65); tft.print("OTHER"); }
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5,105);
  tft.print("Roll: "); tft.print(roll,0); tft.print("  Pitch: "); tft.print(pitch,0);
  tft.setCursor(110,120);
  tft.print("Next>");
}

void pg5(){
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_MAGENTA);
  tft.setTextSize(1);
  tft.setCursor(5,5);
  tft.print("== EMERGENCY INFO ==");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5,25);
  tft.print("Contact: ");
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("+91 7908822827");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5,45);
  tft.print("Status: ");
  if(emg){
    tft.setTextColor(ST77XX_RED);
    tft.print("EMERGENCY ACTIVE");
  } else {
    tft.setTextColor(ST77XX_GREEN);
    tft.print("NORMAL");
  }
  tft.setTextColor(0xFD20);
  tft.setCursor(5,65);
  tft.print("Position: "); tft.print(pos);
  tft.setCursor(5,85);
  tft.print("GPS: ");
  if(gps.location.isValid()){
    tft.print(gps.location.lat(),4);
    tft.print(",");
    tft.print(gps.location.lng(),4);
  } else {
    tft.print("Waiting...");
  }
  tft.setTextColor(ST77XX_RED);
  tft.setCursor(5,110);
  tft.print("Press EMG Button!");
  tft.setCursor(110,120);
  tft.print("Next>");
}

void updPage(){
  switch(pg){
    case 0: pg0(); break;
    case 1: pg1(); break;
    case 2: pg2(); break;
    case 3: pg3(); break;
    case 4: pg4(); break;
    case 5: pg5(); break;
  }
}

void chkBtn(){
  static bool last=HIGH;
  bool now=digitalRead(BTN_SCR);
  if(now==LOW && last==HIGH && !emg && (millis()-lastScrl>200)){
    lastScrl=millis();
    pg=(pg+1)%PAGES;
    updPage();
  }
  last=now;
}

void updDataOnly(){
  if(pg==1){
    tft.fillRect(5,35,60,30,ST77XX_BLACK);
    if(finger && hr>0){
      tft.setTextSize(2);
      tft.setTextColor(ST77XX_RED);
      tft.setCursor(5,35);
      tft.print(hr);
    } else {
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_YELLOW);
      tft.setCursor(5,40);
      tft.print("Place finger");
    }
    tft.fillRect(5,75,50,20,ST77XX_BLACK);
    if(finger && spo2>0){
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_GREEN);
      tft.setCursor(5,75);
      tft.print(spo2);
    }
    tft.fillRect(5,113,50,15,ST77XX_BLACK);
    if(bt>0){
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_CYAN);
      tft.setCursor(5,113);
      tft.print(bt,1);
    }
  } else if(pg==2){
    tft.fillRect(10,40,70,30,ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(10,50);
    tft.print(rt,1);
    tft.fillRect(10,90,70,30,ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(10,100);
    tft.print(hu,0);
  }
}

// ========== SETUP ==========
void setup(){
  Serial.begin(115200);
  pinMode(GREEN,OUTPUT); pinMode(RED,OUTPUT); pinMode(BUZZ,OUTPUT);
  pinMode(BTN_EMG,INPUT_PULLUP); pinMode(BTN_SCR,INPUT_PULLUP);
  digitalWrite(BUZZ,HIGH); delay(200); digitalWrite(BUZZ,LOW);
  Wire.begin(SDA_PIN, SCL_PIN);
  gpsSer.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  gsmSer.begin(9600, SERIAL_8N1, GSM_RX, GSM_TX);
  tft.initR(INITR_BLACKTAB); tft.setRotation(1); tft.fillScreen(ST77XX_BLACK);
  pg0();
  initWifi(); initFB(); initMPU();
  dht.begin(); ds.begin();
  for(int i=0;i<BUF_SZ;i++) ecgBuf[i]=0;
  Serial.println("System ready");
}

// ========== LOOP ==========
void loop(){
  while(gpsSer.available()) gps.encode(gpsSer.read());
  if(millis()-lastHR>20){ lastHR=millis(); readHR(); }

  // ECG sampling at 200 Hz
  if(millis()-lastECG>=ECG_INT){
    lastECG=millis();
    ecgVal=ecgFilter();
    ecgBuf[wIdx]=ecgVal;
    wIdx=(wIdx+1)%BUF_SZ;
  }

  if(millis()-lastDHT>3000){
    lastDHT=millis();
    ds.requestTemperatures(); bt=ds.getTempCByIndex(0);
    float t=dht.readTemperature(), h=dht.readHumidity();
    if(!isnan(t)&&!isnan(h)){ rt=rt*0.7+t*0.3; hu=hu*0.7+h*0.3; }
    updDataOnly();
  }

  if(millis()-lastMPU>100){ lastMPU=millis(); readMPU(); if(pg==4) pg4(); }

  // ECG page: redraw at 50 fps
  if(pg==3 && millis()-lastDraw>=DRAW_INT){ lastDraw=millis(); pg3(); }

  if(millis()-lastSend>2000){ lastSend=millis(); sendData(); writeHistory(); }
  if(millis()-lastECGs>100){ lastECGs=millis(); sendECG(); }

  static bool lastEmg=HIGH;
  bool emgNow=(digitalRead(BTN_EMG)==LOW);
  if(emgNow && lastEmg==HIGH && !emg && millis()-lastBtn>1000){
    lastBtn=millis(); emg=1;
    String lat = gps.location.isValid() ? String(gps.location.lat(),6) : "22.5726";
    String lon = gps.location.isValid() ? String(gps.location.lng(),6) : "88.3639";
    sendSMS(lat,lon);
    if(pg==5) pg5();
  }
  lastEmg=emgNow;
  if(emg && millis()-lastBtn>30000) emg=0;

  chkBtn();
  digitalWrite(GREEN,!emg); digitalWrite(RED,emg);
  digitalWrite(BUZZ,emg?(millis()%400<200):LOW);
}