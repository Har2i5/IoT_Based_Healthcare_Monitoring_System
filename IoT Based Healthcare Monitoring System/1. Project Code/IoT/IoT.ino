
#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "time.h"
// One wire and the Dallas Temperature Library
#include <OneWire.h>
#include <DallasTemperature.h>

// LoadCells and liquidCrystal display Library
#include <LiquidCrystal_I2C.h>
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// set LCD address, number of columns and rows
LiquidCrystal_I2C lcd2(0x27, 20, 4); 

//pins:
const int HX711_dout = 16; //mcu > HX711 dout pin
const int HX711_sck = 4; //mcu > HX711 sck pin

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_calVal_eepromAdress = 0;
unsigned long t = 0;

const int buttonPin = 18;
float weight_kg;
float height_m;
float m_height;
float bmi;

const int trigPin = 25;
const int echoPin = 32;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034

long startTime, endTime, duration;

unsigned long previousMillis = 0;
const long interval = 1000; // 1 second interval

#include <DFRobot_MAX30102.h>

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID ""
#define WIFI_PASSWORD ""


// Insert Firebase project API Key
#define API_KEY ""

// Insert Authorized Email and Corresponding Password
#define USER_EMAIL ""
#define USER_PASSWORD ""

// Insert RTDB URLefine the RTDB URL
#define DATABASE_URL ""

// Define Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variable to save USER UID
String uid;

// Database main path (to be updated in setup with the user UID)
String databasePath;
// Database child nodes
String tempPath = "/temperature";
String heartPath = "/heartRate";
String oxygenPath = "/oxygen";
String weightPath = "/weight";
String heightPath = "/height";
String bmiPath = "/bmi";
String timePath = "/timestamp";

// Parent Node (to be updated in every loop)
String parentPath;

int timestamp;
FirebaseJson json;

const char* ntpServer = "pool.ntp.org";

// Temperature sensor
// GPIO where the DS18B20 is connected to
const int oneWireBus = 26;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
float temperatureC;

// float HeartRate;
DFRobot_MAX30102 particleSensor;

// Timer variables (send new readings every three minutes)
unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 5000;
// unsigned long timerDelay = 180000;
int32_t SPO2; //SPO2
int8_t SPO2Valid; //Flag to display if SPO2 calculation is valid
int32_t heartRate; //Heart-rate
int8_t heartRateValid; //Flag to display if heart-rate calculation is valid 


// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  Serial.println();
}

// Function that gets current epoch time
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}
struct Button{
  const uint8_t PIN;
  bool pressed;
};

Button button1 = {19,false};

unsigned long button_time = 0;
unsigned long last_button_time = 0;

void IRAM_ATTR isr() {
  // Get smoothed value from the dataset:
  button_time = millis();
  if (button_time - last_button_time > 1000)
  {
    button1.pressed = true;
    last_button_time = button_time;
  }
}

void setup(){
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  // pinMode(buttonPin, INPUT);
  // Set up the interrupt on the button pin
  pinMode(button1.PIN, INPUT_PULLUP);
  attachInterrupt(button1.PIN,isr, FALLING);
   // initialize LCD
  lcd2.init();
 // turn on LCD backlight 
  lcd2.backlight();
  Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Starting...");

  float calibrationValue; // calibration value
  calibrationValue = 21.69; // uncomment this if you want to set this value in the sketch
#if defined(ESP8266) || defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch this value from eeprom
#endif
  //EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch this value from eeprom

  LoadCell.begin();
  //LoadCell.setReverseOutput();
  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration factor (float)
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  Serial.print("Calibration value: ");
  Serial.println(LoadCell.getCalFactor());
  Serial.print("HX711 measured conversion time ms: ");
  Serial.println(LoadCell.getConversionTime());
  Serial.print("HX711 measured sampling rate HZ: ");
  Serial.println(LoadCell.getSPS());
  Serial.print("HX711 measured settlingtime ms: ");
  Serial.println(LoadCell.getSettlingTime());
  Serial.println("Note that the settling time may increase significantly if you use delay() in your sketch!");
  if (LoadCell.getSPS() < 7) {
    Serial.println("!!Sampling rate is lower than specification, check MCU>HX711 wiring and pin designations");
  }
  else if (LoadCell.getSPS() > 100) {
    Serial.println("!!Sampling rate is higher than specification, check MCU>HX711 wiring and pin designations");
  }

  // Start the DS18B20 sensor
  sensors.begin();

  // Start Heart Rate Sensor

  while (!particleSensor.begin()) {
    Serial.println("MAX30102 was not found");
    delay(1000);
  }

  particleSensor.sensorConfiguration(/*ledBrightness=*/50, /*sampleAverage=*/SAMPLEAVG_2, \
                        /*ledMode=*/MODE_MULTILED, /*sampleRate=*/SAMPLERATE_200, \
                        /*pulseWidth=*/PULSEWIDTH_411, /*adcRange=*/ADCRANGE_16384);

int32_t SPO2; //SPO2
int8_t SPO2Valid; //Flag to display if SPO2 calculation is valid
int32_t heartRate; //Heart-rate
int8_t heartRateValid; //Flag to display if heart-rate calculation is valid 

  initWiFi();
  Serial.print("RRSI: ");
  configTime(0, 0, ntpServer);

  // Assign the api key (required)
  config.api_key = API_KEY;

  // Assign the user sign in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  // Assign the RTDB URL (required)
  config.database_url = DATABASE_URL;

  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096);

  // Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  // Assign the maximum retry of token generation
  config.max_token_generation_retry = 5;

  // Initialize the library with the Firebase authen and config
  Firebase.begin(&config, &auth);

  // Getting the user UID might take a few seconds
  Serial.println("Getting User UID");
  while ((auth.token.uid) == "") {
    Serial.print('.');
    delay(1000);
  }
  // Print user UID
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);

  // Update database path
  databasePath = "/UsersData/" + uid + "/readings";
}


void loop(){

  static boolean newDataReady = 0;
  const int serialPrintInterval = 500; 
    // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;
    // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      lcd2.setCursor(0,2);
        weight_kg = LoadCell.getData() /1000;
        if(weight_kg > 0){
          lcd2.print("W:");
          lcd2.print(weight_kg);
          lcd2.print("kg");
          Serial.print("Load_cell output val: ");
          Serial.println(weight_kg);

        }

        button1.pressed = false;
      newDataReady = 0;
      t = millis();
    }
  }

  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  // check if last tare operation is complete:
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
 

  // Initialize Temperature Sensor Reading
  sensors.requestTemperatures();
  // Sensing bit rate
  Serial.println(F("Wait about four seconds"));
  particleSensor.heartrateAndOxygenSaturation(/**SPO2=*/&SPO2, /**SPO2Valid=*/&SPO2Valid, /**heartRate=*/&heartRate, /**heartRateValid=*/&heartRateValid);

  // Send new readings to database
  if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();

    //Get current timestamp
    timestamp = getTime();
    Serial.print ("time: ");
    Serial.println (timestamp);

    parentPath= databasePath + "/" + String(timestamp);

    float temperatureC = sensors.getTempCByIndex(0);
    Serial.print(temperatureC);
    Serial.println("ºC");
    if((heartRateValid == 1 && SPO2Valid == 1)){
      //Print result 
      Serial.print(F("heartRate="));
      Serial.print(heartRate, DEC);
      Serial.print(F(", heartRateValid="));
      Serial.print(heartRateValid, DEC);
      Serial.print(F("; SPO2="));
      Serial.print(SPO2, DEC);
      Serial.print(F(", SPO2Valid="));
      Serial.println(SPO2Valid, DEC);
      lcd2.setCursor(0,0);
      lcd2.print("HR:");
      lcd2.print(heartRate);
      lcd2.print("bpm:");
      lcd2.setCursor(11,0);
      lcd2.print("SPO2:");
      lcd2.print(SPO2);
      lcd2.print("%");
      lcd2.setCursor(0,1);
      lcd2.print("Temperature:");
      lcd2.print(temperatureC);
      lcd2.print("C");

      json.set(tempPath.c_str(), String(temperatureC));
      json.set(heartPath.c_str(), String(heartRate));
      json.set(oxygenPath.c_str(), String(SPO2));
      float weight = 0;
      float height = 0;
      json.set(weightPath.c_str(), String(weight));
      json.set(heightPath.c_str(), String(height));

      json.set(timePath, String(timestamp));
      Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());

    }

  }

  if (button1.pressed){
    Serial.print("Button is pressed");
    m_height = height(); // Height in meters
    lcd2.setCursor(11,2);
    lcd2.print("H:");
    lcd2.print(m_height);
    lcd2.print("m");
    bmi = weight_kg / (m_height * m_height);
    lcd2.setCursor(0,3);
    lcd2.print("BMI:");
    lcd2.print(bmi);
    lcd2.print("kg/m2");
    json.set(weightPath.c_str(), String(weight_kg));
    json.set(heightPath.c_str(), String(m_height));
    json.set(bmiPath.c_str(), String(bmi));
    json.set(timePath, String(timestamp));
    if((heartRateValid == 1 && SPO2Valid == 1)){
      json.set(tempPath.c_str(), String(temperatureC));
      json.set(heartPath.c_str(), String(heartRate));
      json.set(oxygenPath.c_str(), String(SPO2));
    }
    else{
      json.set(tempPath.c_str(), String(0));
      json.set(heartPath.c_str(), String(0));
      json.set(oxygenPath.c_str(), String(0));
    }
    Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());
  }
}


float height() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    startTime = micros();
    while (micros() - startTime < 10) {
      // Wait for 10 microseconds
    }
    digitalWrite(trigPin, LOW);
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    while (digitalRead(echoPin) == LOW) {
      startTime = micros();
    }
    
    while (digitalRead(echoPin) == HIGH) {
      endTime = micros();
    }
    
    duration = endTime - startTime;
    
    // Calculate the distance
    height_m = (duration * SOUND_SPEED / 2)/100;
  }
  return 2 - height_m;
}
