// ESP32 Guide: https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/

#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

#define MAX_MPH 30

#define MAGNET_SW_PIN 27
#define BUTTON 12

Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);

sensors_event_t a, g, temp;

JsonDocument docData, offsetData;
const char* dataStored = "/data.json";
const char* offsetStored = "/offset.json";

enum SCREEN {SPEED,ACCELERATION};

int rpm = 0;
int recordCount = 0; // goes however long until reading stops
int recordSection = 0;  // counts as you hit record
bool recordData = false;
SCREEN displayScreen = SPEED;
double pitchOffset = 0;
double rollOffset = 0;

/* Declared Functions */
static void calcRPM(void*);
int_least32_t getMPH(int);
double getPitch();
double getRoll();
double getRawPitch();
double getRawRoll();
void zeroAngles();
void readDataFromFile(JsonDocument*, const char*);
void printFileData();
void writeDataToFile(JsonDocument, const char*);
/* End of Declared functions*/

void setup() {
  Serial.begin(115200);
  pinMode(MAGNET_SW_PIN, INPUT_PULLUP);
  pinMode(BUTTON, INPUT_PULLUP);

  if (!mpu.begin()) {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }
  Serial.println("Found a MPU-6050 sensor");

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    while (1)
      yield();
  }
  display.display();
  delay(500); // Pause for half a second
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);

  // Created task to run rpm calculations in core 0 
  xTaskCreatePinnedToCore(
                    calcRPM, /* Function to implement the task */
                    "task1", /* Name of the task */
                    10000,   /* Stack size in words */
                    NULL,    /* Task input parameter */
                    0,       /* Priority of the task */
                    NULL,    /* Task handle. */
                    0);  /* Core where the task should run */

  // Initialize the SPIFFS object
  if(!SPIFFS.begin(true)) {
    Serial.println("Error initializing SPIFFS");
    while (1)
      yield();
  }
  // SPIFFS.format();
  
  readDataFromFile(&docData, dataStored);
  readDataFromFile(&offsetData, offsetStored);
  pitchOffset = (double)offsetData["pitch"];
  rollOffset = (double)offsetData["roll"];

  Serial.println((double)offsetData["pitch"]);
  Serial.println((double)offsetData["roll"]);
  printFileData();
  Serial.print(SPIFFS.usedBytes()); Serial.print("/"); Serial.print(SPIFFS.totalBytes()); Serial.println(" Bytes");
}

int num1 = millis();
int num2 = millis();
void loop() {
  num1 = millis();
  /* Read values from mpu6050 Sensor */
  mpu.getEvent(&a, &g, &temp);

  double accX = a.acceleration.x;
  double accY = a.acceleration.y;
  double accZ = a.acceleration.z;
  double gyroX = g.gyro.x;
  double gyroY = g.gyro.y;
  double gyroZ = g.gyro.z;

  /* Displaying Data on OLED */
  display.clearDisplay();
  display.setCursor(0, 0);
  if(recordData){
    display.println("                  REC");
  }
  display.setCursor(0, 0);

  if(displayScreen == SPEED){
    display.print("MPH:              ");

    display.drawLine(128/2,63,64-50*cos(PI*getMPH(rpm)/MAX_MPH),63-50*sin(PI*getMPH(rpm)/MAX_MPH),WHITE);
    display.drawCircle(64,63,63,WHITE);

    //0
    display.drawChar(3,56,'0',WHITE,BLACK,1);
    //5
    display.drawChar(12,33,'5',WHITE,BLACK,1);
    //10
    display.drawChar(30,12,'1',WHITE,BLACK,1);
    display.drawChar(36,12,'0',WHITE,BLACK,1);
    //15
    display.drawChar(58,3,'1',WHITE,BLACK,1);
    display.drawChar(64,3,'5',WHITE,BLACK,1);
    //20
    display.drawChar(88,12,'2',WHITE,BLACK,1);
    display.drawChar(94,12,'0',WHITE,BLACK,1);
    //25
    display.drawChar(107,33,'2',WHITE,BLACK,1);
    display.drawChar(113,33,'5',WHITE,BLACK,1);
    //30
    display.drawChar(114,56,'3',WHITE,BLACK,1);
    display.drawChar(120,56,'0',WHITE,BLACK,1);
  }else if(displayScreen == ACCELERATION){
    display.println("\nAccelerometer - m/s^2");
    display.print(accX, 1);
    display.print(", ");
    display.print(accY, 1);
    display.print(", ");
    display.print(accZ, 1);
    display.println("");

    display.println("Gyroscope - rps");
    display.print(gyroX, 1);
    display.print(", ");
    display.print(gyroY, 1);
    display.print(", ");
    display.print(gyroZ, 1);
    display.println("");

    display.print("Pitch (deg): ");
    display.println(getPitch());
    display.print("Roll (deg): ");
    display.println(getRoll());
  }

  display.display();

  // Holding button to
  if(!digitalRead(BUTTON)){
    Serial.println("Button Pressed!");
    int holdingButton = millis();
    while(true){
      // Hold Button < 2 Seconds
      if(millis()-holdingButton < 2000){
        //change display when Released
        if(digitalRead(BUTTON)){
          if(displayScreen == SPEED){
            displayScreen = ACCELERATION;
          }else{
            displayScreen = SPEED;
          }
          Serial.println("Swap display!");
          break;
        }
      }
      // Hold Button for 2-10 Seconds
      else if(millis()-holdingButton < 10000){
        //start recording when released
        if(digitalRead(BUTTON)){
          recordData = !recordData;
          if(recordData == false){
            recordCount = 0;
            recordSection++;
            writeDataToFile(docData, dataStored);
            Serial.println("End Recording!");
          }else{
            Serial.println("Start Recording!");
          }
          break;
        }
      }
      // Hold Button 10 or more Seconds
      else{
        //zeroc tilt when released
        if(digitalRead(BUTTON)){
          // Take a Zero degree measurement
          zeroAngles();
          Serial.println("Zero degrees!");
          break;
        }
      }
    }
  }

  if(recordData){
    docData["speed"][recordCount] = getMPH(rpm);
    docData["acc"]["x"][recordCount] = accX;
    docData["acc"]["y"][recordCount] = accY;
    docData["acc"]["z"][recordCount] = accZ;
    docData["gyro"]["x"][recordCount] = gyroX;
    docData["gyro"]["y"][recordCount] = gyroY;
    docData["gyro"]["z"][recordCount] = gyroZ;
    docData["pitch"][recordCount] = getPitch();
    docData["roll"][recordCount] = getRoll();
    recordCount++;
  }
  
  delay(10);

  num2 = millis();
  Serial.println(num2-num1);

}

// runs in core 0 and updates rpms
static void calcRPM(void* pvParameters){
  bool oneShotRead = false;
  int start = millis();
  while(true){
    // If RPM Switch Triggers
    if(!digitalRead(MAGNET_SW_PIN) && !oneShotRead){
      oneShotRead = true;
      // smooths the result
      rpm = rpm*.2 + 60*1000/(millis()-start)*.8;
      rpm = rpm/2.0;
      start = millis();
      delay(1);
    }
    // If RPM Switch Releases
    if(digitalRead(MAGNET_SW_PIN)){
      oneShotRead = false;
    }

    // Slowly goes to zero
    rpm -= 5;
    if(rpm < 0){
      rpm = 0;
    }
    delay(90);
  }
}

// Get the mph from the rpm of shaft
int getMPH(int rpm){
  // 25" Diameter Tire
  double diameter = 25;
  // circumfrence * rph / 63360" in a mile
  return (int)((PI*diameter)*(rpm*60) / 63360);  // Miles per hour
}

// Accounts for offset
double getPitch(){
  return getRawPitch()-pitchOffset;
}
// Accounts for offset
double getRoll(){
  return getRawRoll()-rollOffset;
}

// Get angle from Front to Back tilt (Pitch)
double getRawRoll(){
  /* Get new sensor events with the readings */
  mpu.getEvent(&a, &g, &temp);

  // this is the acceleration triangle opposite/(hypotenuse) calculations
  double referenceAngle = degrees(asin(a.acceleration.y/sqrt(sq(a.acceleration.z)+sq(a.acceleration.y)))); // add potential offset calibration

  if(a.acceleration.z > 0){
    // when the robot is on a tilt from -90 to 90 degree pitch
    return referenceAngle;
  }else if(a.acceleration.z < 0 && referenceAngle > 0){// Z < 0 is upside down
    // for the pitch tilt from 90 to 180 degree
    return referenceAngle;
  }else if(a.acceleration.z < 0 && referenceAngle < 0){// Z < 0 is upside down
    // for the pitch tilt that is from -90 to -180 degree
    return referenceAngle;
  }else{
    return NAN;
  }
}
// Get the angle Left to Right tilt (roll)
double getRawPitch(){
  /* Get new sensor events with the readings */
  mpu.getEvent(&a, &g, &temp);

  // this is the acceleration triangle opposite/(hypotenuse) calculations
  double referenceAngle = degrees(asin(a.acceleration.x/sqrt(sq(a.acceleration.z)+sq(a.acceleration.x)))); // add potential offset calibration

  if(a.acceleration.z > 0){
    // when the robot is on a tilt from -90 to 90 degree pitch
    return referenceAngle;
  }else if(a.acceleration.z < 0 && referenceAngle > 0){// Z < 0 is upsidedown
    // for the pitch tilt from 90 to 180 degree
    return referenceAngle;
  }else if(a.acceleration.z < 0 && referenceAngle < 0){// Z < 0 is upsidedown
    // for the pitch tilt that is from -90 to -180 degree
    return referenceAngle;
  }else{
    return NAN;
  }
}

// Set angle offsets to zero device
void zeroAngles(){
  pitchOffset = getRawPitch();
  rollOffset = getRawRoll();
  offsetData["pitch"] = pitchOffset;
  offsetData["roll"] = rollOffset;
  
  writeDataToFile(offsetData, offsetStored);
}

// Updates and prints docData with acceleration and rotation data during the recording time period
void readDataFromFile(JsonDocument* doc, const char* filename) {
  // Read JSON data from a file
  File file = SPIFFS.open(filename);
  if(file) {  // if file exists
    deserializeJson(*doc, file); // Deserialize Data
  }
  file.close();
}

// Prints values stored in docData from memory
void printFileData(){
  readDataFromFile(&docData, dataStored);
  // for(int section=0; section<docData["acc"]["x"].size(); section++){
    // Serial.print("Section: ");Serial.println(section+1);
    Serial.println("Acceleration X DATA!!!!!");
    for(int i=0; i<docData["acc"]["x"].size(); i++){
      Serial.print((double)docData["acc"]["x"][i]);
      Serial.print(",");
    }
    Serial.println("\nAcceleration Y DATA!!!!");
    for(int i=0; i<docData["acc"]["y"].size(); i++){
      Serial.print((double)docData["acc"]["y"][i]);
      Serial.print(",");
    }
    Serial.println("\nAcceleration Z DATA!!!!");
    for(int i=0; i<docData["acc"]["z"].size(); i++){
      Serial.print((double)docData["acc"]["z"][i]);
      Serial.print(",");
    }
    Serial.println("\nGyro X DATA!!!!!");
    for(int i=0; i<docData["gyro"]["x"].size(); i++){
      Serial.print((double)docData["acc"]["x"][i]);
      Serial.print(",");
    }
    Serial.println("\nGyro Y DATA!!!!");
    for(int i=0; i<docData["gyro"]["y"].size(); i++){
      Serial.print((double)docData["acc"]["y"][i]);
      Serial.print(",");
    }
    Serial.println("\nGyro Z DATA!!!!");
    for(int i=0; i<docData["gyro"]["z"].size(); i++){
      Serial.print((double)docData["acc"]["z"][i]);
      Serial.print(",");
    }
    Serial.println("\nPitch Degree DATA!!!!");
    for(int i=0; i<docData["pitch"].size(); i++){
      Serial.print((double)docData["pitch"][i]);
      Serial.print(",");
    }
    Serial.println("\nRoll Degree DATA!!!!");
    for(int i=0; i<docData["roll"].size(); i++){
      Serial.print((double)docData["roll"][i]);
      Serial.print(",");
    }
    Serial.println("\nMiles per Hour DATA!!!!");
    for(int i=0; i<docData["speed"].size(); i++){
      Serial.print((double)docData["speed"][i]);
      Serial.print(",");
    }
    Serial.println("");
  // }
}

// saves the acceleration and rotation data during the recording time period in the memory
void writeDataToFile(JsonDocument doc, const char* filename) {
  File outfile = SPIFFS.open(filename,"w"); //set to write data
  
  // Serialize and write data to SPIFFS
  if(serializeJson(doc, outfile)==0) {
    Serial.println("Failed to write to SPIFFS file");
  } else {
    Serial.println("Write Success!");
  }
  outfile.close();  
}
