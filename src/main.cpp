// ESP32 Guide: https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/

#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

#define MAGNET_SW_PIN 19
#define button 13

Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);

sensors_event_t a, g, temp;

int rpm = 0;

JsonDocument docWrite, docRead;
const char* filename = "/data.json";

int recordCount = 0; // goes however long until reading stops
bool buttonONS = false;
bool recordData = false;

/* Declared Functions */
static void calcRPM(void*);
double getPitch();
double getRoll();
void readDataFromFile();
void printFileData();
void writeDataToFile();
/* End of Declared functions*/

void setup() {
  Serial.begin(115200);
  pinMode(MAGNET_SW_PIN, INPUT_PULLDOWN);

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
  readDataFromFile();
}

void loop() {
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

  display.println("Accelerometer - m/s^2");
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

  display.print("RPMs: ");
  display.println(rpm);

  display.display();

  /* Recording Data using SPIFFS */
  if(!buttonONS && touchRead(button) < 50){
    Serial.println("Button Press!");
    buttonONS = true;
    recordData = !recordData;
    if(recordData == false){
      recordCount = 0;
      writeDataToFile();
      readDataFromFile();
    }
  }
  if(touchRead(button) > 50){
    buttonONS = false;
  }

  if(recordData){
    docWrite["acc"]["x"][recordCount] = accX;
    docWrite["acc"]["y"][recordCount] = accY;
    docWrite["acc"]["z"][recordCount] = accZ;
    docWrite["gyro"]["x"][recordCount] = gyroX;
    docWrite["gyro"]["y"][recordCount] = gyroY;
    docWrite["gyro"]["z"][recordCount] = gyroZ;
    recordCount++;
  }

  delay(100);
}

// runs in core 0 and updates rpms
static void calcRPM(void* pvParameters){
  bool oneShotRead = false;
  int start = millis();
  while(true){
    if(digitalRead(MAGNET_SW_PIN) && !oneShotRead){
      oneShotRead = true;
      rpm = rpm*.5 + 60*1000/(millis()-start)*.5;
      start = millis();
      delay(1);
    }
    if(!digitalRead(MAGNET_SW_PIN)){
      oneShotRead = false;
    }
    if(millis()-start > 1000){
      rpm = rpm*.2;
      start = millis();
    }
  }
}

// Get angle from Front to Back tilt (Pitch)
double getPitch(){
  /* Get new sensor events with the readings */
  mpu.getEvent(&a, &g, &temp);

  // this is the acceleration triangle opposite/(hypotenuse) calculations
  double referenceAngle = degrees(asin(a.acceleration.y/sqrt(sq(a.acceleration.z)+sq(a.acceleration.y)))); // add potential offset calibration

  if(a.acceleration.z > 0){
    // when the robot is on a tilt from -90 to 90 degree pitch
    return referenceAngle;
  }else if(a.acceleration.z < 0 && referenceAngle > 0){// Z < 0 is upside down
    // for the pitch tilt from 90 to 180 degree
    return 180 - referenceAngle;
  }else if(a.acceleration.z < 0 && referenceAngle < 0){// Z < 0 is upside down
    // for the pitch tilt that is from -90 to -180 degree
    return -referenceAngle - 180;
  }else{
    return NAN;
  }
}
// Get the angle Left to Right tilt (roll)
double getRoll(){
  /* Get new sensor events with the readings */
  mpu.getEvent(&a, &g, &temp);

  // this is the acceleration triangle opposite/(hypotenuse) calculations
  double referenceAngle = degrees(asin(a.acceleration.x/sqrt(sq(a.acceleration.z)+sq(a.acceleration.x)))); // add potential offset calibration

  if(a.acceleration.z > 0){
    // when the robot is on a tilt from -90 to 90 degree pitch
    return referenceAngle;
  }else if(a.acceleration.z < 0 && referenceAngle > 0){// Z < 0 is upsidedown
    // for the pitch tilt from 90 to 180 degree
    return 180 - referenceAngle;
  }else if(a.acceleration.z < 0 && referenceAngle < 0){// Z < 0 is upsidedown
    // for the pitch tilt that is from -90 to -180 degree
    return -referenceAngle - 180;
  }else{
    return NAN;
  }
}

// Updates and prints docRead with acceleration and rotation data during the recording time period
void readDataFromFile() {
  // Read JSON data from a file
  File file = SPIFFS.open(filename);
  if(file) {  // if file exists
    deserializeJson(docRead, file); // Deserialize Data
  }
  file.close();

  printFileData();
}

// Prints values stored in docRead from memory
void printFileData(){
  Serial.println("Acceleration X DATA!!!!!");
  for(int i=0; i<docRead["acc"]["x"].size(); i++){
    Serial.print((double)docRead["acc"]["x"][i]);
    Serial.print(",");
  }
  Serial.println("\nAcceleration Y DATA!!!!");
  for(int i=0; i<docRead["acc"]["y"].size(); i++){
    Serial.print((double)docRead["acc"]["y"][i]);
    Serial.print(",");
  }
  Serial.println("\nAcceleration Z DATA!!!!");
  for(int i=0; i<docRead["acc"]["z"].size(); i++){
    Serial.print((double)docRead["acc"]["z"][i]);
    Serial.print(",");
  }
  Serial.println("");
}

// saves the acceleration and rotation data during the recording time period in the memory
void writeDataToFile() {
  File outfile = SPIFFS.open(filename,"w"); //set to write data
  
  // Serialize and write data to SPIFFS
  if(serializeJson(docWrite, outfile)==0) {
    Serial.println("Failed to write to SPIFFS file");
  } else {
    Serial.println("Write Success!");
  }
  outfile.close();  
}