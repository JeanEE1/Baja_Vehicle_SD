// Basic OLED demo for accelerometer readings from Adafruit MPU6050

// ESP32 Guide: https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// ESP8266 Guide: https://RandomNerdTutorials.com/esp8266-nodemcu-mpu-6050-accelerometer-gyroscope-arduino/
// Arduino Guide: https://RandomNerdTutorials.com/arduino-mpu-6050-accelerometer-gyroscope/

#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

#define magnetSW 19

Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);

sensors_event_t a, g, temp;

int rpm = 0;

/* Declared Functions */
static void calcRPM(void*);
double getPitch();
double getRoll();
/* End of Declared functions*/

void setup() {
  Serial.begin(115200);
  pinMode(magnetSW, INPUT_PULLDOWN);

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
}

void loop() {
  mpu.getEvent(&a, &g, &temp);

  display.clearDisplay();
  display.setCursor(0, 0);

  display.println("Accelerometer - m/s^2");
  display.print(a.acceleration.x, 1);
  display.print(", ");
  display.print(a.acceleration.y, 1);
  display.print(", ");
  display.print(a.acceleration.z, 1);
  display.println("");

  display.println("Gyroscope - rps");
  display.print(g.gyro.x, 1);
  display.print(", ");
  display.print(g.gyro.y, 1);
  display.print(", ");
  display.print(g.gyro.z, 1);
  display.println("");

  display.println("Orientation - degrees");
  display.print("Pitch: ");
  display.println(getPitch());
  display.print("Roll: ");
  display.print(getRoll());
  display.println("");

  display.print("RPMs: ");
  display.print(rpm);

  display.display();
  delay(100);
}

// runs in core 0 and updates rpms
static void calcRPM(void* pvParameters){
  bool oneShotRead = false;
  int start = millis();
  while(true){
    if(digitalRead(magnetSW) && !oneShotRead){
      oneShotRead = true;
      rpm = rpm*.8 + 60*1000/(millis()-start)*.2;
      start = millis();
      delay(1);
    }
    if(!digitalRead(magnetSW)){
      oneShotRead = false;
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