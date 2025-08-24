#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_AS7341.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//Initialize bno//                    id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

/* Initialize Spec Sensors on channels 1, 2, 3 (BNO055 is on channel 0) */
Adafruit_AS7341 sensors[3]; 

/////////////////////////////////////////////////////////////
//setup for the 4 channel multiplexer
#define MUX_ADDR 0x70  // default I2C address for PCA9546A

void selectMuxChannel(uint8_t channel) {
  if (channel > 3) return;  // only 0-3 valid
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);  // pick channel
  Wire.endTransmission();
}
//end multiplexer setup
/////////////////////////////////////////////////////////////

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10000;

void setup(void)
{
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for serial port to open!

  Wire.begin();

// ---- Initialize BNO055 on channel 0 ----
  selectMuxChannel(0);  
  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);

// ---- Initialize AS7341 on channel 1 ----
  for (int i = 0; i < 3; i++) {
      selectMuxChannel(i + 1);  // +1 because channels 1-3 for spectral sensors
      if (!sensors[i].begin()) {
          Serial.print("Spectral sensor on channel ");
          Serial.print(i + 1);
          Serial.println(" not found!");
      } else {
          Serial.print("Spectral sensor on channel ");
          Serial.print(i + 1);
          Serial.println(" initialized.");
      }
}

  // Configure AS7341 integration params
  //The total integration time is equal to T_int = (𝐴𝑇𝐼𝑀𝐸 + 1) × (𝐴𝑆𝑇𝐸𝑃 + 1) × 2.78μ𝑠 ||| if you do the math the maximum integration time is ~46 seconds at max ATIME and ASTEP
  for (int i = 0; i < 3; i++) {
    selectMuxChannel(i + 1);
    sensors[i].setATIME(255); //0 - 255max, the integration time PER STEP in incraments of 2.78us. So, at ATIME = 1, we can assume each step takes in light for 2.78us, at 255, each step is ~ 709us (still very short)

    sensors[i].setASTEP(1);   //ASTEP 999 = 2.78ms (0 min - 65534 is max value, corresponds to 182ms) This is the number of integration steps...

    sensors[i].setGain(AS7341_GAIN_64X); //gain options: 0.5, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512
}


  Serial.println("Sensors initialized.");
}


void loop(void)
{
  // ------------------ IMU ------------------
  selectMuxChannel(0);

  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

  int8_t boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);
  Serial.println("*************************");


  // ------------------ Spectral Sensor ------------------

// Loop over all spectral sensors (channels 1, 2, 3)
for (int s = 0; s < 3; s++) {
    selectMuxChannel(s + 1);  // +1 because sensor 0 is on channel 1

  uint16_t readings[12];
  if (!sensors[s].readAllChannels(readings)) {
    Serial.print("Error reading AS7341 channels: ");
    Serial.println(s + 1);
  } else {
    Serial.print("Spec ");
    Serial.print(s + 1);
    Serial.println(" : 415, 445, 480, 515, 555, 590, 630, 680, 930");
    
    for (int i = 0; i < 12; i++) {
      Serial.print(readings[i]);
      if (i < 11) Serial.print(", ");
    }

    String dataString = "";
    dataString += " [ASTEP:" + String(sensors[s].getASTEP()) + ",";
    dataString += "ATIME:" + String(sensors[s].getATIME()) + ",";
    dataString += "GAIN:" + String(sensors[s].getGain()) + "]";
    Serial.println(dataString);
    Serial.println();
    }

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);
  
  // ---- Adaptive exposure ----
  int checkChannels[] = {0, 1, 2, 3, 6, 7, 8, 9, 11};
  bool tooHigh = false, tooLow = false;

  for (int i = 0; i < sizeof(checkChannels) / sizeof(checkChannels[0]); i++) {
    int val = readings[checkChannels[i]];
    if (val > 60000) tooHigh = true;
    if (val < 1000) tooLow = true;
  }

  for (int i = 0; i < 3; i++) {
      selectMuxChannel(i + 1);
      uint16_t astep = sensors[i].getASTEP();
      bool tooHigh = false, tooLow = false;
      int checkChannels[] = {0, 1, 2, 3, 6, 7, 8, 9, 11};

      for (int j = 0; j < sizeof(checkChannels)/sizeof(checkChannels[0]); j++) {
          int val = readings[checkChannels[j]];
          if (val > 60000) tooHigh = true;
          if (val < 1000) tooLow = true;
      }

      if (tooHigh && astep > 500) {
          sensors[i].setASTEP(max(500, astep - 2000));
          Serial.print("Sensor ");
          Serial.print(i+1);
          Serial.println(" decreasing ASTEP");
      } else if (tooLow && astep < 60000) {
          sensors[i].setASTEP(min(60000, astep + 2000));
          Serial.print("Sensor ");
          Serial.print(i+1);
          Serial.println(" increasing ASTEP");
      }
  }
}
}  


// ------------ printEvent helper ------------
void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem

  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}


