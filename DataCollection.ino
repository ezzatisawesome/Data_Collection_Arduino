#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h>

static const int baudrate = 115200;

// Vriables for SD utility library:
Sd2Card card;
SdVolume volume;
SdFile root;

// Variables for BNO055 libraries
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
static const int BNO055_SAMPLERATE_DELAY_MS  = 100;


void setup() {
  Serial.begin(baudrate);

  // Prepare SD card
  // while (!Serial) {
  //   ; // wait for serial port to connect. Needed for native USB port only
  // }

  // if (!volume.init(card)) {
  //   Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
  //   while (1);
  // }

  // Init sensor
  if(!bno.begin()) {
    Serial.println("There was a problem detecting the BNO055 ... check your connections");
    while(1);
  }
  bno.setExtCrystalUse(true);

  // Set up SD card writer
}

void loop() {



  // Get all raw data from BNO055
  imu::Vector<3> vectorMag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> vectorGyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> vectorEuler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> vectorAccel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> vectorLinearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> vectorGravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Quaternion quat = bno.getQuat();
  int8_t bnoTemp = bno.getTemp();


  // Print to serial
  Serial.println(
    String(vectorMag.x(), 4) + "," +
    String(vectorMag.y(), 4) + "," +
    String(vectorMag.z(), 4) + "," +

    String(vectorGyro.x(), 4) + "," +
    String(vectorGyro.y(), 4) + "," +
    String(vectorGyro.z(), 4) + "," +

    String(vectorEuler.x(), 4) + "," +
    String(vectorEuler.y(), 4) + "," +
    String(vectorEuler.z(), 4) + "," +

    String(vectorAccel.x(), 4) + "," +
    String(vectorAccel.y(), 4) + "," +
    String(vectorAccel.z(), 4) + "," +

    String(vectorLinearAccel.x(), 4) + "," +
    String(vectorLinearAccel.y(), 4) + "," +
    String(vectorLinearAccel.z(), 4) + "," +

    String(vectorGravity.x(), 4) + "," +
    String(vectorGravity.y(), 4) + "," +
    String(vectorGravity.z(), 4) + "," +

    String(quat.x(), 4) + "," +
    String(quat.y(), 4) + "," +
    String(quat.z(), 4) + "," +
    String(quat.w(), 4) + "," +

    String(bnoTemp, 4)
  );

  delay(BNO055_SAMPLERATE_DELAY_MS);
}