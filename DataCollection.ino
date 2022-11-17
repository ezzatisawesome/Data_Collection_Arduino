#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <SPI.h>

static const int BAUDRATE = 115200;
static const String FILE_PREFIX = "run";
static const String FILE_TYPE = ".txt";
static const String LINE_SIGNIFIER = "#";

/* Variables for BNO055 libraries */
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
static const int BNO055_SAMPLERATE_DELAY_MS  = 100;

/* Variables for writing to SD Card */
File myFile;

/* Control variables */
bool _printToSerial = false;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(BAUDRATE);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }

  // Set up SD card
  Serial.print("Initializing SD card ...");
  if (!SD.begin(4)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialization done.");

  // Create a new file
  int fileI = 0;
  String filename = FILE_PREFIX + "_" + fileI + FILE_TYPE;
  while (SD.exists(filename)) {
    Serial.println(filename + " exists.");
    fileI += 1;
    filename = FILE_PREFIX + "_" + fileI;
  }
  Serial.println("Created new file. Will write to: " + filename);
  myFile = SD.open(filename, FILE_WRITE);

  // Init sensor
  Serial.println("Connecting to BNO055 ...");
  if(!bno.begin()) {
    Serial.println("There was a problem detecting the BNO055 ... check your connections");
    while(1);
  }
  Serial.println("Connected to BNO055 ...");
  bno.setExtCrystalUse(true);
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

  // Read any commands from user
  if (Serial.available() == 0) {
    String userInput = Serial.readString(); //read until timeout
    userInput.trim(); // remove any \r \n whitespace at the end of the String
    if (userInput.length() == 0) {
      ; // Don't print anything
    } else if (userInput == "$print") {
      _printToSerial = !_printToSerial;
      Serial.println("Printing data to serial.");
    } else {
      Serial.println("Command not recognized!");
    }
  }

  // Write to SD card
  myFile.println(
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
  

  // Print to serial if applicable
  if (_printToSerial) {
    Serial.println(
      LINE_SIGNIFIER + 

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
  }

  // Delay loop
  delay(BNO055_SAMPLERATE_DELAY_MS);
}