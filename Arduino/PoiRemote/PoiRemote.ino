#include <SPI.h>
#include "RF24.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

const unsigned long TIMEOUT = 200000; // 200ms
const int SAMPLING_RATE = 10;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 5 & 6 */
RF24 radio(5, 6);

// the pipes we will read and write on
// the pipe address can be up to 5 bytes
const byte remotePipe[6] = "r____";
const byte basePipe[6] = "b____";

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/* Sensor state tracking */
imu::Vector<3> DIRECTION_UP = imu::Vector<3>(0, 0, 1);
imu::Vector<3> DIRECTION_LEFT = imu::Vector<3>(-1, 0, 0);
imu::Vector<3> DIRECTION_FORWARD = imu::Vector<3>(0, 1, 0);
imu::Vector<3> lastSideDirection = imu::Vector<3>(0, 0, 1);

struct remoteData
{
  byte id; 
  double upDot;
  double leftDot;
  double fwdDot;
  byte dir;
} payload;

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

imu::Vector<3> getUpVector(imu::Quaternion rot)
{
  return imu::Vector<3>(
    2.0*rot.x()*rot.y() - 2.0*rot.z()*rot.w(), 
    1.0 - 2.0*rot.x()*rot.x() - 2.0*rot.z()*rot.z(), 
    2.0*rot.y()*rot.z() + 2.0*rot.x()*rot.w());
}

void setup() 
{
  Serial.begin(115200);  
  
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(remotePipe);
  radio.openReadingPipe(1, basePipe);

  payload.id = 0;

  /* Initialise the sensor */
  if (!bno.begin())
  {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1);
  }

  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /*
  *  Look for the sensor's unique ID at the beginning oF EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
      Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);

      Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(calibrationData);

      Serial.println("\n\nCalibration data loaded into BNO055");
      foundCalib = true;
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  if (foundCalib){
      while (!bno.isFullyCalibrated())
      {
          Serial.println("Move sensor slightly to calibrate magnetometers");
          bno.getEvent(&event);
          delay(SAMPLING_RATE);
      }
  }
  else
  {
      Serial.println("Please Calibrate Sensor: ");
      while (!bno.isFullyCalibrated())
      {
          bno.getEvent(&event);

          Serial.print("X: ");
          Serial.print(event.orientation.x, 4);
          Serial.print("\tY: ");
          Serial.print(event.orientation.y, 4);
          Serial.print("\tZ: ");
          Serial.print(event.orientation.z, 4);

          /* Optional: Display calibration status */
          displayCalStatus();

          /* New line for the next sample */
          Serial.println("");

          /* Wait the specified delay before requesting new data */
          delay(SAMPLING_RATE);
      }
  }

  Serial.println("\nFully calibrated!");
  Serial.println("--------------------------------");
  Serial.println("Calibration Results: ");
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);

  Serial.println("\n\nStoring calibration data to EEPROM...");

  eeAddress = 0;
  bno.getSensor(&sensor);
  bnoID = sensor.sensor_id;

  EEPROM.put(eeAddress, bnoID);

  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, newCalib);
  Serial.println("Data stored to EEPROM.");

  Serial.println("\n--------------------------------\n");
  delay(500);
}

void loop() 
{  
  unsigned long startTime = micros();
  
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> sensorUp = getUpVector(quat);
  payload.upDot = DIRECTION_UP.dot(sensorUp);
  payload.leftDot = DIRECTION_LEFT.dot(sensorUp);
  payload.fwdDot = DIRECTION_FORWARD.dot(sensorUp);
  payload.dir; // 0 = up, 1 = down, 2 = left, 3 = right, 4 = forward, 5 = back

  if (payload.upDot > 0.5)
  {
    payload.dir = 0;
  }
  else if (payload.upDot < -0.5)
  {
    payload.dir = 1;
  }
  else if (payload.leftDot > 0.5)
  {
    payload.dir = 2;
  }
  else if (payload.leftDot < -0.5)
  {
    payload.dir = 3;
  }
  else if (payload.fwdDot > 0.5)
  {
    payload.dir = 4;
  }
  else
  {
    payload.dir = 5;
  }
  
  radio.stopListening();

  if (!radio.write(&payload, sizeof(payload)))
  {
    Serial.println(F("Failed sending data"));
    radio.startListening();
  }
  else
  {
    radio.startListening();

    bool timedOut = false;
  
    while (!radio.available())
    {
      if (micros() - startTime > TIMEOUT)
      {
        Serial.println(F("Timed out"));
        timedOut = true;
        break;
      }
    }

    if (!timedOut)
    {
      radio.read(&payload, sizeof(payload));
      Serial.println(F("Heard back"));
    }
  }
  
  unsigned long elapsed = micros() - startTime;
  int elapsedMs = (int)(elapsed / 1000);

  if (elapsedMs < SAMPLING_RATE)
  {
    delay(SAMPLING_RATE - elapsedMs);
  }
}
