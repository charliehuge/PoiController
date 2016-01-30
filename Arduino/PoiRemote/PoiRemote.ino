#include <SPI.h>
#include "RF24.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

const unsigned long TIMEOUT = 200000; // microseconds
const unsigned long SAMPLING_RATE = 10000; // microseconds, note that if this gets longer than about 10ms, switch to using regular delay

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
  long id;
  double upDot;
  double leftDot;
  double fwdDot;
  byte dir;
} payload;

imu::Vector<3> getUpVector(imu::Quaternion rot)
{
  return imu::Vector<3>(
    2.0*rot.x()*rot.y() - 2.0*rot.z()*rot.w(), 
    1.0 - 2.0*rot.x()*rot.x() - 2.0*rot.z()*rot.z(), 
    2.0*rot.y()*rot.z() + 2.0*rot.x()*rot.w());
}

void initRadio()
{
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(remotePipe);
  radio.openReadingPipe(1, basePipe);
}

/*
 * Init the sensor
 * 
 *  NOTE: run PoiRemote_Calibrate on each new remote before uploading this sketch
 *  otherwise calibration data will be garbage
 */
void initImu()
{
  int eeAddress = 0;
  
  EEPROM.get(eeAddress, payload.id);
  eeAddress += sizeof(long);
   
  /* Initialise the sensor */
  if (!bno.begin())
  {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1);
  }  

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  EEPROM.get(eeAddress, calibrationData);
  Serial.println("Got calibration for this sensor in EEPROM.");

  bno.setSensorOffsets(calibrationData);
  Serial.println("Calibration data loaded into BNO055");

  delay(1000);

  bno.setExtCrystalUse(true);

  Serial.println("Done initializing BNO055");
}

void updateSensorData()
{
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
}

void sendPayload(unsigned long startTime)
{
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
      if (micros() < startTime || micros() - startTime > TIMEOUT)
      {
        Serial.println(F("Timed out"));
        timedOut = true;
        return;
      }
    }

    byte statusByte;
    radio.read(&statusByte, sizeof(byte));
    Serial.print(F("Heard back: "));
    Serial.println(statusByte);
  }
}

void setup() 
{
  Serial.begin(115200);  

  initRadio();

  initImu();
}

void loop() 
{  
  unsigned long startTime = micros();

  updateSensorData();

  Serial.println(payload.dir);

  sendPayload(startTime);

  // if we heard back in less time than the sampling rate, AWESOME!
  // but let's not overdo it. Wait until we've caught up with the sampling rate
  unsigned long elapsed = micros() - startTime;

  if (micros() > startTime && elapsed < SAMPLING_RATE)
  {
    delayMicroseconds(SAMPLING_RATE - elapsed);
  }
}
