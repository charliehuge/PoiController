/*
 * Upload this to a remote to calibrate the sensor and assign a permanent ID
 * This is separate to simplify and minimize the actual runtime remote code,
 * and to keep from having to calibrate when trying to use the thing
 * 
 * NOTE: do this whenever your environment changes significantly, as the EMF and location of
 * your area may cause old calibration values to become invalid
 */
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/* 
 *  assign a random id
 * NOTE: this is not perfect, but id collision is statistically unlikely
 */
void assignId(int &eepromAddress)
{
  // seed random with the time since startup
  // this time is randomized thanks to humans, because we wait until we get some input
  randomSeed((long)micros());
  
  long id = random(2147483648);
  EEPROM.put(eepromAddress, id);
  
  Serial.print("assigned id: ");
  Serial.println(id);

  // advance the EEPROM address counter
  eepromAddress += sizeof(long);
}

/*
 * Displays the calibration data so you know how to calibrate it
 */
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

/*
 * Initializes the sensor and forces calibration
 */
void initImu(int &eepromAddress)
{  
  /* Initialise the sensor */
  if (!bno.begin())
  {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1);
  }  

  sensors_event_t event;

  bno.setExtCrystalUse(true);
  
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

      displayCalStatus();

      Serial.println("");

      delay(100);
  }

  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);

  Serial.println("\nFully calibrated!");
  Serial.println("\n\nStoring calibration data to EEPROM...");

  EEPROM.put(eepromAddress, newCalib);
  eepromAddress += sizeof(newCalib);
  
  Serial.println("Data stored to EEPROM.");
  delay(500);
}

void setup() 
{
  Serial.begin(115200);

  while(!Serial.available())
  {
    Serial.println("Enter anything to calibrate and factory reset the remote.");
    delay(1000);
  }

  // clear EEPROM just so we don't have garbage data in there
  for (int i = 0 ; i < EEPROM.length() ; i++) 
  {
    EEPROM.write(i, 0);
  }

  int eepromAddress = 0;

  assignId(eepromAddress);

  initImu(eepromAddress);

  Serial.print("Used ");
  Serial.print(eepromAddress);
  Serial.println(" bytes of EEPROM");

  Serial.println("Done with calibration. Have a lovely day!");
}

void loop()
{
  // empty
}

