#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

void setup() {
  Serial.begin(115200);

  while (!Serial.available())
  {
    delay(100);
  }

  Serial.println(sizeof(imu::Quaternion));

  int dataSize = sizeof(double) * 4;

  Serial.println(dataSize);

}

void loop() {
  // put your main code here, to run repeatedly:

}
