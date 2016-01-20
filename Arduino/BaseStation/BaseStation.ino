#include <SPI.h>
#include "RF24.h"

const unsigned long TIMEOUT = 200000; // 200ms

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 5 & 6 */
RF24 radio(5, 6);

// the pipes we will read and write on
// the pipe address can be up to 5 bytes
const byte remotePipe[6] = "r____";
const byte basePipe[6] = "b____";

byte nextAvailableId = 0;

struct remoteData
{
  byte id; 
  double upDot;
  double leftDot;
  double fwdDot;
  byte dir;
} payload;

void setup() 
{
  Serial.begin(115200);  
  
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(basePipe);
  radio.openReadingPipe(1, remotePipe);
  radio.startListening();
}

void loop() 
{
  if (!radio.available())
  {
    return;
  }

  // empty out the queue, if there is one
  while (radio.available())
  {
    radio.read(&payload, sizeof(payload));

    // if this remote doesn't have an id, give it one
    if (payload.id == 0)
    {
      payload.id = ++nextAvailableId;
    }
  
    radio.stopListening();
  
    radio.write(&payload, sizeof(payload));
    
    radio.startListening();
  
    Serial.print(payload.upDot, 4);
    Serial.print(" ");
    Serial.print(payload.leftDot, 4);
    Serial.print(" ");
    Serial.print(payload.fwdDot, 4);
    Serial.print(" ");
    Serial.println(payload.dir);
  }
}
