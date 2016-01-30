#include <SPI.h>
#include "RF24.h"

#define DEBUG_PRINTS 0

const int REMOTE_ID_LENGTH = 256;
const unsigned long TIMEOUT = 200000; // 200ms

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 5 & 6 */
RF24 radio(5, 6);

// the pipes we will read and write on
// the pipe address can be up to 5 bytes
const byte remotePipe[6] = "r____";
const byte basePipe[6] = "b____";

// you can register up to 256 remotes, because why not?
long remoteIds[REMOTE_ID_LENGTH];
int remotesRegistered = 0; 

// TODO: figure out how to not have to copy this
struct remoteData
{
  long id;
  double upDot;
  double leftDot;
  double fwdDot;
  byte dir;
} payload;

void initRadio()
{
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(basePipe);
  radio.openReadingPipe(1, remotePipe);
  radio.startListening();
}

void setup() 
{
  Serial.begin(115200);  

  initRadio();
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

    // if we haven't heard from this radio before, add it to the list
    int foundId = -1;
    
    for (int i = 0; i < remotesRegistered; ++i)
    {
      if (remoteIds[i] == payload.id)
      {
        foundId = i;
        break;
      }
    }

    if (foundId < 0)
    {
      foundId = remotesRegistered;
      remoteIds[remotesRegistered] = payload.id;
      ++remotesRegistered;

      if (DEBUG_PRINTS)
      {
        Serial.print("Found a new remote with id: ");
        Serial.println(payload.id);
        Serial.print("Total remotes: ");
        Serial.print(remotesRegistered);
      }
    }
  
    radio.stopListening();

    // acknowledge receipt
    // TODO: status codes if needed
    radio.write(0, sizeof(byte));
    
    radio.startListening();

    Serial.print(foundId);
    Serial.print(" ");
    Serial.print(payload.upDot, 4);
    Serial.print(" ");
    Serial.print(payload.leftDot, 4);
    Serial.print(" ");
    Serial.print(payload.fwdDot, 4);
    Serial.print(" ");
    Serial.println(payload.dir);
  }
}
