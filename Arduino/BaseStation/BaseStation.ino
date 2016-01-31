#include <SPI.h>
#include "RF24.h"

const int MAX_REMOTES = 8;
const unsigned long TIMEOUT = 200000; // 200ms

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 5 & 6 */
RF24 radio(5, 6);

// the pipes we will read and write on
// the pipe address can be up to 5 bytes
const byte remotePipe[6] = "r____";
const byte basePipe[6] = "b____";

long remoteIds[MAX_REMOTES];
int remotesRegistered = 0; 

// TODO: figure out how to not have to copy this
struct remoteData
{
  long id;
  bool triggerUpBeat;
  bool triggerDownBeat;
  double accelerationMagnitude;
  double upDot;
} payload;

void initRadio()
{
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(basePipe);
  radio.openReadingPipe(1, remotePipe);
  radio.startListening();
}

void processAndSendRemoteData(int remoteId)
{  
  Serial.print(remoteId);
  Serial.print(" ");
  Serial.print(payload.accelerationMagnitude);
  Serial.print(" ");
  Serial.print(payload.triggerUpBeat);
  Serial.print(" ");
  Serial.print(payload.triggerDownBeat);
  Serial.print(" ");
  Serial.print(payload.upDot);
  Serial.println("");
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
      if (remotesRegistered >= MAX_REMOTES)
      {
        continue;
      }
      
      foundId = remotesRegistered;
      remoteIds[remotesRegistered] = payload.id;
      ++remotesRegistered;
    }
  
    radio.stopListening();

    // acknowledge receipt
    // TODO: status codes if needed
    radio.write(0, sizeof(byte));
    
    radio.startListening();

    processAndSendRemoteData(foundId);
  }
}
