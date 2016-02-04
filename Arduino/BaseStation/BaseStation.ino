#include <SPI.h>
#include "RF24.h"

const uint8_t MAX_REMOTES = 16;
const unsigned long POLLING_INTERVAL = 10;

uint32_t remotes[MAX_REMOTES];
uint8_t registeredRemotes = 0;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 5 & 6 */
RF24 radio(9, 10);

// the pipes we will read and write on
// the pipe address can be up to 5 bytes
const byte remotePipe[6] = "r____";
const byte basePipe[6] = "b____";

unsigned long lastRemotePollTime = 0;

struct __attribute__ ((__packed__))
{
  uint32_t id;
  bool triggerUp;
  bool triggerDown;
  float upDot;
  float accMag;
} payload;

void initRadio()
{
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(false);
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
  // listen for remotes trying to send us data
  while (radio.available())
  {
    radio.read(&payload, sizeof(payload));

    int remoteIdx = -1;

    for (int i = 0; i < registeredRemotes; ++i)
    {
      if (remotes[i] == payload.id)
      {
        remoteIdx = i;
        break;
      }
    }

    if (remoteIdx < 0)
    {
      if (registeredRemotes < MAX_REMOTES)
      {
        remotes[registeredRemotes++] = payload.id; 
        Serial.print("Assigned an id to ");
        Serial.print(payload.id);
        Serial.print(", ");
        Serial.println(registeredRemotes-1);
      }
      else
      {
        Serial.println("Too many remotes."); 
      }
    }
    else
    {
      Serial.print("Got a payload from ");
      Serial.println(payload.id);
      Serial.print(remoteIdx);
      Serial.print(" ");
      Serial.print(payload.triggerUp);
      Serial.print(" ");
      Serial.print(payload.triggerDown);
      Serial.print(" ");
      Serial.print(payload.accMag);
      Serial.print(" ");
      Serial.println(payload.upDot);

      byte channel = (byte)remoteIdx + 1; 
      byte accMag = (byte)constrain(payload.accMag, 0, 70);
      byte accCtl = accMag * 127 / 70;
      byte upDotCtl = (byte)((payload.upDot + 1) * 127 / 2);

      if (accCtl > 31)
      {
        if (payload.triggerUp)
        {
          usbMIDI.sendNoteOn(60, 127, channel);
          usbMIDI.sendNoteOff(61, 127, channel);
        }
        if (payload.triggerDown)
        {
          usbMIDI.sendNoteOn(61, 127, channel);
          usbMIDI.sendNoteOff(60, 127, channel);
        }
      }
      else
      {
          usbMIDI.sendNoteOff(60, 127, channel);
          usbMIDI.sendNoteOff(61, 127, channel);
      }

      usbMIDI.sendControlChange(16, accCtl, channel);
      usbMIDI.sendControlChange(17, upDotCtl, channel);
    }
  }

  while (usbMIDI.read());

  if (millis() > lastRemotePollTime + POLLING_INTERVAL)
  {
    lastRemotePollTime = millis();
    radio.stopListening();
    radio.write(0, sizeof(byte));
    radio.startListening();
  }
}
