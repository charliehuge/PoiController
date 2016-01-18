/*
 * Poi Controller Base Station
 * 
 * Gets signals from poi controller remotes, parses them,
 * and passes data to applications listening on the serial port
 */
 
const byte CODE_START = '!';
const byte CODE_STOP = '?';

bool enabled = false;

void setup() 
{
  Serial.begin(115200);
}

void loop() 
{
  // listen for an application to request transmission
  while (Serial.available() > 0)
  {
    byte currentByte = Serial.read();

    if (currentByte == CODE_START)
    {
      enabled = true;
    }
    else if (currentByte == CODE_STOP)
    {
      enabled = false;
    }
  }

  if (!enabled)
  {
    return;
  }

  Serial.write('!');
}
