#define BASESTEPANGLE (1.8)
#define INAPIN	(8)
#define INBPIN (9)

#define RPM (30)

void setup()
{

  pinMode(INAPIN, OUTPUT);
  pinMode(INBPIN, OUTPUT);
}

void loop()
{
    
    int iStepTime_milli = (int)(1000.0 * BASESTEPANGLE / (RPM * 6));
	// P1:ON, P2:ON, P1B:off, P2B:off
	digitalWrite(INAPIN, LOW);
	digitalWrite(INBPIN, HIGH);
	delay(iStepTime_milli);
	// P1:off, P2:ON, P1B:ON, P2B:off
	digitalWrite(INAPIN, HIGH);
	digitalWrite(INBPIN, HIGH);
	delay(iStepTime_milli);
	// P1:off, P2:off, P1B:ON, P2B:ON
	digitalWrite(INAPIN, HIGH);
	digitalWrite(INBPIN, LOW);
	delay(iStepTime_milli);
	// P1:ON, P2:off, P1B:off, P2B:ON
	digitalWrite(INAPIN, LOW);
	digitalWrite(INBPIN, LOW);
	delay(iStepTime_milli);
  
}
