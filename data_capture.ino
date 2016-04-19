/*
 * This sketch captures data from the Watchman
 * Sonic oil tank level sensor. This product
 * consists of an ultrasonic sensor (mounted on
 * the tank), and an indoor, mains powered
 * receiver. Inside is a neat 5v PSU, LCD driver,
 * a PIC chip, and the SI4320 RX chip. Pins 6 and
 * 7 spit out the required data, and a clock. The
 * data is manchester encoded, 8 bytes (last is
 * 1Wire CRC8), with 3on/3off bits preamble. We
 * can read temperature, and distance from the
 * sensor!
 * 
 * MJC, 16/04/2016
 * 
 */


const byte dataPin = 2; //input pin from receiver
int halfPulseWidth = 1014; //microseconds per 'half pulse', corrected during receive?
float alpha = 0.8; //to correct halfPulseWidth
int pulseTolerance = 50; //tolerance on above time, microseconds. Maybe tightened up?
int minHalfPW = halfPulseWidth-pulseTolerance;
int maxHalfPW = halfPulseWidth+pulseTolerance;
int minWholePW = 2*minHalfPW;
int maxWholePW = 2*maxHalfPW;
int minThreePW = 3*minHalfPW;
int maxThreePW = 3*maxHalfPW;

int maybeTemp = 0;
float temp = 0;
int distance = 0;

volatile bool pulseValue; //a temporary store for the pulse HIGH/LOW
volatile long upTime = 0; //micros() when rising edge detected
volatile long downTime = 0;
volatile long upPulseWidth;
volatile long downPulseWidth;
int rawBuf[130]; //8 bytes, 64 bits, 128 possible pulses, plus the preamble, so 130! Sum of absolute parts of array should be 134 (128 + 6)
volatile int arraySum = 0;
volatile int rawBufPos = 0; //position in raw buffer
volatile bool bufReady = 0; //raw buffer ready for processing
int procBuf[64]; //64 bits
int procBufPos = 0;

void grab() { //grabs the current value of dataPin
  pulseValue = digitalRead(dataPin);
  if(bufReady == 0)
  {
    if(pulseValue) //ie, rising edge
    {
      upTime = micros();
      downPulseWidth = upTime-downTime;
      if(downPulseWidth >= minThreePW && downPulseWidth <= maxThreePW)
      {
        //Three low 'half pulses'. Only valid as second element of array.
        if(rawBufPos == 1)
        {
          rawBuf[1] = -3;
          arraySum = arraySum+3;
          rawBufPos++;
        }
        else //not at element two of array, therefore is noise
        {
          arraySum = 0;
          rawBufPos = 0;
          bufReady = 0;
        }
      }
      else if(downPulseWidth >= minWholePW && downPulseWidth <= maxWholePW)
      {
        //Two low 'half pulses'
        rawBuf[rawBufPos] = 0;
        rawBufPos++;
        rawBuf[rawBufPos] = 0;
        arraySum = arraySum+2;
        rawBufPos++;
        halfPulseWidth = alpha*halfPulseWidth+(1-alpha)*(downPulseWidth*0.5);
      }
      else if(downPulseWidth >= minHalfPW && downPulseWidth <= maxHalfPW)
      {
        //One low 'half pulses'
        rawBuf[rawBufPos] = 0;
        arraySum = arraySum+1;
        rawBufPos++;
        halfPulseWidth = alpha*halfPulseWidth+(1-alpha)*downPulseWidth;
      }
      else
      {
        // Invalid pulse
        arraySum = 0;
        rawBufPos = 0;
        bufReady = 0;
      }
    }
    else
    {
      downTime = micros();
      upPulseWidth = downTime-upTime;
      //if(upPulseWidth >= minThreePW && upPulseWidth <= maxThreePW) //but there may be more 'ones' before the start of the signal
      if(upPulseWidth >= minThreePW)
      {
        //Three high 'half pulses'. Only valid at the start of the array!
        if(rawBufPos == 0)
        {
          rawBuf[0] = 3;
          arraySum = arraySum+3;
          rawBufPos++;
        }
        else //not at start of array, therefore is noise
        {
          rawBufPos = 0;
          bufReady = 0;
          rawBuf[0] = 3;
          arraySum = 3;
        }
      }
      else if(upPulseWidth >= minWholePW && upPulseWidth <= maxWholePW)
      {
        //Two high 'half pulses'
        rawBuf[rawBufPos] = 1;
        rawBufPos++;
        rawBuf[rawBufPos] = 1;
        arraySum = arraySum+2;
        rawBufPos++;
        halfPulseWidth = alpha*halfPulseWidth+(1-alpha)*(upPulseWidth*0.5);
      }
      else if(upPulseWidth >= minHalfPW && upPulseWidth <= maxHalfPW)
      {
        //One high 'half pulses'
        rawBuf[rawBufPos] = 1;
        arraySum = arraySum+1;
        rawBufPos++;
        halfPulseWidth = alpha*halfPulseWidth+(1-alpha)*upPulseWidth;
      }
      else
      {
        // Invalid pulse
        arraySum = 0;
        rawBufPos = 0;
        bufReady = 0;
      }
    }
  }
  if(arraySum>=131) // last received bit will always fail, unless we detect the post-amble
  {
    bufReady = 1;
    arraySum = 0;
    rawBufPos = 0;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(dataPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), grab, CHANGE);
  //Serial.print("Setup complete.\n\n\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  if(bufReady==1)
  {
    procBufPos=0;
    for(int i=2; i<134; i++) //the first two elements will always be [3,-3]
    {
      if(rawBuf[i]==1 && rawBuf[i+1] == 0) //10 transition
      {
        procBuf[procBufPos]=0;
      }
      else if(rawBuf[i]==0 && rawBuf[i+1] == 1) //01 transition
      {
        procBuf[procBufPos]=1;
      }
      procBufPos++;
      i++;
    }
    //for(int j=0; j<(sizeof(procBuf)/sizeof(int)); j++)
    //{
    //  Serial.print(procBuf[j]);
    //}
    //Serial.print("\n");
    /* bytes 0-3 are unit ID, byte 4 is flags, top 6 bits of byte 5 are
     * temperature, rest of byte 5 and all of byte 6 are distance, byte
     * 7 is CRC8 (but will probably be corrupt - ignore).
     * 
     * We're interested in temperature and distance.
     * Temperature is bits 40-45, distance is 46-55.
     */ 
    bitWrite(maybeTemp, 5, procBuf[40]);
    bitWrite(maybeTemp, 4, procBuf[41]);
    bitWrite(maybeTemp, 3, procBuf[42]);
    bitWrite(maybeTemp, 2, procBuf[43]);
    bitWrite(maybeTemp, 1, procBuf[44]);
    bitWrite(maybeTemp, 0, procBuf[45]);
    bitWrite(distance, 9, procBuf[46]);
    bitWrite(distance, 8, procBuf[47]);
    bitWrite(distance, 7, procBuf[48]);
    bitWrite(distance, 6, procBuf[49]);
    bitWrite(distance, 5, procBuf[50]);
    bitWrite(distance, 4, procBuf[51]);
    bitWrite(distance, 3, procBuf[52]);
    bitWrite(distance, 2, procBuf[53]);
    bitWrite(distance, 1, procBuf[54]);
    bitWrite(distance, 0, procBuf[55]);
    temp = ((145 - (5*(float)maybeTemp))/3);
    Serial.print("Temp: ");
    Serial.print(temp);
    Serial.print("deg C.   Distance: ");
    Serial.print(distance);
    Serial.print("cm.\n");
    bufReady = 0;
  }
}
