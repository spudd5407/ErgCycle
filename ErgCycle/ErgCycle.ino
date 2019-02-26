//bl: cad discrete, servo test fail, 022419
//183kc in 15 2/18/19 update = 4

#include <Wire.h>   //i^c for the display
#include <LCDKeypad.h>    //pushbuttons on A0
#include <Adafruit_MotorShield.h>
#include "ErgCycle.h"

//#define DEBUG     //comment this out to stop serial debugging
#define DEBUG1    //connect to RM1 and use lcd for debug
#define DEBUG2		//host comm only no erg hardware

#define DEBOUNCE_TIME 100 //for reading reed

//Create motor shield object with default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

//lcd keypad uses analog input 0 for keys
unsigned int potpin = 1; //analog input for posn pot
unsigned int reedpin = 2;  //dig pin read reed for rpm
unsigned int valf;    //variable to read from pot
unsigned int load;  //resistance level
unsigned int valf0, valf1, valf2;
int diff;  //position error
int diffv;  //velocity error
int rc; //return code 
unsigned char crc ;

unsigned long maxpower = 0;
unsigned long cyctime;  //cycle time
unsigned long cycstart; //cycle start time
volatile byte half_revolutions;
volatile byte halfrevprev;
unsigned int rpm, maxrpm;
unsigned int rpmtarg = 60;  // for local mode
unsigned int rpmupdate = 4; // how many half revs
unsigned long ipm;
unsigned long totrevs, totdist, totbikedist;
unsigned long speedmps = 0;   //speed in meter per second
unsigned long tottime = 0;
unsigned long starttime, pedaltime, timeold; //rpm calc
unsigned long last_interrupt_time = 0; //for debounce check
unsigned long lastupdate;   //timer for screen update
unsigned long worktime, workstart;  //timer for work calc
float power, work, kJoules;
float wtjeff = 0.2; //amateur racer
float eff;  //kCal to KJoule efficiency
unsigned int kCal = 0;
float totdistmi = 0.0;
float totbikemi = 0.0;
bool posONLY = false;
bool negONLY = false;
bool started = false;
bool Connected = false;

// volatile globals - accessed from interrupt handlers
volatile uint8_t buttons = 0;

// i/o message holder
uint8_t buf[7];

volatile int mode;
volatile double loadcmd;
//holders for unpacked command
int type, value8, value12;
int lasttype;
double value; //value to pass to prepareCommand routine could be speed,power,hr or cadence

LCDKeypad lcd;

void setup() {
  uint8_t recd[9];	// array of received bytes
  uint8_t *reply = (uint8_t *)"LinkUp";

  lcd.begin(16, 2);	//display has 16 columns and 2 rows
  lcd.clear();

  Serial.begin(2400); // set up com port computrainer 2400baud
  Serial.println("Beginning Servo Test..");
  AFMS.begin();
  myMotor->setSpeed(100);     //set default speed
  valf = analogRead(potpin);
  lcd.print("Testing:");
  //center the actuator
  while (valf > 450) {
    myMotor->run(BACKWARD);  //get toward center posn
    delay(50);
    valf = analogRead(potpin);
  }
  myMotor->run(RELEASE);
  while (valf < 150) {
    myMotor->run(FORWARD);  //get toward center posn
    delay(50);
    valf = analogRead(potpin);
  }
  myMotor->run(RELEASE);
  delay(50);
  //begin servo test
  valf0 = analogRead(potpin);  // read value betw 0-1023
  Serial.print("Feedback Position before move = ");
  Serial.println(valf0);
  lcd.setCursor(0, 1);
  lcd.print(valf0);
  if (valf0 < 800) {
    myMotor->run(FORWARD);
    delay(1000);
    myMotor->run(RELEASE);     //and stop
    delay(50);
  }
  valf1 = analogRead(potpin); // read value betw 0-1023
  Serial.print("Feedback Position after pos move = ");
  Serial.println(valf1);
  lcd.print(valf1);
  if (valf1 > 200) {
    myMotor->run(BACKWARD);
    delay(1000);
    myMotor->run(RELEASE);     //and stop
    delay(50);
  }
  valf2 = analogRead(potpin); // read value betw 0-1023
  Serial.print("Feedback Position after NEG move = ");
  Serial.println(valf2);
  lcd.print(valf2);
  if ((abs(valf0 - valf2) < 50) && (abs(valf2 - valf1) > 50)) {
    Serial.println("Servo Test Passed!");
    lcd.setCursor(0, 13);
    lcd.print("Pass!");
  }
  else {
    Serial.println("Servo Test Failed. Check power. Check wiring");
    lcd.setCursor(0, 13);
    lcd.print("Fail");
  }
  delay(2000);  //time to read disp
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(reedpin, INPUT_PULLUP); //rpm sensor input pulls low when magnet
  attachInterrupt(digitalPinToInterrupt(reedpin), rpm_fun, FALLING);
  half_revolutions = 0; //there are two magnets per revolution
  rpm = 0;
  maxrpm = 0;
  timeold = millis();
  lcd.clear();
  lcd.setCursor(0, 0);  //use space wisely
  lcd.print("TIME="); //SETUP RUNTIME DISPLAY
  lcd.setCursor(8, 0);
  lcd.print("LOAD=");
  lcd.setCursor(0, 1);
  lcd.print("CAD=");
  lcd.setCursor(8, 1);
  lcd.print("kCAL=");
  lastupdate = millis();  //initialize timer for first calc
  cycstart = millis(); //initial for ride time and 
} //END Setup

// connect the port when Racermate is recd from pc
// host will try all available com ports on startup to find computrainers
int ConnectHost()
{
  int rc;       //return code
  uint8_t recd[9];
  //uint8_t *reply = (uint8_t *)"LinkUp";
  unsigned char reply[] = {0x4C, 0x69, 0x6E, 0x6B, 0x55, 0x70};
  //recd = Serial.readString();
  rc = Serial.readBytes(recd, 9);
  //recd[8] = '\0'; //null character ending
  reply[6] = '\0';	//terminate reply string with null char
  if (rc && strcmp((char *)recd, "RacerMate"))
  {
    Serial.write(reply, 6);
    Connected = true; //global
    return 1;
  }
  return 0;
}



void loop()
{
  //keep loadcmd in bounds
  if (loadcmd < 20) loadcmd = 20;
  if (loadcmd > 80) loadcmd = 80;
  switch (lcd.button())
  {
    case KEYPAD_LEFT:
      buttons = buttons | CT_F1;
      rpmtarg = rpmtarg - 5;
      lcd.setCursor(4, 1);
      lcd.print(rpmtarg);
      delay(2000);
      break;
    case KEYPAD_RIGHT:
      buttons = buttons | CT_F3;
      rpmtarg = rpmtarg + 5;
      lcd.setCursor(4, 1);
      lcd.print(rpmtarg);
      delay(2000);
      break;
    case KEYPAD_DOWN:
      //decrease the resistance
      buttons = buttons | CT_MINUS;
#ifdef DEBUG1
      lcd.setCursor(4, 1); //bottom row
      lcd.print(buttons);
#endif
      loadcmd -= 10;  //decrease the load
      myMotor->run(BACKWARD);
      delay(1000);
      myMotor->run(RELEASE);
      delay(50);
      break;
    case KEYPAD_UP:
      //INCREASE THE RESISTANCE
      buttons = buttons | CT_PLUS;
#ifdef DEBUG1
      lcd.setCursor(4, 1); //bottom row left
      lcd.print(buttons);
#endif
      loadcmd += 10;
      myMotor->run(FORWARD);
      delay(1000);
      myMotor->run(RELEASE);
      delay(50);
      break;
    case KEYPAD_SELECT:
      buttons = buttons | CT_F2;
      lcd.setCursor(4, 1);
      lcd.print(buttons);
      break;
    //reset button does not return a code just resets processor
    default:
      buttons = CT_NONE;  //clear buttons except reset if none pressed
      //lcd.print("NONE  ");
      break;
  }
  if (halfrevprev != half_revolutions)
  {
    halfrevprev = half_revolutions;
  }
  if (half_revolutions >= rpmupdate)
  {
    //update rpm , increase this for better rpm resolution
    //decrease for faster update
    rpm = 30 * 1000 / (millis() - timeold) * half_revolutions;
    half_revolutions = 0; //reset rev counter
    if (rpm > maxrpm)
    {
      maxrpm = rpm;
      if (started == false)
      {
        starttime = millis(); // a good time to start timing
        started = true;
      }
    }
    if (rpm > 10) {
      pedaltime += millis() - cycstart; //only count time if moving
    }
    cycstart = millis();  //reset timer
    lcd.setCursor(5, 0);
    if (pedaltime / 60000 < 10) lcd.print("0");
    lcd.print(pedaltime / 60000);
    timeold = millis();  //time for rpm calc
    //calcs for distance
    totrevs = totrevs + rpmupdate / 2;
    totdist = totrevs * 12 * 12 * 3.14; //inches for flywheel
    totbikedist = totrevs * 2.5 * 82; //typical gear ratio 3:1 and 82" whl circ
    speedmps = rpm * 2.5 * 82 / 2362; //2362 = 60sec/min * 39.37inch/meter
    ipm = 12UL * 5280UL;   //inches per mile
    totdistmi = float(totdist) / ipm;
    totbikemi = float(totbikedist) / ipm;  //bike equivalent distance
    if (millis() - lastupdate > 5000) //update lcd 5 sec
    {
      lcd.setCursor(4, 1);
      if (rpm < 100) lcd.print("0"); //print the leading zero
      lcd.print(rpm);
      valf = analogRead(potpin); // read value betw 0-1023
      lcd.setCursor(13, 0); //load level
      load = valf / 10;
      if (load < 10) lcd.print("0");
      lcd.print(load);
      power = (rpm * load) / 20;       //watts * seconds = joules
      worktime = millis() - workstart; // 1 watt * 1 sec = 1 joule
      work += power * worktime / 1000;
      kJoules = work / 1000;           //kJoules =kWatt*sec
      eff = 4.184 * wtjeff;
      kCal = kJoules / eff; //4184 joule per kCal 15 - 20% efficiency
      lcd.setCursor(13, 1);
      if (kCal < 100) lcd.print("0");
      lcd.print(kCal);
      lastupdate = millis();   //lcd update
      workstart = millis();    //work calc 
    }
    // end of lcd update routine

    if (!Connected)
    {
      Serial.print("Minutes: ");
      Serial.print(pedaltime / 60000);
      Serial.print(", Miles: ");
      Serial.print(totbikemi);
      Serial.print(", Avg Speed: ");
      Serial.print(60 * totbikemi / (pedaltime / 60000));
      Serial.print(", CAD: ");
      Serial.print(rpm, DEC);
      Serial.print(", Load: ");
      Serial.print(load);
      Serial.print(", Power: ");
      Serial.print(power);
      Serial.print(" watts, ");
      Serial.print("Work: ");
      Serial.print(kJoules);
      Serial.print(" kJoules, ");
      Serial.print("kCal: ");
      Serial.println(kCal);
    }
  } //end of rpm calc and screen updates
  if ((millis()-timeold)>200) 
    {
    rpm=0;
    lcd.setCursor(4, 1);
    lcd.print("0"); //print the leading zero
    lcd.print(rpm);
    }
  // valf >920 means magnets are very close to wheel
  //if ((valf < 150) | (valf > 920)){
  //Serial.print("near travel limit or wirebreak - ");
  if (valf < 150) posONLY = true; //only allow positive motion
  else posONLY = false;
  if (valf > 920) negONLY = true; //only allow negative motion
  else negONLY = false;
  // valc = analogRead(cmdpin);  //read the command
  //Serial.print("Command Position = ");
  //Serial.println(valc);
  //diff = valc - valf;       //update the error value
  // diff = map(diff, 0, 1023, 0, 255);
  diffv = rpm - rpmtarg;  //neg val if we are under target spd
  //go slower the closer we get
  myMotor->setSpeed(abs(diffv) + 10); //speed 25 to 255
  if (!Connected)
  {
    //update resistance based on velocity error
    if ((diffv < -10) && (posONLY == false) && (rpm > 15)) {
      myMotor->run(BACKWARD);
    }
    else if ((diffv > 10) && (negONLY == false)) {
      myMotor->run(FORWARD);
    }
    else myMotor->run(RELEASE);  //stop the motor
  }
  else  //if connected to RM1 or
  {
    //move actuator to command position
    while (valf > loadcmd * 11) {
      myMotor->run(BACKWARD);  //get toward center posn
      delay(10);
      valf = analogRead(potpin);
    }
    myMotor->run(RELEASE);
    while (valf < loadcmd * 9) {
      myMotor->run(FORWARD);  //get toward center posn
      delay(10);
      valf = analogRead(potpin);
    }
    myMotor->run(RELEASE);
    delay(10);
  }
  if (power > maxpower) {
    maxpower = power;
  }

  //connect to the host app if not connected
  if (Serial.available() && !Connected)
  {
    ConnectHost();
    rc = 0;
  }

  // Can expect valid control messages from GC every 200ms
  if (Serial.available() && Connected)
  {
    //rc = getMessage();	//this reads the CT style buffer and loads buf array
    rc = Serial.readBytes(buf, 7);
  }
  // only update the trainer data structure following a valid control message
  if (rc > 0)	//if the read returned data
  {
    unpackTelemetry(type, value8, value12);
#ifdef DEBUG //check telemetry decode
    Serial.println();
    Serial.print("type = ");
    Serial.println(type, BIN);
    Serial.print("value8 = ");
    Serial.println(value8, BIN);
    Serial.print("value12= ");
    Serial.println(value12, BIN);
#endif
#ifdef DEBUG1
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Type=");
    lcd.print(type);
    lcd.setCursor(8, 0);
    lcd.print("val8=");
    lcd.print(value8);
    lcd.setCursor(0, 1);
    lcd.print("val12=");
    lcd.print(value12);
#endif
  }

  if (type == 0x40)   //load update yay!
  {
    if (value8 != loadcmd)
    {
      loadcmd = value8; //update if changed
    }
    //move the motor to the cmd posn
    // load is percentage 0x32 = 50% 199 max?
  }
  // move motor towards required position, do this anyway for 100ms refresh
  // MotorController(&data);

  if (rc > 0)	//rotate the message type to host
  {
    switch (lasttype) {

      case CT_SPEED:
        type = CT_POWER;
        value = DEFAULT_POWER;
        break;

      case CT_POWER:
        type = CT_HEARTRATE;
        value = DEFAULT_HEARTRATE;
        break;

      case CT_HEARTRATE:
        type = CT_CADENCE;
        value = rpm;
        break;

      case CT_CADENCE:
        type = CT_SPEED;
        value = speedmps * 10;
        break;

      default:
        break;

    }

    PrepareStatusMessage( type, *buf);
#ifdef DEBUG
    Serial.print("byte array for host: ");
    Serial.write(ERGO_Command, 56);
#endif
    // update pc with current status
    sendCommand();
    lasttype = type;
    rc = 0;	
  }
} //END LOOP

void rpm_fun()  //update the rpm on interrupt
{

  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > DEBOUNCE_TIME)
  {
    half_revolutions++;
    last_interrupt_time = interrupt_time;
  }
  //  if (half_revolutions % 2)  digitalWrite(LED_BUILTIN,HIGH);
  //  else digitalWrite(LED_BUILTIN,LOW);
  //Each rotation, this interrupt function is run twice
}



/*----------------------------------------------------------------------
   COMPUTRAINER PROTOCOL DECODE/ENCODE ROUTINES

   void prepareStatusMessage()    - sets up the command packet according to current settings
   int calcCRC(double value)       - calculates the checksum for the current command
   int unpackTelemetry()  - unpacks from CT protocol layout to byte layout
                            returns non-zero if unknown message type

  ---------------------------------------------------------------------- */


void PrepareStatusMessage(int mode, uint8_t *buf)  //int mode double value
{
  int speed,power,hr,cadence,value;
  //decode value from mode
  if (mode == CT_SPEED) value = int(speed);
  if (mode == CT_POWER) value = int(power);
  if (mode == CT_HEARTRATE) value = int(hr);
  if (mode == CT_CADENCE) value = int(rpm);
  // format the outbound status response packet buffer with current data
  buf[0] = 0x00;  //crc?
  buf[1] = 0x00;
  buf[2] = 0x00;  //spinscan data
  buf[3] = buttons;   //byte 3 is all button data
  buf[4] = mode;//CADENCE  4 BITS 0X06
  buf[5] = 0;
  buf[5] |= (value & (128 + 64 + 32 + 16 + 8 + 4 + 2)) >> 1;
  buf[6] = 128; //SYNC BIT
  buf[6] |= crc & 1 ? 32 : 0; // & 0xFF;  
  buf[6] |= (value & 256) ? 2 : 0;  //(data->current_speed >> 8) & 0xFF;
}

int calcCRC(int value)
{
  return (0xff & (107 - (value & 0xff) - (value >> 8)));
}

// this routine to unpack the command from the pc for the trainer
void unpackTelemetry( int &type, int &value8, int &value12)
{
  
  // inbound data is in the 7 byte array buf[]

  short s1 = buf[0]; // ss data
  short s2 = buf[1]; // ss data
  short s3 = buf[2]; // ss data
  short bt = buf[3]; // button data - NA FROM HOST
  short b1 = buf[4]; // message and value
  short b2 = buf[5]; // value
  short b3 = buf[6]; // the dregs (sync, z and lsb for all the others)


  //these are what we care about
  // 4-bit message type
  type = (b1 & 120) >> 3; //120 = 0111000

  // 8 bit value, 7 bits from b2 and low bit from b3
  value8 = (b2 & ~128) << 1 | (b3 & 1); // 8 bit values 128=(BITWISE NOT)10000000

  // 12 bit value, low 3 bits from b1 and b2 and second lowest bit from b3
  value12 = value8 | (b1 & 7) << 9 | (b3 & 2) << 7;

}	//end of unpackTelemetry


/* ----------------------------------------------------------------------
   LOW LEVEL DEVICE IO ROUTINES - PORT TO QIODEVICE REQUIRED BEFORE COMMIT

   HIGH LEVEL IO
   int sendCommand()        - writes a command to the device
   int ()        - reads an inbound message

   LOW LEVEL IO
   rawRead() - non-blocking read of inbound data
   rawWrite() - non-blocking write of outbound data
   ---------------------------------------------------------------------- */
int sendCommand()      // writes the array
{
   return rawWrite(buf, 7); //sends the current status message
}

int getMessage()  //loads buf array
{
  int rc;

  if ((rc = rawRead(buf, 7)) > 0 && (buf[6] & 128) == 0) {

    // we got 7 bytes but no sync - need to sync
    while ((buf[6] & 128) == 0 && rc > 0) {
      rc = rawRead(&buf[6], 1);	//read byte by byte until sync is found
    }

  }
  return rc;
}

int rawWrite(uint8_t *bytes, int size) 
{
  int rc = 0;
  rc = Serial.write(bytes, size);
  return rc;
}

int rawRead(uint8_t bytes[], int size)
{
  int rc = 0;
  int timeout = 0, i = 0;
  uint8_t byte;

  // read one byte at a time 
  for (i = 0; i < size; i++) {
    timeout = 0;
    rc = 0;
    while (rc == 0 && timeout < CT_READTIMEOUT) {
      rc = Serial.readBytes( &byte, 1);
      if (rc == -1) return -1; // error!
      else if (rc == 0) {
        delay(50); // sleep for 1/20th of a second
        timeout += 50;
      } else {
        bytes[i] = byte;
      }
    }
    if (timeout >= CT_READTIMEOUT) return -1; // timed out
  }

  return i;
}
void WriteData(uint8_t *buf, uint8_t size)
{
  Serial.write(buf, size);  //write binary data to port
}

//FIN----------------------------------------------------
