/*
treadmill_hiddenRewardRFID_(date).ino
Clay 2016

Serial code based on arduino SoftwareSerialExample

NOTE: 
This version is for Arduino UNO
(thus uses software serial for second serial port/rotary encoder)


111616
Added RFID code based upon Bildr tutorial.

112816
made state machine for reward solenoid
Turns on reward for lick during reward zone,
then checks after each rotary input to see if it's over.
Also records licks not in reward zone.

ToDo:
- make sure reward ends even if no rotary input
- make sure checkLicks doesn't interfere with rotary input

120116

Changing serial inputs to hardware serial.
New RFID code from
http://playground.arduino.cc/Code/ID12
Now working, after a few fixes.

ToDo:
- fix checkLicks() to eliminate multiple triggers
- format data for JSON?
- output times for lick events

120816
- track calibration
- button to start and stop

ToDo:
- fix checkLicks() to eliminate multiple triggers

121416
- reversing rotary input direction to match treadmill
- fixed multiple licks by debouncing input because it tended to spontaneously read 0 during a continuous hold

121516
- changed serial output for licks, read tag, and reward (put millis on same line)
- Reward currently only triggers if animal is moving (because it only checks zone within rotary serial input function): NEED TO FIX THIS?
- should add in non-operant reward in zone for training

121616
- made option for velocity reward (isVelRew = 1)
- changed reward solenoid pins to correct ones
- current problem: if velocity reward is on when RFID is read, it stays on: FIXED by resetting prevRew at lap too
- note that currently, reward zone will be active unless prev velocity reward since lap: FIXED by not activating reward zone for isVelRew


011717
- changed RFID code to simpler version, after correcting
  the problem with the tag string (was set to 13 bytes not 12)

030617
Changing to Sebnem experiment parameters
- mult reward zones
- if animal licks in reward zone, can lick for 3sec for additional rewards


070817, 111017
Adding start trig and sync for Inscopix imaging.

122817
This version now using MPR121 for lick sensing.
Changed ledPin to 33 to allow 13 for MPR IRQ pin.

012219
Making option for random reward positions each lap (around line 580).
ToDo:
- finish rew randomization
- output reward locations to processing each lap for plotting

012419
- including maxNumZoneRew for max rew per zone
- not currently using multiple licks for rew trig (can change condition in checkLicks)
- ~line 627 add in output to rotary encoder after lap tag to reset position
- ~line 676 comparing RFID tags, line 573 reading RFID tags
- updatePosition can be changed to read currPos from rotary encoder, ~line 350

013019
- changed a few things and it's basically working, but for some
reason it's not recognizing first reward zone of a series
- look aroudn line 600- no actually 386

120521
- fixed a few problems with reward randomization
- now doesn't print ", " after last reward zone
- fixed int randInd = random(numZones); (i.e. not random(numRew))
- Note that rewPosArr is still not sorted, but it doesn't matter
  because each zone is checked at each updatePos()
- and sorting an array in C is actually not simple(?)
 
021022
- now using ESP32 rotary encoders at 115200 baud serial
- and can then change rotary pins on Esp32Rotary code and thus standardize direction
- so now don't have to use box number var (so commenting out)
 
 */


/// VARIABLES TO CHANGE EXPERIMENT PARAMS
String programName = "treadmill_hiddenRewMultTrigMPRrand_021022a";

int rewDur = 100; 
//int boxNum = 2; // determines polarity of rotary encoder forward/backward
int isMPR = 1;  // 1 if you're detecting licks with MPR121
int isButtonStart = 0;  // this means that prog doesn't print out position data until button is pressed
int isOperant = 1;  // animal has to lick to get reward
int isVelRew = 0;   // reward if animal reaches a certain min velocity (in rotary clicks)
float velThresh = 150;  // for velRew
int velRewTimeout = 2000;

// mult rew locations
int numRew = 8; //8; // number of reward zones
int rewWidth = 450;  // width of reward zone
int rewPosInds[]= {0,1,2,3,4,5,6,7}; //,12}; // indices of reward zones (zero based)
int rewPosArrMaster[] = {100,500,1000,1500,2000,2500,3000,3500}; //, 3600}; // possible start positions of reward zones
int numZones = 8; // the length of rewPosArrMaster
int isRandRew = 1;
int rewPosArr[] = {100,500,1000,1500,2000,2500,3000,3500};
int maxNumZoneRew = 5;

int isCalibrated = 2; // =2 if calibrated, 0 normally at start
int trackLength = 4000;//0;
int startSession = 1;

//int rewBegArr[15] = [
long rewZoneStartTime = 0;
int rewZone; // = 0;
int prevRewZone = 0;
int rewZoneOptTime = 10000;  // ms after entering rewZone animal has the option to lick for rew
long interLickInt = 2;  // make really large if you want single lick choice (but might have to use 'long' var)
int rewTimeout = 30; //5000; // timeout for multiple reward licks within a single zone/epoch
int numZoneRew = 0;
int lastRewZone = 100;

int rewBegPos = 1000;
int rewEndPos = 2000;

// sync pulse params
int syncDur = 500;  // duration of pulse
int syncIntv = 5000;  // interval of pulse train

// MPR121 setup
#include "mpr121.h"
#include <Wire.h>

int irqpin = 13;  // Digital 2
boolean touchStates[12]; //to keep track of the previous touch states

// Other pins
int lickPin = 49;
int spkrPin = 47;
int ledPin = 7;//33;

int buttonPin1 = 30;
int trigPin = 52;  // pin for triggering nVista imaging
int syncPin = 53; // pin for nVista sync
int syncPin2 = 13;  // LED for video sync

int solPin1 = 5; //5;
int solPin2 = 4; //6;

// times
long startTime = 0;
long endTime = 0;

//
int currPos = 0;
int prevPos = 0;
int dy = 0;
float vel = 0;


// rotary encoder serial input variables
char inMess;
char outMess;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

int yInd = 0;
int mInd = 0;
String dyStr = "";
String msStr = "";
long dt = 0;
long reTime = 0;
long prevReTime = 0;
//int currPos = 0;

long syncStartTime = 0;
long prevSyncTime = 0;
int prevSync = 0;
long lastSyncTime = 0;

// RFID vars
int RFIDResetPin = 50;
//char tag1[12] = "1E009A4067A3";  // 120116: not checking tag value right now

char tagString[12];
int indNum = 0;
boolean reading = false;
int ind = 0;


// initialize times and counts
int prevRew = 0;
long rewStartTime = 0;


int prevLick = 0;
long lickTime = 0;
long prevLickTime = 0;
int lickState = 0;
//int prevRew = 0;
int lickStateArr[6];
int numReading = 0;
int lickStateSum = 0;
int justLicked = 0;  // for MPR

long buttonTime = 0;
long prevButtonTime = 0;

int isRewZone = 0;
int rewZoneSum = 0;

// SETUP ///////////////////
void setup()
{
  // set up pins
  pinMode(lickPin, INPUT);
  //pinMode(spkrPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(solPin1, OUTPUT);
  pinMode(solPin2, OUTPUT);
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(syncPin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(syncPin2, OUTPUT);

  digitalWrite(solPin1, LOW);

  // MPR121
  pinMode(irqpin, INPUT);
  digitalWrite(irqpin, HIGH); //enable pullup resistor
  Wire1.begin();
  mpr121_setup();
  
  // Open serial communications
  Serial.begin(38400);
  //NOTE: need to delay if using ESP32
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.println(programName);

  // set the data rate for the rotary encoder port
  // NOTE: must match the rate of the rotary encoder nano
  Serial3.begin(115200);//(19200);
  pinMode(15, INPUT); // was having problems and a link suggested this
  digitalWrite(15, HIGH);
  //Serial3.println("Hello, world?");// reserve 200 bytes for the inputString:
  inputString.reserve(200);

  // RFID serial port
  Serial1.begin(9600);
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  pinMode(19, INPUT);
  digitalWrite(19, HIGH);
  pinMode(RFIDResetPin, OUTPUT);
  digitalWrite(RFIDResetPin, HIGH);

  // Now print out some header information with behavioral program parameters
  Serial.println("numRew=" + String(numRew)); // number of reward zones
  Serial.println("rewWidth=" + String(rewWidth));  // width of reward zone
  Serial.println("rewZoneOptTime=" + String(rewZoneOptTime));  // ms after entering rewZone animal has the option to lick for rew
  Serial.println("interLickInt=" + String(interLickInt));  // make really large if you want single lick choice (but might have to use 'long' var)
  Serial.println("rewTimeout=" + String(rewTimeout)); 

  Serial.print("rewPosArr={");
  for (int i=0; i<numRew; i++) {
    if (i!=numRew-1) {
    Serial.print(String(rewPosArr[rewPosInds[i]]) + ", ");}
    else {Serial.print(String(rewPosArr[rewPosInds[i]]));}
  }
  Serial.println("}");

//  Serial.print("rewBegPos=");
//  Serial.println(rewBegPos);
//  Serial.print("rewEndPos=");
//  Serial.println(rewEndPos);

  Serial.print("isRandRew=");
  Serial.println(isRandRew);

  Serial.print("rewDur=");
  Serial.println(rewDur);
  Serial.print("isButtonStart=");
  Serial.println(isButtonStart);
  Serial.print("isOperant=");
  Serial.println(isOperant);
  Serial.print("isVelRew=");
  Serial.println(isVelRew);
  Serial.print("velThresh=");
  Serial.println(velThresh);
  Serial.println("END HEADER");

  randomSeed(analogRead(A4));
  
}  // end SETUP

// start LOOP  ///////////////////////////
void loop() // run over and over
{



    // read rotary encoder input
    serialEvent1(); //call the function
    
//    // print the string when a newline arrives:
//    if (stringComplete) {
//      //Serial.println(inputString);
//      updatePosition();  // update position and see if this is a reward zone
//      
//      // clear the string:
//      inputString = "";
//      stringComplete = false;
//    }
      
    // see if you need to turn off reward valve (for any given reward)
    if (prevRew == 1) {
      checkRewState();
    }
  
    // read RFID input
    serialEvent2();
  
    checkLicks();

    if (isButtonStart == 1 && isCalibrated == 2) {
      checkButton();
    }
    if (startSession == 1) {
      checkSyncState();
    }

}  // end LOOP


// SUBFUNCTIONS ////////////////////



// use rotary input to update position (and check if reward zone)
void updatePosition() {

    //prevPos = currPos;
    
    yInd = inputString.indexOf("dy"); // ind of "dy" in rotary encoder message
    mInd = inputString.indexOf("millis");

    dyStr = inputString.substring(yInd+4, mInd-3); 
//    Serial.println(dyStr);
    dy = dyStr.toInt();
//    if (boxNum==1) {
//      dy = -dy;  // reverse direction to match treadmill
//    }
    msStr = inputString.substring(mInd+8, inputString.length()-1); 
    reTime = msStr.toInt();
    dt = reTime - prevReTime;
//    Serial.println(dt);
//    Serial.println(reTime);
//    Serial.println(dy);
    prevReTime = reTime;

    vel = float(dy)/float(dt)*1000;
    currPos = prevPos + dy;
    prevPos = currPos;

    // only print output and check reward zone after track calibrated and session started
    if (isCalibrated == 2 && startSession == 1) {

      Serial.print("vel=" + String(vel) + ", ");
      Serial.println("currPos=" + String(currPos) + ", millis=" + String(millis()));
  
  //    if (prevRew == 1) {
  //      checkRewState();
  //    }

      // see if mouse is in any reward zone (but need to change so he can't go back through)
      rewZoneSum = 0;
      for (int i=0; i<numRew; i++) { // check through all possible reward zones
        if (currPos>=rewPosArr[i] && currPos<=rewPosArr[i]+rewWidth) {  // if currPos is in a rewZone
          rewZone = i;
          if (prevRewZone==0 && rewZone!=lastRewZone) {  // if new entry into new zone (this prevents re-entry rewards)
            rewZone = i;
            lastRewZone = i;
            rewZoneStartTime = millis();
            Serial.println("rewZone=" + String(rewZone) + ", millis=" + String(rewZoneStartTime));
//            isRewZone = 1;
//            prevRewZone = 1;
//            rewZoneSum = rewZoneSum +1;
            //break;
          }
          isRewZone = 1;
          prevRewZone = 1;
          rewZoneSum = rewZoneSum +1;
        }
//        else {  // else if not in a reward zone
//          isRewZone = 0;
//          prevRewZone = 0;
//          
//        }
      }

      if (rewZoneSum == 0) {  // else if not in any reward zone
          isRewZone = 0;
          prevRewZone = 0;
          numZoneRew = 0;
      }
  
      if (isVelRew == 0 && isRewZone == 1) {    // currPos >= rewBegPos && currPos <= rewEndPos) {
        if (isOperant == 0 && prevRew == 0) {
          prevRew = 1;
          giveReward();
        }
//        else if (digitalRead(lickPin) && prevRew == 0) {
//          prevRew = 1;
//          giveReward();
//        }

        digitalWrite(ledPin, HIGH); // just turn on LED while in rew zone
        //Serial.println("reward ON: " + String(millis()));
  //      delay(1000);
  //      digitalWrite(ledPin, LOW);
      }  // end IF in rewZone
      else {
        digitalWrite(ledPin, LOW);
        //Serial.println("reward OFF: " + String(millis()));
        //prevRew = 0;  // reset reward if not in rew zone
        isRewZone = 0;
      }

      // if pos is more than estimate of track length, start new lap
      if (currPos >= trackLength + 100) {
        prevPos = 0;
        prevRew = 0;  // reset reward
      }

      // if velRew and vel is high enough, give reward
      if (isVelRew == 1 && vel > velThresh && millis()-rewStartTime > velRewTimeout) {
        giveReward();
        prevRew = 1;
      }

      /// ADD NEW POSITION EVENTS HERE
      /// e.g.  if (currPos >= eventPos1 && currPos < eventPos1+zoneWidth) {tone(pin, freq);} else {noTone(pin);}
      ///
      ///
      
    } // end IF isCalibrated
//    else {  // if track not calibrated, then calibrate
//      Serial.println("begin calibration");
//      
//    }
}

////////////////////////////////////////////
void checkLicks() {
  if (isMPR == 0) {
    //Serial.println(digitalRead(lickPin));
    ++numReading;
    lickState = digitalRead(lickPin);
    lickStateSum = lickStateSum + lickState;
  
    if (numReading == 5) {  // every 5 readings
      if (lickStateSum > 4) { // if all readings were lick ON
        if (prevLick == 0) { // if new lick
          prevLick = 1;
          lickTime = millis();
          Serial.print("lick, millis=");
          Serial.println(lickTime);
  
          if (isOperant == 1 && isRewZone == 1 && prevRew == 0 && millis()-rewZoneStartTime<=rewZoneOptTime && millis()-rewStartTime>rewTimeout && numZoneRew<maxNumZoneRew) {  //  && lickTime-prevLickTime<interLickInt
            prevRew = 1;
            numZoneRew = numZoneRew +1;
            giveReward();
          }
        }
      }
      else if (lickStateSum == 0) {  //if (prevLick == 1 && lickState == 0 && millis()-lickTime>50) {
        prevLick = 0;
      }
      numReading = 0;
      lickStateSum = 0;
    }
  //  Serial.println(prevLick);
  //  delay(10);
  }
  else { // else if using MPR121 touch sensor
    readTouchInputs();
    
    if (justLicked == 1 && prevLick == 0) { 
      prevLick = 1;
      lickTime = millis();
      Serial.print("lick, millis=");
      Serial.println(lickTime);

      if (isOperant == 1 && isRewZone == 1 && prevRew == 0 && millis()-rewZoneStartTime<=rewZoneOptTime && millis()-rewStartTime>rewTimeout  && numZoneRew<maxNumZoneRew) {  //  && lickTime-prevLickTime<interLickInt
        prevRew = 1;
        numZoneRew = numZoneRew +1;
        giveReward();
      }
    }
  }  // END if justLicked == 1
  
}  // END readTouchInputs();

//////////////////////////////////////
// reward functions
void giveReward() {
  rewStartTime = millis();
  digitalWrite(solPin1, HIGH);
  Serial.print("REWARD, millis=");
  Serial.println(rewStartTime);
}

void checkRewState() {
  if (prevRew == 1 && millis()-rewStartTime>=rewDur) {
    digitalWrite(solPin1, LOW);
    prevRew = 0;
  }
}



/////////////////////////////////////
/* ROTARY ENCODER SERIAL INPUT
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent1() {
  while (Serial3.available()) {
    // get the new byte:
    char inChar = (char)Serial3.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      //Serial.println(inputString);
      updatePosition();  // update position and see if this is a reward zone
      
      // clear the string:
      inputString = "";
      stringComplete = false;
    }
  }
}

////////////////////////////////////////
// RFID input
void serialEvent2() {
  // now look for RFID serial
  //char tagString[12];
  //int ind = 0;
  //boolean reading = false;

  if(Serial1.available() > 0) {
    //Serial.println("Reading tag");
    while(Serial1.available()){

      int readByte = Serial1.read(); //read next available byte
  
      //Serial.println(readByte);
      
      if(readByte == 2) {
        reading = true; //begining of tag
        //Serial.println("Reading tag");
      }
      if(readByte == 3) reading = false; //end of tag
  
      // NOTE: ASCII 10 is newline and 13 is carriage return
      if(reading && readByte != 2 && readByte != 10 && readByte != 13){ // && ind <13){
        //store the tag
        tagString[ind] = readByte;
        ind ++;
      }  // end IF data byte then put in array
    } // end WHILE Serial.available

      // Output to Serial:
      //Serial.println(tagString);

    if (ind == 12) {                          // if 12 digit read is complete

      Serial.print("read tag, millis=");
      Serial.println(millis());
      Serial.print("tagString=");
      Serial.println(tagString);
      //currPos=0;
      prevPos = 0;    // now just resetting position if any tag is read
      prevRew = 0;  // reset reward
      digitalWrite(solPin1, LOW);  // 121616: added this in case velocity reward ON during lap reset

      // randomize reward positions?
      if (isRandRew == 1) {

        // trick for generating random rewPosInds (switch each index with random other one, so no repeats)
        for (int i= 0; i< numZones; i++) {
             int randInd = random(numZones); // gen random number
             int t = rewPosInds[i];  // load in index
             rewPosInds[i] = rewPosInds[randInd]; // change this index to random other one
             rewPosInds[randInd] = t; // make the random index equal to other one (i.e. switch)
        }

        // now use random reward positions this lap (and print this array)
        Serial.print("rewPosArr={");
        for (int j=0;j<numRew;j++) {
         rewPosArr[j] = rewPosArrMaster[rewPosInds[j]];
         Serial.print(rewPosArr[j]);
         if (j!=numRew-1) {
          Serial.print(", ");
         }
         else {
          Serial.println("}");
         }
        }
      }
      
      if (isCalibrated == 0) {
        isCalibrated = 1;
        Serial.println("starting track calibration");
      }
      else if (isCalibrated == 1) {
        isCalibrated = 2;
        Serial.println("calibration complete");
        trackLength = currPos;
        Serial.print("track length=");
        Serial.println(trackLength);

        // send this to rotary encoder
        // Serial1.print("lap");

        if (isButtonStart==0) { 
          startSession = 1;
          Serial.print("START SESSION, millis=");
          Serial.println(millis());
        }  // end IF isButtonStart
      }  // end elseif isCalibrated

      ind = 0;
    }  // end IF ind = 12

  }  // end IF Serial.available
}  // end VOID serialEvent


void checkTag(char tag[]){
///////////////////////////////////
//Check the read tag against known tags
///////////////////////////////////

  if(strlen(tag) == 0) return; //empty, no need to contunue

  //if(compareTag(tag, tag1)){ // if matched tag1, do this
    //lightLED(2);

  //}else{
    Serial.println(tag); //read out any unknown tag
//    digitalWrite(ledPin, HIGH);
//    delay(1000);
//    digitalWrite(ledPin, LOW);
    currPos = 0;
  //}

}

void clearTag(char one[]){
///////////////////////////////////
//clear the char array by filling with null - ASCII 0
//Will think same tag has been read otherwise
///////////////////////////////////
  for(int i = 0; i < strlen(one); i++){
    one[i] = 0;
  }
}

boolean compareTag(char one[], char two[]){

  if (strcmp(one, two)) {
    Serial.println("yes it's correct");
  }

  return true; //no mismatches
}

//////////////////////////////
void checkButton() {
  if (digitalRead(buttonPin1) == 0) {
    if (millis() - prevButtonTime > 2000) {
    
      if (startSession == 0) {
        startSession = 1;
        startTime = millis();
        digitalWrite(trigPin, HIGH);
        Serial.print("START SESSION button, millis = ");
        Serial.println(startTime);
        Serial.print("trigTime, millis=");
        Serial.println(startTime);
        prevButtonTime = startTime;
        
      }
      else {
        startSession = 0;
        endTime = millis();
        digitalWrite(trigPin, LOW);
        digitalWrite(syncPin, LOW);
        digitalWrite(syncPin2, LOW);
        Serial.print("END session button, millis=");
        Serial.println(endTime);
        prevButtonTime = endTime;
      }
    }
  }
}

void checkSyncState() {
  if (prevSync == 1 && millis()-syncStartTime>=syncDur) {
    digitalWrite(syncPin, LOW);
    digitalWrite(syncPin2, LOW);
    prevSync = 0;
  }
  else if (millis()- lastSyncTime >= syncIntv) {
    digitalWrite(syncPin, HIGH);
    digitalWrite(syncPin2, HIGH);
    syncStartTime = millis();
    Serial.print("syncOut, millis = ");
    Serial.println(syncStartTime);
    lastSyncTime = syncStartTime;
    prevSync = 1;
  }
}

// MPR121 functions
void mpr121_setup(void){

  set_register(0x5A, ELE_CFG, 0x00); 
  
  // Section A - Controls filtering when data is > baseline.
  set_register(0x5A, MHD_R, 0x01);
  set_register(0x5A, NHD_R, 0x01);
  set_register(0x5A, NCL_R, 0x00);
  set_register(0x5A, FDL_R, 0x00);

  // Section B - Controls filtering when data is < baseline.
  set_register(0x5A, MHD_F, 0x01);
  set_register(0x5A, NHD_F, 0x01);
  set_register(0x5A, NCL_F, 0xFF);
  set_register(0x5A, FDL_F, 0x02);
  
  // Section C - Sets touch and release thresholds for each electrode
  set_register(0x5A, ELE0_T, TOU_THRESH);
  set_register(0x5A, ELE0_R, REL_THRESH);
 
  set_register(0x5A, ELE1_T, TOU_THRESH);
  set_register(0x5A, ELE1_R, REL_THRESH);
  
  set_register(0x5A, ELE2_T, TOU_THRESH);
  set_register(0x5A, ELE2_R, REL_THRESH);
  
  set_register(0x5A, ELE3_T, TOU_THRESH);
  set_register(0x5A, ELE3_R, REL_THRESH);
  
  set_register(0x5A, ELE4_T, TOU_THRESH);
  set_register(0x5A, ELE4_R, REL_THRESH);
  
  set_register(0x5A, ELE5_T, TOU_THRESH);
  set_register(0x5A, ELE5_R, REL_THRESH);
  
  set_register(0x5A, ELE6_T, TOU_THRESH);
  set_register(0x5A, ELE6_R, REL_THRESH);
  
  set_register(0x5A, ELE7_T, TOU_THRESH);
  set_register(0x5A, ELE7_R, REL_THRESH);
  
  set_register(0x5A, ELE8_T, TOU_THRESH);
  set_register(0x5A, ELE8_R, REL_THRESH);
  
  set_register(0x5A, ELE9_T, TOU_THRESH);
  set_register(0x5A, ELE9_R, REL_THRESH);
  
  set_register(0x5A, ELE10_T, TOU_THRESH);
  set_register(0x5A, ELE10_R, REL_THRESH);
  
  set_register(0x5A, ELE11_T, TOU_THRESH);
  set_register(0x5A, ELE11_R, REL_THRESH);
  
  // Section D
  // Set the Filter Configuration
  // Set ESI2
  set_register(0x5A, FIL_CFG, 0x04);
  
  // Section E
  // Electrode Configuration
  // Set ELE_CFG to 0x00 to return to standby mode
  set_register(0x5A, ELE_CFG, 0x0C);  // Enables all 12 Electrodes
  
  
  // Section F
  // Enable Auto Config and auto Reconfig
  /*set_register(0x5A, ATO_CFG0, 0x0B);
  set_register(0x5A, ATO_CFGU, 0xC9);  // USL = (Vdd-0.7)/vdd*256 = 0xC9 @3.3V   set_register(0x5A, ATO_CFGL, 0x82);  // LSL = 0.65*USL = 0x82 @3.3V
  set_register(0x5A, ATO_CFGT, 0xB5);*/  // Target = 0.9*USL = 0xB5 @3.3V
  
  set_register(0x5A, ELE_CFG, 0x0C);
  
}

boolean checkInterrupt(void){
  return digitalRead(irqpin);
}

void set_register(int address, unsigned char r, unsigned char v){
    Wire1.beginTransmission(address);
    Wire1.write(r);
    Wire1.write(v);
    Wire1.endTransmission();
}

void readTouchInputs(){
  if(!checkInterrupt()){
    
    //read the touch state from the MPR121
    Wire1.requestFrom(0x5A,2); 
    
    byte LSB = Wire1.read();
    byte MSB = Wire1.read();
    
    uint16_t touched = ((MSB << 8) | LSB); //16bits that make up the touch states

    
    for (int i=0; i < 12; i++){  // Check what electrodes were pressed
      if(touched & (1<<i)){
      
        if(touchStates[i] == 0){
          //pin i was just touched
//          Serial.print("pin ");
//          Serial.print(i);
//          Serial.println(" was just touched");
          if (i==0 && justLicked == 0) {
            justLicked = 1;
            //prevLick = 1;
          }
        
        }else if(touchStates[i] == 1){
          //pin i is still being touched
        }  
      
        touchStates[i] = 1;      
      }else{
        if(touchStates[i] == 1){
          //Serial.print("pin ");
          //Serial.print(i);
          //Serial.println(" is no longer being touched");

          if (i==0) {
            justLicked = 0;
            prevLick = 0;
          }
          
          //pin i is no longer being touched
       }
        
        touchStates[i] = 0;
      }
    
    }
    
  }
}
