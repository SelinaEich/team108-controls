#include <Servo.h>
#include <Wire.h>
#include <avr/sleep.h>

#define address 0x1E  //0011110b, I2C 7bit address of HMC5883
#define ServoPin 10
#define SolenoidPin 9
#define Reed 2 
#define Switch 2
#define deltaSw 2.4737 // (in inches per reed trigger) Circumference/#Magnets or 2*pi*wheelRaduis/numMagnets, MUST BE A DECIMAL NUMBER!
#define PotPin A0
#define buttPin 5 //Button to start executing code is in pin 5

Servo myServo;                      // compass servo

/*IDEAS FOR OPTIMIZATION: max steering angle? easiest: piston fire interval? best/coolest: control gain?!*/

/*********************** Global variables go here ***************************/
/* Magnetometer Variables*/
int   magX, magY, magZ;             // triple axis data
float theta;                        // magnetometer angle in PCB plane (around Z-axis) in degrees, Servo.write(theta) turns robot towards magnetic north, theta is the angle/direction that the robot is currently pointing
float thetaOffset = -191;             // a value of -191 makes 0 degrees represent North, UPDATE THIS!!!!
/*update thetaOffset*/

float servoBearing = 135;          // where you want the robot to point
float servoAngle;                 // desired change in servo angle

/*Servo Variables*/
int servoPosition = 1;
unsigned long lastTimeServo;
int servoInterval = 500; // (in milliseconds) sets time interval at which servo adjusts

/*Servo Position Btwn 0 & 180*/
int minServoPosition = 0;
int maxServoPosition = 180;

/*Solenoid Variables*/
int solenoidState = 0; // zero is closed (valve)
unsigned long lastTimeSolenoid;
//time
int solenoidInterval = 500; // (in milliseconds) sets time interval at which solenoid fires

unsigned long startTime;
unsigned long currentTime;

/*Reed Variables*/
int lastTimeReed = currentTime;

float wheelSpeed = 0; 
float wheelSpeedCM;
float wheelSpeedSEC;
int numReedTriggers = 0;

volatile unsigned long lastInterrupt;
long debounceTime = 15;

/*Robot Specification Variables*/
const float pi = 3.14159;
float a = 5.5; // (in inches) distance between rear wheels
float b = 5.5; // (in inches) distance between rear axle and front wheel 
float backWheelRadius = 1.25; // (in inches) roller blade back wheel radius

/*Position Variables*/
float distanceX = 0; // (in inches) distance traveled in the x-direction
float distanceY = 0; // (in inches) distance traveled in the y-direction
float distance = 0; // (in inches) distance traveled
float currentPosX; // current x-position (NEW)
float currentPosY; // current y-position (NEW)
float startingPosX; // starting x-position (will vary depending on selected starting position) (NEW)
float startingPosY; // starting y-position (will vary depending on selected starting position) (NEW)


/*Error Variables*/

float error_p_dist; // distance from current position to target position (NEW)
float p_angle; // direction from robot to target (NEW)
float alpha_p; // error between orientation (theta) and error angle (p_angle) (NEW)


float lead_angle; // direction of lead target (NEW)
float alpha_L; // error between orientation (theta) and lead_angle (NEW)
float error_theta; // error between estimated and desired orientation (NEW)
float error_speed; // error between estimated and desired speed (NEW)

/*Potentiometer Variables*/
int startingBlock; // notes starting block position 0 or 1


/******************************* setup *************************************/
void setup() {
  
  pinMode (buttPin,INPUT_PULLUP); //reads 5V until button is pressed again, note: logic is inverted
  while(digitalRead(buttPin));
   
  delay(5000); // Delay the code 15 seconds (15000 ms)
  startTime = millis(); 
  
  myServo.attach(ServoPin); // Attach the servo to pin ServoPin
  myServo.write(minServoPosition);
  
  pinMode(SolenoidPin, OUTPUT);

  /* Initialize Serial and I2C communications */
  Serial.begin(9600);
  Wire.begin();
  /* Put the HMC5883 IC into the correct operating mode */
  Wire.beginTransmission(address);  //open communication with HMC5883
  Wire.write(0x02);                 //select mode register
  Wire.write(0x00);                 //continuous measurement mode
  Wire.endTransmission();
  
  pinMode (Reed,INPUT_PULLUP); //reads 5V until switch turns off, "unfloat" the switch when open for a reliable reading, note: logic is inverted
  attachInterrupt(digitalPinToInterrupt(Switch),debounceInterrupt,FALLING);
  pinMode(PotPin, INPUT);
}

/************************* debounce interrupt ********************************************/
void debounceInterrupt() {
  if((long)(micros() - lastInterrupt) >= debounceTime * 1000) { 
    // checks time since last interrupt, if < debounceTime do nothing, if > debounceTime call readReed()
    readReed();
    lastInterrupt = micros();
  }
}

/*********************** read the Reed Switch & Update wheelSpeed ***************************/
void readReed() {
  currentTime = millis(); 
  int timeDiff = currentTime - lastTimeReed;
  wheelSpeed = deltaSw / timeDiff; // (in inches per millisecond)
  wheelSpeedCM = wheelSpeed * 2.54; // convert (in per ms) to (cm per ms)
  wheelSpeedSEC = wheelSpeedCM / .001; //convert to cm/s
  numReedTriggers++; // number of times a magnet has passed the reed switch
  //Serial.print("Speed (cm/s):"); Serial.print("\t"); 
  Serial.println(wheelSpeedSEC);
  //Serial.print("Distance Traveled:"); Serial.print("\t"); Serial.println(numReedTriggers*deltaSw); // print distance traveled
  
  /*Print for Troubleshooting*/
 /* Serial.print("deltaSw:"); Serial.print("\t"); Serial.println(deltaSw);
  Serial.print("timeDiff:"); Serial.print("\t"); Serial.println(timeDiff);
  Serial.print("wheelSpeed:"); Serial.print("\t"); Serial.println(wheelSpeed);
  Serial.print("numReedTriggers:"); Serial.print("\t"); Serial.println(numReedTriggers); */

  estimate(); //call estimate function to update position each time the reed triggers
  lastTimeReed = currentTime;
}

/************************** main loop **************************************/
void loop() {
  currentTime = millis(); //measures current time of the system
  
  /*Serial.print("currentTime:"); Serial.print("\t"); Serial.println(currentTime);
  Serial.print("startTime:"); Serial.print("\t"); Serial.println(startTime);
  Serial.print("currentTime - startTime:"); Serial.print("\t"); Serial.println(currentTime - startTime);*/
  
  if (currentTime - startTime >= 59000){ //Stop actuating 59 seconds after the code starts running (after delay)
    cli();
    sleep_enable();
    sleep_cpu();
  }
     
  /* Update the sensor readings and solenoid state */
  updateMag();
  updateSolenoid(); 
  
/* Sweeps the servo to check that it functions */
 /* Serial.print("currentTime:");   Serial.print("\t"); Serial.println(currentTime);
  Serial.print("lastTimeServo:"); Serial.print("\t"); Serial.println(lastTimeServo);
  Serial.print("servoInterval:"); Serial.print("\t"); Serial.println(servoInterval);
  Serial.print("servo position"); Serial.print("\t"); Serial.println(servoPosition);*/
  
 /* if ((currentTime - lastTimeServo) >= servoInterval) {//checks if enough time passed since the last time the servo moved
    if (servoPosition > maxServoPosition) {
      servoPosition = minServoPosition;
      Serial.println("sweep left");
    } else  if (servoPosition <= minServoPosition){
      servoPosition = maxServoPosition;
      Serial.println("sweep right");
    }*/
  /*  myServo.write(servoPosition);
    servoPosition++;
    lastTimeServo = currentTime;*/
    /*Serial.println("lastTimeServo");*/
  //}


  /* Tell robot which starting block it is on (currently just x-direction)*/
  /*startingBlock = analogRead(PotPin) / 513;
  Serial.print ("Starting Block:"); Serial.print("\t"); Serial.println(startingBlock);

  if (startingBlock == 0) {
    servoBearing = atan(3./5.); //insert info for starting block 0
  } else if (startingBlock == 1) {
    servoBearing = atan((3. - (1./3.)) / (5. - .5)); //insert info for starting block 1
  }

  /* update servo position */
  servoAngle = servoBearing - theta; //where you want robot to point - where it currently points = desired change in servo angle
  servoAngle = modulo(servoAngle, 360.); //make sure servoAngle is between 0 and 360 degrees,*/ 
  /* does mod absolute value of #s?/take negative numbers into account? NO! See below(?). */
  /*  float turn;
   *  if (servoBearing > -180 && servoBearing <= 180){
   *    turn = servoBearing;
   *  }else if(servoBearing > 180){
   *    turn = servoBearing - 360;
   *  }else if(servoBearing <= -180){
   *    turn = servoBearing + 360;
   *  }
   */
  
  if (servoAngle > 270){ //test and adjust number to servo range
    servoAngle = 0;
  }
  else if (servoAngle > 180){ //test and adjust number to servo range
    servoAngle = 180;
  }
  
  myServo.write(servoAngle); // input can be between 0 and 180 degrees (note abovementioned changes)*/
  
 /*print for troubleshooting*/
  // Serial.print("Theta:"); Serial.print("\t"); Serial.println(theta);
  // Serial.print("Servo Angle:"); Serial.print("\t"); Serial.println(servoAngle);
  //Serial.print("Servo Position:"); Serial.print("\t"); Serial.println(servoPosition);
  //Serial.print("Solenoid State:"); Serial.print("\t"); Serial.println(solenoidState);
}

/******************** update magnetometer function ************************/
void updateMag(void) {
  /* Tell the HMC5883L where to begin reading data */
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  /* Read data from each axis, 2 registers per axis */
  Wire.requestFrom(address, 6);
  if (6 <= Wire.available()) {
    magX = Wire.read() << 8; //X msbs // Bitshift
    magX |= Wire.read();    //X lsb
    magZ = Wire.read() << 8; //Z msb
    magZ |= Wire.read();    //Z lsb
    magY = Wire.read() << 8; //Y msb
    magY |= Wire.read();    //Y lsb
  }

  /* print for troubleshooting */
    //Serial.print("X:"); Serial.print("\t"); Serial.print(magX); Serial.print("\t");
    //Serial.print("Y:"); Serial.print("\t"); Serial.print(magY); Serial.print("\t");
    //Serial.print("Z:"); Serial.print("\t"); Serial.print(magZ); Serial.print("\t");*/

    /* Calculate magnetometer angle */
  theta = 180. / pi * atan2(magY, magX) + thetaOffset; // convert to degrees, apply offset
  theta = modulo(theta, 360.); // ensure that theta is between 0 and 360 degrees
}

/************************* Update Solenoid *********************************/
void updateSolenoid(){ 
    if ((currentTime - lastTimeSolenoid) >= solenoidInterval && solenoidState == HIGH) {
      solenoidState = LOW;
      digitalWrite(SolenoidPin, solenoidState); // Opens the solenoid valve
      lastTimeSolenoid = currentTime;
      //Serial.println("SolenoidOff");
    } 
    if ((currentTime - lastTimeSolenoid) >= solenoidInterval && solenoidState == LOW) {
      solenoidState = HIGH; 
      digitalWrite(SolenoidPin, solenoidState); // Closes the solenoid valve
      lastTimeSolenoid = currentTime;
      //Serial.println("SolenoidOn");
    }
    //Serial.println("Solenoid");
    //Serial.print("solenoidInterval:"); Serial.print("\t"); Serial.println(solenoidInterval);
}

/************************* position estimate/distance traveled update *************************************/
void estimate(){
  /*assume deltaSw is >>1, therefore neglect curvature and use trig. to estimate x & y distances traveled*/
  distanceX += cos(theta) * deltaSw; // updates distance traveled in x-direction since last reed trigger
  distanceY += sin(theta) * deltaSw; // updates distance travled in y-direction since last reed trigger
  distance += deltaSw; // update distance traveled since last reed trigger

  float distanceCM = (distance * 2.54); //convert distance traveled from inches to cm
  //Serial.print("Distance Traveled (in cm):"); Serial.print("\t"); Serial.println(distanceCM);
}

/************************* modulo for floats *******************************/
float modulo(float dividend, float divisor) {
  float quotient = floor(dividend/divisor); // find quotient rounded down
  return dividend - divisor*quotient; // return the remainder of the divison
}