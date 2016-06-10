#include <Servo.h>
#include <Wire.h>
#include <avr/sleep.h>

#define address 0x1E  //0011110b, I2C 7bit address of HMC5883
#define ServoPin 10
#define SolenoidPin 3
#define Reed 2 
#define Switch 2
#define deltaSw 2.4737 // (in inches per reed trigger) Circumference/#Magnets or 2*pi*wheelRadius/numMagnets, MUST BE A DECIMAL NUMBER!
#define PotPin A0
#define buttPin 5 //Button to start executing code is in pin 5
#define startPin 0 // Button to define starting positions

Servo myServo;                      // compass servo

/*IDEAS FOR OPTIMIZATION: max steering angle? easiest: piston fire interval? best/coolest: control gain?!*/

/*********************** Global variables go here ***************************/
/* Magnetometer Variables*/
int   magX, magY, magZ;             // triple axis data
float phi;                        // magnetometer angle in PCB plane (around Z-axis) in degrees, Servo.write(theta) turns robot towards magnetic north, theta is the angle/direction that the robot is currently pointing
/* this variable phi used to be theta, I changed it to make the code less confusing for implementing the controller */
float thetaOffset = -117;             // a value of -117 makes 0 degrees represent down the channel
/*update thetaOffset*/

float servoBearing;          // originally as servoBearing; has been changed to phi_desired to represent the desired ORIENTATION of the robot
float servoAngle;                 // desired change in servo angle; should be changed to theta_target as that is representative of the desired change in the robot's steering angle

/*Servo Variables*/
int servoPosition = 1;
unsigned long lastTimeServo;
int servoInterval = 100; // (in milliseconds) sets time interval at which servo adjusts

/*Servo Position Btwn 0 & 180*/
int minServoPosition = 0;
int maxServoPosition = 180;

/*Solenoid Variables*/
int solenoidState = 0; // zero is closed (valve)
unsigned long lastTimeSolenoid;
//time
int solenoidInterval = 200; // (in milliseconds) sets time interval at which solenoid fires

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
float backWheelRadius = 1.5748; // (in inches) roller blade back wheel radius
float theta_max = 90; // maximum change in servo angle (relative to center)

/*Position Variables*/
float distanceX = 0; // (in inches) distance traveled in the x-direction
float distanceY = 0; // (in inches) distance traveled in the y-direction
float distance = 0; // (in inches) distance traveled
float currentPosX; // current x-position (NEW)
float currentPosY; // current y-position (NEW)
float startingPosX; // starting x-position (will vary depending on selected starting position) (NEW)
float startingPosY; // starting y-position (will vary depending on selected starting position) (NEW)
float originX; // x-position of origin, to be set at the center of the channel gate (NEW)
float originY; // y-position of origin, to be set at the center of the channel gate (NEW)
float finish_lineX; // x-position of finish line, to be set at the end of the channel (NEW)
float finish_lineY; // y-position of finish line, to be set at the end of the channel (NEW)

/*Desired and Target Variables*/
float p_desiredX; // desired x-position
float p_desiredY; // desired y-position
float phi_desired; // desired orientation of robot
float theta_target; // desired change in steering angle
float target_lead = 33; // leading distance for channel control

/*Error Variables*/

float error_pX;// x-position error
float error_pY; // y-position error
float error_p_distance; // magnitude of position error
float error_phi; // error between desired and current orientations
float p_angle; // direction from robot to target (NEW)
float alpha_p; // error between orientation (theta) and error angle (p_angle)

/*Control Gain Variables*/
float K_alpha_p;
float K_phi;

/*Potentiometer Variables*/
int startingBlock; // notes starting block position (16 possibilities)

/*Start Button Variables*/
int buttonState;


/******************************* setup *************************************/
void setup() {
  Serial.begin(9600);
  
  pinMode(startPin,INPUT_PULLUP);
  pinMode(PotPin, INPUT);
  buttonState = digitalRead(startPin);
  startingBlock = floor(analogRead(PotPin) / 129);
  Serial.print("Button State:"); Serial.print("\t"); Serial.println(buttonState);
  Serial.print("Starting Block:"); Serial.print("\t"); Serial.println(startingBlock);
  start();
  Serial.print("startingPosX:"); Serial.print("\t"); Serial.println(startingPosX);
  Serial.print("startingPosY:"); Serial.print("\t"); Serial.println(startingPosY);
  
  K_alpha_p = 1; //Pat had this set to 1
  K_phi = 0;
  
  currentPosX = startingPosX; // initializes the starting x-position of the robot for future use in the code (NEW)
  currentPosY = startingPosY; // initializes the starting y-position of the robot for future in in the code (NEW)
  
  originX = 0;
  originY = 0;
  finish_lineX = 528;
  finish_lineY = 0;

    /* Initialize Serial and I2C communications */
  
  Wire.begin();
  
  pinMode (buttPin,INPUT_PULLUP); //reads 5V until button is pressed again, note: logic is inverted
  while(digitalRead(buttPin));
  myServo.attach(ServoPin); // Attach the servo to pin ServoPin
  myServo.write(90); // I assume this is where you set the servo to 90





  
   
  delay(5000); // Delay the code 15 seconds (15000 ms)
  startTime = millis(); 
  
 
  pinMode(SolenoidPin, OUTPUT);


  /* Put the HMC5883 IC into the correct operating mode */
  Wire.beginTransmission(address);  //open communication with HMC5883
  Wire.write(0x02);                 //select mode register
  Wire.write(0x00);                 //continuous measurement mode
  Wire.endTransmission();
  
  pinMode (Reed,INPUT_PULLUP); //reads 5V until switch turns off, "unfloat" the switch when open for a reliable reading, note: logic is inverted
  attachInterrupt(digitalPinToInterrupt(Switch),debounceInterrupt,FALLING);
  pinMode(PotPin, INPUT);

}
/************************* position estimate/distance traveled update *************************************/
void estimate(){
  /*assume deltaSw is >>1, therefore neglect curvature and use trig. to estimate x & y distances traveled*/
  distanceX += cos(phi) * deltaSw; // updates distance traveled in x-direction since last reed trigger
  distanceY += sin(phi) * deltaSw; // updates distance travled in y-direction since last reed trigger
  distance += deltaSw; // update distance traveled since last reed trigger
  currentPosX += distanceX; // updates the new current x-position of the robot (NEW)
  currentPosY += distanceY; // updates the new current y-position of the robot (NEW)
  
  float distanceCM = (distance * 2.54); //convert distance traveled from inches to cm
  //Serial.print("Distance Traveled (in cm):"); Serial.print("\t"); Serial.println(distanceCM);
  Serial.print("currentPosX:");Serial.print("\t");Serial.println(currentPosX);
  Serial.print("currentPosY:");Serial.print("\t");Serial.println(currentPosY);
}
/************************* modulo for floats *******************************/
float modulo(float x, float y) {
  float n = floor(x/y); // find quotient rounded down
  return x - n*y; // return the remainder of the divison
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
  phi = 180. / pi * atan2(magY, magX) + thetaOffset; // convert to degrees, apply offset
  phi = modulo(phi, 360.); // ensure that theta is between 0 and 360 degrees

   Serial.print("phi"); Serial.print("\t"); Serial.print(phi); Serial.println("\t");
  // Serial.print("servo angle:"); Serial.print("\t"); Serial.println(theta_target);
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
  steering();
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

  /* update servo position 
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
  /*
  if (servoAngle > 270){ //test and adjust number to servo range
    servoAngle = 0;
  }
  else if (servoAngle > 180){ //test and adjust number to servo range
    servoAngle = 180;
  }
  
  myServo.write(servoAngle); // input can be between 0 and 180 degrees (note abovementioned changes)*/
  
 /*print for troubleshooting
  // Serial.print("Theta:"); Serial.print("\t"); Serial.println(theta);
  // Serial.print("Servo Angle:"); Serial.print("\t"); Serial.println(servoAngle);
  //Serial.print("Servo Position:"); Serial.print("\t"); Serial.println(servoPosition);
  //Serial.print("Solenoid State:"); Serial.print("\t"); Serial.println(solenoidState);*/
}


/********** signum function **********************/
int sign(float x) {
    return (x > 0) - (x < 0);
}

/**********fix angle function***************/
float fix(float x) {
  return modulo(x+180,360)-180; // limits the input argument x to be between +/- pi (180 degrees)
}

/************* Controller (TO BE CHECKED) **********************/
void steering(){
  if((currentPosY > -33) && (currentPosY < 33)) {// determines whether or not the robot is in the channel to run the channel control law
    p_desiredX = currentPosX + target_lead;
    p_desiredY = 0;
    phi_desired = 0; 
  }
  // phi_desired was originally servoBearing; this variable refers to the desired orientation of the robot
  else  {// the statements following this else should have the robot continue to move forwards
    p_desiredX = startingPosX;
    p_desiredY = 0;
    phi_desired = -modulo(atan2(currentPosY,0)*(180/pi),360);
  }

  error_pX = currentPosX - p_desiredX; // calculates the x-position error between the current position and the desired position
  error_pY = currentPosY - p_desiredY; // calculates the y-position error between the current position and the desired position
  error_p_distance = sqrt(pow(error_pX,2)+pow(error_pY,2)); // calculates the magnitude of the error between the current position and the desired position

  p_angle = modulo(atan2(-1*error_pY,-1*error_pX)*(180/pi),360); // calculates the angle formed by the position error vector and the horizontal axis then converts to degrees
  alpha_p = fix(p_angle-phi); // calculates the error formed by the orientation of the robot and the desired position vector

  error_phi = modulo(phi_desired-phi,360); 

  theta_target = K_alpha_p*alpha_p+K_phi*error_phi; // calculation of the desired change in steering angle
 // theta_target = sign(theta_target)*min(theta_max,abs(theta_target)); // limits the orientation of the SERVO within its limits (0-180 degrees)
  theta_target = -1*theta_target;
  float servoAngle = 90 - theta_target; //Subtraction instead of addition to account for our servo being upside down
  if (servoAngle > 270 || servoAngle < 60){
    servoAngle = 60;
  }
  else if (servoAngle > 120){
    servoAngle = 120;
  }
  myServo.write(servoAngle);
}

/************** Potentiometer/Starting Position Base Code *************/
void start() {
  if (buttonState == 1){ 
    if (startingBlock == 0){ // Position A1
      startingPosX = -99;
      startingPosY = 154;
      Serial.println("Position A1");
    }
    else if (startingBlock == 1){ // Position A2
      startingPosX = -121;
      startingPosY = 132;
      Serial.println("Position A2");
    }
    else if (startingBlock == 2){ // Position A3
      startingPosX = -99;
      startingPosY = 110;
      Serial.println("Position A3");
    }
    else if (startingBlock == 3){ // Position A4
      startingPosX = -77;
      startingPosY = 132;
      Serial.println("Position A4");
    }
    else if (startingBlock == 4){ // Position B1
      startingPosX = -33;
      startingPosY = 154;
      Serial.println("Position B1");
    }
    else if (startingBlock == 5){ // Position B2
      startingPosX = -55;
      startingPosY = 132;
      Serial.println("Position B2");
    }
    else if (startingBlock == 6){ // Position B3
      startingPosX = -33;
      startingPosY = 110;
      Serial.println("Position B3");
    }
    else if (startingBlock == 7){ // Position B4
      startingPosX = -11;
      startingPosY = 132;
      Serial.println("Position B4");
    }
  }
  else if(buttonState == 0) {
    if (startingBlock == 0){ // Position C1
      startingPosX = -99;
      startingPosY = -110;
      Serial.println("Position C1");
    }
    else if (startingBlock == 1){ // Position C2
      startingPosX = -121;
      startingPosY = -132;
      Serial.println("Position C2");
    }
    else if (startingBlock == 2){ // Position C3
      startingPosX = -99;
      startingPosY = -154;
      Serial.println("Position C3");
    }
    else if (startingBlock == 3){ // Position C4
      startingPosX = -77;
      startingPosY = -132;
      Serial.println("Position C4");
    }
    else if (startingBlock == 4){ // Position D1
      startingPosX = -33;
      startingPosY = -110;
      Serial.println("Position D1");
    }
    else if (startingBlock == 5){ // Position D2
      startingPosX = -55;
      startingPosY = -132;
      Serial.println("Position D2");
    }
    else if (startingBlock == 6){ // Position D3
      startingPosX = -33;
      startingPosY = -154;
      Serial.println("Position D3");
    }
    else if (startingBlock == 7){ // Position D4
      startingPosX = -11;
      startingPosY = -132;
      Serial.println("Position D4");
    }
  }
  }
