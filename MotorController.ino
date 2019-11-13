//Varuables for motorController A
//---------------------------------
int currentPos_A = 0; //goto_Pos to hold in motorMode 0
int goto_Pos_A = 0; //goto_Pos to go to in motorMode 1
int last_Pos_A = 0; //Variable to hold the last possition
int current_speed_A = 0; //Variable to hold the speed value
int desired_speed_A = 0; //Variable to hold the decired speed
int speed_Calc_A = 0;

//Varuables for motorController B
//---------------------------------
int currentPos_B = 0; //goto_Pos to hold in motorMode 0
int goto_Pos_B = 0; //goto_Pos to go to in motorMode 1
int last_Pos_B = 0; //Variable to hold the last possition
int current_speed_B = 0; //Variable to hold the speed value
int desired_speed_B = 0; //Variable to hold the decired speed
int speed_Calc_B = 0;

//General PID controller values
//-----------------------------
volatile boolean mode_0_set_A = false;
volatile boolean mode_1_set_A = false;
volatile boolean mode_2_set_A = false;
volatile boolean mode_3_set_A = false;
volatile boolean mode_4_set_A = false;
volatile boolean mode_0_set_B = false;
volatile boolean mode_1_set_B = false;
volatile boolean mode_2_set_B = false;
volatile boolean mode_3_set_B = false;
volatile boolean mode_4_set_B = false;
volatile boolean mode_A_Done = false;
volatile boolean mode_B_Done = false;

//General PID controller values
//-----------------------------
int sampleTime = 100; //10ms
double errorVal = 0.0; //The direct error value
double errorSum_A = 0.0; //The sum of error over time
double errorSum_B = 0.0; //The sum of error over time
double errorD = 0.0;//Change of error since last
double errorLast_A = 0.0;//The error at the last run
double errorLast_B = 0.0;//The error at the last run
double P_POS = 60.0; //Position P
double I_POS = 0.0001; //Position I
double D_POS = 200.0; //Position D
double P_SPD = 160.0; //Speed P
double I_SPD = 0.5; //Speed I
double D_SPD = 5.0; //Speed D
int PID_A = 0; //PID output control value for motor A
int PID_B = 0; //PID output control value for motor B
double Iterm = 0; //Used as part of the speed controller
int errorPos_A = 0; //Value holding the possition error in mode 4
int errorPos_B = 0; //Value holding the possition error in mode 4


//Variables for tick counter on motor A
//-------------------------------------
volatile boolean ENC_Trig_A1 = false; //Varuable indicating if encoder A is triggered
int ENC_A1 = 14; //Pin for encoder 1 on Motor A (need pin 2 for interrupt)
int ENC_A2 = 2; //Pin for encoder 2 on Motor A
int val_ENC_A1 = 0; //Value of pin 6
int val_ENC_A2 = 0; //Value of pin 7
int posVal_A = 0;

//Variables for tick counter on motor B
//-------------------------------------
volatile boolean ENC_Trig_B1 = false; //Varuable indicating if encoder A is triggered
int ENC_B1 = 15; //Pin for encoder 1 on Motor A (need pin 3 for interrupt)
int ENC_B2 = 4; //Pin for encoder 2 on Motor A
int val_ENC_B1 = 0; //Value of pin 4
int val_ENC_B2 = 0; //Value of pin 5
int posVal_B = 0;

//Variables for the value where the timer will interrupt
//------------------------------------------------------
int timer1_counter;


//General variables
//-----------------
int led = 13;
int or_Interrupt = 2;

//Pin difinition for motorboard
//-----------------------------
int dirA = 12;
int dirB = 13;
int pwmA = 3;
int pwmB = 11;


//Motor mode variable
// Mode 0: Stop
// Mode 1: Break
// Mode 2: Position
// Mode 3: Speed
//-----------------------------
int motorMode_A = 0;
int motorMode_B = 0;


//Pin difinition for distance sensor board
//----------------------------------------
#define trigPin 7
#define echoPin 8
unsigned long duration;
int distance;



//Interrupt attempt
#include <EnableInterrupt.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Adafruit motorshiled
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor01 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor02 = AFMS.getMotor(2);



/*
  //-------The Setup routine-------
  //-------------------------------
  void setup() {
  }
*/
/*
  //-------The loop routine-------
  //------------------------------
  void loop() {

  //Serial.println("mode 2");
  motorModeControlA(0,360);
  motorModeControlB(0,4);
  //delay(5000);
  //Serial.println(posVal_A);

  Serial.println("mode 3");
  motorModeControl(3,2);
  delay(5000);
  Serial.println(posVal_A);

  Serial.println("mode 1");
  motorModeControl(1,0);
  delay(5000);
  Serial.println(posVal_A);

  Serial.println("mode 2");
  motorModeControl(2,180);
  delay(5000);
  Serial.println(posVal_A);

  Serial.println("mode 0");
  motorModeControl(0,0);

  while(1){

  //motorModeControl(3,-5);
  delay(200);
  //Serial.print("Speed: ");
  //Serial.println(current_speed_A);
  //Serial.print("ErrorVal: ");
  //Serial.println(errorVal);
  //Serial.println(errorSum);
  //Serial.println(Iterm);
  //Serial.println(errorVal);
  Serial.print("PosA: ");
  Serial.println(posVal_A);
  Serial.print("PosB: ");
  //Serial.println(posVal_B);
  //Serial.print("PID: ");
  //Serial.println(abs(PID_A));
  //motorControl();
  }
  }
*/

void configController()
{
  AFMS.begin();

  // Set the speed to start, from 0 (off) to 255 (max speed)
  //myMotor02->setSpeed(50);

  //myMotor02->run(FORWARD);

  //General pinMode
  //---------------
  pinMode(led, OUTPUT);
  pinMode(2, INPUT);

  //Pin modes for the encoders
  pinMode(ENC_A1, INPUT);     //set the pin to input
  digitalWrite(ENC_A1, HIGH); //use the internal pullup resistor
  pinMode(ENC_B1, INPUT);     //set the pin to input
  digitalWrite(ENC_B1, HIGH); //use the internal pullup resistor

  // Attach a PinChange Interrupt to our pin on the
  // rising edge used to handle the interrupts from
  // the two encoders.
  //-----------------------------------------------
  enableInterrupt(ENC_A1,enc_Int_MotorA,CHANGE);
  enableInterrupt(ENC_B1,enc_Int_MotorB,CHANGE);

  //Setting up the serial communication
  //-----------------------------------
  Serial.begin(9600);

  //Configuration of the motorboard port A
  //--------------------------------------
  pinMode(dirA, OUTPUT); //Pin controlling the direction of the motor
  pinMode(pwmA, OUTPUT); //Pin for controlling the speed of the motor
  digitalWrite(dirA, LOW); //Setting the direction of the motor with either HIGH or LOW
  analogWrite(pwmA, 0); //Initializing the motor at speed 0

  //Configuration of the motorboard port B
  //--------------------------------------
  pinMode(dirB, OUTPUT); //Pin controlling the direction of the motor
  pinMode(pwmB, OUTPUT); //Pin for controlling the speed of the motor
  digitalWrite(dirB, LOW); //Setting the direction of the motor with either HIGH or LOW
  analogWrite(pwmB, 0); //Initializing the motor at speed 0

  // initialize timer1 used for the PID goto_Pos controller
  //-------------------------------------------------------
  //noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz (10ms)
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz

  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  //interrupts();             // enable all interrupts


  // Setting up the pins for the HC-SR04 distance sensor
  //-------------------------------------------------------
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW); //Making sure our trigPin is low
  delay(20);
}

//---Interrupt service routine for handling the timier interrupt---
//-----------------------------------------------------------------
ISR(TIMER1_OVF_vect)        // interrupt service routine
{
  interrupts();
  TCNT1 = timer1_counter;   // preload timer
  //digitalWrite(led, digitalRead(led) ^ 1);
  motorControl();
}

//---------------------Motor Mode Controller A----------------------
// The function controles the mode of the motors. To change the mode
// of a motor, call this function with the mode parameter:
// Mode = 0 : Stop
// Mode = 1 : Break
// Mode = 2 : Position
// Mote = 3 : Speed
void motorModeControlA(int mode, int inputVal)
{
  if (mode == 0 && mode_0_set_A == false)
  {
    //Sets a variable that can be checked to see if a function is done
    mode_A_Done = false;

    //motorMode_A = 0; //Sets the actual motor mode for the PID regulator

    mode_0_set_A = true; //Set boolean that the mode is set.
    mode_1_set_A = false; //Set boolean that the mode is not set.
    mode_2_set_A = false; //Set boolean that the mode is not set.
    mode_3_set_A = false; //Set boolean that the mode is not set.
    mode_4_set_A = false; //Set boolean that the mode is not set.
  }
  if (mode == 1 && mode_1_set_A == false)
  {
    //Sets a variable that can be checked to see if a function is done
    mode_A_Done = false;

    currentPos_A = posVal_A; //Saves the current position
    //motorMode_A = 1; //Sets the actual motor mode for the PID regulator

    mode_0_set_A = false; //Set boolean that the mode is set.
    mode_1_set_A = true; //Set boolean that the mode is not set.
    mode_2_set_A = false; //Set boolean that the mode is not set.
    mode_3_set_A = false; //Set boolean that the mode is not set.
    mode_4_set_A = false; //Set boolean that the mode is not set.
  }
  if (mode == 2 && mode_2_set_A == false)
  {
    //Sets a variable that can be checked to see if a function is done
    mode_A_Done = false;

    posVal_A = 0;
    goto_Pos_A = inputVal;
    //motorMode_A = 2; //Sets the actual motor mode for the PID regulator

    mode_0_set_A = false; //Set boolean that the mode is not set.
    mode_1_set_A = false; //Set boolean that the mode is set.
    mode_2_set_A = true; //Set boolean that the mode is not set.
    mode_3_set_A = false; //Set boolean that the mode is not set.
    mode_4_set_A = false; //Set boolean that the mode is not set.
  }
  if (mode == 3)
  {
    //Sets a variable that can be checked to see if a function is done
    mode_A_Done = true;

    desired_speed_A = inputVal;
    //motorMode_A = 3; //Sets the actual motor mode for the PID regulator

    mode_0_set_A = false; //Set boolean that the mode is not set.
    mode_1_set_A = false; //Set boolean that the mode is set.
    mode_2_set_A = false; //Set boolean that the mode is not set.
    mode_3_set_A = true; //Set boolean that the mode is not set.
    mode_4_set_A = false; //Set boolean that the mode is not set.
  }
}


//---------------------Motor Mode Controller A (overload) -----------------
// The function controles the mode of the motors. To change the mode
// of a motor, call this function with the mode parameter:
// Mode = 4 : Possition, Speed
void motorModeControlA(int mode, int inputVal1, int inputVal2)
{
  if (mode == 4 && mode_4_set_A == false)
  {
    //Sets a variable that can be checked to see if a function is done
    mode_A_Done = false;

    desired_speed_A = inputVal2;
    goto_Pos_A = inputVal1;
    posVal_A = 0;

    //motorMode_A = 4; //Sets the actual motor mode for the PID regulator

    mode_0_set_A = false; //Set boolean that the mode is not set.
    mode_1_set_A = false; //Set boolean that the mode is set.
    mode_2_set_A = false; //Set boolean that the mode is not set.
    mode_3_set_A = false; //Set boolean that the mode is not set.
    mode_4_set_A = true; //Set boolean that the mode is not set.
  }
}

//---------------------Motor Mode Controller B----------------------
// The function controles the mode of the motors. To change the mode
// of a motor, call this function with the mode parameter:
// Mode = 0 : Stop
// Mode = 1 : Break
// Mode = 2 : Position
// Mode = 3 : Speed
void motorModeControlB(int mode, int inputVal)
{
  if (mode == 0 && mode_0_set_B == false)
  {
    //Sets a variable that can be checked to see if a function is done
    mode_B_Done = false;

    //motorMode_B = 0; //Sets the actual motor mode for the PID regulator

    mode_0_set_B = true; //Set boolean that the mode is set.
    mode_1_set_B = false; //Set boolean that the mode is not set.
    mode_2_set_B = false; //Set boolean that the mode is not set.
    mode_3_set_B = false; //Set boolean that the mode is not set.
    mode_4_set_B = false; //Set boolean that the mode is not set.
  }
  if (mode == 1 && mode_1_set_B == false)
  {
    //Sets a variable that can be checked to see if a function is done
    mode_B_Done = false;

    currentPos_B = posVal_B; //Saves the current position
    //motorMode_B = 1; //Sets the actual motor mode for the PID regulator

    mode_0_set_B = false; //Set boolean that the mode is set.
    mode_1_set_B = true; //Set boolean that the mode is not set.
    mode_2_set_B = false; //Set boolean that the mode is not set.
    mode_3_set_B = false; //Set boolean that the mode is not set.
    mode_4_set_B = false; //Set boolean that the mode is not set.
  }
  if (mode == 2 && mode_2_set_B == false)
  {
    //Sets a variable that can be checked to see if a function is done
    mode_B_Done = false;

    posVal_B = 0;
    goto_Pos_B = inputVal;
    //motorMode_B = 2; //Sets the actual motor mode for the PID regulator

    mode_0_set_B = false; //Set boolean that the mode is not set.
    mode_1_set_B = false; //Set boolean that the mode is set.
    mode_2_set_B = true; //Set boolean that the mode is not set.
    mode_3_set_B = false; //Set boolean that the mode is not set.
    mode_4_set_B = false; //Set boolean that the mode is not set.
  }
  if (mode == 3)
  {
    //Sets a variable that can be checked to see if a function is done
    mode_B_Done = true;

    desired_speed_B = inputVal;
    //motorMode_B = 3; //Sets the actual motor mode for the PID regulator

    mode_0_set_B = false; //Set boolean that the mode is not set.
    mode_1_set_B = false; //Set boolean that the mode is set.
    mode_2_set_B = false; //Set boolean that the mode is not set.
    mode_3_set_B = true; //Set boolean that the mode is not set.
    mode_4_set_B = false; //Set boolean that the mode is not set.
  }
}

//---------------------Motor Mode Controller B (overload) -----------------
// The function controles the mode of the motors. To change the mode
// of a motor, call this function with the mode parameter:
// Mode = 4 : Possition, Speed
void motorModeControlB(int mode, int inputVal1, int inputVal2)
{
  if (mode == 4 && mode_4_set_B == false)
  {
    //Sets a variable that can be checked to see if a function is done
    mode_B_Done = false;

    desired_speed_B = inputVal2;
    goto_Pos_B = inputVal1;
    posVal_B = 0;

    //motorMode_B = 4; //Sets the actual motor mode for the PID regulator

    mode_0_set_B = false; //Set boolean that the mode is not set.
    mode_1_set_B = false; //Set boolean that the mode is set.
    mode_2_set_B = false; //Set boolean that the mode is not set.
    mode_3_set_B = false; //Set boolean that the mode is not set.
    mode_4_set_B = true; //Set boolean that the mode is not set.
  }
}


//------------------PID controller Motor A & B---------------------
// Depending of the mode setting this controller wil either control
// goto_Pos or speed of the motors. The routine is divided into two
// main segments, one for each motor. Then depending of the mode of
// each motor, it will run either goto_Pos or speed for that motor
//-----------------------------------------------------------------
void motorControl()
{
  //Calculating the speed of motor A
  current_speed_A = (posVal_A - last_Pos_A);
  last_Pos_A = posVal_A;

  //Calculating the speed of motor B
  current_speed_B = (posVal_B - last_Pos_B);
  last_Pos_B = posVal_B;

  // Setting the PID controller in motorMode 0
  if (motorMode_A == 0)
  {
    PID_A = 0;
  }

  // Setting the PID controller in motorMode 0
  if (mode_0_set_B == true)
  {
    PID_B = 0;
    mode_0_set_B = false;
  }

  // Setting the PID controller in motorMode 1
  if (mode_1_set_A == true)
  {
    //Calculating error values
    errorVal = currentPos_A - posVal_A; //The goto_Pos error
    errorSum_A += errorVal; //The total error over time (should be zero)
    errorD = (errorVal - errorLast_A); //Change in error

    PID_A = (int)((P_POS * errorVal) + (I_POS * errorSum_A) + (D_POS * errorD));

    errorLast_A = errorVal;

    //Indicates the function is done
    mode_A_Done = true;
  }

  // Setting the PID controller in motorMode 1
  if (mode_1_set_B == true)
  {
    //Calculating error values
    errorVal = currentPos_B - posVal_B; //The goto_Pos error
    errorSum_B += errorVal; //The total error over time (should be zero)
    errorD = (errorVal - errorLast_B); //Change in error

    PID_B = (int)((P_POS * errorVal) + (I_POS * errorSum_B) + (D_POS * errorD));

    errorLast_B = errorVal;

    //Indicates the function is done
    mode_B_Done = true;
  }

  // Setting the PID controller in motorMode 2
  if (mode_2_set_A == true)
  {
    //Calculating error values
    errorVal = goto_Pos_A - posVal_A; //The position error
    errorSum_A += errorVal; //The total error over time (should be zero)
    errorD = (errorVal - errorLast_A); //Change in error

    PID_A = (int)((P_POS * errorVal) + (I_POS * errorSum_A) + (D_POS * errorD));

    errorLast_A = errorVal;

    if (errorVal < 10 && errorVal > -10)
    {
      //Renables the mode
      mode_2_set_A = false;
      //Call the break motor command
      motorModeControlA(1, 0);
    }
  }

  // Setting the PID controller in motorMode 2
  if (mode_2_set_B == true)
  {
    //Calculating error values
    errorVal = goto_Pos_B - posVal_B; //The position error
    errorSum_B += errorVal; //The total error over time (should be zero)
    errorD = (errorVal - errorLast_B); //Change in error

    PID_B = (int)((P_POS * errorVal) + (I_POS * errorSum_B) + (D_POS * errorD));

    errorLast_B = errorVal;

    if (errorVal < 10 && errorVal > -10)
    {
      //Renables the mode
      mode_2_set_B = false;
      //Call the break motor command
      motorModeControlB(1, 0);
    }
  }

  // Setting the PID controller in motorMode 3
  if (mode_3_set_A == true)
  {
    //Calculating error values
    errorVal = desired_speed_A - current_speed_A; //The position error
    errorSum_A += errorVal; //The total error over time (should be zero)
    errorD = (errorVal - errorLast_A); //Change in error

    Iterm = (I_SPD * errorSum_A);
    if (Iterm > 200.0)
    {
      Iterm = 200.0;
      errorSum_A = 0.0;
    }
    else if (Iterm < -200.0)
    {
      Iterm = -200.0;
      errorSum_A = 0.0;
    }

    PID_A = (int)((P_SPD * errorVal) + Iterm + (D_SPD * errorD));

    errorLast_A = errorVal;
  }

  // Setting the PID controller in motorMode 3
  if (mode_3_set_B == true)
  {
    //Calculating error values
    errorVal = desired_speed_B - current_speed_B; //The position error
    errorSum_B += errorVal; //The total error over time (should be zero)
    errorD = (errorVal - errorLast_B); //Change in error

    Iterm = (I_SPD * errorSum_B);
    if (Iterm > 200.0)
    {
      Iterm = 200.0;
      errorSum_B = 0.0;
    }
    else if (Iterm < -200.0)
    {
      Iterm = -200.0;
      errorSum_B = 0.0;
    }

    PID_B = (int)((P_SPD * errorVal) + Iterm + (D_SPD * errorD));

    errorLast_B = errorVal;
  }

  // Setting the PID controller in motorMode 4
  if (mode_4_set_A == true)
  {
    //Calculating the position error
    errorPos_A = goto_Pos_A - abs(posVal_A);

    //Calculating error values
    errorVal = desired_speed_A - current_speed_A; //The position error
    errorSum_A += errorVal; //The total error over time (should be zero)
    errorD = (errorVal - errorLast_A); //Change in error

    Iterm = (I_SPD * errorSum_A);
    if (Iterm > 100.0)
    {
      Iterm = 200.0;
      errorSum_A = 0.0;
    }
    else if (Iterm < -100.0)
    {
      Iterm = -200.0;
      errorSum_A = 0.0;
    }

    PID_A = (int)((P_SPD * errorVal) + Iterm + (D_SPD * errorD));

    errorLast_A = errorVal;

    if (errorPos_A < 10 && errorPos_A > -10)
    {
      //Renables the mode
      mode_4_set_A = false;
      //Call the break motor command
      motorModeControlA(1, 0);
    }
  }

  // Setting the PID controller in motorMode 4
  if (mode_4_set_B == true)
  {
    //Calculating the position error
    errorPos_B = goto_Pos_B - abs(posVal_B);

    //Calculating error values
    errorVal = desired_speed_B - current_speed_B; //The position error
    errorSum_B += errorVal; //The total error over time (should be zero)
    errorD = (errorVal - errorLast_B); //Change in error

    Iterm = (I_SPD * errorSum_B);
    if (Iterm > 100.0)
    {
      Iterm = 200.0;
      errorSum_B = 0.0;
    }
    else if (Iterm < -100.0)
    {
      Iterm = -200.0;
      errorSum_B = 0.0;
    }

    PID_B = (int)((P_SPD * errorVal) + Iterm + (D_SPD * errorD));

    errorLast_B = errorVal;

    if (errorPos_B < 10 && errorPos_B > -10)
    {
      //Renables the mode
      mode_4_set_B = false;
      //Call the break motor command
      motorModeControlB(1, 0);
    }
  }

  //Decides if the motor should turn right or left
  if (PID_A < 0)
  {
    //digitalWrite(dirA, HIGH);
    myMotor02->run(FORWARD);
  }
  else
  {
    //digitalWrite(dirA, LOW);
    myMotor02->run(BACKWARD);
  }

  //Limits the maximum output
  if (abs(PID_A) > 255)
  {
    PID_A = 255;
  }
  //analogWrite(pwmA, abs(PID_A));
  myMotor02->setSpeed(abs(PID_A));



  //Decides if the motor should turn right or left
  if (PID_B < 0)
  {
    //digitalWrite(dirB, HIGH);
    myMotor01->run(FORWARD);
  }
  else
  {
    //digitalWrite(dirB, LOW);
    myMotor01->run(BACKWARD);
  }

  //Limits the maximum output
  if (abs(PID_B) > 255)
  {
    PID_B = 255;
  }
  //analogWrite(pwmB, abs(PID_B));
  myMotor01->setSpeed(abs(PID_B));

}


//--------------------Tick interrupt routine-----------------------
// This is the interrupt service routine that handles the interrupt
// from the encoders on both motor A and B. Through a diode or gate
// the general interrupt is triggered, and afterwards it is deter-
// mined which motor (or both) that made the interrupt.
//-----------------------------------------------------------------
void enc_Int_MotorA()
{
  noInterrupts(); //Disables other interrupts
  val_ENC_A1 = digitalRead(ENC_A1); //Reads in value on enc_A1
  val_ENC_A2 = digitalRead(ENC_A2); //Reads in value on enc_A2

  if (val_ENC_A1 == 1) //Checks if the interrupt was falling or rising
  {
    if (val_ENC_A2 == 1) //Checks which way the motor turns
    {
      posVal_A++;
    }
    else
    {
      posVal_A--;
    }
  }
  else
  {
    if (val_ENC_A2 == 0) //Checks which way the motor turns
    {
      posVal_A++;
    }
    else
    {
      posVal_A--;
    }
  }
  interrupts();//Enable interrupts
}

void enc_Int_MotorB()
{
  noInterrupts(); //Disables other interrupts
  val_ENC_B1 = digitalRead(ENC_B1); //Reads in value on enc_B1
  val_ENC_B2 = digitalRead(ENC_B2); //Reads in value on enc_B2

  if (val_ENC_B1 == 1) //Checks if the interrupt was falling or rising
  {
    if (val_ENC_B2 == 1) //Checks which way the motor turns
    {
      posVal_B++;
    }
    else
    {
      posVal_B--;
    }
  }
  else
  {
    if (val_ENC_B2 == 0) //Checks which way the motor turns
    {
      posVal_B++;
    }
    else
    {
      posVal_B--;
    }
  }
  interrupts();//Enable interrupts
}


//-------------------------Get Distance ----------------------------
// This function returns the distance to an object in mm. The sensor
// used is the HC-SR04 that takes a 10us pulse on the trig pin to
// start the measurement, and returns a high push on the echo pin.
// The lenght of the puls defines the length to the object in front.
//------------------------------------------------------------------
int hentAfstand()
{
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration / 5.8;
  return distance;
}

