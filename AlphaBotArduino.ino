// ------------------------------- Libraries ----------------------------------//
#include <Encoder.h>
#include <NewPing.h>

// --------------------------- Variables definition ---------------------------//
// ----------------------------- Motors variables -----------------------------//
#define PWMA   3            //Left Motor Speed pin (ENA)
#define AIN2   A0           //Motor-L forward (IN2).
#define AIN1   A1           //Motor-L backward (IN1)
#define PWMB   5            //Right Motor Speed pin (ENB)
#define BIN1   A2           //Motor-R forward (IN3)
#define BIN2   A3           //Motor-R backward (IN4)

// ---------------------- IR remote control ------------------------ //
#define IR  4               //he infrare remote receiver pin 

// --------------- IR remote control variables -------------//
#define KEY2 0x18           //Key:2 
#define KEY8 0x52           //Key:8 
#define KEY4 0x08           //Key:4 
#define KEY6 0x5A           //Key:6 
#define KEY1 0x0C           //Key:1 
#define KEY3 0x5E           //Key:3 
#define KEY5 0x1C           //Key:5
#define SpeedDown 0x07      //Key:VOL-
#define SpeedUp 0x15        //Key:VOL+
#define ResetSpeed 0x09     //Key:EQ
#define AddDifSpeed 0x40    //Key:Next
#define SubDifSpeed 0x44    //Key:Prev
#define AddDifPos 0x46      //Key:Ch
#define SubDifPos 0x45      //Key:Ch-
#define Repeat 0xFF         //press and hold the key

// ---------------------- Encoder variables ------------------------//
#define L_OUT_A 8           //Left Motor Encoder Output A
#define L_OUT_B 9           //Left Motor Encoder Output B
#define R_OUT_A 10          //Right Motor Encoder Output A
#define R_OUT_B 11          //Right Motor Encoder Output B

// ------------------------ Initialization -------------------------//
Encoder LEnc(L_OUT_A, L_OUT_B);
Encoder REnc(R_OUT_A, R_OUT_B);

unsigned int pingSpeed = 50;  // How frequently are we going to send out a ping (in milliseconds). 
                              //                                  50ms would be 20 times a second.
unsigned long pingTimer;      // Holds the next ping time.
int DISTANCE = 100;           // Distance from the object at the front

// -----------------------  Encoder Position variables ----------------------------- //
long OLD_L_POS  = -999;           // Old left position, used while position changed
long OLD_R_POS  = -999;           // Old right position, used while position changed
long TARG_L_POS = 0;              // Target position for left motor
long TARG_R_POS = 0;              // Target position for right motor
long L_MOTOR_REAL_POS  = 0;       // Real position from left motor
long R_MOTOR_REAL_POS  = 0;       // Real position from right motor

const float cm = 3.46;    // constant - 346 encoder points = 100 cm

// --- Var for calculate X and Y real position in reference to start position --- //
long X = 0;               // X real position
long Y = 0;               // Y real position
long TRAV_ROUTE = 0;      // Traveled route

long X_NEG = 0;           // X position in negative direction
long X_POS = 0;           // X position in positive direction
long Y_NEG = 0;           // Y position in negative direction
long Y_POS = 0;           // Y position in positive direction

long REF_TRAV_ROUTE = 0;  // Last position before change direction
long REF_Y_POS = 0;       // Last Y positive before change direction
long REF_Y_NEG = 0;       // Last Y negative before change direction
long REF_X_POS = 0;       // Last X positive before change direction
long REF_X_NEG = 0;       // Last X negative before change direction

byte DIRECTION = 1;       // Direction while riding / 1 = Forward / 0 = Backward / 2 = other 
byte POSITION = 0;        // 0 = UP / 1 = Right / 2 = Down / 3 = Left

// -------------------- Bluetooth variables ---------------------- //

String comdata = "";            // Data command
byte flag_bt = 1;               // flag_irstate for bluetooth

// ---------------- IR Remote control variables ------------------ //

unsigned long lasttime = 0;     // 
unsigned char results;          // received results 
byte flag_ir = 0;               // flag_irstate for ir

// --------------- Motor speed variables ------------------------- //
int Speed = 100;                // Main speed of robot
int SpeedLeft = Speed;          // Speed on left wheel
int SpeedRight = Speed;         // Speed on right wheel

// ---------- Variables for calculate speed correction ----------- //
short SetDPos = 1;          // Disccrepancy of points for pattern
short SetDSpeed = 5;        // Speed varriable for pattern
int dPos;                   // Difference between left and right motor
int L_DIFF = 0;             // Left wheel difference
int R_DIFF = 0;             // Right wheel difference

// -------------------- Transport task states -------------------- //
bool Task = false;
bool DriveTask = false;
bool IS_X_AXIS = false;
bool IS_Y_AXIS = true;
bool Y_COMPLETE = false;
bool X_COMPLETE = false;

byte IS_LOCKED = 0;    // about 4 states

int X_TARG = 0;
int Y_TARG = 0;

// ------------- Function initialization -------------- //

char IR_decode(unsigned char code);
void translateIR();
void forward();
void backward();
void right();
void left();
void stop();
void IR_Read();
void EncoderPositionRead();
void Logger();
void SpeedCompensation();
void onTurnRight();
void onTurnLeft();
void BluetoothSerialRead();
void Read_XY();

void setup() {
  //Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(IR, INPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pingTimer = millis();
}

// ----------------- Main loop of program ---------------- //

void loop() {

  if (Task == true) {
    TransportTask();        // Transport task 
  }
  EncoderPositionRead();    // Motors position read
  Drive();                  // Position correction
  Read_XY();                // Read current position on X,Y axis
  SpeedCompensation();      // Compensation motor speed while driving
  Logger();                 // Write all data in serial
  IR_Read();                // IR remote control
  BluetoothSerialRead();    // Reads commands on bluetooth serial port
}

void TransportTask()
{
  if (DIRECTION == 4) {
    DriveTask = true;
  }
  if (Y_TARG + 5 > Y && Y_TARG - 5 < Y) Y_COMPLETE = true; else Y_COMPLETE = false;

  if (X_TARG + 5 > X && X_TARG - 5 < X) X_COMPLETE = true; else X_COMPLETE = false;

  if (DriveTask == true)
  {
    if (Y_COMPLETE == false && IS_Y_AXIS == true && Y_TARG + 5 > Y)     
    {                       // --- if positive Y direction not reached --- //
      if (POSITION == 2) {
        right(); right();
      }
      else forward();
    }
    else if (Y_COMPLETE == false && IS_Y_AXIS == true && Y_TARG - 5 < Y)     
    {                       // --- if negative Y direction not reached --- //
      if (POSITION == 0) {
        right(); right();
      }
      else forward();
    }
    else if (Y_COMPLETE == true && X_COMPLETE == false && IS_Y_AXIS == true)
    {                          // --- if direction Y reached but X not --- //
      if (X_TARG > X) {
        right();
      }
      else left();
    }
    else if (X_COMPLETE == true && Y_COMPLETE == false && IS_X_AXIS == true)  
    {                          // --- if direction X reached but Y not --- //
      if (Y_TARG > Y) {
        left();
      }
      else left();
    }
    else if (X_COMPLETE == false && IS_X_AXIS == true && X_TARG + 5 > X)  
    {                       // --- if positive X direction not reached --- //
      if (POSITION == 3) {
        left(); left();
      }
      else forward();
    }
  }
  if (Y_TARG + 4 > Y && Y_TARG - 4 < Y && X_TARG + 4 > X && X_TARG - 4 < X )
  {
    Task = false;             // Task completed!
  }
}

void forward(){
  DIRECTION = 1;
  TARG_L_POS = TARG_L_POS + 173;   
  TARG_R_POS = TARG_R_POS + 173;   
  DriveTask = false;
}
void forward(int distance){
  DIRECTION = 1;
  TARG_L_POS = TARG_L_POS + distance;    
  TARG_R_POS = TARG_R_POS + distance;    
  DriveTask = false;
}
void backward(){
  DIRECTION = 0;
  TARG_L_POS = TARG_L_POS - 346;    
  TARG_R_POS = TARG_R_POS - 346;    
  DriveTask = false;
}
void right(){
  DIRECTION = 2;
  TARG_L_POS = TARG_L_POS + 24;
  TARG_R_POS = TARG_R_POS - 24;
  L_DIFF = L_DIFF + 24;
  R_DIFF = R_DIFF - 24;
  onTurnRight();                // change orientation
  SpeedLeft = Speed;            // Reset speed of motors
  SpeedRight = Speed;           // Reset speed of motors
  DriveTask = false;
}
void left(){
  DIRECTION = 2;
  TARG_L_POS = TARG_L_POS - 24;
  TARG_R_POS = TARG_R_POS + 24;
  L_DIFF = L_DIFF - 24;
  R_DIFF = R_DIFF + 24;
  onTurnLeft();                 // change orientation
  SpeedLeft = Speed;            // Reset speed of motors
  SpeedRight = Speed;           // Reset speed of motors
  DriveTask = false;
}
void stop() {
}

void EncoderPositionRead() {
  L_MOTOR_REAL_POS = LEnc.read();
  R_MOTOR_REAL_POS = REnc.read();
}

void Drive() {
  if (L_MOTOR_REAL_POS == TARG_L_POS){
    analogWrite(PWMA, 0);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  if (R_MOTOR_REAL_POS == TARG_R_POS){
    analogWrite(PWMB, 0);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  if (L_MOTOR_REAL_POS >= TARG_L_POS - 1 && R_MOTOR_REAL_POS >= TARG_R_POS - 1) 
  {
    DIRECTION = 4;
  }
  if (L_MOTOR_REAL_POS < TARG_L_POS ){
    analogWrite(PWMA, SpeedLeft);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  if (L_MOTOR_REAL_POS > TARG_L_POS ){
    analogWrite(PWMA, SpeedLeft);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  if (R_MOTOR_REAL_POS < TARG_R_POS ){
    analogWrite(PWMB, SpeedRight);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  if (R_MOTOR_REAL_POS > TARG_R_POS ){
    analogWrite(PWMB, SpeedRight);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
}

void onTurnRight(){
  if (POSITION == 3) POSITION = 0;
  else POSITION++;
  REF_TRAV_ROUTE = TRAV_ROUTE;
  REF_Y_POS = Y_POS;
  REF_Y_NEG = Y_NEG;
  REF_X_POS = X_POS;
  REF_X_NEG = X_NEG;
  SetActualTaskPosition();
}
void onTurnLeft(){
  if (POSITION == 0) POSITION = 3;
  else POSITION--;
  REF_TRAV_ROUTE = TRAV_ROUTE;
  REF_Y_POS = Y_POS;
  REF_Y_NEG = Y_NEG;
  REF_X_POS = X_POS;
  REF_X_NEG = X_NEG;
  SetActualTaskPosition();
}
void SetActualTaskPosition(){
  switch (POSITION){
    case 0:
      IS_Y_AXIS = true;
      IS_X_AXIS = false;
      break;
    case 1:
      IS_Y_AXIS = false;
      IS_X_AXIS = true;
      break;
    case 2:
      IS_Y_AXIS = true;
      IS_X_AXIS = false;
      break;
    case 3:
      IS_Y_AXIS = false;
      IS_X_AXIS = true;
      break;
  }
}

void Read_XY()
{
  TRAV_ROUTE = ((L_MOTOR_REAL_POS + R_MOTOR_REAL_POS) / 2);

  Y = Y_POS - Y_NEG;
  X = X_POS - X_NEG;

  switch (POSITION)
  {
    case 0: Y_POS = REF_Y_POS + TRAV_ROUTE - REF_TRAV_ROUTE;
      break;
    case 1: X_POS = REF_X_POS + TRAV_ROUTE - REF_TRAV_ROUTE;
      break;
    case 2: Y_NEG = REF_Y_NEG + TRAV_ROUTE - REF_TRAV_ROUTE;
      break;
    case 3: X_NEG = REF_X_NEG + TRAV_ROUTE - REF_TRAV_ROUTE;
      break;
  }
}
void SpeedCompensation()
{
  dPos = (L_MOTOR_REAL_POS - L_DIFF ) - (R_MOTOR_REAL_POS - R_DIFF);
  switch (DIRECTION)
  {
    case 1: // when FORWARD
      if (dPos < SetDPos) {
        SpeedLeft = Speed + SetDSpeed * abs(dPos);
        SpeedRight = Speed - SetDSpeed * abs(dPos);
      }
      else if (dPos > SetDPos) {
        SpeedLeft = Speed - SetDSpeed * abs(dPos);
        SpeedRight = Speed + SetDSpeed * abs(dPos);
      }
      else{ SpeedLeft = Speed; SpeedRight = Speed; }
      break;
    case 0: // when BARKWARD
      if (dPos > SetDPos) {
        SpeedLeft = Speed - SetDSpeed * abs(dPos);
        SpeedRight = Speed + SetDSpeed * abs(dPos);
      }
      else if (dPos < SetDPos) {
        SpeedLeft = Speed + SetDSpeed * abs(dPos);
        SpeedRight = Speed - SetDSpeed * abs(dPos);
      }
      else{  SpeedLeft = Speed; SpeedRight = Speed; }
      break;
  }
  if (SpeedLeft <= 30)  SpeedLeft = 35;      
  else if (SpeedLeft >= 255)  SpeedLeft = 250; 
  if (SpeedRight <= 30)  SpeedRight = 35;       
  else if (SpeedRight >= 255)  SpeedRight = 250;
}
void Logger()
{
  if (L_MOTOR_REAL_POS != OLD_L_POS || R_MOTOR_REAL_POS != OLD_R_POS) // Logging data on changed position
  {
    OLD_R_POS = R_MOTOR_REAL_POS;
    OLD_L_POS = L_MOTOR_REAL_POS;
//    Serial.print("");
//    Serial.print(L_MOTOR_REAL_POS );
//    Serial.print(";");
//    Serial.print(dPos);
//    Serial.print(";");
//    Serial.print(SpeedLeft);
//    Serial.print(";");
//    Serial.print(R_MOTOR_REAL_POS );
//    Serial.print(";");
//    Serial.println(SpeedRight);
//    Serial.print(" | Direction: ");
//    Serial.print(DIRECTION);
//    Serial.print(" | Position: ");
//    Serial.print(Message("POSITION"));
//    Serial.print(" | Traveled forward: ");
//    Serial.print(TRAV_ROUTE / cm );
//    Serial.print("cm | Left diff.: ");
//    Serial.print(L_DIFF / cm );
//    Serial.print("cm | Right diff.: ");
//    Serial.print(R_DIFF / cm );
//    Serial.print(" | X+: ");
//    Serial.print(X_POS);
//    Serial.print(" | X-: ");
//    Serial.print(X_NEG);
//    Serial.print(" | Y+: ");
//    Serial.print(Y_POS);
//    Serial.print(" | Y-: ");
//    Serial.print(Y_NEG);
//    Serial.print(" | Target Position: ");
//    Serial.println((TARG_L_POS + TARG_R_POS) / 2);
//    Serial.print(" | X: ");
//    Serial.print(X);
//    Serial.print(" | Y: ");
//    Serial.print(Y);
//    Serial.print(" | DISTANCE: ");
//    Serial.print(DISTANCE);
//    Serial.print(" | Target Y: ");
//    Serial.print(Y_TARG);
//    Serial.print(" | Target X: ");
//    Serial.print(X_TARG);
//    Serial.print(" | Drive Task? ");
//    Serial.print(MessageBool(DriveTask));
//    Serial.print(" | IS_X_AXIS_? : ");
//    Serial.print(MessageBool(IS_X_AXIS));
//    Serial.print(" | IS_Y_AXIS_? : ");
//    Serial.print(MessageBool(IS_Y_AXIS));
//    Serial.print(" | IsTask_? : ");
//    Serial.print(MessageBool(Task));
//    Serial.print(" | X_Is_Complete: ");
//    Serial.print(MessageBool(X_COMPLETE));
//    Serial.print(" | Y_Is_Complete: ");
//    Serial.print(MessageBool(Y_COMPLETE));
//    Serial.print(" | Is Locked: ");
//    Serial.println(IS_LOCKED);
  }
}

String MessageBool(bool Value) {
  String Message;
  switch (Value) {
    case true: Message = "True";
      break;
    case false: Message = "False";
      break;
  }
  return Message;
}

String Message(String Name) {
  String Message;
  if (Name == "POSITION")
  {
    switch (POSITION)
    {
      case 0: Message = "Up";
        break;
      case 1: Message = "Right";
        break;
      case 2: Message = "Down";
        break;
      case 3: Message = "Left";
        break;
    }
    return Message;
  }
  else return Message = "Null";
}

void BluetoothSerialRead()
{
  while (Serial1.available() > 0){
    comdata += char(Serial1.read());
    delay(2);
  }
  if (comdata.length() > 0)
  {
    //Serial.println(comdata);
    const char* command = comdata.c_str();
    if (strcmp(command, "Forward") == 0)
    {   
      flag_bt = 0;
      forward();
    }
    else if (strcmp(command, "Backward") == 0)
    {
      flag_bt = 0;
      backward();
    }
    else if (strcmp(command, "Left") == 0)
    {
      flag_bt = 0;
      left();
    }
    else if (strcmp(command, "Right") == 0)
    {
      flag_bt = 0;
      right();
    }
    else if (strcmp(command, "Stop") == 0)
    {
      flag_bt = 1;
      stop();
    }
    else if (comdata.substring(0, 2) == "XY")
    {
      flag_bt = 0;
      Task = true;
      X_TARG = comdata.substring(comdata.indexOf("(")+1,comdata.indexOf(",")).toInt();
      Y_TARG = comdata.substring(comdata.indexOf(",")+1,comdata.indexOf(")")).toInt();
      //Serial.print("Y Targ: ");
      //Serial.print(Y_TARG);
      //Serial.print(" | X Targ: ");
      //Serial.println(X_TARG);
    }
  comdata = "";
  }
}

void IR_Read()
{
  if (IR_decode(&results))
  {
    flag_ir= 1;
    lasttime = millis();
    translateIR();
  }
  else
  {
    if (flag_ir== 1)
    {
      if (millis() - lasttime > 150 )
      {
        flag_ir= 0;
        stop();
        //Serial.println("stop");
      }
    }
  }
}

void translateIR() {
  switch (results)
  {
    case KEY2:
      forward(); break;
    case KEY4:
      left(); break;
    case KEY5:
      stop(); break;
    case KEY6:
      right(); break;
    case KEY8:
      backward(); break;
    case SpeedUp:
      Speed += 10;
      if (Speed > 255)Speed = 250;
      //Serial.print("Speed: ");
      //Serial.println(Speed); break;
    case SpeedDown:
      Speed -= 10;
      if (Speed < 0)Speed = 0;
      //Serial.print("Speed: ");
      //Serial.println(Speed); break;
    case ResetSpeed:
      Speed = 150;
      //Serial.print("Speed reseted: ");
      //Serial.println(Speed); break;
    case AddDifSpeed:
      SetDSpeed ++;
      //Serial.print("dPos Speed: ");
      //Serial.println(SetDSpeed); break;
    case SubDifSpeed:
      SetDSpeed --;
      //Serial.print("dPos Speed: ");
      //Serial.println(SetDSpeed); break;
    case AddDifPos:
      SetDPos ++;
      //Serial.print("dPos: ");
      //Serial.println(SetDPos); break;
    case SubDifPos:
      SetDPos --;
      //Serial.print("dPos: ");
      //Serial.println(SetDPos);break;
    case Repeat: break;
    default: stop();
  }
}
char IR_decode(unsigned char * code) {
  char value = 0;
  unsigned int count = 0;
  unsigned char i, index, cnt = 0, data[4] = {0, 0, 0, 0};
  if (digitalRead(IR) == LOW)
  {
    count = 0;
    while (digitalRead(IR) == LOW && count++ < 200)  //9ms
      delayMicroseconds(60);

    count = 0;
    while (digitalRead(IR) == HIGH && count++ < 80)   //4.5ms
      delayMicroseconds(60);

    for (i = 0; i < 32; i++)
    {
      count = 0;
      while (digitalRead(IR) == LOW && count++ < 15) //0.56ms
        delayMicroseconds(60);
      count = 0;
      while (digitalRead(IR) == HIGH && count++ < 40) //0: 0.56ms; 1: 1.69ms
        delayMicroseconds(60);
      if (count > 20)data[index] |= (1 << cnt);
      if (cnt == 7)
      {
        cnt = 0;
        index++;
      }
      else cnt++;
    }
    if (data[0] + data[1] == 0xFF && data[2] + data[3] == 0xFF) //check
    {
      code[0] = data[2];
      // Serial.println(code[0], HEX);
      value = 1;
    }
    if (data[0] == 0xFF && data[1] == 0xFF && data[2] == 0xFF && data[3] == 0xFF)
    {
      code[0] = 0xFF;
      // Serial.println("rep");
      value = 1;
    }
  }
  return value;
}

