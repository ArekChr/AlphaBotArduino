/*
 * Układ bezprzewodowego sterowania platformą mobilną realizującą zadanie transportowe
 * Arkadiusz Chrabąszczewski
 */

// ------------------------------- Libraries --------------------------------//
#include <Encoder.h>

// --------------------------- Variables  -----------------------------------//
// ----------------------------- Motor Variables -----------------------------//
#define PWMA   3            //Speed left motor - pin (ENA)
#define AIN2   A0           //Left motor - forward (IN2)
#define AIN1   A1           //Left motor - backward (IN1)
#define PWMB   5            //Speed right motor - pin (ENB)
#define BIN1   A2           //right motor - forward (IN3)
#define BIN2   A3           //right motor - backward (IN4)

// ---------------------- Pilot IR ------------------------ //
#define IR  4

// --------------- IR remote control variables -------------//
#define KEY2 0x18           //Button:2 
#define KEY8 0x52           //Button:8 
#define KEY4 0x08           //Button:4 
#define KEY6 0x5A           //Button:6 
#define KEY1 0x0C           //Button:1 
#define KEY3 0x5E           //Button:3 
#define KEY5 0x1C           //Button:5
#define SpeedDown 0x07      //Button:VOL-
#define SpeedUp 0x15        //Button:VOL+
#define ResetSpeed 0x09     //Button:EQ
#define AddDifSpeed 0x40    //Button:Next
#define SubDifSpeed 0x44    //Button:Prev
#define AddDifPos 0x46      //Button:Ch
#define SubDifPos 0x45      //Button:Ch-
#define Repeat 0xFF         //Button long press

// ---------------------- Encoder Variables ------------------------//
#define L_OUT_A 8           // Left encoder output A
#define L_OUT_B 9           // left encoder output B
#define R_OUT_A 10          // Right encoder output A
#define R_OUT_B 11          // Right encoder output B

// ------------------------ Initialization -------------------------//
Encoder LEnc(L_OUT_A, L_OUT_B);
Encoder REnc(R_OUT_A, R_OUT_B);

// ----------------------- encoder position variables ----------------------------- //
long OLD_L_POS = -999;      // Position left wheel, changing position
long OLD_R_POS = -999;      // Position right wheel, changing position
long TARG_L_POS = 0;        // Target position left wheel
long TARG_R_POS = 0;        // Target position right wheel
long L_MOTOR_REAL_POS = 0;  // Actual position left wheel
long R_MOTOR_REAL_POS = 0;  // Actual position right wheel

const float cm = 3.46;      // constans - 346 encoder unit = 100 cm

// --- Variables for count X & Y axis and real position reference --- //
long X = 0;               // Real position X axis
long Y = 0;               // Real position Y axis
long TRAV_ROUTE = 0;      // Counter of covered route

long X_NEG = 0;           // Position X negative direction
long X_POS = 0;           // Position Y positive direction
long Y_NEG = 0;           // Position Y negative direction
long Y_POS = 0;           // Position Y positive direction

long REF_TRAV_ROUTE = 0;  // Last position before change direction
long REF_Y_POS = 0;       // Last position Y axis positive before change direction
long REF_Y_NEG = 0;       // Last position Y axis negative before change direction
long REF_X_POS = 0;       // Last position X axis positive before change direction
long REF_X_NEG = 0;       // Last position X axis negative before change direction

byte DIRECTION = 1;       // Direction while driving / 1 = Forward / 0 = Backward / 2 = Rest 
byte POSITION = 0;        // 0 = UP / 1 = RIGHT / 2 = DOWN / 3 = LEFT

// -------------------- Bluetooth Variables ---------------------- //

String comdata = "";            // Data command
byte flag_bt = 1;               // Bluetooth flag

// -------------------- IR Control Variables ---------------------- //

unsigned long lasttime = 0;     // 
unsigned char results;          // reveived signal
byte flag_ir = 0;               // Flag for IR state

// ------------------- Speed Variables ----------------------------- //
int DefaultSpeedRight = 86;  
int DefaultSpeedLeft = 80;              // Default Speed
int SpeedLeft = DefaultSpeedLeft;       // Speed of right wheel
int SpeedRight = DefaultSpeedRight;     // Speed of left wheel
int SpeedOnTurn = 80;

// ------------- Variables for calculate driving correction ---------------- //
short SetDPos = 1;          // position discrepancy for odriometry pattern
short SetDSpeed = 5;        // Variables speed disparity
int dPos;                   // Disparity variable L/R wheel
int L_DIFF = 0;             // Disparity variable left wheel
int R_DIFF = 0;             // Disparity variable right wheel

// -------------------- Transport task flags -------------------- //
bool Task = false;
bool DriveTask = false;
bool IS_X_AXIS = false;
bool IS_Y_AXIS = true;
bool Y_COMPLETE = false;
bool X_COMPLETE = false;

int X_TARG = 0;
int Y_TARG = 0;

// ------------- Void Initialization -------------- //

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
//  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(IR, INPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
}

// ----------------- Main Loop ---------------- //

void loop() {

  if (Task == true) {
    TransportTask();        
  }
  EncoderPositionRead();    
  Drive();                  
  Read_XY();           
  SpeedCompensation();    
//  Logger();
  IR_Read();              
  BluetoothSerialRead();   
}

void TransportTask()
{
  if (DIRECTION == 4) {
    DriveTask = true;
  }
  if (Y_TARG > Y - 2 && Y_TARG < Y + 2 ) {
    Y_COMPLETE = true; 
  } else {
    Y_COMPLETE = false;
  }

  if (X_TARG > X - 2 && X_TARG < X + 2) {
    X_COMPLETE = true; 
  } else {
    X_COMPLETE = false;
  }

  if (Y_COMPLETE && X_COMPLETE ) {
    Task = false;
  }

  if (DriveTask == true)
  {
    if(Y_COMPLETE == false) {
      
      if (Y_TARG > Y + 2)
      {
        if (POSITION == 2) { right(); right(); }
        if (POSITION == 3) { right(); }
        if (POSITION == 1) { left(); }
        if (DIRECTION == 4) {
          forward(abs(Y_TARG - Y));
        }
      }
      else if (Y_TARG < Y - 2)     
      {
        if (POSITION == 0) { right(); right(); }
        if (POSITION == 3) { left(); }
        if (POSITION == 1) { right(); }
        if (DIRECTION == 4) {
          forward(abs(Y_TARG - Y));
        }
      }
    }

    if (Y_COMPLETE == true && X_COMPLETE == false){
      if (X_TARG > X + 2)
      {
        if (POSITION == 3) { right(); right(); }
        if (POSITION == 2) { left(); }
        if (POSITION == 0) { right(); }
        if (DIRECTION == 4) {
          forward(abs(X_TARG - X));
        }
      }
      if (X_TARG < X - 2)  
      {
        if (POSITION == 1) { right(); right(); }
        if (POSITION == 0) { left(); }
        if (POSITION == 2) { right(); }
        if (DIRECTION == 4) {
          forward(abs(X_TARG - X));
        }
      }
    }
  }
}

void forward(){
  DIRECTION = 1;
  TARG_L_POS = TARG_L_POS + 50;   
  TARG_R_POS = TARG_R_POS + 50;   
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
  TARG_L_POS = TARG_L_POS - 50;    
  TARG_R_POS = TARG_R_POS - 50;    
  DriveTask = false;
}
void right(){
  DIRECTION = 2;
  TARG_L_POS = TARG_L_POS + 24;
  TARG_R_POS = TARG_R_POS - 24;
  L_DIFF = L_DIFF + 24;
  R_DIFF = R_DIFF - 24;
  onTurnRight();              
  SpeedLeft = SpeedOnTurn;   
  SpeedRight = SpeedOnTurn;   
  DriveTask = false;
}
void left(){
  DIRECTION = 2;
  TARG_L_POS = TARG_L_POS - 24;
  TARG_R_POS = TARG_R_POS + 24;
  L_DIFF = L_DIFF - 24;
  R_DIFF = R_DIFF + 24;
  onTurnLeft();              
  SpeedLeft = SpeedOnTurn;   
  SpeedRight = SpeedOnTurn;
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
  if (L_MOTOR_REAL_POS == TARG_L_POS  && R_MOTOR_REAL_POS == TARG_R_POS ) 
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
    case 1: // gdy kierunek = przód
      if (dPos < SetDPos) {
        SpeedLeft = DefaultSpeedLeft + SetDSpeed * abs(dPos);
        SpeedRight = DefaultSpeedRight - SetDSpeed * abs(dPos);
      }
      else if (dPos > SetDPos) {
        SpeedLeft = DefaultSpeedLeft - SetDSpeed * abs(dPos);
        SpeedRight = DefaultSpeedRight + SetDSpeed * abs(dPos);
      }
      else{ 
        SpeedLeft = DefaultSpeedLeft; 
        SpeedRight = DefaultSpeedRight; 
      }
      break;
    case 0: // gdy kierunek = tył
      if (dPos < SetDPos) {
        SpeedLeft = DefaultSpeedLeft - SetDSpeed * abs(dPos);
        SpeedRight = DefaultSpeedRight + SetDSpeed * abs(dPos);
      }
      else if (dPos > SetDPos) {
        SpeedLeft = DefaultSpeedLeft + SetDSpeed * abs(dPos);
        SpeedRight = DefaultSpeedRight - SetDSpeed * abs(dPos);
      }
      else{  
        SpeedLeft = DefaultSpeedLeft; 
        SpeedRight = DefaultSpeedRight; 
      }
      break;
  }
  if (SpeedLeft <= 30)  SpeedLeft = 35;      
  else if (SpeedLeft >= 255)  SpeedLeft = 250; 
  if (SpeedRight <= 30)  SpeedRight = 35;       
  else if (SpeedRight >= 255)  SpeedRight = 250;
}
void Logger()
{
  if (L_MOTOR_REAL_POS != OLD_L_POS || R_MOTOR_REAL_POS != OLD_R_POS) 
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
//    Serial.print(SpeedRight);
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
//    Serial.print((TARG_L_POS + TARG_R_POS) / 2);
//    Serial.print(" | X: ");
//    Serial.print(X);
//    Serial.print(" | Y: ");
//    Serial.print(Y);
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
//    Serial.println(MessageBool(Y_COMPLETE));
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
      Y_TARG = Y_TARG * cm;
      X_TARG = X_TARG * cm;
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
      DefaultSpeedRight += 10;
      DefaultSpeedLeft += 10;
      if (DefaultSpeedRight > 255) DefaultSpeedRight = 250;
      if (DefaultSpeedLeft > 255) DefaultSpeedLeft = 250;
      //Serial.print("Speed: ");
      //Serial.println(Speed); break;
    case SpeedDown:
      DefaultSpeedRight -= 10;
      DefaultSpeedLeft -= 10;
      if (DefaultSpeedRight < 0)DefaultSpeedRight = 0;
      if (DefaultSpeedLeft < 0)DefaultSpeedLeft = 0;
      //Serial.print("Speed: ");
      //Serial.println(Speed); break;
    case ResetSpeed:
      DefaultSpeedRight = 150;
      DefaultSpeedLeft = 150;
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

