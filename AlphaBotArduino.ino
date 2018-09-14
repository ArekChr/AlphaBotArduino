/*
 * Praca inżynierska:
 * Układ bezprzewodowego sterowania platformą mobilną realizującą zadanie transportowe
 * Program sterowaia robota
 * Arkadiusz Chrabąszczewski
 */

// ------------------------------- Biblioteki --------------------------------//
#include <Encoder.h>

// --------------------------- Definicja zmiennych ---------------------------//
// ----------------------------- Zmienne silnika -----------------------------//
#define PWMA   3            //Szybkość lewego silnika pin (ENA)
#define AIN2   A0           //Lewy silnik - przód (IN2)
#define AIN1   A1           //Lewy silnik - tył (IN1)
#define PWMB   5            //Szybkość prawego silnika pin(ENB)
#define BIN1   A2           //Prawy silnik - przód (IN3)
#define BIN2   A3           //Prawy silnik - tył (IN4)

// ---------------------- Pilot IR ------------------------ //
#define IR  4

// --------------- IR remote control variables -------------//
#define KEY2 0x18           //Przycisk:2 
#define KEY8 0x52           //Przycisk:8 
#define KEY4 0x08           //Przycisk:4 
#define KEY6 0x5A           //Przycisk:6 
#define KEY1 0x0C           //Przycisk:1 
#define KEY3 0x5E           //Przycisk:3 
#define KEY5 0x1C           //Przycisk:5
#define SpeedDown 0x07      //Przycisk:VOL-
#define SpeedUp 0x15        //Przycisk:VOL+
#define ResetSpeed 0x09     //Przycisk:EQ
#define AddDifSpeed 0x40    //Przycisk:Next
#define SubDifSpeed 0x44    //Przycisk:Prev
#define AddDifPos 0x46      //Przycisk:Ch
#define SubDifPos 0x45      //Przycisk:Ch-
#define Repeat 0xFF         //Przytrzymanie przycisku

// ---------------------- Zmienne ekodera ------------------------//
#define L_OUT_A 8           //Lewy enkoder wyjście A
#define L_OUT_B 9           //Lewy enkoder wyjście B
#define R_OUT_A 10          //Prawy enkoder wyjście A
#define R_OUT_B 11          //Prawy enkoder wyjście B

// ------------------------ Inicjalizacja -------------------------//
Encoder LEnc(L_OUT_A, L_OUT_B);
Encoder REnc(R_OUT_A, R_OUT_B);

// -----------------------  Zmienne pozycji enkodera ----------------------------- //
long OLD_L_POS = -999;      //Pozycja lewego koła, używana do zmiany pozycji
long OLD_R_POS = -999;      //Pozycja prawego koła, używana do zmiany pozycji
long TARG_L_POS = 0;        //Pozycja docelowa dla lewego koła
long TARG_R_POS = 0;        //Pozycja docelowa dla prawego koła
long L_MOTOR_REAL_POS = 0;  //Pozycja aktualna dla lewego koła
long R_MOTOR_REAL_POS = 0;  //Pozycja aktualna dla prawego koła

const float cm = 3.46;      // stała - 346 jednostek enkodera = 100 cm

// --- Zmienne do obliczania osi X i Y oraz rzeczywistej pozycji odniesienia --- //
long X = 0;               // Rzeczywista pozycja osi X
long Y = 0;               // Rzeczywista pozycja osi Y
long TRAV_ROUTE = 0;      // Licznik pokoannej trasy

long X_NEG = 0;           // Pozycja X kierunek ujemny
long X_POS = 0;           // Pozycja Y kierunek dodatni
long Y_NEG = 0;           // Pozycja Y kierunek ujemny
long Y_POS = 0;           // Pozycja Y kierunek dodatni

long REF_TRAV_ROUTE = 0;  // Ostatnia pozycja przed zmianąmiana kierunku
long REF_Y_POS = 0;       // Ostatnia pozycja osi Y dodatniej przez zmianą kierunku
long REF_Y_NEG = 0;       // Ostatnia pozycja osi Y ujemnej przez zmianą kierunku
long REF_X_POS = 0;       // Ostatnia pozycja osi X dodatniej przez zmianą kierunku
long REF_X_NEG = 0;       // Ostatnia pozycja osi X ujemnej przez zmianą kierunku

byte DIRECTION = 1;       // Kierunek podczas jazdy / 1 = Naprzód / 0 = Cofanie / 2 = Pozostałe 
byte POSITION = 0;        // 0 = Góra / 1 = Prawo / 2 = Dół / 3 = Lewo

// -------------------- Zmienne Bluetooth ---------------------- //

String comdata = "";            // Komenda danych
byte flag_bt = 1;               // Flaga stanu dla bluetooth

// -------------------- Zmienne pilota IR ---------------------- //

unsigned long lasttime = 0;     // 
unsigned char results;          // Otrzymany sygnał
byte flag_ir = 0;               // Flaga stanu dla IR

// ------------------- Zmienne prędkości ----------------------------- //
int DefaultSpeedRight = 86;  
int DefaultSpeedLeft = 80;     // Główna prędkość
int SpeedLeft = DefaultSpeedLeft;          // Prędkość lewego koła
int SpeedRight = DefaultSpeedRight;         // prędkość prawego kołą
int SpeedOnTurn = 80;

// ------------- Zmienne do obliczania korekcji jazdy ---------------- //
short SetDPos = 1;          // Rozbieżność punktów dla wzoru odriometrii
short SetDSpeed = 5;        // Zmienna różnicy predkości
int dPos;                   // Zmienna róznicy L/P koła
int L_DIFF = 0;             // Zmienna różnicy lewego koła
int R_DIFF = 0;             // Zmienna różnicy prawego koła

// -------------------- Stany zadania transportowego -------------------- //
bool Task = false;
bool DriveTask = false;
bool IS_X_AXIS = false;
bool IS_Y_AXIS = true;
bool Y_COMPLETE = false;
bool X_COMPLETE = false;

int X_TARG = 0;
int Y_TARG = 0;

// ------------- Inicjalizacja funkcji -------------- //

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

// ----------------- Główna pętla programu ---------------- //

void loop() {

  if (Task == true) {
    TransportTask();        // Zadanie transportowe
  }
  EncoderPositionRead();    // Odczyt pozycji kół
  Drive();                  // Algorytm jazdy
  Read_XY();                // Odczyt pozycji (x,y)
  SpeedCompensation();      // Korekta rozbieżności pozycji kół
//  Logger();                 // Czytanie danych
  IR_Read();                // Czytanie komend IR
  BluetoothSerialRead();    // Czytanie komend bluetooth
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
  onTurnRight();                // zmiana orientacji
  SpeedLeft = SpeedOnTurn;         // Reset prędkości lewego silnika
  SpeedRight = SpeedOnTurn;       // Reset prędkości prawego silnika
  DriveTask = false;
}
void left(){
  DIRECTION = 2;
  TARG_L_POS = TARG_L_POS - 24;
  TARG_R_POS = TARG_R_POS + 24;
  L_DIFF = L_DIFF - 24;
  R_DIFF = R_DIFF + 24;
  onTurnLeft();                 // zmiana orientacji
  SpeedLeft = SpeedOnTurn;         // Reset predkości lewego silnika
  SpeedRight = SpeedOnTurn;       // Reset predkości prawego silnika
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
  if (L_MOTOR_REAL_POS != OLD_L_POS || R_MOTOR_REAL_POS != OLD_R_POS) // Czytanie danych podczas zmiany pozycji
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

