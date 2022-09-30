#include <Servo.h>
#include <IMU.h>
#include <Dynamixel2Arduino.h>
#define DXL_SERIAL   Serial3


//-------------------
#define dirPin 3   //디지털 3번핀 스텝모터 드라이버 방향핀
#define stepPin 4  //디지털 4번핀 스텝모터 스텝핀(PWM)
#define limitSw1 5 //디지털 5번핀 리미트 스위치1 입력핀
#define limitSw2 6 //디지털 6번핀 리미트 스위치2 입력핀

#define TRIG_FRONT 7 //TRIG 핀 설정 (초음파 보내는 핀)
#define ECHO_FRONT 8 //ECHO 핀 설정 (초음파 받는 핀)
#define TRIG_BACK 11 //TRIG 핀 설정 (초음파 보내는 핀)
#define ECHO_BACK 12 //ECHO 핀 설정 (초음파 받는 핀)


//---------------------
char state1 = 0; //리미트 스위치 상태 저장용 변수
char state2 = 0; //
Servo head_x;
Servo head_y;

cIMU    IMU;

uint8_t   err_code;
int step_angle = 0;
const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
const uint8_t LS_DXL_ID = 102;
const uint8_t LE_DXL_ID = 104;
const uint8_t LW_DXL_ID = 106;
const uint8_t RS_DXL_ID = 103;
const uint8_t RE_DXL_ID = 105;
const uint8_t RW_DXL_ID = 107;

const uint8_t RWH1_DXL_ID = 4;
const uint8_t RWH2_DXL_ID = 6;
const uint8_t LWH1_DXL_ID = 3;
const uint8_t LWH2_DXL_ID = 1;

const float DXL_PROTOCOL_VERSION = 1.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

int DXL_ID_ARM[] = { LS_DXL_ID, LE_DXL_ID, LW_DXL_ID, RS_DXL_ID, RE_DXL_ID, RW_DXL_ID };
int DXL_ID_WH[] = { RWH1_DXL_ID, RWH2_DXL_ID, LWH2_DXL_ID, LWH1_DXL_ID };
int r, l;
int cm_FRONT, cm_BACK;
int Straight_BEAT = 1;
int Back_BEAT = 1;
int Stop_BEAT = 1;
int seread;
int Flag;
int n_Flag = 0;
//함수----------------------------------------------

void position_setting() {
  //포지션 모드 0~300도 까지가능
  dxl.setGoalPosition(LS_DXL_ID, 178, UNIT_DEGREE);           //LS ,183도,도단위사용
  dxl.setGoalPosition(LE_DXL_ID, 60, UNIT_DEGREE);            //LE ,183도,도단위사용
  dxl.setGoalPosition(LW_DXL_ID, 160, UNIT_DEGREE);           //LW ,183도,도단위사용
  dxl.setGoalPosition(RS_DXL_ID, 168, UNIT_DEGREE);           //RS ,183도,도단위사용
  dxl.setGoalPosition(RE_DXL_ID, 58, UNIT_DEGREE);            //RE ,183도,도단위사용
  dxl.setGoalPosition(RW_DXL_ID, 162, UNIT_DEGREE);           //RW ,183도,도단위사용
}
int dis_FRONT() {
  long duration_FRONT, distance_FRONT;
  digitalWrite(TRIG_FRONT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_FRONT, LOW);
  duration_FRONT = pulseIn(ECHO_FRONT, HIGH);
  distance_FRONT = duration_FRONT * 17 / 1000;
  return distance_FRONT;
}
int dis_BACK() {
  long duration_BACK, distance_BACK;
  digitalWrite(TRIG_BACK, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_BACK, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_BACK, LOW);
  duration_BACK = pulseIn(ECHO_BACK, HIGH);
  distance_BACK = duration_BACK * 17 / 1000;
  return distance_BACK;
}
void Straight()
{ //정회전
  if (Straight_BEAT == 1)
  {
    Serial.println("Straight!\n");
    for (int i = 0; i <= 79; i++)
    { //속도%가 0%~80%까지 천천히 증가 딜레이 10당 0.8초임
      if (i == 0) {
        r = -100, l = 0;
      }
      dxl.setGoalVelocity(RWH1_DXL_ID, r, UNIT_PERCENT);
      dxl.setGoalVelocity(LWH1_DXL_ID, l, UNIT_PERCENT);
      dxl.setGoalVelocity(RWH2_DXL_ID, l, UNIT_PERCENT);
      dxl.setGoalVelocity(LWH2_DXL_ID, r, UNIT_PERCENT);
      r++; l++;
      delay(10);
    }
    Straight_BEAT = 0;
  }
  else if (Straight_BEAT == 0)
  {
    dxl.setGoalVelocity(RWH1_DXL_ID, r, UNIT_PERCENT);
    dxl.setGoalVelocity(LWH1_DXL_ID, l, UNIT_PERCENT);
    dxl.setGoalVelocity(RWH2_DXL_ID, l, UNIT_PERCENT);
    dxl.setGoalVelocity(LWH2_DXL_ID, r, UNIT_PERCENT);
    Stop_BEAT = 1;
  }
  else;
}
void Back() {//역회전
  if (Back_BEAT == 1)
  {
    Serial.println("Back!\n");
    for (int i = 0; i <= 79; i++)
    {
      if (i == 0) {
        r = 0;
        l = -100;
      }
      dxl.setGoalVelocity(RWH1_DXL_ID, r, UNIT_PERCENT);
      dxl.setGoalVelocity(LWH1_DXL_ID, l, UNIT_PERCENT);
      dxl.setGoalVelocity(RWH2_DXL_ID, l, UNIT_PERCENT);
      dxl.setGoalVelocity(LWH2_DXL_ID, r, UNIT_PERCENT);
      r++; l++;
      delay(10);
    }
    Back_BEAT = 0;
  }
  else if (Back_BEAT == 0)
  {
    dxl.setGoalVelocity(RWH1_DXL_ID, r, UNIT_PERCENT);
    dxl.setGoalVelocity(LWH1_DXL_ID, l, UNIT_PERCENT);
    dxl.setGoalVelocity(RWH2_DXL_ID, l, UNIT_PERCENT);
    dxl.setGoalVelocity(LWH2_DXL_ID, r, UNIT_PERCENT);
    Stop_BEAT = 1;
  }
  else;
}
void STOP() {//정지
  if (Stop_BEAT == 1)
  {
    Serial.println("Stop!\n");
    for (int i = 0; i <= 79; i++) { //속도%가 80%~0%까지 천천히 감소 딜레이 10당 0.8초임
      dxl.setGoalVelocity(RWH1_DXL_ID, r, UNIT_PERCENT);
      dxl.setGoalVelocity(LWH1_DXL_ID, l, UNIT_PERCENT);
      dxl.setGoalVelocity(RWH2_DXL_ID, l, UNIT_PERCENT);
      dxl.setGoalVelocity(LWH2_DXL_ID, r, UNIT_PERCENT);
      r--; l--;
      delay(10);
    }
    Stop_BEAT = 0;
  }
  else if (Stop_BEAT == 0)
  {
    dxl.setGoalVelocity(RWH1_DXL_ID, -100, UNIT_PERCENT);
    dxl.setGoalVelocity(LWH1_DXL_ID, -100, UNIT_PERCENT);
    dxl.setGoalVelocity(RWH2_DXL_ID, -100, UNIT_PERCENT);
    dxl.setGoalVelocity(LWH2_DXL_ID, -100, UNIT_PERCENT);
  }
  Back_BEAT = 1;
  Straight_BEAT = 1;
}
void turn_front()     //전진하다가 뒤집기할때
{
  Serial.println("Turn!\n");
  //찝기1
  dxl.setGoalPosition(LS_DXL_ID, 184, UNIT_DEGREE);           //LS ,183도,도단위사용
  dxl.setGoalPosition(LE_DXL_ID, 54, UNIT_DEGREE);            //LE ,183도,도단위사용
  dxl.setGoalPosition(LW_DXL_ID, 80, UNIT_DEGREE);           //LW ,183도,도단위사용
  dxl.setGoalPosition(RS_DXL_ID, 180, UNIT_DEGREE);           //RS ,183도,도단위사용
  dxl.setGoalPosition(RE_DXL_ID, 43, UNIT_DEGREE);            //RE ,183도,도단위사용
  dxl.setGoalPosition(RW_DXL_ID, 80, UNIT_DEGREE);           //RW ,183도,도단위사용
  delay(2000);
  //찝기2
  dxl.setGoalPosition(LS_DXL_ID, 126, UNIT_DEGREE);           //LS ,183도,도단위사용
  dxl.setGoalPosition(LE_DXL_ID, 258, UNIT_DEGREE);            //LE ,183도,도단위사용
  dxl.setGoalPosition(LW_DXL_ID, 56, UNIT_DEGREE);           //LW ,183도,도단위사용
  dxl.setGoalPosition(RS_DXL_ID, 128, UNIT_DEGREE);           //RS ,183도,도단위사용
  dxl.setGoalPosition(RE_DXL_ID, 244, UNIT_DEGREE);            //RE ,183도,도단위사용
  dxl.setGoalPosition(RW_DXL_ID, 59, UNIT_DEGREE);           //RW ,183도,도단위사용
  delay(2000);
  //뒤집기 후
  dxl.setGoalPosition(LS_DXL_ID, 135, UNIT_DEGREE);           //LS ,183도,도단위사용
  dxl.setGoalPosition(LE_DXL_ID, 251, UNIT_DEGREE);            //LE ,183도,도단위사용
  dxl.setGoalPosition(LW_DXL_ID, 127, UNIT_DEGREE);           //LW ,183도,도단위사용
  dxl.setGoalPosition(RS_DXL_ID, 124, UNIT_DEGREE);           //RS ,183도,도단위사용
  dxl.setGoalPosition(RE_DXL_ID, 242, UNIT_DEGREE);            //RE ,183도,도단위사용
  dxl.setGoalPosition(RW_DXL_ID, 129, UNIT_DEGREE);           //RW ,183도,도단위사용
}
void turn_back()      //후진하다가 뒤집기할떄
{
  Serial.println("Turn!\n");
  //찝기3
  dxl.setGoalPosition(LS_DXL_ID, 128, UNIT_DEGREE);           //LS ,183도,도단위사용
  dxl.setGoalPosition(LE_DXL_ID, 250, UNIT_DEGREE);            //LE ,183도,도단위사용
  dxl.setGoalPosition(LW_DXL_ID, 230, UNIT_DEGREE);           //LW ,183도,도단위사용
  dxl.setGoalPosition(RS_DXL_ID, 126, UNIT_DEGREE);           //RS ,183도,도단위사용
  dxl.setGoalPosition(RE_DXL_ID, 250, UNIT_DEGREE);            //RE ,183도,도단위사용
  dxl.setGoalPosition(RW_DXL_ID, 230, UNIT_DEGREE);           //RW ,183도,도단위사용
  delay(2000);
  //찝기4
  dxl.setGoalPosition(LS_DXL_ID, 179, UNIT_DEGREE);           //LS ,183도,도단위사용
  dxl.setGoalPosition(LE_DXL_ID, 60, UNIT_DEGREE);            //LE ,183도,도단위사용
  dxl.setGoalPosition(LW_DXL_ID, 230, UNIT_DEGREE);           //LW ,183도,도단위사용
  dxl.setGoalPosition(RS_DXL_ID, 167, UNIT_DEGREE);           //RS ,183도,도단위사용
  dxl.setGoalPosition(RE_DXL_ID, 70, UNIT_DEGREE);            //RE ,183도,도단위사용
  dxl.setGoalPosition(RW_DXL_ID, 230, UNIT_DEGREE);           //RW ,183도,도단위사용
  delay(2000);
  //초기
  dxl.setGoalPosition(LS_DXL_ID, 178, UNIT_DEGREE);           //LS ,183도,도단위사용
  dxl.setGoalPosition(LE_DXL_ID, 60, UNIT_DEGREE);            //LE ,183도,도단위사용
  dxl.setGoalPosition(LW_DXL_ID, 160, UNIT_DEGREE);           //LW ,183도,도단위사용
  dxl.setGoalPosition(RS_DXL_ID, 168, UNIT_DEGREE);           //RS ,183도,도단위사용
  dxl.setGoalPosition(RE_DXL_ID, 58, UNIT_DEGREE);            //RE ,183도,도단위사용
  dxl.setGoalPosition(RW_DXL_ID, 162, UNIT_DEGREE);           //RW ,183도,도단위사용
}

void stepMotorPinSetting() { //스텝모터관련 핀 세팅
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(limitSw1, INPUT);
  pinMode(limitSw2, INPUT);
  digitalWrite(dirPin, LOW);
}

void servo_init() { //짐벌초기자세 잡는 함수
  head_x.attach(9);
  head_y.attach(10);
  head_x.write(94);
  head_y.write(92);
}

void step_swing() { //스텝모터 회전하는 기본 코드
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(500);
  digitalWrite(stepPin, LOW);
  delay(1);
}

void step_init() { //스텝모터 초기세팅

  while (digitalRead(limitSw1) == 0) {//스위치 눌림 감지시 while 문 탈출
    step_swing();
  }
  delay(200);                     //방향 전환을 위한 딜레이
  digitalWrite(dirPin, HIGH);     //방향전환
  for (int x = 0; x < 101; x++) { //영점으로 이동(180=1.8*100)
    step_swing();
  }
  step_angle = 180; //영점 각도
}

//---------------------------------------------------------------------------------
void setup() {
  Serial1.begin(115200);         //시리얼 속도
  Serial1.println("시리얼통신시작");//통신시작을 알림
  IMU.begin();                   //IMU와 통신 시작
  stepMotorPinSetting();         //스텝모터 핀 세팅 함수
  servo_init();                  //서보모터 초기 자세 세팅 x,y(90도 세팅)
  step_init();                   //스텝모터 초기 세팅(영점 세팅)
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  //초음파센서 설정

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_BACK, OUTPUT);
  pinMode(ECHO_BACK, INPUT);
  //다이나믹셀 설정
  for (int i = 0; i < 6; i++) {
    dxl.torqueOff(DXL_ID_ARM[i]);
    dxl.setOperatingMode(DXL_ID_ARM[i], OP_POSITION);      //각도모드
    dxl.torqueOn(DXL_ID_ARM[i]);
    //DEBUG_SERIAL.println(dxl.ping(DXL_ID_ARM[i]));         //ping 함수로 LS 연결 체크, 안써도 됨
  }

  for (int i = 0; i < 4; i++) {
    dxl.torqueOff(DXL_ID_WH[i]);
    dxl.setOperatingMode(DXL_ID_WH[i], OP_VELOCITY);       //DC모드
    dxl.torqueOn(DXL_ID_WH[i]);
    //DEBUG_SERIAL.println(dxl.ping(DXL_ID_WH[i]));         //ping 함수로 LS 연결 체크, 안써도 됨
  }
  position_setting();
}

void loop() {
  static uint32_t tTime[3];
  static uint32_t imu_time = 0;
  static int32_t head_x_val = 0;
  static int32_t head_y_val = 0;
  static int32_t head_z_val = 0;

  state1 = digitalRead(limitSw1); //1번 리미트 스위치 상태 저장 변수
  state2 = digitalRead(limitSw2); //2번 리미트 스위치 상태 저장 변수
  cm_FRONT = dis_FRONT();
  cm_BACK = dis_BACK();
  if (Flag == 0)
    {
        if ((cm_FRONT <= 30) && (Straight_BEAT == 0))
        {
            STOP();
            turn_front();
            Back();
            n_Flag = 1;
        }
        else if ((cm_BACK <= 30) && (Back_BEAT == 0))
        {
            STOP();
            turn_back();
            Straight();
            n_Flag = 0;
        }
        else
        {
            if (n_Flag == 0)Straight();
            else if (n_Flag == 1)Back();
        }
    }
    else if (Flag == 1)
    {
        STOP();
    }
    else if (Flag == 2)
    {
      if(n_Flag==0)
      {
        STOP();
        
        turn_front();
        n_Flag = 1;
        Flag = 0;
      }
      else if(n_Flag==1)
      {
        STOP();
        turn_back();
        n_Flag = 0;
        Flag = 0;
      }
    }
    
  if (Serial1.available() > 0) {//주 메인문 [젯슨>>opencr 시리얼 조건문]
    seread = Serial1.read();
    if (seread == 'n')
    {
      //Serial1.println("기본상태 ");
      Flag = 0;
    }
    else if (seread == 's')
    {
      //Serial1.println("정지");
      Flag = 1;
    }
    else if (seread == 't')
        {
            Flag = 2;
    }
    else if (seread == 'f') {
      //Serial1.println("pir 앞");
      if (step_angle == 360) {
        for (int i = 0; i < 101; i++) {
          step_swing();
        }
      }
      else;
    }
    else if (seread == 'b') {
      
      Serial1.println("pir 뒤");
      digitalWrite(dirPin, LOW);
      step_angle = 0;
      while (digitalRead(limitSw2) == 0) {
        step_swing();
      }
      digitalWrite(dirPin, HIGH);
      step_angle = 360;
      
    }
    else;
  }

  //IMU관련 코드(절대 수정하면 안됨!!!!!!!!!)===============================
  if ( (millis() - tTime[0]) >= 500 )//IMU 관련
  {
    tTime[0] = millis();
  }

  tTime[2] = micros();
  if ( IMU.update() > 0 ) imu_time = micros() - tTime[2];

  if ( (millis() - tTime[1]) >= 50 )
  {
    tTime[1] = millis();
  }
  //=====================================================================
  /* //step motor test code
    if(state1==1){
    digitalWrite(dirPin,HIGH);
    state2=0;
    }
    else if(state2==1){
    digitalWrite(dirPin,LOW);
    state1=0;
    }
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(200);
    digitalWrite(stepPin, LOW);
    delay(1);
  */
  head_x_val = IMU.rpy[0];//기울어진 각도 x
  head_y_val = IMU.rpy[1];//            y
  head_z_val = IMU.rpy[2];//            z

  if (head_x_val > 10)head_x_val = 10;
  else if (head_x_val < -10)head_x_val = -10;
  if (head_y_val > 10)head_y_val = 10;
  else if (head_y_val < -10)head_y_val = -10;

  head_x.write(94 - head_x_val); //짐벌 x축 각도 보정(조건 보정)
  head_y.write(92 - head_y_val); //짐벌 y축 각도 보정(항상 보정)
}
