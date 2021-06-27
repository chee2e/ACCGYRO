#include <SoftwareSerial.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
#include "DFRobot_OxygenSensor.h"
#include "MQ7.h"

MPU6050 accelgyro1(0x68);
MPU6050 accelgyro2(0x69);

//가속도 자이로 센서 관련 변수
int16_t ax1, ay1, az1;
int16_t gx1, gy1, gz1;


double angle1 = 0, deg ; // angle, deg data (각도계산)
double angle2 = 0;
double dgy_x ;
long mapping_value = 1000;
long normal_x, normal_y, normal_z, deltha_x[3], deltha_y[3], deltha_z[3], deltha, angle_1[3], angle_2[3] ;
long event_value = 1000;
boolean ACCESS_ACCEL = false;
  
int16_t ac_x, ac_y, ac_z, gy_x, gy_y, gy_z ; //acc, gyro data (acc, gyro 계산 수식)
double angleAcX;
double angleAcY;
double anglemain;
const double RADIAN_TO_DEGREE = 180 / 3.14159;
int Nomal_Angle = 60;

// 기타 변수
SoftwareSerial BtSerial(2, 3);
DFRobot_OxygenSensor Oxygen;
MQ7 mq7(A0,5.0);

#define OUTPUT_READABLE_ACCELGYRO
#define mpu_add 0x68
#define COLLECT_NUMBER    30             
#define Oxygen_IICAddress ADDRESS_3

void value_init();
//void ACCEL_ACCESS();

void setup() {
  Wire.begin();
  Serial.begin(115200);
  accelgyro1.initialize();
  accelgyro2.initialize();
  BtSerial.begin(115200);
  Oxygen.begin(Oxygen_IICAddress);
  pinMode(10, OUTPUT);
}

void loop() {
  
  accelgyro1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1); 
  float oxygenData = Oxygen.ReadOxygenData(COLLECT_NUMBER); //LED 제어를 위한 산소 변수
  float ppm = mq7.getPPM();  // LED 제어를 위한 일산화탄소 변수
  
  Oxygen_Data();  //산소측정 함수
  CO_Data();    // 일산화탄소 측정함수
  value_init(); // 변수 초기화
  accel_calculate(); // 가속도 측정 -> 각도계산

  if (BtSerial.available()) { //블루투스에서 넘어온 데이터가 있다면
    Serial.write(BtSerial.read()); //시리얼모니터에 데이터를 출력
  }
  if (Serial.available()) {    //시리얼모니터에 입력된 데이터가 있다면
    BtSerial.write(Serial.read());  //블루투스를 통해 입력된 데이터 전달
  }
  
  if( oxygenData < 18 )
  {
    digitalWrite(10 , 255);
  }else if ( ppm > 50) {
    digitalWrite(10 , 255);
  } else {
    digitalWrite(10 , LOW);
  }

}

void accel_calculate() {
  ac_x = 0; ac_y = 0; ac_z = 0;

  Wire.beginTransmission(mpu_add) ; // 번지수 찾기
  Wire.write(0x3B) ; // 가속도 데이터 보내달라고 컨트롤 신호 보내기
  Wire.endTransmission(false) ; // 기달리고,
  Wire.requestFrom(mpu_add, 6, true) ; // 데이터를 받아 처리

  ac_x = Wire.read() << 8 | Wire.read() ;
  ac_y = Wire.read() << 8 | Wire.read() ;
  ac_z = Wire.read() << 8 | Wire.read() ;

  normal_x = map(int(ac_x), -16384, 16384, 0, mapping_value);
  normal_y = map(int(ac_y), -16384, 16384, 0, mapping_value);
  normal_z = map(int(ac_z), -16384, 16384, 0, mapping_value);

  angleAcX = atan(ac_y / sqrt(pow(ac_x, 2) + pow(ac_z, 2)));
  angleAcX *= RADIAN_TO_DEGREE;   // 좌, 우 (Roll)
  
  angleAcY = atan(-ac_x / sqrt(pow(ac_y, 2) + pow(ac_z, 2)));
  angleAcY *= RADIAN_TO_DEGREE;   // 앞, 뒤 (Pitch)

  anglemain = sqrt(pow(angleAcX, 2) + pow(angleAcY, 2));

  angle1 = abs(angleAcX);
  angle2 = abs(angleAcY);

  /*
  //Serial.print(" Moving : ");
  Serial.print(anglemain, 1);
  Serial.print(",");
  Serial.print("\n");
  */
  
  if(anglemain >= Nomal_Angle)
  {
     Serial.print("확인필요");
     Serial.print(",");
     Serial.println();

     BtSerial.print("확인필요");
     BtSerial.print(",");
     BtSerial.println();
     
  }else{
     Serial.print("활동중");
     Serial.print(",");
     Serial.println();

     BtSerial.print("활동중");
     BtSerial.print(",");
     BtSerial.println();
  }

  angle1 = abs(angleAcX);
  angle2 = abs(angleAcY);

  /*
  BtSerial.print(anglemain, 1);
  BtSerial.print(",");
  BtSerial.println();
  */
}

void value_init() {
  normal_x = 0; normal_x = 0; normal_x = 0;
  for (int i = 0; i < 3; i++) {
    deltha_x[i] = 0;
    deltha_y[i] = 0;
    deltha_z[i] = 0;
    angle1 = 0;
    angle2 = 0;
  }
}

void Oxygen_Data() {
  float oxygenData = Oxygen.ReadOxygenData(COLLECT_NUMBER);
  //Serial.print(" Oxygen : ");
  Serial.print(oxygenData, 1);
  Serial.print("%");
  Serial.print(",");
  //delay(100);

  BtSerial.print(oxygenData, 1);
  BtSerial.print("%");
  BtSerial.print(",");

  if( oxygenData < 18 )
  {
    digitalWrite(10 , 255);
  }else {
    digitalWrite(10 , LOW);
  }
}

void CO_Data() {

  float ppm = mq7.getPPM();
  
  //Serial.print(" CO : ");
  Serial.print(mq7.getPPM(), 1);
  Serial.print("ppm");
  Serial.print(",");
  //delay(100);

  BtSerial.print(mq7.getPPM(), 1);
  BtSerial.print("ppm");
  BtSerial.print(",");

    if( ppm > 50 )
  {
    digitalWrite(10 , 255);
  }else {
    digitalWrite(10 , LOW);
  }

}
