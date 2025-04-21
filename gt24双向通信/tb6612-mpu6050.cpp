/*
适用于四轮小车项目
制作原理：

1、用NR24L01实现遥控功能，一个arduino做遥控器，一个做为接收器。用一个摇杆控制油门和方向；
2、用两个舵机来控制炮塔（摄像头）的方位和俯仰；
3、另外，有4个开关（最多可以有8个），用于控制灯，音乐等设备； 
4、GT24，低8位传输油门信号，8~15位传输方向信号，16~24位，各4位传输炮塔的方位和俯仰信号。最高8位传输开关信号

接线方法：
  D3、D9接左侧驱动的控制；
  A6、A7接右侧驱动的控制；
  D5、D6分别为左右侧PWM控制；
  D10接方位舵机；
  A0接俯仰舵机；
  A4 SDA接pcf8574
  A5 SCL接pcf8574

  NR24L01接线：
  CSN  <---> D7
  CE   <---> D8
  MOSI <---> D11
  MISO <---> D12
  SCK  <---> D13
  VCC  <---> 3.3V

  数据IO D2、D3、D4、D5分别的4个开关 
  枪控油门范围： 中点：130，上限185，下限80；
  枪控方向范围： 中点：125，上限205，下限45；
*/

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <Servo.h>
#include <Adafruit_PCF8574.h>
//#include <C:\Users\Administrator\Documents\Arduino\libraries\Adafruit-Motor-Shield-library\AFMotor.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

// Ws2812 引脚
#define PIN 4
#define __DEBUG__

#ifdef __DEBUG__
#define DEBUG(...) Serial.print(__VA_ARGS__)
#define DEBUGL(...) Serial.println(__VA_ARGS__)
//#define DEBUG(...) Serial.println(__VA_ARGS__); \
                   Serial.print(" @ [SRC]:      "); \
                   Serial.println(__FILE__); \
                   Serial.print(" @ [LINE]:     "); \
                   Serial.println(__LINE__); \
                   Serial.print(" @ [FUNCTION]: "); \
                   Serial.println(__func__); 
#else
#define DEBUG(...)
#define DEBUGL(...)
#endif

Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, PIN, NEO_GRB + NEO_KHZ800);

//SoftwareSerial S2(A0, A3); //A0为接收，A3为发送；

Adafruit_PCF8574 pcf;
Servo fwservo; // 建立方位舵机对象
Servo fyservo; // 创建俯仰舵机对象
//SoftwareSerial S2(A2, A3); //A2为接收，A3为发送；

//定义驱动板控制引脚
int left1 = 3;
int left2 = 9;
int right1 = 2;
int right2 = 10;
int leftPwm = 5;
int rightPwm = 6;
//速度
int leftSpeed = 128;
int rightSpeed = 128;
int forwardSpeed = 128;
int lowSpeed = 80;  //轮子最低速度，低于此值电机无法转动 
//轮子的转弯系数 数值越大，两轮的转速差距越大 2.1
float turnIndex = 2.1;
//方向的停止范围
int turnUp = 140;
int turnDown = 120;
//方向的上、下限
int fxUp = 255;
int fxDown = 45;  
//油门的停止范围
int forwardUP = 135;
int forwardDown = 120;
//油门上、下限
int ymUp = 255;
int ymDown = 5;
//运行状态
bool isForward = 1;
//停机相关设置
long lastStopTime = 0;
long stopInterval = 1000; //1秒
uint8_t ym,fx,con_value,ptValue,lastPtValue = 0;
uint32_t recValue = 0;
int fwval,fyval,camval,lastFyval,lastFwval = 0;
// 数据上报间隔
long lastreport = 0; //记录上报时间
long reportTime = 2000; //2秒上报一次
//接收数据联合体
union transData
{ 
   long newvalue;
   byte buffer[4];
};

transData recUnion,sendUnion;
/////////////////////////////////
//PID gain and limit settings
/////////////////////////////////
float pid_p_gain_roll = 0.95;               //Gain setting for the roll P-controller 横滚
float pid_i_gain_roll = 0.03;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 17.0;                //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller 俯仰
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 3.0;                //Gain setting for the pitch P-controller 方位
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

/////////////////////////////////
//Declaring Variables
/////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int cal_int, start;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte highByte, lowByte;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float                 x_gyro = 0;
float                 y_gyro = 0;
float                 z_gyro = 0;

float gyro[3];
float gyroScaleFactor = radians(1000.0 / 32768.0);  //角度转为弧度
//float gyroScaleFactor = (0.0174532 / 16.4);


uint16_t sensors_detected = 0x00;

uint8_t gyroSamples = 0;

int16_t gyroRaw[3];
float gyroSum[3];

int16_t gyro_offset[3];
float gyro_x_cal=0.0;
float gyro_y_cal=0.0;
float gyro_z_cal=0.0;



/////////////////////////////////
//Defining Variables
/////////////////////////////////
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

#define MPU6050_ADDRESS         0x68
#define MPUREG_WHOAMI           0x75
#define MPUREG_SMPLRT_DIV       0x19
#define MPUREG_CONFIG           0x1A
#define MPUREG_GYRO_CONFIG      0x1B
#define MPUREG_ACCEL_CONFIG     0x1C
#define MPUREG_FIFO_EN          0x23
#define MPUREG_INT_PIN_CFG      0x37
#define MPUREG_INT_ENABLE       0x38
#define MPUREG_INT_STATUS       0x3A
#define MPUREG_ACCEL_XOUT_H     0x3B
#define MPUREG_ACCEL_XOUT_L     0x3C
#define MPUREG_ACCEL_YOUT_H     0x3D
#define MPUREG_ACCEL_YOUT_L     0x3E
#define MPUREG_ACCEL_ZOUT_H     0x3F
#define MPUREG_ACCEL_ZOUT_L     0x40
#define MPUREG_TEMP_OUT_H       0x41
#define MPUREG_TEMP_OUT_L       0x42
#define MPUREG_GYRO_XOUT_H      0x43
#define MPUREG_GYRO_XOUT_L      0x44
#define MPUREG_GYRO_YOUT_H      0x45
#define MPUREG_GYRO_YOUT_L      0x46
#define MPUREG_GYRO_ZOUT_H      0x47
#define MPUREG_GYRO_ZOUT_L      0x48
#define MPUREG_USER_CTRL        0x6A
#define MPUREG_PWR_MGMT_1       0x6B
#define MPUREG_PWR_MGMT_2       0x6C
#define MPUREG_FIFO_COUNTH      0x72
#define MPUREG_FIFO_COUNTL      0x73
#define MPUREG_FIFO_R_W         0x74


// Configuration bits
#define BIT_SLEEP               0x40
#define BIT_H_RESET             0x80
#define BITS_CLKSEL             0x07
#define MPU_CLK_SEL_PLLGYROX    0x01
#define MPU_CLK_SEL_PLLGYROZ    0x03
#define MPU_EXT_SYNC_GYROX      0x02
#define BITS_FS_250DPS          0x00
#define BITS_FS_500DPS          0x08
#define BITS_FS_1000DPS         0x10
#define BITS_FS_2000DPS         0x18
#define BITS_FS_MASK            0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR    0x10
#define BIT_RAW_RDY_EN          0x01
#define BIT_I2C_IF_DIS          0x10
#define BIT_INT_STATUS_DATA     0x01

#define pi 3.14159 
#define RAD_TO_DEG 57.295779513082320876798154814105



void forward(int speedL=128, int speedR=128);
void backward(int speedL=128, int speedR=128);
void speedControl(int speedL, int speedR);
void stop();
void yuanDiReturn(int speedL=90, int speedR=90);
void gyro_signalen();
void mpu6050_initialize();
void calculate_pid();

void setup()
{
  Serial.begin(115200);
  //S2.begin(9600);
  // 初始化pcf8574对象，默认地址为0x20，如果地址改了，要改！！
  if (!pcf.begin(0x20, &Wire)) {
    Serial.println("Couldn't find PCF8574");
    while (1);
  }
  for (uint8_t p=0; p<8; p++) {
    pcf.pinMode(p, OUTPUT);
  }
    Mirf.spi = &MirfHardwareSpi;
    Mirf.init();
    Mirf.setRADDR((byte *)"RECVE"); //设置自己的地址（发送端地址），使用5个字符
    Mirf.payload = 4;  // 设置传送位数，16位是2，32位是4；
    Mirf.channel = 90;              //设置所用信道
    Mirf.config();

   // Read and print RF_SETUP 无线模块初始化检查
  byte rf_setup = 0;
  Mirf.readRegister( RF_SETUP, &rf_setup, sizeof(rf_setup) );
  Serial.print( "rf_setup = " );
  Serial.println( rf_setup, BIN );
  //舵机初始化
  /*
  servo.attach(pin) 
  servo.attach(pin, min, max)
  min(可选)：脉冲宽度，以微秒为单位，对应于舵机上的最小(0度)角度(默认为544)
  max(可选)：脉冲宽度，以微秒为单位，对应于舵机上的最大(180度)角度(默认为2400)
  */
  fwservo.attach(A3,600,2000);
  fwservo.write(90);//回到中间位置
  fyservo.attach(A0,600,2000);
  fyservo.write(90);//回到中间位置
  //控制引脚设置
  pinMode(left1, OUTPUT);
  //pinMode(left2, OUTPUT);
  pinMode(right1, OUTPUT);
  //pinMode(right2, OUTPUT);
  pinMode(leftPwm, OUTPUT);
  pinMode(rightPwm, OUTPUT);
  stop();
  mpu6050_initialize();    

  delay(20);
 
 //Let's take multiple samples so we can determine the average gyro offset
  Serial.print("Starting calibration...");           //Print message 
   for (int cal_int = 0 ; cal_int <= 100 ; cal_int++){
    gyro_signalen();
    gyro_roll = (gyroRaw[XAXIS]*gyroScaleFactor)*RAD_TO_DEG;
    gyro_pitch = (gyroRaw[YAXIS]*gyroScaleFactor)*RAD_TO_DEG;
    gyro_yaw = (gyroRaw[ZAXIS]*gyroScaleFactor)*RAD_TO_DEG;
     
    gyro_x_cal += gyro_roll;
    gyro_y_cal += gyro_pitch;
    gyro_z_cal += gyro_yaw;
     
    if(cal_int%10 == 0)Serial.print(".");           //Print a dot every 100 readings
     
     digitalWrite(13, LOW);
     delay(20);
     digitalWrite(13, HIGH);
     delay(20);
   } 
   //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset
  Serial.println(" done!");                          //2000 measures are done!
   
   gyro_x_cal = gyro_x_cal/100.0;
   gyro_y_cal = gyro_y_cal/100.0;
   gyro_z_cal = gyro_z_cal/100.0;  
   

    DEBUG("gyro_x_cal: ");DEBUGL(gyro_x_cal);
    DEBUG("gyro_y_cal: ");DEBUGL(gyro_y_cal);
    DEBUG("gyro_z_cal: ");DEBUGL(gyro_z_cal);   
    delay(200);


}
 
void loop()
{
  if(Mirf.dataReady()) {  //当接收到程序，便从串口输出接收到的数据
  Mirf.getData((byte *) &recValue);
  //DEBUG("Recive Data: "); 
  //DEBUGL(recValue,BIN); 
  recUnion.newvalue = recValue;
  //fx = (recValue & 0xff00) >> 8;
  fx = recUnion.buffer[1];
  //ym = recValue & 0xff;
  ym = recUnion.buffer[0];
  //con_value = (recValue >> 24) & 0xff;
  con_value = recUnion.buffer[3];
  //ptValue = (recValue >> 16) & 0xff;
  ptValue = recUnion.buffer[2];
  DEBUG("fx = ");
  DEBUG(recUnion.buffer[1]);
  DEBUG(",");
  DEBUG("ym = ");
  DEBUGL(recUnion.buffer[0]);
  DEBUG("con_value: "); 
  DEBUGL(recUnion.buffer[3],BIN);  
  DEBUG("PaoTa: ");
  DEBUGL(recUnion.buffer[2],BIN);
 }
 delay(100); //这个延迟要放到这里，否则程序错乱，一直会有垃圾数据输出
  //Let's get the current gyro data and scale it to degrees per second for the pid calculations.
  gyro_signalen();
  
    gyro_roll = (gyroRaw[XAXIS]*gyroScaleFactor)*RAD_TO_DEG-gyro_x_cal;
    gyro_pitch = ((gyroRaw[YAXIS]*gyroScaleFactor)*RAD_TO_DEG-gyro_y_cal)*-1;
    gyro_yaw = ((gyroRaw[ZAXIS]*gyroScaleFactor)*RAD_TO_DEG-gyro_z_cal)*-1;
    
    gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll) * 0.2);            //Gyro pid input is deg/sec.
    gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch) * 0.2);         //Gyro pid input is deg/sec.
    gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_yaw) * 0.2);               //Gyro pid input is deg/sec.
    
    // Serial.print(gyro_roll_input);Serial.print("\t");
    // Serial.print(gyro_pitch_input);Serial.print("\t");
    // Serial.print(gyro_yaw_input);Serial.print("\t");  
    // Serial.print("\n");  

//原地360度调头优先
if(bitRead(con_value, 0) == 1) {
  DEBUG("YuanDiReturn!");
  leftSpeed = rightSpeed = map(ym, forwardUP, ymUp, lowSpeed, 255);
  yuanDiReturn(leftSpeed, rightSpeed);
}
else{
  //有刷电机油门控制，控制器的中间位置，保持不动。
  if(ym >= forwardDown and ym <= forwardUP) {
    stop();
  }
  // 前进控制
  //if(ym < forwardDown and ym >= 0) { // 摇杆 前进
  else if(ym > forwardUP and ym <= ymUp) { // 前进 双回位
    //左侧电机速度根据油门大小决定
    leftSpeed = rightSpeed = map(ym, forwardUP, ymUp, lowSpeed, 255); //枪控
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
    //转弯控制
    if(fx > turnUp) { //如果方向偏右,致迪摇杆是反的，右转，右侧的电机速度低
      //leftSpeed = map(fx, turnDown, 0, rightSpeed/turnIndex, lowSpeed); //越往左，速度越慢  摇杆控制 
      rightSpeed = map(fx, turnUp, fxUp, leftSpeed/turnIndex, lowSpeed); //越往左，速度越慢  枪控
    }
    else if(fx <= turnDown) { //否则
      leftSpeed = map(fx, turnDown, fxDown, rightSpeed/turnIndex, lowSpeed); //越往右，速度越慢
      //rightSpeed = map(fx, turnUp, 255, lowSpeed, leftSpeed/turnIndex); //越往右，速度越慢 摇杆控制
    }
   else {
      //在不转弯的情况下，要保持直线行驶，右侧电机的转速通过PID控制
      pid_yaw_setpoint = 0;
      calculate_pid(); 
      //往右机转向时方位输出为正，说明右边的转速需要增加才能保证直线行驶
      int after_speed;
      after_speed=ym+pid_output_yaw;
      if(after_speed >= ymUp) after_speed=ymUp;
      if(after_speed <= forwardUP) after_speed=forwardUP;
      rightSpeed = map(after_speed,forwardUP, ymUp, lowSpeed, 255);
    }
    //DEBUG("leftSpeed: ");
    //DEBUG(leftSpeed);
    //DEBUG(" rightSpeed: ");
    //DEBUGL(rightSpeed);
    if(!isForward){ //如果是倒退的状态，先翻转状态。
      isForward = !isForward;
      stop();
      delay(200);
      forward();
    }
    else{
      forward(leftSpeed, rightSpeed);
    }
    DEBUG("forward...");
    DEBUG("leftSpeed: ");
    DEBUG(leftSpeed);
    DEBUG(" rightSpeed: ");
    DEBUGL(rightSpeed);
    //speedControl(leftSpeed, rightSpeed);
  }
  // 后退控制
  //else if(ym >= forwardUP and ym < 256) { //摇杆 后退
  else if(ym < forwardDown and ym >= ymDown) {
    //左侧电机速度根据油门大小决定
    leftSpeed = rightSpeed = (ym, ymDown, forwardDown, 255, lowSpeed); //枪控
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
    if(fx > turnUp) { //如果方向偏右，实际倒车往左
      //leftSpeed = map(fx,turnDown,0,rightSpeed/turnIndex,lowSpeed); //越往左，速度越慢 //摇杆控制
      rightSpeed = map(fx,turnUp,fxUp,leftSpeed/turnIndex,lowSpeed); //越往左，速度越慢 //枪控
    }
    else if(fx < turnDown){ //否则左转
      //rightSpeed = map(fx, turnUp, 255, leftSpeed/turnIndex, lowSpeed); //越往左，速度越慢  //摇杆控制
      leftSpeed = map(fx, turnDown, fxDown, rightSpeed/turnIndex, lowSpeed); //越往左，速度越慢 //枪控
    }
    else{
      //在不转弯的情况下，要保持直线行驶，右侧电机的转速通过PID控制
      pid_yaw_setpoint = 0;
      calculate_pid(); 
      //往右机转向时方位输出为正，说明右边的转速需要增加才能保证直线行驶
      int after_speed=0;
      after_speed=ym+pid_output_yaw; 
      if(after_speed >= ymUp) after_speed=ymUp;
      if(after_speed <= forwardUP) after_speed=forwardUP;
      rightSpeed = map(after_speed, ymDown, forwardDown, 255, lowSpeed);
    }

    if(isForward){  //如果是前进状态，状态翻转，延迟后再换方向
      isForward = !isForward;
      stop();
      delay(200);
      backward();
    }
    else{
      backward(leftSpeed, rightSpeed);
    }
    DEBUG("backward...");
    DEBUG("leftSpeed: ");
    DEBUG(leftSpeed);
    DEBUG(" rightSpeed: ");
    DEBUGL(rightSpeed);
    //speedControl(leftSpeed, rightSpeed);
  }
  else{
    if(millis()-lastStopTime >= stopInterval){
      stop();
      lastStopTime = millis();
    }
    
    //DEBUGL("motor stop!");
  }
  }
  // 控制值处理
  //pcf.digitalReadByte();
  pcf.digitalWriteByte(con_value<<4);
  //炮台控制 低4位为方位，高4位为俯仰
  //方位控制 如果同上次的值对比，有变化，再操作舵机，避免每次对舵机进行操作。
  fwval = ptValue & 0x0f;
  if(fwval != lastFwval){
    //DEBUG("paota FW: ");
    //DEBUGL(fwval);
    fwval = map(fwval,0,15,160,10);  //160,10为舵机的可转动范围，根据需要调整；
    fwservo.write(fwval);
    lastFwval = fwval;
  }

  //俯仰控制 如果同上次的值对比，有变化，再操作舵机，避免每次对舵机进行操作。
  fyval = (ptValue>>4) & 0x0f;

  if(fyval != lastFyval){
    //DEBUG("paota FY: ");
    //DEBUGL(fyval);
    fyval = map(fyval,0,15,90,135);
    fyservo.write(fyval);
    lastFyval = fyval;
  }
  //返回数据到发送端
  Mirf.setTADDR((byte *)"SENDE");           //设置接收端地址
  if(millis() - lastreport > reportTime) {
    lastreport = millis();
    int batVol = analogRead(A1);
    int current = analogRead(A0);
    sendUnion.buffer[0] = map(batVol,0,1024,0,255);
    sendUnion.buffer[1] = map(current,0,1024,0,255);
    sendUnion.buffer[2] = 0;
    sendUnion.buffer[3] = 0;
    //发送数据并打印
    Mirf.send((byte *)&sendUnion.newvalue);     //发送指令，组合后的数据
    while(Mirf.isSending()) delay(1);          //直到发送成功，退出循环 
    delay(50);
  }

}

//电机速度调整
void speedControl(int speedL, int speedR){
  analogWrite(leftPwm, speedL);
  analogWrite(rightPwm, speedR);
}
//电机前进
void forward(int speedL=128, int speedR=128){
  digitalWrite(left1, LOW);
  digitalWrite(left2, HIGH);
  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);  
  analogWrite(leftPwm, speedL);
  analogWrite(rightPwm, speedR);
}
// 电机后退
void backward(int speedL, int speedR){
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
  digitalWrite(right1, LOW);
  digitalWrite(right2, HIGH);  
  analogWrite(leftPwm, speedL);
  analogWrite(rightPwm, speedR);
}
// 电机停止
void stop(){
  //pcf.digitalWriteByte(0);
  digitalWrite(left1, LOW);
  digitalWrite(left2, LOW);
  digitalWrite(right1, LOW);
  digitalWrite(right2, LOW); 
}
//原地掉头
void yuanDiReturn(int speedL, int speedR){
  digitalWrite(left1, LOW);
  digitalWrite(left2, HIGH);
  digitalWrite(right1, LOW);
  digitalWrite(right2, HIGH);  
  analogWrite(leftPwm, speedL);
  analogWrite(rightPwm, speedR);
}

/////////////////////////////////
//Subroutine for reading the gyro
/////////////////////////////////
void gyro_signalen()
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_GYRO_XOUT_H);
    Wire.endTransmission();
    
    Wire.requestFrom(MPU6050_ADDRESS, 6);
    while(Wire.available() < 6);                       //Wait until the 6 bytes are received
    
    gyroRaw[XAXIS] = ((Wire.read() << 8) | Wire.read());
    gyroRaw[YAXIS] = ((Wire.read() << 8) | Wire.read());
    gyroRaw[ZAXIS] = ((Wire.read() << 8) | Wire.read());
 
}

/////////////////////////////////
//Subroutine for calculating pid outputs
/////////////////////////////////

void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint; // 根据加速度传感器和控制器输入求出偏差；
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp; //偏差乘以pid_gain系数，得到一个值，并且进行累积
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll; // 设置不超过上限
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1; //设置不超过下限，*-1相当于取反
  // 横滚输出 = p增益*偏差+i增益*偏差+d增益*（本次偏差-上次偏差）
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll; //设置不超过上限
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1; //设置不超过下限
  
  pid_last_roll_d_error = pid_error_temp;  // 数据更新
  
  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
    
  pid_last_pitch_d_error = pid_error_temp;
    
  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}

/////////////////////////////////
//Gyro Initilization
/////////////////////////////////

void mpu6050_initialize()
{
    // Chip reset
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_PWR_MGMT_1);
    Wire.write(BIT_H_RESET);
    Wire.endTransmission();  
       
    
    // Startup delay 
    delay(100);  
    
    // Check if sensor is alive
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_WHOAMI);
    Wire.endTransmission();
    
    Wire.requestFrom(MPU6050_ADDRESS, 1);
    
    uint8_t register_value = Wire.read();
    
//    if (register_value == 0x68) {
//        sensors_detected |= GYROSCOPE_DETECTED;
//        sensors_detected |= ACCELEROMETER_DETECTED;
//    } else {
//        return;
//    }   
    
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_INT_PIN_CFG);
    Wire.write(0x02);
    Wire.endTransmission();       

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_PWR_MGMT_1);
    Wire.write(MPU_CLK_SEL_PLLGYROZ);
    Wire.endTransmission();   

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_PWR_MGMT_2);
    Wire.write(0);
    Wire.endTransmission();   

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_SMPLRT_DIV);
    Wire.write(0x00);
    Wire.endTransmission();   

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_CONFIG);
    Wire.write(BITS_DLPF_CFG_42HZ);
    Wire.endTransmission();   

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_GYRO_CONFIG);
    Wire.write(BITS_FS_1000DPS);
    Wire.endTransmission();   

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_ACCEL_CONFIG);
    Wire.write(0x08);
    Wire.endTransmission();    
 
    delay(1500);   
}