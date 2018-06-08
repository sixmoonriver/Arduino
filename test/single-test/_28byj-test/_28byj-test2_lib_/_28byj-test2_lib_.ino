#include<Stepper.h>
 
// 参考文件
/*
    28BYJ-48.pdf
 
    该参数根据电机每一转的步数来修改
*/
const int stepsPerRevolution = 100;
 
// 初始化步进电机要使用的Arduino的引脚编号
Stepper myStepper(stepsPerRevolution, 2, 3, 4, 5);
 
void setup()
{
    // 设置转速，单位r/min
    myStepper.setSpeed(60);
 
    // 初始化串口
    Serial.begin(9600);
}
 
void loop()
{
    // 顺时针一次旋转
    Serial.println("clockwise");
    myStepper.step(stepsPerRevolution);
    delay(500);
 
    // 逆时针一次旋转
    Serial.println("counterclockwise");
    myStepper.step(-stepsPerRevolution);
    delay(500);
}
