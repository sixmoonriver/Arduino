/* 
 *  4脚接运行灯，绿色，控制二级管阴极
 *  3脚接按键，按下按键为低电平
 */
// 定义按键输入针脚号常量，
// 并初始化为2号针脚。
const int buttonPin = 3;
// 定义LED输入针脚号常量，
// 并初始化为13号针脚。
// 注：此处我们使用的LED神灯是Arduino UNO电路板自带，
// 此神灯对应的针脚号默认为13，此数值不得随意更改，
// 所以这里定义的数值13是为了和默认值相对应。
const int ledPin = 4;
// 定义记录LED神灯当前状态的变量，
// 并初始化状态为HIGH。
int ledState = HIGH;
// 定义记录按键当前状态的变量
int buttonState;
// 定义记录按键最近一次状态变化的变量，
// 并初始化状态为LOW。
int lastButtonState = LOW;
// 定义记录最近一次抖动的时间变量，
// 并初始化时间为0毫秒。
long lastDebounceTime = 0;
// 定义延迟抖动的时间变量，
// 并初始化为50毫秒。
long debounceDelay = 50;

// 对Arduino电路板或相关状态进行初始化方法
void setup() {
  // 设置按键的针脚为输入状态
  pinMode(buttonPin, INPUT_PULLUP);
  // 设置电路板上LED神灯的针脚状态为输出状态
  pinMode(ledPin, OUTPUT);
  // 设置电路板上LED神灯的初始状态，
  // 此处因为变量ledState的初始状态为HIGH，
  // 所以LED神灯的初始状态为亮。
  digitalWrite(ledPin, ledState);
}

// 系统调用，无限循环方法
void loop() {
  // 读取按键的状态
  int reading = digitalRead(buttonPin);
  // 判断当前的按键状态是否和之前有所变化
  if (reading != lastButtonState) {
    // 如果按键发生了变化，
    // 则重新设置最近一次抖动的时间。
    lastDebounceTime = millis();
  }
  // 判断按键按下或抬起的状态时间间隔是否大于延迟抖动的时间长度。
  // 方法millis()可以获取当前时间，单位统一为毫秒。
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // 判断当前的按键状态是否和之前有所变化
    if (reading != buttonState) {
      // 如果发生了变化，
      // 则更新按键状态变量。
      buttonState = reading;
      // 判断按键的状态是否为按下，
      // 只有在按键按下的时候，
      // 才改变LED神灯的状态。
      if (buttonState == HIGH) {
        // 如果LED神灯当前为亮度，
        // 则变为灭。如果为灭，
        // 则变为亮。
        ledState = !ledState;
      }
    }
  }
  // 最终改变电路板上LED神灯的状态
  digitalWrite(ledPin, ledState);
  // 更新按键最近一次状态变化的变量
  lastButtonState = reading;
}
