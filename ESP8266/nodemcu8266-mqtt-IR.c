#include <ESP8266WiFi.h>
#include <PubSubClient.h>
// Define NodeMCU D3 pin connect to LED
#define LED_PIN 2
#define buttonPin 0 

// Update these with values suitable for your network.
const char* ssid = "Wange";
const char* password = "shangY0012#$";
const char* mqtt_server = "192.168.18.106";
//const char* mqtt_server = "iot.eclipse.org 5";

int buttonState = 0;             // 记录按键的状态
int lastButtonState = 0;   // 上一次按键的状态
int LedState = 1; 
// 以下代码以long类型声明，因为时间值以毫秒为单位(用整型会很快溢出)
long lastDebounceTime = 0;  // 按键最后一次被触发
//long debounceDelay = 50;    // 为了滤去抖动暂停的时间，如果发现输出不正常增加这个值
long debounceDelay = 50000000; //灯持续的时间；

int getIRsensor() {
  // 读取按键状态并存储到变量中:
  int reading = digitalRead(buttonPin);
 //检查按键是否与上一次状态不同，如果不同，记录下当前启动的时间；
  if (reading) {
    lastDebounceTime = millis();
  //  Serial.println("Button is pressed,but....");
  }
  //lastButtonState = reading;
// 如果当前的时间减去上触发的时间小于灯亮的时间 ；
  if ((millis() - lastDebounceTime) < debounceDelay) {    
    // 如果按键状态改变了，将当前状态保存为上一次状态，打印、并且返回1，如果没有改变，返回0
      return 1;
    }
	else{
	  return 0;	
	}
  }

int getswchange() {
  // 读取按键状态并存储到变量中:
  int reading = digitalRead(buttonPin);
 //检查按键是否与上一次状态不同，如果不同，记录下当前启动的时间；
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  //  Serial.println("Button is pressed,but....");
  }
  lastButtonState = reading;
// 如果当前的时间减去上次按键状态变化的时间间隔大于抖动时间，说明按键状态进入稳定期；
  if ((millis() - lastDebounceTime) > debounceDelay) {    
    // 如果按键状态改变了，将当前状态保存为上一次状态，打印、并且返回1，如果没有改变，返回0
    if (reading != buttonState) {
      buttonState = reading;
      lastButtonState = buttonState;
      Serial.println("Button is pressed");
      return 1;
    }
  }
  return 0;
}

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup_wifi() {
delay(100);
// We start by connecting to a WiFi network
Serial.print("Connecting to ");
Serial.println(ssid);
WiFi.begin(ssid, password);
while (WiFi.status() != WL_CONNECTED)
{
delay(500);
Serial.print(".");
}
randomSeed(micros());
Serial.println("");
Serial.println("WiFi connected");
Serial.println("IP address: ");
Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length)
{
Serial.print("Command from MQTT broker is : [");
Serial.print(topic);
int p =(char)payload[0]-'0';
// if MQTT comes a 0 turn off LED on D2
if(p==0)
{
digitalWrite(LED_PIN, LOW);
LedState = 0;
//lastDebounceTime = millis() - debounceDelay -10;
Serial.println(" Turn Off LED! " );
}
// if MQTT comes a 1, turn on LED on pin D2
if(p==1)
{
digitalWrite(LED_PIN, HIGH);
LedState = 1;
//lastDebounceTime = millis();
Serial.println(" Turn On LED! " );
}
Serial.println();
} //end callback

void reconnect() {
// Loop until we're reconnected
while (!client.connected())
{
Serial.print("Attempting MQTT connection…");
// Create a random client ID
String clientId = "ESP8266Client-";
clientId += String(random(0xffff), HEX);
// Attempt to connect
//if you MQTT broker has clientID,username and password
//please change following line to if (client.connect(clientId,userName,passWord))
//if (client.connect(clientId.c_str()))
if (client.connect(clientId.c_str(),"mqtt","gameingo"))
{
Serial.println("connected");
//once connected to MQTT broker, subscribe command if any
client.subscribe("DoorLight");
} else {
Serial.print("failed, rc=");
Serial.print(client.state());
Serial.println(" try again in 5 seconds");
// Wait 6 seconds before retrying
delay(6000);
}
}
} //end reconnect()
void setup() {
pinMode(buttonPin, INPUT);
Serial.begin(115200);
setup_wifi();
client.setServer(mqtt_server, 1883);
client.setCallback(callback);
pinMode(LED_PIN, OUTPUT);
digitalWrite(LED_PIN, LOW);
}

void loop() {
if (!client.connected()) {
reconnect();
}
client.loop(); 

if (getIRsensor){
	digitalWrite(LED_PIN,  LOW);
  }
  else{
    digitalWrite(LED_PIN,  HIGH);
  }
}