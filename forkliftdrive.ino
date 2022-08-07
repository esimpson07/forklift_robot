#include <WiFi.h>
#include <ESP32Servo.h>

#define BAUD_SERIAL 115200
#define RXBUFFERSIZE 1024
#define STACK_PROTECTOR  512
#define MAX_SRV_CLIENTS 5

#define ENA 12
#define IN1 14
#define IN2 27
#define IN3 26
#define IN4 25
#define ENB 33

#define AEN1 32
#define AEN2 35
#define BEN1 34
#define BEN2 39

#define SERVOPIN 13

#define pwmCh1 5
#define pwmCh2 6

Servo sServo;

const int freq = 20000;
const int resolution = 8;
const int MAX_DUTY_CYCLE = (int)(pow(2, resolution) - 1);

int dSpeed = 0;
int sTurn = 0;
int sTarget = 0;
int sTrim = 98; //90 degree offset for straight, 8 degree offset for servo horn

int pastTime = 0;
int timeDelay = 17;
int currentTime = 0;

String sbuf;

WiFiServer server(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];

const char* ssid = "PandoraAccessPoint";
const char* password = "b8070ecd10cd20";

void setup() {
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  pinMode(AEN1,INPUT);
  pinMode(AEN2,INPUT);
  pinMode(BEN1,INPUT);
  pinMode(BEN2,INPUT);
  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  sServo.setPeriodHertz(50);
  sServo.attach(SERVOPIN, 1000, 2000);
  
  ledcAttachPin(ENA, pwmCh1);
  ledcAttachPin(ENB, pwmCh2);

  ledcSetup(pwmCh1, freq, resolution);
  ledcSetup(pwmCh2, freq, resolution);

  
  Serial.begin(115200);

  serverInit();
}

void serverInit() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.println();
  Serial.print("connected, address=");
  Serial.println(WiFi.localIP());
  server.begin();
  server.setNoDelay(true);
  delay(1000);
}

void lift(int driveSpeed) {
  /*
   * MUST BE CHANGED!!!!
   * 
   * 
   * 
   * 
   * 
   * 
   * REMEMBER TO NOT ACTUALLY APPLY THIS!!!!!!
   */
   if(driveSpeed > 0) {
    ledcWrite(pwmCh2, 255);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
  } else if(driveSpeed < 0) {
    ledcWrite(pwmCh2, 255);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
  } else {
    ledcWrite(pwmCh2, 0);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,HIGH);
  }
}

void drive(int driveSpeed) {
  if(driveSpeed > 0) {
    ledcWrite(pwmCh1, abs(driveSpeed));
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
  } else if(driveSpeed < 0) {
    ledcWrite(pwmCh1, abs(driveSpeed));
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
  } else {
    ledcWrite(pwmCh1, 0);
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,HIGH);
  }
}

void serverUpdate(){
  if (server.hasClient()) {
    for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (!serverClients[i]) {
        serverClients[i] = server.available();
        Serial.print("New client idx: ");
        Serial.println(i);
        break;
      }

      if (i >= MAX_SRV_CLIENTS) {
        server.available().println("no sessions available");
        Serial.print("Server exceeded max connections: ");
        Serial.println(MAX_SRV_CLIENTS);
      }
    }
  }

  for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
    while (serverClients[i].available()) {
      size_t maxToSerial = std::min(serverClients[i].available(), Serial.availableForWrite());
      maxToSerial = std::min(maxToSerial, (size_t)STACK_PROTECTOR);
      uint8_t buf[maxToSerial];
      size_t tcp_got = serverClients[i].read(buf, maxToSerial);
      sbuf = String((char*)buf);
      if(sbuf.substring(0,3).toInt() - 510 != -510) {
        dSpeed = sbuf.substring(0,3).toInt() - 510;
      }
      if((sbuf.substring(3,6).toInt() - 510) / (17 / 3) != -102) {
        sTarget = -(sbuf.substring(3,6).toInt() - 510) / (17 / 3);
      }  
      if(abs(sTarget) > 45) {
        if(sTarget > 0){
          sTarget = 45;
        } else {
          sTarget = -45;
        }
      }
      Serial.println(dSpeed);
      Serial.println(sTurn);
    }
  }
}

void loop() {
  currentTime = millis();
  drive(dSpeed);
  if(sTurn != sTarget && currentTime >= timeDelay + pastTime) {
    pastTime = currentTime;
    if(sTarget > sTurn) {
      sTurn ++;
    } else {
      sTurn --;
    }
  }
  sServo.write(sTurn + sTrim);
  serverUpdate();
}
