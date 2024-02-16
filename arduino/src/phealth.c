#include <Arduino.h>
#include <SetTimeout.h>
#include <mbed.h>
//#include <WiFi.h>

#define TICKS 10   //msec
#define VCAL 3.25 //V
#define DCAL 1024 //bits

#define TEMP_SENS A1
#define TEMP_ENABLE D0
#define TEMP_GND D2
#define TEMP_CAL 0.01  //V/deg
#define TEMP_ZERO 0.6  // V at zero
#define TEMP_THRES 40

#define TEMP_LAMP D9
#define UPS_LAMP D8
#define RUN_LAMP D7

float temp=0;
int battery=0;
int run=0;
int error=0;

void sampler(){
  digitalWrite(TEMP_ENABLE,HIGH);
  int aval=analogRead(TEMP_SENS);
  float volt=VCAL*aval/DCAL;
  float t=(volt-TEMP_ZERO)/TEMP_CAL;
  temp=temp+(t-temp)*0.1;
  setTimeout.set(sampler,100);
}
void logger(){
  Serial.print("inner_temp=");
  Serial.println(temp);
  setTimeout.set(logger,1000);
}

void TEMP_scan(){
  if(temp>TEMP_THRES){
    digitalWrite(TEMP_LAMP,HIGH);
    setTimeout.set([]{
      digitalWrite(TEMP_LAMP,LOW);
    },50);
    setTimeout.set(TEMP_scan,700);
  }
  else{
    digitalWrite(TEMP_LAMP,LOW);
    setTimeout.set(TEMP_scan,1000);
  }
}

void UPS_scan(){
  if(battery==0){
    digitalWrite(UPS_LAMP,HIGH);
    setTimeout.set([]{
      digitalWrite(UPS_LAMP,LOW);
    },50);
    setTimeout.set(UPS_scan,700);
  }
  else{
    digitalWrite(UPS_LAMP,HIGH);
    setTimeout.set(UPS_scan,1000);
  }
}

void RUN_scan(){
  if(run==0){
    digitalWrite(RUN_LAMP,HIGH);
    setTimeout.set([]{
      digitalWrite(RUN_LAMP,LOW);
    },50);
    setTimeout.set(RUN_scan,700);
  }
  else{
    digitalWrite(RUN_LAMP,HIGH);
    setTimeout.set(RUN_scan,1000);
  }
}

void setup() {
//  WiFi.disconnect(true);
//  WiFi.mode(WIFI_OFF);
  Serial.begin(115200);
  pinMode(TEMP_ENABLE,OUTPUT);
  pinMode(TEMP_GND,OUTPUT);
  pinMode(RUN_LAMP,OUTPUT);
  pinMode(UPS_LAMP,OUTPUT);
  pinMode(TEMP_LAMP,OUTPUT);
  logger();
  sampler();
  run=1;
  RUN_scan();
  UPS_scan();
  TEMP_scan();
}

void loop() {
  setTimeout.spinOnce();
  Serial.setTimeout(100);
  String str = Serial.readString();
  int index = split(str, '=', args);
  switch (args[0]) {
    case "battery":
      battery = args[1].toInt();
    case "error":
      error = args[1].toInt();
    default:
      print("Unprocessable protocol.");
  }
  delay(10);
}
