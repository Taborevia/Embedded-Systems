#include "Wheels.h"
#include <LiquidCrystal_I2C.h>
#include "TimerOne.h"
#include "PinChangeInterrupt.h"
#include "lcd.h"
#include <IRremote.h>
#include <Servo.h>

// piny dla sonaru (HC-SR04)
#define TRIG A2
#define ECHO A3
// pin kontroli serwo (musi być PWM)
#define SERVO 3

#define BEEPER 13
#define INTINPUT0 A0
#define INTINPUT1 A1

Servo serwo;

long int intPeriod = 700000;
Wheels w;
LCD l;
volatile char cmd;
const int RECV_PIN = 2;
IRrecv irrecv(RECV_PIN);
decode_results results;
int receivedCode = 0;
int speed = 0;
unsigned long int time;
unsigned int distance;
int state;
bool autoPilot = false;
int lastPilotCode;
unsigned long oldTime = millis();
unsigned long timeFromLastSignal = 0;

void setup() {
  // put your setup code here, to run once:
  w.attach(10,8,9,6,7,5);
  // w.attach(8,12,11,2,4,3);
  pinMode(TRIG, OUTPUT);    // TRIG startuje sonar
  pinMode(ECHO, INPUT);  

  Serial.begin(9600);
  Serial.println("Forward: WAD");
  Serial.println("Back: ZXC");
  Serial.println("Stop: S");
  
  
  // Timer1.initialize();
  pinMode(BEEPER, OUTPUT);

  pinMode(INTINPUT0, INPUT);
  pinMode(INTINPUT1, INPUT);
  l.start();
  irrecv.enableIRIn();
  irrecv.blink13(true);  

  PCICR  = 0x02;  // włącz pin change interrupt dla 1 grupy (A0..A5)
  PCMSK1 = 0x03;  // włącz przerwanie dla A0, A1
}


// aktualizuje Timer1 aktualną wartością intPeriod
// void timerUpdate() {
//   Timer1.detachInterrupt();
//   Timer1.attachInterrupt(doBeep, intPeriod);
// }
void calculateDistance(){
  digitalWrite(TRIG, HIGH);
  delay(10);
  digitalWrite(TRIG, LOW);
  time = pulseIn(ECHO, HIGH);
  distance = time/58;
}
// zmienia wartość pinu BEEPER
void doBeep() {
  digitalWrite(BEEPER, digitalRead(BEEPER) ^ 1);
}
void stop(){
  w.setSpeed(speed);
  l.stop();
  w.stop();
  Timer1.detachInterrupt();
  digitalWrite(BEEPER, 0);
}void forward(){
  w.setSpeed(speed);
  w.forward();
  l.forward();
  Timer1.detachInterrupt();
  digitalWrite(BEEPER, 0);
}
void back(){
  // intPeriod += 100000; timerUpdate();
  w.setSpeed(speed);
  w.back();
  l.back();
}
void goRight(){
  forward();
  right();
  delay(500);
  w.stop();
}
void right(){
  w.forwardLeft();
  l.right();
  w.setSpeedLeft(speed);
  w.setSpeedRight(0);
}
void left(){
  w.forwardRight();
  l.left();
  w.setSpeedLeft(0);
  w.setSpeedRight(speed);
}
void setSpeed(int value){
  speed = value;
  w.setSpeed(value);
  l.clear(1,3,0);
  l.clear(12,14,0);
  l.write(1,0,value);
  l.write(12,0,value);
}
bool checkDistance(){
  calculateDistance();
  l.clear(7,10,0);
  l.write(7,0,distance);
  if (distance < 30){
    stop();
    state = 0;
    return false;
  }else{
  return true;
  }
}

void obstacle(){

}
void checkSignal(){
  timeFromLastSignal = millis() - oldTime;
  if (IrReceiver.decode()) {
      oldTime = millis();
      receivedCode = (IrReceiver.decodedIRData.command);
      lastPilotCode = receivedCode;
      switch(IrReceiver.decodedIRData.command){
        case 0x18: if(checkDistance()){Serial.println(distance);forward();} state = 1; break;
        case 0x52: back(); state = 2; l.clear(7,10,0); break;
        case 0x8: left(); break;
        case 0x5A: right(); break;
        case 0x1C: stop(); state = 0; l.clear(7,10,0); break;
        case 0x45: setSpeed(50); break;
        case 0x46: setSpeed(100); break;
        case 0x47: setSpeed(150); break; 
        case 0x44: setSpeed(200); break; 
        case 0x40: setSpeed(250); break;
        case 0x43: setSpeed(255); break;
        case 0x19: setSpeed(0); break;
        case 0xD: autoPilot = !autoPilot; state = 1; forward(); setSpeed(100); break;
        default: IrReceiver.printIRResultShort(&Serial);break;
      }
      IrReceiver.resume(); // Enable receiving of the next value
    }else if(timeFromLastSignal>100){
      // Serial.println(timeFromLastSignal);
      stop(); state = 0; l.clear(7,10,0);
    }  
}
void loop() {
  // Serial.println(autoPilot);
  // states: 0 - stop, 1 - forward, 2 - back, 3 - goBack, 4 - turnRight
  if(autoPilot){
    switch(state){
      case 0: break;
      case 1: checkDistance(); break;
      case 2: break;
      case 3: w.goBack(10); state = 4; break;
      case 4: goRight(); state = 1; forward(); break; 
    }
  }else{
    switch(state){
      case 0: break;
      case 1: checkDistance(); break;
      case 2: break;
    }
  }
  checkSignal();
}
ISR(PCINT1_vect) {
  if(digitalRead(INTINPUT0))
    w.cntPlus(0);
  else if(digitalRead(INTINPUT1))
    w.cntPlus(1);
}
