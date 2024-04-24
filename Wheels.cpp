#include <Arduino.h>

#include "Wheels.h"


#define SET_MOVEMENT(side,f,b) digitalWrite( side[0], f);\
                               digitalWrite( side[1], b)

#define BEEPER 13
#define INTINPUT0 A0
#define INTINPUT1 A1
volatile int cnt0=0, cnt1=0;
int cntlimit = 0;

Wheels::Wheels() 
{ }

void Wheels::cntPlus(int n){
  if (n==0){
    cnt0++;
  }else{
    cnt1++;
  }
}
int Wheels::cntGet(int n){
  if (n==0){
    return cnt0;
  }else{
    return cnt1;
  }
}

void Wheels::attachRight(int pF, int pB, int pS)
{
    pinMode(pF, OUTPUT);
    pinMode(pB, OUTPUT);
    pinMode(pS, OUTPUT);
    this->pinsRight[0] = pF;
    this->pinsRight[1] = pB;
    this->pinsRight[2] = pS;
}


void Wheels::attachLeft(int pF, int pB, int pS)
{
    pinMode(pF, OUTPUT);
    pinMode(pB, OUTPUT);
    pinMode(pS, OUTPUT);
    this->pinsLeft[0] = pF;
    this->pinsLeft[1] = pB;
    this->pinsLeft[2] = pS;
}
void Wheels::checkStatus(){
  if (cntlimit!=0){
    if (cnt0>=cntlimit){
      stop();
      cntlimit = 0;
    }
  }
}
void Wheels::goForward2(int cm){
  cntlimit = 2*cm;
  cnt0 = 0;
  cnt1 = 0;
  forward();
}

// void Wheels::goForward(int cm){
//   setSpeed(100);
//   forward();
//   delay(25*cm);
//   stop();
// }
void Wheels::goBack(int cm){
  cntlimit = 2*cm;
  cnt0 = 0;
  cnt1 = 0;
  back();
}

void Wheels::setSpeedRight(uint8_t s)
{
    analogWrite(this->pinsRight[2], s);
}

void Wheels::setSpeedLeft(uint8_t s)
{
    analogWrite(this->pinsLeft[2], s);
}

void Wheels::setSpeed(uint8_t s)
{
    setSpeedLeft(s);
    setSpeedRight(s);
}

void Wheels::attach(int pRF, int pRB, int pRS, int pLF, int pLB, int pLS)
{
    this->attachRight(pRF, pRB, pRS);
    this->attachLeft(pLF, pLB, pLS);
}

void Wheels::forwardLeft() 
{
    SET_MOVEMENT(pinsLeft, HIGH, LOW);
}

void Wheels::forwardRight() 
{
    SET_MOVEMENT(pinsRight, HIGH, LOW);
}

void Wheels::backLeft()
{
    SET_MOVEMENT(pinsLeft, LOW, HIGH);
}

void Wheels::backRight()
{
    SET_MOVEMENT(pinsRight, LOW, HIGH);
}

void Wheels::forward()
{
    this->forwardLeft();
    this->forwardRight();
}

void Wheels::back()
{
    this->backLeft();
    this->backRight();
}

void Wheels::stopLeft()
{
    SET_MOVEMENT(pinsLeft, LOW, LOW);
}

void Wheels::stopRight()
{
    SET_MOVEMENT(pinsRight, LOW, LOW);
}

void Wheels::stop()
{
    this->stopLeft();
    this->stopRight();
}



