#include "Arduino.h"
#include <LiquidCrystal_I2C.h>
#include "lcd.h"

uint8_t arrowUp[8] =
{
    0b00100,
    0b01110,
    0b11111,
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b00100
};
uint8_t arrowDown[8] =
{
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b11111,
    0b01110,
    0b00100
};
uint8_t arrowLeft[8] =
{
    0b00010,
    0b00100,
    0b01000,
    0b10000,
    0b10000,
    0b01000,
    0b00100,
    0b00010
};
uint8_t arrowRight[8] =
{
    0b01000,
    0b00100,
    0b00010,
    0b00001,
    0b00001,
    0b00010,
    0b00100,
    0b01000
};

byte LCDAddress = 0x27;
LiquidCrystal_I2C lcd(LCDAddress, 16, 2);

LCD::LCD(){}

void LCD::start(){
lcd.init();
lcd.backlight();
lcd.createChar(0, arrowDown);
lcd.createChar(1, arrowUp);
lcd.createChar(2, arrowLeft);
lcd.createChar(3, arrowRight);
}
void LCD::clear(int x1, int x2, int y){
lcd.setCursor(x1,y);
while (x1<=x2){
  lcd.print(" ");
  x1++;
}
}

void LCD::write(int x, int y, int input){
  lcd.setCursor(x,y);
  lcd.print(input);
}
void LCD::leftForward(){
  lcd.setCursor(0, 0);
  lcd.write(1);
  lcd.setCursor(0, 1);
  lcd.print('|');
}
void LCD::rightForward(){
  lcd.setCursor(15, 0);
  lcd.write(1);
  lcd.setCursor(15, 1);
  lcd.print('|');
}
void LCD::leftBack(){
  lcd.setCursor(0, 0);
  lcd.print('|');
  lcd.setCursor(0, 1);
  lcd.write(0);
}
void LCD::rightBack(){
  lcd.setCursor(15, 0);
  lcd.print('|');
  lcd.setCursor(15, 1);
  lcd.write(0);
}
void LCD::stop(){
  lcd.setCursor(0,0);
  lcd.print('-');
  lcd.setCursor(0,1);
  lcd.print('-');
  lcd.setCursor(15, 0);
  lcd.write('-');
  lcd.setCursor(15, 1);
  lcd.print('-');
}
void LCD::forward(){
  leftForward();
  rightForward();
}
void LCD::back(){
  leftBack();
  rightBack();
}
void LCD::left(){
  lcd.setCursor(0,0);
  lcd.write(2);
  lcd.setCursor(0,1);
  lcd.write(2);
  lcd.setCursor(15, 0);
  lcd.write(2);
  lcd.setCursor(15, 1);
  lcd.write(2);
}
void LCD::right(){
  lcd.setCursor(0,0);
  lcd.write(3);
  lcd.setCursor(0,1);
  lcd.write(3);
  lcd.setCursor(15, 0);
  lcd.write(3);
  lcd.setCursor(15, 1);
  lcd.write(3);
}

