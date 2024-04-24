/* 
 * prosta implementacja klasy obsługującej 
 * silniki pojazdu za pośrednictwem modułu L298
 *
 * Sterowanie odbywa się przez:
 * 1)  powiązanie odpowiednich pinów I/O Arduino metodą attach() 
 * 2)  ustalenie prędkości setSpeed*()
 * 3)  wywołanie funkcji ruchu
 *
 * TODO:
 *  - zabezpieczenie przed ruchem bez attach()
 *  - ustawienie domyślnej prędkości != 0
 */


#include <Arduino.h>


#ifndef LCD_h
#define LCD_h



class LCD {
    public: 
        LCD();
        void start();
        void write(int x, int y, int input);
        void clear(int x1, int x2, int y);
        void left();
        void right();
        void leftForward();
        void rightForward();
        void leftBack();
        void rightBack();
        void forward();
        void back();
        void stop();
    private: 
};



#endif
