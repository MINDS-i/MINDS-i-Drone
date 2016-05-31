#pragma once

namespace ServoPWM{
    void setup(uint16_t frameMicroseconds){
        uint16_t frameTicks = frameMicroseconds * 2;

        // Fast PWM mode; WGM = 14 (0b1110)
        //   TOP is ICRn
        //   TOVn flag set on TOP
        //   OCRnX updated at BOTTOM
        // 8x prescalar; CS = 2 (0b010)
        //   One tick per 8 clock cycles

        TCCR1A =((1<<WGM11));
        TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
        ICR1   = frameTicks;
        OCR1A  = 0xFFFF;
        OCR1B  = 0xFFFF;

        TCCR4A =((1<<WGM41));
        TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
        ICR4   = frameTicks;
        OCR4A  = 0xFFFF;
        OCR4B  = 0xFFFF;
        OCR4C  = 0xFFFF;

        TCCR3A =((1<<WGM31));
        TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
        ICR3   = frameTicks;
        OCR3A  = 0xFFFF;
        OCR3B  = 0xFFFF;
        OCR3C  = 0xFFFF;

        pinMode(12, OUTPUT); // CH_1 (PB6/OC1B)
        pinMode(11, OUTPUT); // CH_2 (PB5/OC1A)
        pinMode(8,  OUTPUT); // CH_3 (PH5/OC4C)
        pinMode(7,  OUTPUT); // CH_4 (PH4/OC4B)
        pinMode(6,  OUTPUT); // CH_5 (PH3/OC4A)
        pinMode(3,  OUTPUT); // CH_6 (PE5/OC3C)
        pinMode(2,  OUTPUT); // CH_7 (PE4/OC3B)
        pinMode(5,  OUTPUT); // CH_8 (PE3/OC3A)

        TCCR1A |= (1<<COM1B1); // CH_1 : OC1B
        TCCR1A |= (1<<COM1A1); // CH_2 : OC1A
        TCCR4A |= (1<<COM4C1); // CH_3 : OC4C
        TCCR4A |= (1<<COM4B1); // CH_4 : OC4B
        TCCR4A |= (1<<COM4A1); // CH_5 : OC4A
        TCCR3A |= (1<<COM3C1); // CH_6 : OC3C
        TCCR3A |= (1<<COM3B1); // CH_7 : OC3B
        TCCR3A |= (1<<COM3A1); // CH_8 : OC3A
    }

    void set(uint8_t output, uint16_t us){
        // in mode 14 (Fast PWM) OCRnX is buffered automatically
        uint16_t pwm = us*2;
        switch(output){
            case 1:  OCR1B=pwm; break;  // out1
            case 2:  OCR1A=pwm; break;  // out2
            case 3:  OCR4C=pwm; break;  // out3
            case 4:  OCR4B=pwm; break;  // out4
            case 5:  OCR4A=pwm; break;  // out5
            case 6:  OCR3C=pwm; break;  // out6
            case 7:  OCR3B=pwm; break;  // out7
            case 8:  OCR3A=pwm; break;  // out8
        }
    }

    class APMoutput{
        uint8_t pin;
        APMoutput(uint8_t APMoutput): pin(APMoutput) {}
        void write(uint8_t sig){
            set(pin, sig*10 + 600); // convert [0,180] to [600,2400]
        }
        void writeMicroseconds(uint16_t us){
            set(pin, us);
        }
    };
}
