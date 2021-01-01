#include "PWM.hpp"

const byte MAX_ISR_COUNT = 20;

byte isr_count = 0;
byte isr_pin[MAX_ISR_COUNT];
unsigned int isr_value[MAX_ISR_COUNT];
bool isr_last_state[MAX_ISR_COUNT];
bool isr_trigger_state[MAX_ISR_COUNT];
unsigned long isr_timer[MAX_ISR_COUNT];
unsigned long isr_age[MAX_ISR_COUNT];

void ISR_generic(byte isr) {
    unsigned long now = micros();
    bool state_now = digitalRead(isr_pin[isr]);
    if (state_now != isr_last_state[isr]) {
        if (state_now == isr_trigger_state[isr]) {
            isr_timer[isr] = now;
        } else {
            isr_value[isr] = (unsigned int)(now - isr_timer[isr]);
            isr_age[isr] = now;
        }
        isr_last_state[isr] = state_now;
    }
}

void ISR_0() {
    ISR_generic(0);
}

void ISR_1() {
    ISR_generic(1);
}

void ISR_2() {
    ISR_generic(2);
}

void ISR_3() {
    ISR_generic(3);
}

void ISR_4() {
    ISR_generic(4);
}

void ISR_5() {
    ISR_generic(5);
}

void ISR_6() {
    ISR_generic(6);
}

void ISR_7() {
    ISR_generic(7);
}

void ISR_8() {
    ISR_generic(8);
}

void ISR_9() {
    ISR_generic(9);
}

void ISR_10() {
    ISR_generic(10);
}

void ISR_11() {
    ISR_generic(11);
}

void ISR_12() {
    ISR_generic(12);
}

void ISR_13() {
    ISR_generic(13);
}

void ISR_14() {
    ISR_generic(14);
}

void ISR_15() {
    ISR_generic(15);
}

void ISR_16() {
    ISR_generic(16);
}

void ISR_17() {
    ISR_generic(17);
}

void ISR_18() {
    ISR_generic(18);
}

void ISR_19() {
    ISR_generic(19);
}

PWM::PWM(byte pin) {
    my_isr = isr_count;
    isr_count++;
    
    isr_pin[my_isr] = pin;
    pinMode(isr_pin[my_isr], INPUT);
}

int PWM::begin(bool measure_pulse_high) {
    isr_last_state[my_isr] = digitalRead(isr_pin[my_isr]);
    isr_trigger_state[my_isr] = measure_pulse_high;
    
    switch (my_isr) {
        case 0:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_0, CHANGE);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_1, CHANGE);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_2, CHANGE);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_3, CHANGE);
            break;
        case 4:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_4, CHANGE);
            break;
        case 5:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_5, CHANGE);
            break;
        case 6:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_6, CHANGE);
            break;
        case 7:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_7, CHANGE);
            break;
        case 8:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_8, CHANGE);
            break;
        case 9:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_9, CHANGE);
            break;
        case 10:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_10, CHANGE);
            break;
        case 11:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_11, CHANGE);
            break;
        case 12:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_12, CHANGE);
            break;
        case 13:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_13, CHANGE);
            break;
        case 14:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_14, CHANGE);
            break;
        case 15:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_15, CHANGE);
            break;
        case 16:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_16, CHANGE);
            break;
        case 17:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_17, CHANGE);
            break;
        case 18:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_18, CHANGE);
            break;
        case 19:
            attachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]), ISR_19, CHANGE);
            break;
        default:
            return -1; // Error.
    }
    return 0; // Success.
}

unsigned int PWM::getValue() {
    return isr_value[my_isr];
}

void PWM::end() {
    detachInterrupt(digitalPinToInterrupt(isr_pin[my_isr]));
}

unsigned long PWM::getAge() {
    return (micros() - isr_age[my_isr]);
}
