/*
 * File:   main.c
 * Author: abusa
 *
 * Created on May 10, 2025, 8:35 PM
 */

/*
 * File:   main.c
 * Author: abusa
 * Created on May 10, 2025, 8:35 PM
 */

#pragma config FOSC = HS
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config BOREN = OFF
#pragma config LVP = OFF
#pragma config CPD = OFF
#pragma config WRT = OFF
#pragma config CP = OFF

#include <xc.h>
#include <stdio.h>


#define _XTAL_FREQ 8000000
#define TEMP_THRESHOLD 19
#define LVL_THRESHOLD 5


void delay_ms(unsigned int);
void delay_us(unsigned int);
void init_timer();
void Initialize_Hbridge();
unsigned int adc();
void Ultrasonic_Init(void);
unsigned int Read_Ultrasonic(void);
void PWM_Init_S();
void PWM_Init_H();
void Set_PWM_Duty(unsigned int);
void openDoor();
void closeDoor();
void H_Bridge_PWM(unsigned int);
void Pump_on();
void Pump_off();
void Fan_on();
void Fan_off();
void init_interrupt();
void __interrupt() myISR(void);
void isr();




unsigned int cntr = 0;
unsigned int tmp;
char water_flag = 0;

void main(void) {
    Ultrasonic_Init();
    Initialize_Hbridge();
    PWM_Init_S();

    TRISB = TRISB & 0xFE;  // RB0 output (RELAY SPIN)
    PORTB = PORTB | 0x01;  // RB0 high (relay off)
    TRISB = TRISB | 0x02;  // RB1 input (IR)
    TRISD = TRISD | 0x01;  // RD0 input (IR)
    TRISC = TRISC & 0xDF;  // RC5 output (Buzzer)
    TRISC = TRISC & 0xBF;  // RC6 output (Relay)
    TRISD = TRISD | 0x10;  // RD4 input (Push button)
    TRISD = TRISD & 0xF7;  // RD3 output (LED)
    PORTD = PORTD & 0xF7;
    TRISD = TRISD & 0xFB;  // RD2 output (LED)
    PORTD = PORTD & 0xFB;
    PORTB = PORTB & 0x3F;  // RB6, RB7 low (fan off)

    openDoor();
    while (!(PORTD & 0x10)) {
        asm("nop");
    }
    closeDoor();

    delay_ms(1000);
    PORTD = PORTD & 0xF3;  // Clear RD3, RD2
    delay_ms(1000);
    PORTD = PORTD | 0x0C;  // Set RD3, RD2
    delay_ms(1000);
    PORTD = PORTD & 0xF3;  // Clear RD3, RD2 again

    init_interrupt();

    while (1) {
        tmp = adc();
        if (tmp >= 0 && tmp <= 20);
        else if (tmp < TEMP_THRESHOLD) {
            Fan_on();
        } else {
            Fan_off();
        }

        unsigned int lev = Read_Ultrasonic();
        if (lev != (unsigned int)-1) {
            if (lev > LVL_THRESHOLD) {
                if (water_flag == 0) {
                    INTCON = INTCON & 0x7F;  // GIE = 0
                    PORTC = PORTC | 0x20;    // RC5 high
                    delay_ms(250);
                    PORTC = PORTC & 0xDF;    // RC5 low
                    water_flag = 1;
                    INTCON = INTCON | 0x80;  // GIE = 1
                }
            } else {
                water_flag = 0;
            }
        }
    }
}

void init_interrupt() {
    TMR0 = 0;
    OPTION_REG = 0x07;
    INTCON = INTCON | 0xA0;  // T0IE=1, GIE=1
}

void isr() {
    PORTB = PORTB & 0xFE; // RB0 = 0
    delay_ms(300);
    PORTB = PORTB | 0x01; // RB0 = 1
    delay_ms(2500);

    if (PORTD & 0x01) {
        openDoor();
        while (!(PORTD & 0x10)) {
            asm("nop");
        }
        closeDoor();
        return;
    }

    char pill_taken = 0;

    while (1) {
        PORTC = PORTC | 0x20; // RC5 = 1

        if (!pill_taken)
            PORTD = PORTD | 0x04; // RD2 = 1
        else
            PORTD = PORTD & 0xFB; // RD2 = 0

        if (!water_flag && (PORTB & 0x02)) {
            PORTD = PORTD | 0x08; // RD3 = 1
        }

        if (!water_flag && !(PORTB & 0x02)) {
            PORTD = PORTD & 0xF7; // RD3 = 0
            PORTC = PORTC & 0xDF; // RC5 = 0
            Pump_on();
            Pump_off();
            PORTC = PORTC | 0x20; // RC5 = 1
            water_flag = 1;
        }

        if (water_flag)
            PORTD = PORTD & 0xF7; // RD3 = 0

        if (PORTD & 0x01) {
            pill_taken = 1;
        }

        if (pill_taken && water_flag) {
            PORTC = PORTC & 0xDF; // RC5 = 0
            water_flag = 0;
            PORTD = PORTD & 0xFB; // RD2 = 0
            return;
        }
    }
}

void __interrupt() myISR(void) {
    if (INTCON & 0x04) { // Check T0IF
        TMR0 = 0;
        cntr++;
        if (cntr == 625) {
            INTCON = INTCON & 0x7F; // GIE = 0
            cntr = 0;
            isr();
            INTCON = INTCON | 0x80; // GIE = 1
        }
        INTCON = INTCON & 0xFB; // Clear T0IF
    }
}




void PWM_Init_S() {
      // Set RC2 as output
    PR2 = 249; // Period register for 50 Hz (20 ms)
    T2CON = 0b00000111; // Timer2 ON, Prescaler 1:16
    CCP1CON = 0b00001100; // CCP1 in PWM mode
    CCPR1L = 0; // Initial duty cycle (0%)
}


void PWM_Init_H() {
    // Set RC1 as output (PWM2)

    // Configure CCP2 module
    CCP2CON = 0b00001100; // PWM mode

    //     Set PWM frequency: PR2 = [(Fosc / (PWM_freq * 4 * TMR2_Prescale)) - 1]
    // For 5kHz PWM, PR2 = [(8000000 / (5000 * 4 * 16)) - 1] = ~24
    PR2 = 199; // Use this for ~1kHz PWM with prescaler 16 (adjust as needed)

    T2CON = 0b00000111; // Timer2 on, prescaler 1:16

    TMR2 = 0; // Clear Timer2
}

void H_Bridge_PWM(unsigned int duty_percent) {
    unsigned int duty;

    // Convert percentage to 10-bit duty (max 1023)
    duty = ((unsigned long) duty_percent * 1023) / 100;

    // Set the upper 8 bits of the duty cycle
    CCPR2L = duty >> 2;

    // Clear the two LSBs in CCP2CON (bits 5 and 4)
    CCP2CON &= 0xCF; // 11001111

    // Set CCP2X (bit 5) and CCP2Y (bit 4) based on LSBs of duty
    CCP2CON |= ((duty & 0x03) << 4); // shift lower 2 bits into positions 4 and 5
}


// Set PWM Duty Cycle for Servo (Pulse Width)

void Set_PWM_Duty(unsigned int duty_us) {
    // Calculate the PWM duty cycle from the pulse width (in microseconds)
    unsigned int duty = (duty_us * (_XTAL_FREQ / 4)) / (16 * (PR2 + 1) * 1000);
    
    // Set the upper 8 bits of the duty cycle in CCPR1L
    CCPR1L = duty >> 2; // Upper 8 bits
    
    // Clear CCP1X and CCP1Y bits in CCP1CON to prepare for the lower 2 bits
    CCP1CON &= 0xCF; // 0xCF = 11001111, clears bits 5 and 4
    
    // Set CCP1X (bit 5) and CCP1Y (bit 4) based on the lower 2 bits of the duty
    CCP1CON |= ((duty & 0x03) << 4);
    
}

void openDoor() {
    Set_PWM_Duty(60);
}

void closeDoor() {
    Set_PWM_Duty(319);
}

void Ultrasonic_Init(void) {
    TRISC &= ~(0b00010000); //TRISC4 = 0;
    TRISC |=  (0b00001000); //TRISC3 = 1;
}

unsigned int Read_Ultrasonic(void) {
    unsigned int timeout = 300;
    unsigned int time = 0;

    PORTC |= (1 << 4);           // RC4 = 1
    delay_us(10);
    PORTC &= ~(1 << 4);          // RC4 = 0

    while (!(PORTC & (1 << 3)) && timeout--) delay_us(1);  // Wait for RC3 high (echo start)
    if (timeout == 0) return -1;

    timeout = 30000;
    while ((PORTC & (1 << 3)) && timeout--) {  // Wait while RC3 is high
        delay_us(1);
        time++;
    }
    if (timeout == 0) return -1;

    return time / 5.8;
}


void Initialize_Hbridge() {
    TRISB &= ~(0b11110000);
}

void Pump_on() {
    PORTB |= (0b00010000);
    PORTB &= ~(0b00100000);

    PWM_Init_H();
    delay_ms(200);

    // 80% Speed
    H_Bridge_PWM(80);
    delay_ms(10000);
    // 50% Speed
    H_Bridge_PWM(50);
    delay_ms(6000);
    // 30% Speed
    H_Bridge_PWM(30);
    delay_ms(3000);
    Pump_off();
}

void Pump_off() {
    PORTB &= ~(0b00110000);
    H_Bridge_PWM(0);
    PWM_Init_S();
}

void Fan_on() {
    PORTB &= ~(0b01000000);
    PORTB |=  (0b10000000);
}

void Fan_off() {
    PORTB &= ~(0b11000000);
}

unsigned int adc() {
    unsigned int adcval;
    ADCON1 = 0xc0; //right justified
    ADCON0 = 0x85; //adc on, fosc/64
    while (ADCON0 & 0b00000100); //wait until conversion is finished
    adcval = ((ADRESH << 8) | (ADRESL)); //store the result
    adcval = (adcval / 3) - 1;
    return adcval;
}

void delay_ms(unsigned int x) {
    unsigned int i, j;
    for (i = 0; i < x; i++) {
        delay_us(1000);
    }
}
void delay_us(unsigned int x) {
    unsigned int i;
    for (i = 0; i < x; i++) {
        asm("NOP");
        asm("NOP");
    }
}
