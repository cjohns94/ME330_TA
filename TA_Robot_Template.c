/*
 * File:   TA_Robot_Template.c
 * Author: Curtis Johnson
 *
 * Created on March 18, 2020, 4:32 PM
 * 
 * 
 *               PIN OUT DIAGRAM
 *                  ------------
 *                  |1       20| 3v3
 *                  |2       19| GND
 *                  |3       18|
 *    Stepper PWM   |4       17| Right Sensor 
 * Lift Servo PWM   |5       16| Front Sensor
 *  Right Motor Dir |6       15|
 *   Left Motor Dir |7       14| Steppers PWM
 *           Enable |8       13| Back Touch CN
 *                  |9       12| Front Touch CN
 *                  |10      11| 
 *                  ------------

 
 
 
 */
 
//Include to use delay function
#define FCY 4000000UL
#include <libpic30.h>

#include "xc.h"
#pragma config FNOSC = FRC   // select 8 MHz 

//global vars
int steps = 0;

// Configuration function prototypes
void config_steppers();
void config_touchSensors();
void config_ADC();

//Interrupt function prototypes
void __attribute__((interrupt, no_auto_psv)) _CNInterrupt();
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void);


int main(void) {
    //reset global vars
    steps = 0;
    
    //Configure peripherals
    config_steppers();
    
    __delay_ms(20); //delay to make sure everything is configured properly
    
    while(1){};
    return 0;
}

void config_steppers(){ 
    //Parameters to change
    int motor_freq = 700; //Hz
    double duty_cycle = 0.5;
    
    OC1CON1 = 0;    // Clear control bits initially
    OC1CON2 = 0;    
   
    //Setup PWM frequency and duty cycle
    int period = (4000000)/motor_freq;
    OC1RS = period; //Total period
    OC1R = OC1RS * duty_cycle;  //On time      duty cycle% = OC2R/OC2RS 
    
    //Set up PWM module on microcontroller
    OC1CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
    OC1CON2bits.SYNCSEL = 0x1F; 
    OC1CON2bits.OCTRIG = 0;   
    OC1CON1bits.OCM = 0b110;
    
    //CONFIGURE OUTPUT PIN 6
    TRISBbits.TRISB2 = 0; // Pin 6 is output (DIR1)
    ANSBbits.ANSB2 = 0; //make digital
    
    //CONFIGURE OUTPUT PIN 7
    TRISAbits.TRISA2 = 0; // Pin 7 is output (DIR2)
    ANSAbits.ANSA2 = 0; //make digital
    
    //CONFIGURE OUTPUT PIN 8
    TRISAbits.TRISA3 = 0;        //Ouput
    ANSAbits.ANSA3 = 0;         //Digital
    
    //CONFIGURE OUTPUT COMPARE 1 INTERUPT (used to count steps)
    IEC0bits.OC1IE = 1; //enable interrupt
    IPC0bits.OC1IP = 5; // priority level 5

   //KEEP MOTORS OFF by turning enable off
    LATAbits.LATA3 = 0;
}

void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void){
    steps = steps+1;
    
    IFS0bits.OC1IF = 0;
}
