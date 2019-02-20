// PIC18F2431 Configuration Bit Settings

#include <xc.h>
#include<math.h>
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON        // Internal External Oscillator Switchover bit (Internal External Switchover mode enabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled)
// BORV = No Setting

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDPS = 32768     // Watchdog Timer Postscale Select bits (1:32768)
#pragma config WINEN = OFF      // Watchdog Timer Window Enable bit (WDT window disabled)

// CONFIG3L
#pragma config PWMPIN = OFF     // PWM output pins Reset state control (PWM outputs disabled upon Reset (default))
#pragma config LPOL = HIGH      // Low-Side Transistors Polarity (PWM0, 2, 4 and 6 are active-high)
#pragma config HPOL = HIGH      // High-Side Transistors Polarity (PWM1, 3, 5 and 7 are active-high)
#pragma config T1OSCMX = ON     // Timer1 Oscillator MUX (Low-power Timer1 operation when microcontroller is in Sleep mode)

// CONFIG3H
#pragma config MCLRE = ON      // MCLR Pin Enable bit (Disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Low-Voltage ICSP Enable bit (Low-voltage ICSP enabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000200-000FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (001000-001FFF) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (002000-002FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (003000-003FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000200-000FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (001000-001FFF) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (002000-002FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (003000-003FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000200-000FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (001000-001FFF) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (002000-002FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (003000-003FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)


#define _XTAL_FREQ   20000000

//---------------global integers--------------//
long int old_pos=0;
long int new_pos=0;
int poscnt_rolovr=0;
int poscnth;
int poscntl;
int act_vel;
int actual_pos=0;
int err_vel=0;
int a_err=0;
int p_err=0;
int d_err=0;
int count=1;
int des_vel=70;
int tmr5_rolovr=0;
char rec_vel;
long int velr_vh;
int velrh;
int velrl;
//----------------high priority interrupts------//

void interrupt isr()
{
    if(PIR1bits.SSPIF==1)
    {
        unsigned char temp;
        char address,data;
        temp = SSPSTAT & 0x2D;
        if((temp ^ 0x09) == 0x00)             // 1: write operation, last byte was address
            address = SSPBUF;

        else if((temp ^ 0x29) == 0x00)
        {
            // 2: write operation, last byte was data
             rec_vel = SSPBUF;

            char c= rec_vel & 0x80;
            des_vel=rec_vel;
            if (c==0x80)
                des_vel|=0xFF00;
            else
                des_vel&=0x00FF;
        }
        else if((temp ^ 0x0C) == 0x00)       // 3: read operation, last byte was address
                SSPBUF=act_vel;

        else if ((temp ^ 0x2C) == 0x00)      // 4: read operation, last byte was data
                 SSPBUF=act_vel;
                                             // 5: slave logic reset by NACK from master


        PIR1bits.SSPIF=0;
        SSPCON1bits.CKP=1;
    }

    if(PIR3bits.IC2QEIF==1)
    {

        if(QEICONbits.UP_DOWN==1)
            poscnt_rolovr++;
        else
            poscnt_rolovr--;
        PIR3bits.IC2QEIF=0;
    }

    if(PIR1bits.TMR1IF==1)
    {

        a_err+=err_vel;
        count++;
        if(count==4)
        {
            d_err=err_vel-p_err;
            count=1;
        }
        TMR1=0xEC78;

        PIR1bits.TMR1IF=0;
    }
   
}


//----------------prototypes-----------------------------//

void init_pwm();
void pwm_generator(int DC);
void init_QEI();
void interrupt_enable();
void vel_read();
void pos_read();
void delay_10us(int a);
void init_time_base();
void init_I2C();

void main()
{

//----------------initialisations-----------------//
    ANSEL0=0X00;      //CONFIGURING THE PORT A AS digital I/O
    ADCON0bits.ADON=0;  //disabling ADC
    interrupt_enable();
    init_QEI();
    init_pwm();
    init_I2C();
    TRISB=0;
    init_time_base();
    TRISAbits.RA0=0;
//-----------------MAIN-------------------------//

    
    int DC;
    int u;
    while(1)
    {
     //--------------calculating error---------------------//
        PORTAbits.RA0=~PORTAbits.RA0;
        vel_read();
        if(QEICONbits.UP_DOWN==0)
        act_vel=(-act_vel);

        err_vel=des_vel-act_vel;


    //---------------------------PID-----------------------//
         u=(13*err_vel)+(0.01*a_err);//+(2*d_err);  //amplifier gain;
       
    //---------calculating corresponding DC---------------//

         if (des_vel<7 && des_vel>-7)
             DC=50;
         else
             DC= ((u*10)/45)+50;

   //----------saturator --------------------------------//
        if (DC>98)
           DC=97;
        if (DC<2)
           DC=3;
     //----------------producing desired DC----------------//
        pwm_generator(DC);
        p_err=err_vel;
        
    }

}

void init_pwm()
{
    TRISBbits.RB0=0;
    TRISBbits.RB1=0;
    PTMRH=0;
    PTMRL=0;
    PTPERH=0;
    PTPERL=200;// 25KHz  40usec
    DTCON=0x81;  // Fosc/4, 0.2 usec delay;
    PWMCON0=0x5C;// pwm 0,1,2,3 all enabled as pwm
    PWMCON1=0x00;
    PTCON0=0x00; // prescaler 1,free running
    PTCON1=0x80; // START TIMER, UP COUNTING

}

void pwm_generator(int DC)
{
    int on_time=DC*2;        //(DC*200)/100;
    on_time=on_time<<2;
    PDC0L=on_time & 0xFF;
    PDC0H=(on_time&0xFF00)>>8;
}

void init_QEI()
{
    TRISAbits.RA3=1;
    TRISAbits.RA4=1;

    DFLTCON=0x31;
    QEICON=0x19;  // velocity mode, x4 count up, 1 postscale for velcap
    MAXCNTH=0xFF;
    MAXCNTL=0xFF;

    POSCNTH=0;
    POSCNTL=0;

    TMR5=0x00;
    T5CON=0x01;  // synchronous, prescalar 0, timer ON
    CAP1CONbits.CAP1REN=1;  //reset enable for TMR5
}

void interrupt_enable()
{
    INTCONbits.GIE=1; // global enable
    INTCONbits.PEIE=1; // peripheral enable
    PIE3bits.IC2QEIE=1;  // QEI interrupt enable
    PIR3bits.IC2QEIF==0;
    
}
void init_I2C(void)
{
    INTCONbits.PEIE=1;
    PIE1bits.SSPIE = 1;         // Enable SSP interrupt
    PIR1bits.SSPIF=0;
    TRISC=0x30;             // SDA and SCL as input pin
    SSPSTAT |= 0x00;        // Slew rate disabled
    SSPCON = 0x3E;          // SSPEN = 1, I2C slave mode,
    SSPADD = 0x04;          // SLAVE ADDRESS

}

void vel_read()
{
    int temp=VELRH;
    long int pulse_count=(temp<<8)+VELRL;
    int freq= 5000000/(pulse_count); //16 counts in 1 rev of motor
    act_vel=((freq/16)*60)/50; // gear ratio 1:50

}
void delay_10us(int a)
{

    T0CON=0X08;
    for (int x=1; x<=a; x++ )
    {
        TMR0H=0xFF;
        TMR0L=0xE7;

        T0CONbits.TMR0ON=1;
        while(INTCONbits.TMR0IF==0);
        T0CONbits.TMR0ON=0;
        INTCONbits.TMR0IF=0;
    }
}
void init_time_base()
{

    //TMR1=0x0000;   // interrupt after 13.107 ms
    TMR1=0xEC78;     //  interrupt at 1 ms
    PIE1bits.TMR1IE=1;
    PIR1bits.TMR1IF=0;
    T1CON=0b10000001; //timer1 ON,1:1 prescale, osc Fosc/4
}


