
// PIC18F4550 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator (HS))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)



#define _XTAL_FREQ   20000000
//#define trig_s2      PORTBbits.RB5
//#define echo_s2      PORTBbits.RB4
#define trig_s3      PORTBbits.RB2
#define echo_s3      PORTAbits.RA0
#define trig_s4      PORTBbits.RB3
#define echo_s4      PORTAbits.RA1
#define trig_s5      PORTBbits.RB4
#define echo_s5      PORTDbits.RD4
#define trig_s6      PORTBbits.RB5
#define echo_s6      PORTDbits.RD5
#define trig_s7      PORTBbits.RB6
#define echo_s7      PORTDbits.RD6
#define trig_s8      PORTBbits.RB7
#define echo_s8      PORTDbits.RD7
//#define trig_s9      PORTDbits.RD0
//#define echo_s9      PORTDbits.RD1
#define lcd_data   PORTD
#define rs         PORTCbits.RC0
#define en         PORTCbits.RC2


float s2,s3,s4,s5,s6,s7,s8,s9,s_front_right,s_front,s_front_left;
int s2_old=0;
int s3_old=0;
int s4_old=0;
int s5_old=0;
int s6_old=0;
int s7_old=0;
int s8_old=0;
int s9_old=0;
//-------------------slav addresses-----------------------------------
char slave_rw=0x02;
char slave_lw=0x04;

//--------------------------obstacle avoidance------------------------

float Mem_front[3] ;
float Mem_left[3] ;
float Mem_right[3] ;

float lw_vel_obs,rw_vel_obs;
int mul_obs,width_obs;


float premise_obstacle_avoid_lw[3][3][3];
float area_obstacle_avoid_lw [3][3][3];



float obstacle_rule_lw[3][3][3] ={
                                 {10,10,10,
                                   40,10,10,
                                   40,10,10},

                                   {20,10,10,
                                   40,40,20,
                                   40,40,40},

                                   {20,10,10,
                                   40,50,50,
                                   40,50,60}
                                   };
float obstacle_rule_rw[3][3][3] ={
                                 {40,40,40,
                                   10,40,40,
                                   10,10,40},

                                   {30,40,40,
                                   20,40,40,
                                   10,20,40},

                                   {20,40,40,
                                   20,50,50,
                                   20,50,60}
                                   };

int lw_velocity,rw_velocity;
//-------------------------prototypes--------------------------

void init_ultrasonic();
void read_ultrasonic_s2();
void read_ultrasonic_s3();
void read_ultrasonic_s4();
void read_ultrasonic_s5();
void read_ultrasonic_s6();
void read_ultrasonic_s7();
void read_ultrasonic_s8();
void read_ultrasonic_s9();
void s_delay (int a);
void read_sensors();
float area(float w,float h);
float min(float x,float y);
float step_func(float x);
float Pos_left_Mem(float y1,float z1,float d);
float Pos_Centre_Mem(float x2,float y2,float z2,float d);
float Pos_Right_Mem( float x3,float y3,float d);
void  Mem_values_obs();
void obstacle_avoid_controller();
void SetWheelSpeed(int lw,int rw);
//----------------for lcd---------------------------------------

void init_lcd();
void lcdcmd(unsigned char value);
void lcdcmd_4bit (unsigned char value);
void lcddata_4bit (unsigned char value);
void long_delay(int a);
void int_to_char(char a);

//-----------------------Main----------------------------------------
void main()
{
    mul_obs=40;       //  in centimeteres
    width_obs=20;  //width for obstacle avoidance
    TRISB=0;

    init_lcd();
    init_ultrasonic();
    OpenI2C( MASTER, SLEW_OFF);  //init I2C  bus
    SSPADD = 0x31;

    while(1)
    {

        read_sensors();
        lcdcmd_4bit(0x80);
        int_to_char(s_front_left);
        lcdcmd_4bit(0x84);
        int_to_char(s_front);
        lcdcmd_4bit(0x88);
        int_to_char(s_front_right);
        //lcdcmd_4bit(0x8C);
        //int_to_char(s4);//s_front_right);

        Mem_values_obs();
        obstacle_avoid_controller();

        lw_velocity=lw_vel_obs;
        rw_velocity=rw_vel_obs;
        SetWheelSpeed(lw_velocity,rw_velocity);
        //SetWheelSpeed(100,100);
        lcdcmd_4bit(0xC3);
        int_to_char(lw_velocity);
        lcdcmd_4bit(0xCA);
        int_to_char(rw_velocity);

        //long_delay(500);
        //long_delay(1000);
        //long_delay(1000);
//        long_delay(1000);
    }
}
void init_ultrasonic()
{
    TRISBbits.RB2=0;
    TRISAbits.RA0=1;
    TRISBbits.RB3=0;
    TRISAbits.RA1=1;
    TRISBbits.RB4=0;
    TRISDbits.RD4=1;
    TRISBbits.RB5=0;
    TRISDbits.RD5=1;
    TRISBbits.RB6=0;
    TRISDbits.RD6=1;
    TRISBbits.RB7=0;
    TRISDbits.RD7=1;
//    trig_s2=0;
//    echo_s2=0;
    trig_s3=0;
    echo_s3=0;
    trig_s4=0;
    echo_s4=0;
    trig_s5=0;
    echo_s5=0;
    trig_s6=0;
    echo_s6=0;
    trig_s7=0;
    echo_s7=0;
    trig_s8=0;
    echo_s8=0;
//    trig_s9=0;
//    echo_s9=0;
    T1CON=0xB0;
    T3CON=0x48;
    TMR1=0x0000;
    ADCON0=0x00;
    ADCON1=0x0F;
}
void read_sensors()
{

    //read_ultrasonic_s2();
    read_ultrasonic_s3();
    read_ultrasonic_s4();
    read_ultrasonic_s5();
    read_ultrasonic_s6();
    read_ultrasonic_s7();
    read_ultrasonic_s8();
    //read_ultrasonic_s9();
    s_front_right=min(s3,s4);
    s_front=min(s5,s6);
    s_front_left=min(s7,s8);
}
//void read_ultrasonic_s2()
//{
//    TMR1=0x0000;
//    int value=0x0000;
//    trig_s2=1;
//    s_delay(15);
//    trig_s2=0;
//    while(!echo_s2);
//    T1CONbits.TMR1ON=1;
//    while(echo_s2 && value<14706)
//        value=TMR1;
//    T1CONbits.TMR1ON=0;
//    s2=(TMR1*0.0136)+1.093;//s2=(TMR1*0.0544)+1.093;//s2=(TMR1*0.0272)+1.093;
//    if(s2<2)
//        s2=s2_old;
//    else
//        s2_old=s2;
//}
void read_ultrasonic_s3()
{
    TMR1=0x0000;
    int value=0x0000;
    trig_s3=1;
    //while(1);
    __delay_us(15);//s_delay(25);
    trig_s3=0;
    while(!echo_s3);
    T1CONbits.TMR1ON=1;
    while(echo_s3 && value<7353)
        value=TMR1;
    T1CONbits.TMR1ON=0;
    s3=(TMR1*0.0272)+1.093;
    if(s3<2)
        s3=s3_old;
    else
        s3_old=s3;
}
void read_ultrasonic_s4()
{
    TMR1=0x0000;
    int value=0x0000;
    trig_s4=1;
    s_delay(15);
    trig_s4=0;
    while(!echo_s4);
    T1CONbits.TMR1ON=1;
    while(echo_s4 && value<7353)
        value=TMR1;
    T1CONbits.TMR1ON=0;
    s4=(TMR1*0.0272)+1.093;
    if(s4<2)
        s4=s4_old;
    else
        s4_old=s4;
}
void read_ultrasonic_s5()
{

    TMR1=0x0000;
    int value=0x0000;
    trig_s5=1;
    s_delay(15);
    trig_s5=0;
    while(!echo_s5);
    T1CONbits.TMR1ON=1;
    while(echo_s5 && value<7353)
        value=TMR1;
    T1CONbits.TMR1ON=0;
    s5=(TMR1*0.0272)+1.093;
    if(s5<2)
        s5=s5_old;
    else
        s5_old=s5;
}
void read_ultrasonic_s6()
{
    TMR1=0x0000;
    int value=0x0000;
    trig_s6=1;
    s_delay(15);
    trig_s6=0;
    while(!echo_s6);
    T1CONbits.TMR1ON=1;
    while(echo_s6 && value<7353)//14706)
        value=TMR1;
    T1CONbits.TMR1ON=0;
    s6=(TMR1*0.0272)+1.093;
    if(s6<2)
        s6=s6_old;
    else
        s6_old=s6;
}
void read_ultrasonic_s7()
{
    TMR1=0x0000;
    int value=0x0000;
    trig_s7=1;
    s_delay(15);
    trig_s7=0;
    while(!echo_s7);
    T1CONbits.TMR1ON=1;
    while(echo_s7 && value<7353)
        value=TMR1;
    T1CONbits.TMR1ON=0;
    s7=(TMR1*0.0272)+1.093;
    if(s7<2)
        s7=s7_old;
    else
        s7_old=s7;
}
void read_ultrasonic_s8()
{
    TMR1=0x0000;
    int value=0x0000;
    trig_s8=1;
    s_delay(15);
    trig_s8=0;
    while(!echo_s8);
    T1CONbits.TMR1ON=1;
    while(echo_s8 && value<7353)
        value=TMR1;
    T1CONbits.TMR1ON=0;
    s8=(TMR1*0.0272)+1.093;
    if(s8<2)
        s8=s8_old;
    else
        s8_old=s8;
}
//void read_ultrasonic_s9()
//{
//    TMR1=0x0000;
//    int value=0x0000;
//    trig_s9=1;
//    s_delay(15);
//    trig_s9=0;
//    while(!echo_s9);
//    T1CONbits.TMR1ON=1;
//    while(echo_s9 && value<14706)
//        value=TMR1;
//    T1CONbits.TMR1ON=0;
//    s9=(TMR1*0.0136)+1.093;//(TMR1*0.0272)+1.093;
//    if(s9<2)
//        s9=s9_old;
//    else
//        s9_old=s9;
//}

//-----------------------------------------------------------Area--------------------------------------------------
float area(float w,float h)
{
    float area;
    area=w*(h-(h*h/2));
    return area;
}
//----------------------------------------------------------Minimum------------------------------------------------
float min(float x,float y)
{
    float z;
    if (x<=y)
        z=x;
    else
        z=y;
    return z;
}

//----------------------------------------------------Step Function------------------------------------------------

float step_func(float x)
{
    float temp;
    if (x<0)
    temp=0;
    else
    temp=1;
    return temp;
}
//------------------------------------------------------------------------------------------------------------------
//----------------------------------------Membership functions------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------

float Pos_left_Mem(float y1,float z1,float d)
{
    float Pos_left_Mem;
    Pos_left_Mem=(((y1-d)/(z1-y1))+1)*((step_func(d-y1)-step_func(d-z1)));
    return Pos_left_Mem;
}
float Pos_Centre_Mem(float x2,float y2,float z2,float d)
{
    float Pos_Centre_Mem;
    float temp=(((-1/(z2-y2))*d + (1*y2/(z2-y2)) + 1)* (step_func(d-y2)-step_func(d-z2)));
    Pos_Centre_Mem=(((1/(y2-x2))*d - (x2*y2/(y2-x2)))* (step_func(d-x2)-step_func(d-y2)))+ temp;
    return Pos_Centre_Mem;
}

float Pos_Right_Mem( float x3,float y3,float d)
{
    float Pos_Right_Mem;
    Pos_Right_Mem=(((1/(y3-x3))*d - (x3/(y3-x3)))* (step_func(d-x3)-step_func(d-y3)))+ step_func(d-y3);
    return Pos_Right_Mem;
}

//--------------------------Mem_values-------------------------------------------
void Mem_values_obs()
{


    Mem_front[0] =Pos_left_Mem( 0,1*mul_obs,s_front);
    Mem_front[1] =Pos_Centre_Mem(0,1*mul_obs,2*mul_obs,s_front);
    Mem_front[2] =Pos_Right_Mem(1*mul_obs,2*mul_obs,s_front);


    Mem_left[0] =Pos_left_Mem(0,1*mul_obs,s_front_left );
    Mem_left[1] =Pos_Centre_Mem(0,1*mul_obs,2*mul_obs,s_front_left);
    Mem_left[2] =Pos_Right_Mem(1*mul_obs,2*mul_obs,s_front_left);


    Mem_right[0] =Pos_left_Mem(0,1*mul_obs,s_front_right );
    Mem_right[1] =Pos_Centre_Mem(0,1*mul_obs,2*mul_obs,s_front_right);
    Mem_right[2] =Pos_Right_Mem(1*mul_obs,2*mul_obs,s_front_right);

}

void obstacle_avoid_controller()
{
  //for left wheel
    float num,den;
    float premise_temp;

    float premise_temp=0;
    num=0;
    den=0.0001;

    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            for(int k=0; k<3; k++)
            {
                premise_temp=min(Mem_front[i],Mem_right[j]);
                premise_obstacle_avoid_lw[i][j][k]=min(premise_temp,Mem_left[k]);
            }
        }
    }
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            for(int k=0; k<3; k++)
                 area_obstacle_avoid_lw[i][j][k]=area(width_obs,premise_obstacle_avoid_lw[i][j][k]);
        }
    }

    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            for(int k=0; k<3; k++)
            {
                num=num+area_obstacle_avoid_lw[i][j][k] * obstacle_rule_lw[i][j][k];
                den=den+area_obstacle_avoid_lw[i][j][k];
            }
        }
    }

    lw_vel_obs=num/den;
    int temp=lw_vel_obs;
    float temp1=lw_vel_obs-temp;
    if ( temp1>=0.5)
        lw_vel_obs=temp+1;

// for right wheel
    num=0;
    den=0.0001;
    premise_temp=0;



    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            for(int k=0; k<3; k++)
            {
                num=num+area_obstacle_avoid_lw[i][j][k] * obstacle_rule_rw[i][j][k];
                den=den+area_obstacle_avoid_lw[i][j][k];
            }
        }
    }

    rw_vel_obs=num/den;
    int temp2=rw_vel_obs;
    float temp3=rw_vel_obs-temp2;
    if ( temp3>=0.5)
        rw_vel_obs=temp2+1;


}
void SetWheelSpeed(int lw,int rw)
{
    //lw=-lw;
    IdleI2C();
    StartI2C();
    IdleI2C();
    WriteI2C(slave_lw);
    IdleI2C();
    WriteI2C(lw);
    IdleI2C();
    StopI2C();
    //----------------------------------
    IdleI2C();
    StartI2C();
    IdleI2C();
    WriteI2C(slave_rw);
    IdleI2C();
    WriteI2C(rw);
    IdleI2C();
    StopI2C();

}

void s_delay(int a)  // 1usec delay  40 MHz
{
    T0CON=0X08;
    for (int x=1; x<=a; x++ )
    {
        TMR0H=0XFF;
        TMR0L=0XFE;

        T0CONbits.TMR0ON=1;
        while(INTCONbits.TMR0IF==0);
        T0CONbits.TMR0ON=0;
        INTCONbits.TMR0IF=0;

    }
}

void init_lcd()
{
    TRISD=0;
    TRISC=0;
    PORTCbits.RC1=0;
    en=0;
    long_delay(200);   //15ms delay
    lcdcmd(0x30);
    long_delay(80);     //4.1 ms delay
    lcdcmd(0x30);
    long_delay(1);    // 100 usec delay
    lcdcmd(0x30);
    long_delay(41);    // 4.1 ms delay
    lcdcmd(0x20);
    long_delay(100);
    lcdcmd_4bit(0x0E);   // 2 lines 5*7 matrix and cursor blink
    lcdcmd_4bit(0x01);    //clear screen
    long_delay(17);      //1.7 ms delay
    lcdcmd_4bit(0x06);     // something something
    lcdcmd_4bit(0x28);    // this too does something
    lcdcmd_4bit(0x80);    // this too does something

}

void lcdcmd(unsigned char value)
{
    lcd_data=(value & 0xF0)>>4;
    rs=0;
    en=1;
    s_delay(1);
    en=0;
}

void lcdcmd_4bit (unsigned char value)
{
    lcd_data=(value & 0xF0)>>4;
    rs=0;
    en=1;
    s_delay(1);
    en=0;
    lcd_data=(value & 0x0F);
    rs=0;
    en=1;
    s_delay(1);
    en=0;
    s_delay(41);
}
void lcddata_4bit (unsigned char value)
{
    lcd_data=(value & 0xF0)>>4;
    rs=1;
    en=1;
    s_delay(1);
    en=0;
    lcd_data=(value & 0x0F);
    rs=1;
    en=1;
    s_delay(1);
    en=0;
    s_delay(41);  // delay for command to get completed
}
void long_delay(int a)  // 100 usec delay for 40MHz
{

    T0CON=0X08;
    for (int x=1; x<=a; x++ )
    {
        TMR0H=0XFE; //delay for 20 MHZ
        TMR0L=0X0C;

        T0CONbits.TMR0ON=1;
        while(INTCONbits.TMR0IF==0);
        T0CONbits.TMR0ON=0;
        INTCONbits.TMR0IF=0;

    }
}


void int_to_char(char a)
{
    char temp;
    char b[3];
    b[0]=a/100;
    temp=a-(b[0]*100);
    b[1]=temp/10;
    b[2]=temp-(b[1]*10);
    for(int i=0;i<3;i++)
        lcddata_4bit(b[i]+0x30);
}