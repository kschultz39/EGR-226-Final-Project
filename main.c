#include "msp.h"
#include "msp432.h"
#include <stdio.h>
#include "math.h"




/**
 *Authors: Kelly Schultz and Nathan Gruber
 *Instructors: Professor Zuidema
 *Date: 11/28/18
 *Assignment: Alarm Clock Project
 * 
 */
void InitializeAll(void);

void SysTick_Init(void);
void LCD_init(void);
void delay_micro(unsigned microsec);
void delay_ms (unsigned ms);
void PulseEnablePin(void);
void pushNibble (uint8_t nibble);
void pushByte(uint8_t byte);
void commandWrite(uint8_t command);
void dataWrite(uint8_t data);

void configRTC(void);



int alarmflag=0;
int setalarmflag=0;
int setclockflag=0;




int displayhour=0;
int result=0;

// global struct variable called clock
struct
{
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    char daynight;
} clock;

// global struct variable called alarm
struct
{
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    char daynight;
} alarm;



//Alarm Clock Tones
#define F4 349.23
#define F4SHARP 369.99

//Defining the lengths of notes
#define QUARTER 1000000
#define QUARTERPLUS 1500000

#define MAX_NOTE 1000 // How many notes are in the song matrix

void SetupTimer32s();

int note = 0;       //The note in the music sequence we are on
int breath = 0;     //Take a breath after each note.  This creates seperation

//FOR TIME FROM ZUIDEMA
int time_update = 0, alarm_update = 0;
   uint8_t hours, mins, secs;

enum states{
    DEFAULT,//Default state
    SETALARM,
    SETTIME,
};

void main(void)
{
    InitializeAll();
    char buffer[50];
    int i;
    //int seconds;

    clock.daynight= 'A';





//
//
//    //TIMER 32 COUNT DOWN IN SECONDS
//    TIMER32_1->CONTROL = 0b11101010;  // Periodic, Wrapping, Interrupt, Divide by 256, Enabled, 32bit
//        TIMER32_1->LOAD = 703200-1;  //0.25 seconds @ 3MHz * 4 = 1 sec * 60 = 703200 for 60 seconds
//        seconds = TIMER32_1->LOAD;
//        second = (seconds / 11720);
//    //    TIMER32_1->LOAD = 1012500000-1;  //24 hours @ 3 MHz
//        NVIC_EnableIRQ(T32_INT1_IRQn);
//        __enable_interrupt();

    //RTC CODE FROM LECTURE
 RTC_C->CTL0 = (0xA500) ;
RTC_C->CTL13 = 0;
    //initialize time to 2:45 pm
    //RTC_C->TIM0 = 0x2000; //45 min, 0 sec
    RTC_C->TIM0 = 45 << 8 | 55; //45 min, 0 seconds
    RTC_C->TIM1 = 1 << 8 | 14;
    RTC_C->YEAR = 2018;
    //Alarm set at 2:45 om
    RTC_C->AMINHR = 14 << 8 | 46 | BIT(15) | BIT(7); //bit 15 adn 7 are alarm enable bits
    RTC_C->ADOWDAY = 0;
    RTC_C->PS1CTL = 0b11010; //1 second interrupt

    RTC_C->CTL0 = (0xA500) | BIT5;
    RTC_C-> CTL13 = 0;
    NVIC_EnableIRQ(RTC_C_IRQn);





while(1)
{


    if(time_update){
        time_update = 0;
        if(clock.hour>12)
            displayhour=(clock.hour)-(12);
        else
            displayhour=(clock.hour);


        printf("%02d:%02d:%02d %cM\n", displayhour, clock.minute, clock.second, clock.daynight);
       commandWrite(0x0F); //turn off blinking cursor
        commandWrite(0x0C);
                      commandWrite(0xC0);  //moves the cursor to the second
                      delay_ms(500);
        sprintf(buffer, "%d:%02d:%02d %cM                     ", displayhour, clock.minute, clock.second, clock.daynight);
      // commandWrite(0x0C); //Prints to line 1 of LCD
      for(i=0; i<16; i++)
          dataWrite(buffer[i]);
     //commandWrite(0xC0); //Prints to line 2 of LCD

           //TEMP SENSOR CODE
      commandWrite(0xD0); //Prints to line 3 of LCD

         ADC14->CTL0 |= 1;  //Start conversion
         while (!ADC14->IFGR0);  // wait till conversion completes  read is ADC14IFGRO
         float voltage;
         float Cheat;
         float Fheat;

         voltage = ((3.3 / 4096) * result);  //function that will convert 12-bit resolution output into a voltage reading depending on the position of the potentiometer

         Cheat = ( ( ( voltage * 1000) - 500 ) / 10 );

         Fheat = ((Cheat * (9.0/5.0)) + 32.0);

         printf("Fheat is %lf\n", Fheat);

         sprintf(buffer, "      %.1f", Fheat);    //puts X value in the string of buffer through use of sprintf() function
         for(i=0; i<10; i++)   //prints string with information about X in the first line
             dataWrite(buffer[i]);
         dataWrite('F');
    }

    if(alarm_update)
    {
        printf("ALARM\n");
        alarm_update = 0;

    }



}

//     commandWrite(0x0F); //turn off blinking cursor
//     //sprintf(buffer, "%d:%d:%d %cM                     ", hour, minute, second, time);
//     commandWrite(0x0C); //Prints to line 1 of LCD
//     for(i=0; i<16; i++)
//         dataWrite(buffer[i]);
//     commandWrite(0xC0); //Prints to line 2 of LCD

//     ADC14->CTL0 |= 1;  //Start conversion
//     while (!ADC14->IFGR0);  // wait till conversion completes  read is ADC14IFGRO
//     float voltage;
//     float Cheat;
//     float Fheat;

//     voltage = ((3.3 / 4096) * result);  //function that will convert 12-bit resolution output into a voltage reading depending on the position of the potentiometer

//     Cheat = ( ( ( voltage * 1000) - 500 ) / 10 );

//     Fheat = ((Cheat * (9.0/5.0)) + 32.0);

//     delay_ms(500);
//     sprintf(buffer, "      %.1f", Fheat);    //puts X value in the string of buffer through use of sprintf() function
//     for(i=0; i<10; i++)   //prints string with information about X in the first line
//         dataWrite(buffer[i]);
//     dataWrite('F');
//     delay_ms(300);






}

void InitializeAll(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    SysTick_Init();
    P3->SEL0 &= ~(BIT2|BIT3); //sets P3.2 and P3.3 as GPIO
    P3->SEL1 &= ~(BIT2|BIT3);   //sets P3.2 and P3.3 as GPIO
    P3->DIR |= (BIT2|BIT3); //sets P3.2 and P3.3 as OUTPUT
    //P3->OUT &= ~(BIT2|BIT3); //sets P3.2 and P3.3 to 0 for RS RW =0


            //LCD
            P2->SEL0 &= ~(BIT4|BIT5|BIT6|BIT7); //sets (DB4-DB7) P2.4, P2.5, P2.5, P2.6, P2.7 as GPIO
            P2->SEL1 &= ~(BIT4|BIT5|BIT6|BIT7);
            P2->DIR |= (BIT4|BIT5|BIT6|BIT7); //sets pins 4.4-4.7 to OUTPUT




            P6->SEL0 &=~BIT4; //sets P6.4 to GPIO (ENABLE PIN)
            P6->SEL1 &=~BIT4; //sets P6.4 to GPIO (ENABLE PIN)
            P6->DIR |= BIT4; //sets as output
            P6->OUT &= ~BIT4; //sets Enable pin to 0 initially


    LCD_init();

    //Pin enables for PWM LEDs (Referencing code from Zuidema In-class example Week 5 part 1)
    P7->SEL0 |= (BIT4|BIT5|BIT6); //sets SEL0=1;
    P7->SEL1 &= (BIT4|BIT5|BIT6);  //SEL1 = 0. Setting SEL0=1 and SEL1=0 activates PWM function
    P7->DIR |= (BIT4|BIT5|BIT6);    // Set pins as  PWM output.
    P7->OUT &= ~(BIT4|BIT5|BIT6);

    //Timer A
    TIMER_A1->CCR[0] = 999;  //1000 clocks = 0.333 ms.  This is the period of everything on Timer A1.  0.333 < 16.666 ms so the on/off shouldn't
                                 //be visible with the human eye.  1000 makes easy math to calculate duty cycle.  No particular reason to use 1000.

    TIMER_A1->CCTL[2] = 0b0000000011100000;  //reset / set compare.   Duty Cycle = CCR[1]/CCR[0].
    TIMER_A1->CCR[2] = 0;  //P7.6 initialize to 0% duty cycle
    TIMER_A1->CCTL[3] = 0b0000000011100000;
    TIMER_A1->CCR[3] = 0;  //P7.5 intialize to 0% duty cycle
    TIMER_A1->CCTL[4] = 0b0000000011100000;
    TIMER_A1->CCR[4] = 0;  //7.4 0% duty cycle

    //The next line turns on all of Timer A1.  None of the above will do anything until Timer A1 is started.
    TIMER_A1->CTL = 0b0000001000010100;

    //BUTTON INITIALIZATION
    //BUTTON 1.6 and BUTTON 1.7 (Button 1.6 SET ALARM, Button 1.7 SET TIME)
    P1-> SEL0 &= ~(BIT6|BIT7);
    P1 -> SEL1 &= ~(BIT6|BIT7);
    P1 -> DIR &= ~(BIT6|BIT7);
    P1 -> REN |= (BIT6|BIT7);
    P1->OUT |= (BIT6|BIT7);
    P1->IE |= (BIT6|BIT7);
    P1->IES |= (BIT6|BIT7);
    
        //BUTTON INITIALIZATION FOR ON/OFF/UP and SNOOZE/DOWN
    P5-> SEL0 &= ~(BIT1|BIT2);
    P5 -> SEL1 &= ~(BIT1|BIT2);
    P5 -> DIR &= ~(BIT1|BIT2);
    P5 -> REN |= (BIT1|BIT2);
    P5->OUT |= (BIT1|BIT2);
//    P5->IE |= (BIT1|BIT2);
//    P5->IES |= (BIT1|BIT2);

    //Initialization for Temperature sensor
    ADC14->CTL0 = 0x00000010;  // power on and disabled during configuration
    ADC14->CTL0 |= 0x04080300; //S/H pulse mode, sysclk, 32 sample clock, software trigger
    ADC14->CTL1 = 0x00000020;  //12-bit resolution, should we use 14 as specified in class?
    ADC14->MCTL[5] = 5;        //A5 input, single-ended, Vref-AVCC

    P5->SEL1 |= (BIT0);  //configure P5.0 for A5, this will then be attached to the 10k external Pot
    P5->SEL0 |= (BIT0);

    ADC14->CTL1 |= 0x00050000;  //convert for mem reg 5
    ADC14->CTL0 |= 2;           //enable ADC after configuration

    //Interrupt functions
    TIMER32_1->CONTROL = 0b11101010;  // Periodic, Wrapping, Interrupt, Divide by 256, Enabled, 32bit
    TIMER32_1->LOAD = 5860-1;  //0.25 seconds @ 3MHz

    NVIC_EnableIRQ(T32_INT1_IRQn);
    __enable_interrupt();

}
//This function goes through the entire initialization sequence as shown in Figure 4
void LCD_init(void)
{
    //P3->OUT &= ~BIT2;    //P3.2 is RS, set to 0 because sending command

    //P3->OUT &= ~BIT2;    //P3.2 is RS, set to 0 because sending command




          commandWrite(0x03);   //3 in HEX
          delay_ms(100);  //waits 100 ms
          commandWrite(0x03);   //3 in HEX
          delay_micro(200);   //waits 200 microseconds
          commandWrite(0x03); //3 in HEX
          delay_ms(100);  //waits 100 ms




          commandWrite(0x02); //2 in HEX
          delay_micro(100); //waits 100 microseconds
          commandWrite(0x02); //2 in HEX
          delay_micro(100); ///waits 100 microseconds




          commandWrite(0x08); //8 in HEX
          delay_micro(100); //waits 100 microseconds
          commandWrite(0x0F); //HEX 0F MOD
          delay_micro(100); //waits 100 microseconds
          commandWrite(0x01); //01 HEX
          delay_micro(100); //waits 100 microseconds
          commandWrite(0x06); //HEX 06
          delay_ms(10); //waits 10 microseconds


          //INitialization complete according to Figure 4 in prelab
    //INitialization complete according to Figure 4 in prelab
}

//Use the SysTick Timer peripheral to generate a delay in microseconds
//must be able to generate delays between 10 and 100 microseconds
void delay_micro(unsigned microsec)
{
    SysTick ->LOAD = ((microsec*3)-1); //1ms count down to 0
    SysTick ->VAL =0; //any write to CVR clears it and COUNTFLAG in CSR

    //wait for flag to be SET (Timeout happened)
    while((SysTick -> CTRL & 0x00010000) ==0);

}
//uses the SysTick timer peripheral to generate a delay in milliseconds
//function must be able to generate a delay of at least 60 ms
void delay_ms (unsigned ms)
{
    SysTick ->LOAD = ((ms*3000)-1); //1microsecond count down to 0
    SysTick ->VAL =0; //any write to CVR clears it and COUNTFLAG in CSR

    //wait for flag to be SET (Timeout happened)
    while((SysTick -> CTRL & 0x00010000) ==0);

}
//Sequence the Enable (E) pin as shown in Figure 6
void PulseEnablePin(void)
{
    P6->OUT &= ~BIT4; //sets enable pin to LOW
    delay_micro(100); //waits 10 microseconds

    P6 ->OUT |= BIT4; //sets enable pin to HIGH
    delay_micro(100); //waits 10 microseconds

    P6->OUT &= ~BIT4; //sets enable pin to LOW
    delay_micro(100);
}
//Pushes 1 nibble onto the data pins and pulses the Enable pin
void pushNibble (uint8_t nibble)
{
    P2->OUT &= ~(BIT4|BIT5|BIT6|BIT7); //clears values
         P2->OUT |= ((nibble & 0x0F)<<4);
        // delay_micro(100);

    PulseEnablePin();
}

//Pushes the most significant 4 bits of the byte onto the data pins by calling the pushNibble() function
//Then pushes the least significant 4 bits onto the data pins by calling the pushNibble() function again
void pushByte(uint8_t byte)
{
    uint8_t temp;
           temp= ((byte & 0xF0)>>4);


           //MOST SIGNIFICANT
           pushNibble(temp);


           //LEAST SIGNIFICANT
           temp= (byte & 0x0F);
           pushNibble(temp);
           delay_micro(100);


}
//write one byte of COMMAND by calling the pushByte() function with the COMMAND parameter
void commandWrite(uint8_t command)
{
    //RW to zero
           P3->OUT &= ~(BIT2); //pulls RS pin LOW (expects instructions)


           //RS to zero
           P3 ->OUT &=~(BIT3);
           pushByte(command);
           delay_ms(100);
}
//writes one byte of DATA by calling the pushByte() function within the DATA parameter
void dataWrite(uint8_t data)
{
    P3->OUT |= BIT2; //pulls RS pin HIGH (expects data)
          pushByte(data);



}

void SysTick_Init(void)
{
    SysTick -> CTRL=0; //disable SysTick during setup
    SysTick -> LOAD= 0x00FFFFFF; //maximum reload value
    SysTick -> VAL= 0; //any write to current value clears it
    SysTick -> CTRL= 0x00000005; //enable SysTIck, CPU clk, no interrupts
}

//Interrupt function, set up to read conversion result 2 times a second (FOR TEMPERATURE SENSOR)
void T32_INT1_IRQHandler()
{
    TIMER32_1->INTCLR = 1;      // Clear interrupt needs to happen first for some reason (unknown)

// add to seconds

    result = ADC14->MEM[5];  //read conversion result, STORES TO MEM LOCATION 5
}
//Interupt function for Set Alarm and Set Time
void PORT1_IRQHandler()
{
    //Button 1.6 is for SET ALARM
    //Button 1.7 is for SET TIME
    if(P1->IFG & BIT6)
    {
        P1 -> IFG &= ~BIT6; //clears interrupt
        if(setalarmflag>=0 && setalarmflag<=3)
            setalarmflag+=1;
        else if(setalarmflag==3)
            setalarmflag=0;

    }
    //if(buttonP17_pressed())
    if(P1->IFG & BIT7)
    {
        P1 -> IFG &= ~BIT6; //clears interrupt
        if(setalarmflag>=0 && setalarmflag<3)
            setalarmflag+=1;
        else if(setalarmflag==3)
            setalarmflag=0;
    }


}

void RTC_C_IRQHandler()
{
    if(RTC_C->PS1CTL & BIT0)
    {
        time_update = 1;
        clock.hour = RTC_C->TIM1 & 0x00FF;
        clock.minute = (RTC_C->TIM0 & 0xFF00) >> 8;
        clock.second = RTC_C->TIM0 & 0x00FF;
        RTC_C->PS1CTL &= ~BIT0;
    }

    if(RTC_C->CTL0 & BIT1)
    {

        alarm_update = 1;
        RTC_C->CTL0 = (0xA500) | BIT5;
    }
}
void sethour(void)
 {
    if(setalarmflag==1)
    {
        while(!((P1->IN & BIT6) == BIT6))
        {
            alarm.hour+=1;
            if(alarm.hour==12)
                {
                alarm.hour+=1;
                configRTC();
                }
            if(alarm.hour==12)
            {
                if(alarm.daynight== 'A')
                    alarm.daynight='P';
                else if(alarm.daynight=='P')
                    {
                    alarm.daynight= 'A';
                    }
                configRTC();
                delay_ms(300); //wait
               alarm.hour=1;
            }

            delay_ms(300); //wait
        }
    }
    if(setclockflag==1)
    {
        while(!((P1->IN & BIT7) == BIT7))
        {
            clock.hour+=1;
            if(clock.hour==12)
               {clock.hour+=1;
               configRTC();
               }
            configRTC();
            if(clock.hour==12)
            {
                if(clock.daynight== 'A')
                    clock.daynight='P';
                else if(clock.daynight=='P')
                    clock.daynight= 'A';
                configRTC();
                delay_ms(300); //wait
                clock.hour=1;

             }
            delay_ms(300); //wait
         }
    }
}

void setminute(void)
{
    if(setalarmflag==2)
    {
        while(!((P1->IN & BIT6) == BIT6))
        {
            alarm.minute+=1;
            if(alarm.minute!=59)
            { alarm.minute+=1;
            configRTC();
            }
            if(alarm.minute==59)
            {
                delay_ms(300); //wait
                alarm.minute=0;
                alarm.minute=0;
                alarm.hour+=1;
                configRTC();
            }
            delay_ms(300); //wait
        }
    }

   if(setclockflag==2)
   {
       while(!((P1->IN & BIT7) == BIT7))
       {
           clock.minute+=1;
           if(clock.minute!=59)
               {clock.minute+=1;
               configRTC();
               }
           if(clock.minute==59)
           {
               delay_ms(300); //wait
               clock.minute=0;
               clock.minute+=1;
               configRTC();
           }
           delay_ms(300); //wait
        }
   }
}
void setsecond(void)
{
    if(setalarmflag==3)
    {
        while(!((P1->IN & BIT6) == BIT6))
        {
            alarm.second+=1;
            if(alarm.second!=59)
                {alarm.second+=1;
                configRTC();
                }
            if(alarm.second==59)
            {
                delay_ms(300); //wait
                alarm.second=0; //if it goes to 00 seconds
                alarm.minute+=1; //it should be incremented by a minute
                configRTC();
            }
            delay_ms(300); //wait
        }
    }

   if(setclockflag==3)
   {
       while(!((P1->IN & BIT7) == BIT7))
       {
           clock.second+=1;
           if(clock.second!=59)
               {clock.second+=1;
               configRTC();
               }
           if(clock.second==59)
           {
               delay_ms(300); //wait
               clock.second=0;
               clock.minute+=1;
               configRTC();
           }
           delay_ms(300); //wait
        }
   }
}

void configRTC(void)
{
    RTC_C->CTL0     =   0xA500;     //Write Code, IE on RTC Ready
    RTC_C->CTL13    =   0x0000;
    RTC_C->TIM0     = (clock.minute)<<8 | (clock.second); //minutes, seconds
    RTC_C->TIM1     = (clock.hour)<<8 | 12;

    RTC_C->PS1CTL   = 0b11010;

    RTC_C->AMINHR   = 12<<8 | 31 | BIT(15) | BIT(7);
    RTC_C->ADOWDAY = 0;
    RTC_C->CTL0     = ((0xA500) | BIT5);

}
