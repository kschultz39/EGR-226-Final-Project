#include "msp.h"
#include "msp432.h"
#include <stdio.h>
#include "math.h"




/**
   main.c
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





char blinkhour[50];
char blinkminute[50];
char hourdisplay[50];
char minutedisplay[50];

void sethour(void);
void setminute(void);
void PORT1_IRQHandler();
int alarmflag = 0;
int setflag = 0;

int displayhour = 0;
int result = 0;

int hour = 0;
int minute = 0;
int second = 0;
char daynight = 'A';




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


//SOUNDER CONTENT FOR ALARM///////////////////////////////////////////////////////
void SetupTimer32s();

int note = 0;       //The note in the music sequence we are on
int breath = 0;     //Take a breath after each note.  This creates seperation

//Defining the musical notes needed for the song (frequencies)
//F notes
#define F4 349.23
#define F4SHARP 369.99
#define F5 698.46

//E notes
#define E4 329.63
#define E5 659.25

//D notes
#define D4 293.66
#define D5 587.33

//C notes
#define C4 262.63
#define C5 524.25

//B notes
#define B4 493.88
#define B5 987.77

//A notes
#define A4 440
#define A5 880

//G notes
#define G4 392
#define G5 783.99

#define REST 0

//Defining the lengths of notes
#define QUARTER 1000000
#define QUARTERPLUS 1500000
#define EIGHTH 500000
#define HALF 2000000
#define HALFPLUS 3000000
#define WHOLE 4000000
#define BREATH_TIME 50000

#define MAX_NOTE 100 // How many notes are in the song below

float music_note_sequence[][2] = {  // Deck the halls song

  //FIRST
  {C5, QUARTERPLUS},
  {B4, EIGHTH},
  {A4, QUARTER},
  {G4, QUARTER},

  //SECOND
  {F4, QUARTER},
  {G4, QUARTER},
  {A4, QUARTER},
  {F4, QUARTER},

  //THIRD
  {G4, EIGHTH},
  {A4, EIGHTH},
  {B4, EIGHTH},
  {G4, EIGHTH},
  {A4, QUARTER},
  {G4, EIGHTH},

  //FOURTH
  {F4, QUARTER},
  {E4, QUARTER},
  {F4, HALF},


  //////////////////////////////////Second line of music/////////////////////////////////////////

  //FIRST
  {C5, QUARTERPLUS},
  {B4, EIGHTH},
  {A4, QUARTER},
  {G4, QUARTER},


  //SECOND
  {F4, QUARTER},
  {G4, QUARTER},
  {A4, QUARTER},
  {F4, QUARTER},

  //THIRD
  {G4, EIGHTH},
  {A4, EIGHTH},
  {B4, EIGHTH},
  {G4, EIGHTH},
  {A4, QUARTER},
  {G4, EIGHTH},

  //FOURTH
  {F4, QUARTER},
  {E4, QUARTER},
  {F4, HALF},

  ////////////////////////////////Third line of music/////////////////////////////////////////

  //First
  {G4, QUARTERPLUS},
  {A4, EIGHTH},
  {B4, QUARTER},
  {G4, QUARTER},

  //Second
  {A4, QUARTERPLUS},
  {B4, EIGHTH},
  {C5, QUARTER},
  {G4, QUARTER},


  //Third
  {A4, EIGHTH},
  {B4, EIGHTH},
  {C5, QUARTER},
  {D5, EIGHTH},
  {E5, EIGHTH},
  {F5, QUARTER},

  //Fourth
  {E5, QUARTER},
  {D5, QUARTER},
  {C5, HALF},


  ////////////////////////////////Fourth line of music/////////////////////////////////////////

  //First
  {C5, QUARTERPLUS},
  {B4, EIGHTH},
  {A4, QUARTER},
  {G4, QUARTER},

  //Second
  {F4, QUARTER},
  {G4, QUARTER},
  {A4, QUARTER},
  {F4, QUARTER},


  //THIRD

  {D5, EIGHTH},
  {D5, EIGHTH},
  {D5, EIGHTH},
  {D5, EIGHTH},
  {C5, QUARTERPLUS},
  {B4, EIGHTH},


  //FOURTH

  {A4, QUARTER},
  {G4, QUARTER},
  {F4, HALF},

  {REST, WHOLE},
  {REST, WHOLE},
};

///SERIAL INIT CODE
// Making a buffer of 100 characters for serial to store to incoming serial data
#define BUFFER_SIZE 100
char INPUT_BUFFER[BUFFER_SIZE];
// initializing the starting position of used buffer and read buffer
uint8_t storage_location = 0; // used in the interrupt to store new data
uint8_t read_location = 0; // used in the main application to read valid data that hasn't been read yet

void writeOutput(char *string); // write output charactrs to the serial port
void readInput(char* string); // read input characters from INPUT_BUFFER that are valid
void setupP1(); // Sets up P1.0 as an output to drive the on board LED
void setupSerial(); // Sets up serial for use and enables interrupts
unsigned int Serial_flag = 0;


//FOR TIME FROM ZUIDEMA
int time_update = 0, alarm_update = 0;
uint8_t hours, mins, secs;

void RTC_Init();
enum states
{
  DEFAULT,
  SETHOURCLOCK,
  SETMINUTECLOCK,
  SETSECONDCLOCK,
  SETHOURALARM,
  SETMINUTEALARM,
};

void main(void)
{

  WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; //stop watchdog timer
  InitializeAll();
  char buffer[50];
  //serial communication stuf
  char string[BUFFER_SIZE]; // Creates local char array to store incoming serial commands

  int i;
  //int seconds;

  clock.daynight = 'A';

  //SERIAL CODE
  INPUT_BUFFER[0] = '\0'; // Sets the global buffer to initial condition of empty
  setupSerial();

  __enable_irq();  // Enable all interrupts (serial)

  RTC_Init(); //enable RTC

  __enable_interrupt();




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
  // RTC_C->CTL0 = (0xA500) ;
  //RTC_C->CTL13 = 0;
  //    //initialize time to 2:45 pm
  //    //RTC_C->TIM0 = 0x2000; //45 min, 0 sec
  //    RTC_C->TIM0 = (clock.minute) << 8 | (clock.second); //45 min, 0 seconds
  //    RTC_C->TIM1 = 1 << 8 | (clock.hour);
  //    RTC_C->YEAR = 2018;
  //    //Alarm set at 2:46 om
  //    RTC_C->AMINHR = alarm.hour << 8 | alarm.minute | BIT(15) | BIT(7); //bit 15 adn 7 are alarm enable bits
  //    RTC_C->ADOWDAY = 0;
  //    RTC_C->PS1CTL = 0b11010; //1 second interrupt
  //
  //    RTC_C->CTL0 = (0xA500) | BIT5;
  //    RTC_C-> CTL13 = 0;
  //    NVIC_EnableIRQ(RTC_C_IRQn);







  enum states state = DEFAULT;

  while (1)
  {
    switch (state)
    {
      case DEFAULT:
        //workingclock();
        setflag = 0;
        // printf("Alarm set to %d: %2d\n", alarm.hour, alarm.minute);
        //printf("Clock set to %d: %d\n", clock.hour, clock.minute);

        //SERIAL READ STUFF//////////////////////////
        if (Serial_flag)
        {
          readInput(string); // Read the input up to \n, store in string.  This function doesn't return until \n is received

          if (string[0] != '\0') // if string is not empty, check the inputted data.
          {
            //entrynumbers = atoi(&string[1]);

            //Set Time of clock through serial
            if (string[0] == 'S' && string[1] == 'E' && string[2] == 'T' && string[3] == 'T' && string[4] == 'I' && string[5] == 'M' && string[6] == 'E')
            {
              printf("\nset time through serial");

              writeOutput("\n");
              writeOutput("Set Time");

              clock.hour = (string[8] - 48) * 10 + (string[9] - 48);
              clock.minute = (string[11] - 48) * 10 + (string[12] - 48);
              clock.second = (string[14] - 48) * 10 + (string[15] - 48);

              writeOutput(string);
              writeOutput("\n");
              RTC_Init(); //enable RTC
            }

            //Set alarm clock through serial
            if (string[0] == 'S' && string[1] == 'E' && string[2] == 'T' && string[3] == 'A' && string[4] == 'L' && string[5] == 'A' && string[6] == 'R'  && string[7] == 'M')
            {
              printf("\nset alarm through serial");

              writeOutput("Set Alarm");
              writeOutput("\nSet Alarm\n ");

              alarm.hour = (string[9] - 48) * 10 + (string[10] - 48);
              alarm.minute = (string[12] - 48) * 10 + (string[13] - 48);

              writeOutput(string);
              writeOutput("\n");
              RTC_Init();
            }

            //READ Current time through serial
            if (string[0] == 'R' && string[1] == 'E' && string[2] == 'A' && string[3] == 'D' && string[4] == 'T' && string[5] == 'I' && string[6] == 'M' && string[7] == 'E')
            {
              printf("\nRead time through Serial");
              // writeOutput("\nTHE CURRENT TIME IS\n ");
              char buffer[100];
              sprintf(buffer, "The current Time is %2d:%2d:%2d", clock.hour, clock.minute, clock.second);
              writeOutput(buffer);
            }

            //READ Current Alarm through serial
            if (string[0] == 'R' && string[1] == 'E' && string[2] == 'A' && string[3] == 'D' && string[4] == 'A' && string[5] == 'L' && string[6] == 'A' && string[7] == 'R' && string[8] == 'M')
            {
              printf("\nRead ALARM through Serial");
              // writeOutput("\nCURRENT ALARM IS\n  ");
              char bufferalarm[100];
              sprintf(bufferalarm, "The the alarm time is %2d:%2d:00", alarm.hour, alarm.minute);
              writeOutput(bufferalarm);
            }

          }

          Serial_flag = 0;
        }

        //RTC CLOCK
        if (time_update) {
          time_update = 0;
          if (clock.hour > 12)
          {
            displayhour = (clock.hour) - (12);
            clock.daynight = 'P';
          }
          else
          {
            displayhour = (clock.hour);

            clock.daynight = 'A';
          }


          printf("   %02d:%02d:%02d %cM\n", displayhour, clock.minute, clock.second, clock.daynight);
          commandWrite(0x0F); //turn off blinking cursor
          commandWrite(0x0C);
          commandWrite(0xC0);  //moves the cursor to the second
          delay_ms(500);
          sprintf(buffer, "%d:%02d:%02d %cM                     ", displayhour, clock.minute, clock.second, clock.daynight);
          // commandWrite(0x0C); //Prints to line 1 of LCD
          for (i = 0; i < 16; i++)
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

          Fheat = ((Cheat * (9.0 / 5.0)) + 32.0);

          printf("Fheat is %lf\n", Fheat);

          sprintf(buffer, "      %.1f", Fheat);    //puts X value in the string of buffer through use of sprintf() function
          for (i = 0; i < 10; i++) //prints string with information about X in the first line
            dataWrite(buffer[i]);
          dataWrite('F');


          if (alarm_update)
          {
            printf("ALARM\n");


            SetupTimer32s();   //FOR SOUNDER ALARMInitializes Timer32_1 as a non-interrupt timer and Timer32_2 as a interrupt timers.  Also initializes TimerA and P2.4 for music generation.

            alarm_update = 0;

            // state = DEFAULT;

          }


          if (setflag == 1)
          {
            printf("State: Set hour alarm\n");
            state = SETHOURALARM;
          }
          if (setflag == 2)
          {
            printf("State: Set hour clock\n");
            state = SETHOURCLOCK;
          }
          break;
        case SETHOURALARM:
          printf("Setting hour\n");
          setflag = 0;
          sethour();
          if (setflag == 1)
          {
            printf("State: Set minute alarm\n");
            state = SETMINUTEALARM;

          }
          break;
        case SETMINUTEALARM:
          printf("Setting Minute Alarm\n");
          setflag = 0;
          setminute();
          if (setflag == 1)
          {
            printf("State: Default\n");
            alarm.hour = hour;
            alarm.minute = minute;
            alarm.daynight = daynight;
            printf("Alarm set to %d: %2d", alarm.hour, alarm.minute );
            RTC_Init();
            state = DEFAULT;
          }
          break;
        case SETHOURCLOCK:
          printf("setting hour clock\n");
          setflag = 0;
          sethour();
          if (setflag == 2)
          {
            printf("State: Set minute alarm\n");
            state = SETMINUTECLOCK;
          }
          break;
        case SETMINUTECLOCK:
          printf("Setting Minute clock\n");
          setflag = 0;
          setminute();
          if (setflag == 2)
          {
            printf("State: Default\n");
            clock.hour = hour;
            clock.minute = minute;
            clock.daynight = daynight;
            printf("Alarm set to %d: %2d", clock.hour, clock.minute );
            RTC_Init();
            state = DEFAULT;
          }
          break;



        }
    }

  }

}

void InitializeAll(void)
{
  WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

  SysTick_Init();
  P3->SEL0 &= ~(BIT2 | BIT3); //sets P3.2 and P3.3 as GPIO
  P3->SEL1 &= ~(BIT2 | BIT3); //sets P3.2 and P3.3 as GPIO
  P3->DIR |= (BIT2 | BIT3); //sets P3.2 and P3.3 as OUTPUT
  //P3->OUT &= ~(BIT2|BIT3); //sets P3.2 and P3.3 to 0 for RS RW =0


  //LCD
  P2->SEL0 &= ~(BIT4 | BIT5 | BIT6 | BIT7); //sets (DB4-DB7) P2.4, P2.5, P2.5, P2.6, P2.7 as GPIO
  P2->SEL1 &= ~(BIT4 | BIT5 | BIT6 | BIT7);
  P2->DIR |= (BIT4 | BIT5 | BIT6 | BIT7); //sets pins 4.4-4.7 to OUTPUT




  P6->SEL0 &= ~BIT4; //sets P6.4 to GPIO (ENABLE PIN)
  P6->SEL1 &= ~BIT4; //sets P6.4 to GPIO (ENABLE PIN)
  P6->DIR |= BIT4; //sets as output
  P6->OUT &= ~BIT4; //sets Enable pin to 0 initially


  LCD_init();

  //Pin enables for PWM LEDs (Referencing code from Zuidema In-class example Week 5 part 1)
  P7->SEL0 |= (BIT4 | BIT5 | BIT6); //sets SEL0=1;
  P7->SEL1 &= (BIT4 | BIT5 | BIT6); //SEL1 = 0. Setting SEL0=1 and SEL1=0 activates PWM function
  P7->DIR |= (BIT4 | BIT5 | BIT6); // Set pins as  PWM output.
  P7->OUT &= ~(BIT4 | BIT5 | BIT6);

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
  //BUTTON 1.6 and BUTTON 1.7 (Button 1.6 SET ALARM, Button 1.7 SET TIME
  P1->SEL0 &= ~(BIT6 | BIT7);
  P1->SEL1 &= ~(BIT6 | BIT7);
  P1->DIR  &= ~(BIT6 | BIT7);
  P1->REN  |=  (BIT6 | BIT7);
  P1->OUT  |=  (BIT6 | BIT7);
  P1->IE   |=  (BIT6 | BIT7);
  NVIC_EnableIRQ(PORT1_IRQn);

  //BUTTON INITIALIZATION FOR ON/OFF/UP (5.1) and SNOOZE/DOWN (5.2)
  P5-> SEL0 &= ~(BIT1 | BIT2);
  P5 -> SEL1 &= ~(BIT1 | BIT2);
  P5 -> DIR &= ~(BIT1 | BIT2);
  P5 -> REN |= (BIT1 | BIT2);
  P5->OUT |= (BIT1 | BIT2);
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
  TIMER32_1->LOAD = 5860 - 1; //0.25 seconds @ 3MHz

  NVIC_EnableIRQ(T32_INT1_IRQn);
  __enable_interrupt();


  //INITIALIZE SERIAL STUFF
  P1->SEL0 |=  (BIT2 | BIT3); // P1.2 and P1.3 are EUSCI_A0 RX
  P1->SEL1 &= ~(BIT2 | BIT3); // and TX respectively.

  EUSCI_A0->CTLW0 = BIT0;
  EUSCI_A0->CTLW0 = 0b10000110000001;


  EUSCI_A0->CTLW0  = BIT0; // Disables EUSCI. Default configuration is 8N1
  EUSCI_A0->CTLW0 |= BIT7; // Connects to SMCLK BIT[7:6] = 10
  //EUSCI_A0->CTLW0 |= (BIT(15)|BIT(14)|BIT(11));  //BIT15 = Parity, BIT14 = Even, BIT11 = Two Stop Bits
  // Baud Rate Configuration
  // 3000000/(16*9600) = 19.53125  (3 MHz at 96000 bps is fast enough to turn on over sampling (UCOS = /16))
  // UCOS16 = 1 (0ver sampling, /16 turned on)
  // UCBR  = 19 (Whole portion of the divide)
  // UCBRF = .531 * 16 = 8 (0x08) (Remainder of the divide)
  // UCBRS = 3000000/9600 remainder=0.5 -> 0xAA (look up table 22-4)
  EUSCI_A0->BRW = 19;  // UCBR Value from above
  EUSCI_A0->MCTLW = 0xAA81; //UCBRS (Bits 15-8) & UCBRF (Bits 7-4) & UCOS16 (Bit 0)

  EUSCI_A0->CTLW0 &= ~BIT0;  // Enable EUSCI
  EUSCI_A0->IFG &= ~BIT0;    // Clear interrupt
  EUSCI_A0->IE |= BIT0;      // Enable interrupt
  NVIC_EnableIRQ(EUSCIA0_IRQn);

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
  SysTick ->LOAD = ((microsec * 3) - 1); //1ms count down to 0
  SysTick ->VAL = 0; //any write to CVR clears it and COUNTFLAG in CSR

  //wait for flag to be SET (Timeout happened)
  while ((SysTick -> CTRL & 0x00010000) == 0);

}
//uses the SysTick timer peripheral to generate a delay in milliseconds
//function must be able to generate a delay of at least 60 ms
void delay_ms (unsigned ms)
{
  SysTick ->LOAD = ((ms * 3000) - 1); //1microsecond count down to 0
  SysTick ->VAL = 0; //any write to CVR clears it and COUNTFLAG in CSR

  //wait for flag to be SET (Timeout happened)
  while ((SysTick -> CTRL & 0x00010000) == 0);

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
  P2->OUT &= ~(BIT4 | BIT5 | BIT6 | BIT7); //clears values
  P2->OUT |= ((nibble & 0x0F) << 4);
  // delay_micro(100);

  PulseEnablePin();
}

//Pushes the most significant 4 bits of the byte onto the data pins by calling the pushNibble() function
//Then pushes the least significant 4 bits onto the data pins by calling the pushNibble() function again
void pushByte(uint8_t byte)
{
  uint8_t temp;
  temp = ((byte & 0xF0) >> 4);


  //MOST SIGNIFICANT
  pushNibble(temp);


  //LEAST SIGNIFICANT
  temp = (byte & 0x0F);
  pushNibble(temp);
  delay_micro(100);


}
//write one byte of COMMAND by calling the pushByte() function with the COMMAND parameter
void commandWrite(uint8_t command)
{
  //RW to zero
  P3->OUT &= ~(BIT2); //pulls RS pin LOW (expects instructions)


  //RS to zero
  P3 ->OUT &= ~(BIT3);
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
  SysTick -> CTRL = 0; //disable SysTick during setup
  SysTick -> LOAD = 0x00FFFFFF; //maximum reload value
  SysTick -> VAL = 0; //any write to current value clears it
  SysTick -> CTRL = 0x00000005; //enable SysTIck, CPU clk, no interrupts
}

//Interrupt function, set up to read conversion result 2 times a second (FOR TEMPERATURE SENSOR)
void T32_INT1_IRQHandler()
{
  TIMER32_1->INTCLR = 1;      // Clear interrupt needs to happen first for some reason (unknown)

  // add to seconds

  result = ADC14->MEM[5];  //read conversion result, STORES TO MEM LOCATION 5
}

//Interupt function for Set Alarm and Set Time
void PORT1_IRQHandler(void)
{
  if (P1->IFG & BIT6) {                               //If P1.1 had an interrupt
    setflag = 1;
    printf("1.6 pressed");
  }
  if (P1->IFG & BIT7) {                               //If P1.4 had an interrupt
    setflag = 2;
    printf("1.7 pressed");
  }
  P1->IFG = 0;                                        //Clear all flags
}

void RTC_C_IRQHandler()
{
  if (RTC_C->PS1CTL & BIT0)
  {
    time_update = 1;
    clock.hour = RTC_C->TIM1 & 0x00FF;
    clock.minute = (RTC_C->TIM0 & 0xFF00) >> 8;
    clock.second = RTC_C->TIM0 & 0x00FF;

    RTC_C->PS1CTL &= ~BIT0;
  }

  if (RTC_C->CTL0 & BIT1)
  {

    alarm_update = 1;
    RTC_C->CTL0 = (0xA500) | BIT5;
  }
}


//FOR SOUNDER CODE************************************************************************************
/*
  void TA0_N_IRQHandler()

  Interrupt Handler for Timer A1.  The name of this function is set in startup_msp432p401r_ccs.c

  This handler clears the status of the interrupt for Timer32_A0 CCTL 1 and 2


  Turns on when CCTL1 interrupts.  Turns off with CCTL2 interrupts.

  -------------------------------------------------------------------------------------------------------------------------------*/
void TA0_N_IRQHandler()
{
  if (TIMER_A1->CCTL[4] & BIT0) {                 //If CCTL1 is the reason for the interrupt (BIT0 holds the flag)
  }
  if (TIMER_A1->CCTL[1] & BIT0) {                 //If CCTL1 is the reason for the interrupt (BIT0 holds the flag)
  }
}

/*-------------------------------------------------------------------------------------------------------------------------------

   void SetupTimer32s()

   Configures Timer32_1 as a single shot (runs once) timer that does not interrupt so the value must be monitored.
   Configures Timer32_2 as a single shot (runs once) timer that does interrupt and will run the interrupt handler 1 second
   after this function is called (and the master interrupt is enabled).

  -------------------------------------------------------------------------------------------------------------------------------*/
void SetupTimer32s()
{
  TIMER32_1->CONTROL = 0b11000011;                //Sets timer 1 for Enabled, Periodic, No Interrupt, No Prescaler, 32 bit mode, One Shot Mode.  See 589 of the reference manual
  TIMER32_2->CONTROL = 0b11100011;                //Sets timer 2 for Enabled, Periodic, With Interrupt, No Prescaler, 32 bit mode, One Shot Mode.  See 589 of the reference manual
  NVIC_EnableIRQ(T32_INT2_IRQn);                  //Enable Timer32_2 interrupt.  Look at msp.h if you want to see what all these are called.
  TIMER32_2->LOAD = 3000000 - 1;                  //Set to a count down of 1 second on 3 MHz clock

  TIMER_A1->CCR[0] = 0;                           // Turn off timerA to start
  TIMER_A1->CCTL[4] = 0b0000000011110100;         // Setup Timer A1_ Reset/Set, Interrupt, No Output
  TIMER_A1->CCR[4] = 0;                           // Turn off timerA to start

  //possible issues here////
  TIMER_A1->CCTL[1] = 0b0000000011110100;         // Setup Timer A1_1 Reset/Set, Interrupt, No Output
  TIMER_A1->CCR[1] = 0;                           // Turn off timerA to start
  ///////////////////////
  TIMER_A1->CTL = 0b0000001000010100;             // Count Up mode using SMCLK, Clears, Clear Interrupt Flag

  NVIC_EnableIRQ(TA0_N_IRQn);                     // Enable interrupts for CCTL1-6 (if on)

  P7->SEL0 |= BIT4;                               // Setup the P7.4 to be an output for the ALARM.  This should drive a sounder.
  P7->SEL1 &= ~BIT4;
  P7->DIR |= BIT4;
}

/*-------------------------------------------------------------------------------------------------------------------------------

   void T32_INT2_IRQHandler()

   Interrupt Handler for Timer 2.  The name of this function is set in startup_msp432p401r_ccs.c

   This handler clears the status of the interrupt for Timer32_2

   Sets up the next note to play in sequence and loads it into TimerA for play back at that frequency.
   Enables a new Timer32 value to interrupt after the note is complete.

  -------------------------------------------------------------------------------------------------------------------------------*/

void T32_INT2_IRQHandler()
{
  TIMER32_2->INTCLR = 1;                                      //Clear interrupt flag so it does not interrupt again immediately.
  if (breath) {                                               //Provides separation between notes
    TIMER_A1->CCR[0] = 0;                                   //Set output of TimerA to 0
    TIMER_A1->CCR[4] = 0;
    TIMER_A1->CCR[1] = 0;
    TIMER32_2->LOAD = BREATH_TIME;                          //Load in breath time to interrupt again
    breath = 0;                                             //Next Timer32 interrupt is no longer a breath, but is a note
  }
  else {                                                      //If not a breath (a note)
    TIMER32_2->LOAD = music_note_sequence[note][1] - 1;     //Load into interrupt count down the length of this note
    if (music_note_sequence[note][0] == REST) {             //If note is actually a rest, load in nothing to TimerA
      TIMER_A1->CCR[0] = 0;
      TIMER_A1->CCR[4] = 0;
      TIMER_A1->CCR[1] = 0;

      //P1->OUT |= BIT0; //Turn ON
    }




    else {
      TIMER_A1->CCR[0] = 3000000 / music_note_sequence[note][0];  //Math in an interrupt is bad behavior, but shows how things are happening.  This takes our clock and divides by the frequency of this note to get the period.
      TIMER_A1->CCR[4] = 1500000 / music_note_sequence[note][0];  //50% duty cycle
      TIMER_A1->CCR[1] = TIMER_A0->CCR[0];                        //Had this in here for fun with interrupts.  Not used right now
    }
    note = note + 1;                                                //Next note
    if (note >= MAX_NOTE) {                                         //Go back to the beginning if at the end
      note = 0;
    }
    breath = 1;                                             //Next time through should be a breath for separation.
  }
}


/*----------------------------------------------------------------
   void writeOutput(char *string)

   Description:  This is a function similar to most serial port
   functions like printf.  Written as a demonstration and not
   production worthy due to limitations.
   One limitation is poor memory management.
   Inputs: Pointer to a string that has a string to send to the serial.
   Outputs: Places the data on the serial output.
  ----------------------------------------------------------------*/
void writeOutput(char *string)
{
  int i = 0;  // Location in the char array "string" that is being written to

  while (string[i] != '\0') {
    EUSCI_A0->TXBUF = string[i];
    i++;
    while (!(EUSCI_A0->IFG & BIT1));
  }
}

/*----------------------------------------------------------------
   void readInput(char *string)

   Description:  This is a function similar to most serial port
   functions like ReadLine.  Written as a demonstration and not
   production worthy due to limitations.
   One of the limitations is that it is BLOCKING which means
   it will wait in this function until there is a \n on the
   serial input.
   Another limitation is poor memory management.
   Inputs: Pointer to a string that will have information stored
   in it.
   Outputs: Places the serial data in the string that was passed
   to it.  Updates the global variables of locations in the
   INPUT_BUFFER that have been already read.
  ----------------------------------------------------------------*/
void readInput(char *string)
{
  int i = 0;  // Location in the char array "string" that is being written to

  // One of the few do/while loops I've written, but need to read a character before checking to see if a \n has been read
  do
  {
    // If a new line hasn't been found yet, but we are caught up to what has been received, wait here for new data
    while (read_location == storage_location && INPUT_BUFFER[read_location] != 13);
    string[i] = INPUT_BUFFER[read_location];  // Manual copy of valid character into "string"
    INPUT_BUFFER[read_location] = '\0';
    i++; // Increment the location in "string" for next piece of data
    read_location++; // Increment location in INPUT_BUFFER that has been read
    if (read_location == BUFFER_SIZE) // If the end of INPUT_BUFFER has been reached, loop back to 0
      read_location = 0;
  }
  while (string[i - 1] != 13); // If a \n was just read, break out of the while loop

  string[i - 1] = '\0'; // Replace the \n with a \0 to end the string when returning this function
}

/*----------------------------------------------------------------
   void EUSCIA0_IRQHandler(void)

   Description: Interrupt handler for serial communication on EUSCIA0.
   Stores the data in the RXBUF into the INPUT_BUFFER global character
   array for reading in the main application
   Inputs: None (Interrupt)
   Outputs: Data stored in the global INPUT_BUFFER. storage_location
   in the INPUT_BUFFER updated.
  ----------------------------------------------------------------*/
void EUSCIA0_IRQHandler(void)
{
  if (EUSCI_A0->IFG & BIT0)  // Interrupt on the receive line
  {
    INPUT_BUFFER[storage_location] = EUSCI_A0->RXBUF; // store the new piece of data at the present location in the buffer
    EUSCI_A0->IFG &= ~BIT0; // Clear the interrupt flag right away in case new data is ready
    storage_location++; // update to the next position in the buffer
    if (storage_location == BUFFER_SIZE) // if the end of the buffer was reached, loop back to the start
      storage_location = 0;

    if (EUSCI_A0->RXBUF == 13)
      Serial_flag = 1;
    else
      Serial_flag = 0;
  }
}


/*----------------------------------------------------------------
   void setupSerial()
   Sets up the serial port EUSCI_A0 as 115200 8E2 (8 bits, Even parity,
   two stops bit.)  Enables the interrupt so that received data will
   results in an interrupt.
   Description:
   Inputs: None
   Outputs: None
  ----------------------------------------------------------------*/
void setupSerial()
{
  P1->SEL0 |=  (BIT2 | BIT3); // P1.2 and P1.3 are EUSCI_A0 RX
  P1->SEL1 &= ~(BIT2 | BIT3); // and TX respectively.

  EUSCI_A0->CTLW0 = BIT0;
  EUSCI_A0->CTLW0 = 0b10000110000001;


  EUSCI_A0->CTLW0  = BIT0; // Disables EUSCI. Default configuration is 8N1
  EUSCI_A0->CTLW0 |= BIT7; // Connects to SMCLK BIT[7:6] = 10
  //EUSCI_A0->CTLW0 |= (BIT(15)|BIT(14)|BIT(11));  //BIT15 = Parity, BIT14 = Even, BIT11 = Two Stop Bits
  // Baud Rate Configuration
  // 3000000/(16*9600) = 19.53125  (3 MHz at 96000 bps is fast enough to turn on over sampling (UCOS = /16))
  // UCOS16 = 1 (0ver sampling, /16 turned on)
  // UCBR  = 19 (Whole portion of the divide)
  // UCBRF = .531 * 16 = 8 (0x08) (Remainder of the divide)
  // UCBRS = 3000000/9600 remainder=0.5 -> 0xAA (look up table 22-4)
  EUSCI_A0->BRW = 19;  // UCBR Value from above
  EUSCI_A0->MCTLW = 0xAA81; //UCBRS (Bits 15-8) & UCBRF (Bits 7-4) & UCOS16 (Bit 0)

  EUSCI_A0->CTLW0 &= ~BIT0;  // Enable EUSCI
  EUSCI_A0->IFG &= ~BIT0;    // Clear interrupt
  EUSCI_A0->IE |= BIT0;      // Enable interrupt
  NVIC_EnableIRQ(EUSCIA0_IRQn);
}


//RTC CODE
void RTC_Init()
{
  //RTC CODE FROM LECTURE
  RTC_C->CTL0 = (0xA500) ;
  RTC_C->CTL13 = 0;
  //initialize time to 2:45 pm
  //RTC_C->TIM0 = 0x2000; //45 min, 0 sec
  RTC_C->TIM0 = (clock.minute) << 8 | (clock.second); //45 min, 0 seconds
  RTC_C->TIM1 = 1 << 8 | (clock.hour);
  RTC_C->YEAR = 2018;
  //Alarm set at 2:46 om
  RTC_C->AMINHR = alarm.hour << 8 | alarm.minute | BIT(15) | BIT(7); //bit 15 adn 7 are alarm enable bits
  RTC_C->ADOWDAY = 0;
  RTC_C->PS1CTL = 0b11010; //1 second interrupt

  RTC_C->CTL0 = (0xA500) | BIT5;
  RTC_C-> CTL13 = 0;
  NVIC_EnableIRQ(RTC_C_IRQn);

}


void sethour(void)
{
  int i = 0;
  int pva_up=0;
  int pva_down=0;
  while (setflag == 0)

  {
    if (!((P5->IN & BIT1) == BIT1))
    {
      if (hour == 11)
      {
        hour=12;
        daynight= 'P';

        printf("HOURINC: %d\n", hour);
        sprintf(hourdisplay, "%d:%2d:%2d %cM", hour, minute, second, daynight);
        commandWrite(0xC0);
        for (i = 0; i < 10; i++)
          dataWrite(hourdisplay[i]);
        __delay_cycles(300000);
        pva_up+=1;

      }
      else if (hour != 11)
      {
          if(hour==12)
              {
              if(pva_up==1)
              { hour=1;
              daynight= 'P';
              }

              if(pva_up==2)
              {
                  hour=1;
                  daynight= 'A';
                  pva_up=0;
              }

              }
          else if(hour!=12)
              hour += 1;
        printf("HOURINC: %d\n", hour);
        __delay_cycles(300000);
//        if (hour == 12)
//        {
//          daynight = 'P';
//        }
        sprintf(hourdisplay, "%d:%2d:%2d %cM", hour, minute, second, daynight);
        commandWrite(0xC0);
        for (i = 0; i < 10; i++)
          dataWrite(hourdisplay[i]);

      }
    }

    if (!((P5->IN & BIT2) == BIT2))
    {
      if (hour == 1)
      {
        hour = 12;



        printf("HOURDEC: %d\n", hour);
        sprintf(hourdisplay, "%d:%2d:%2d %cM", hour, minute, second, daynight);
        commandWrite(0xC0);
        for (i = 0; i < 10; i++)
          dataWrite(hourdisplay[i]);
        __delay_cycles(300000);
        pva_down+=1;


      }
      else if (hour != 1)
      {
          if(hour==12)
                        {
                       
                      if(pva_up==1)
                        { hour=11;
                        daynight= 'P';
                        }

                        if(pva_up==2)
                        {
                            hour=11;
                            daynight= 'A';
                            pva_up=0;
                        }

                        }
          else if(hour!=12)

          { hour -= 1;

          }
        printf("HOURDEC: %d\n", hour);
        __delay_cycles(300000);

        sprintf(hourdisplay, "%d:%2d:%2d %cM", hour, minute, second, daynight);
        commandWrite(0xC0);
        for (i = 0; i < 10; i++)
          dataWrite(hourdisplay[i]);
      }

    }

//    while (((((P5->IN & BIT1) == BIT1)) && ((P5->IN & BIT2) == BIT2)))
//    {
//    sprintf(hourdisplay, "%d:%2d:%2d %cM", hour, minute, second, daynight);
//    commandWrite(0xC0);
//    for (i = 0; i < 10; i++)
//      dataWrite(hourdisplay[i]);
//    __delay_cycles(300000);
//    sprintf(blinkhour, "  :%2d:%2d", minute, second);
//    commandWrite(0xC0);
//    for (i = 0; i < 10; i++)
//      dataWrite(blinkhour[i]);
//    __delay_cycles(300000);
//    }


  }
}
void setminute(void)
{
  int i = 0;
  while (setflag == 0)

  {
    if (!((P5->IN & BIT1) == BIT1))
    {
      if (minute == 59)
      {
        minute = 0;
        __delay_cycles(300000);
        sprintf(minutedisplay, "%d: %2d:%2d %cM", hour, minute, second, daynight);
        commandWrite(0xC0);
        for (i = 0; i < 10; i++)
          dataWrite(minutedisplay[i]);
        printf("MININC: %d\n", minute);
      }
      else if (minute != 59)
      {
        minute += 1;
        sprintf(minutedisplay, "%d: %2d:%2d %cM", hour, minute, second, daynight);
        commandWrite(0xC0);
        for (i = 0; i < 10; i++)
          dataWrite(minutedisplay[i]);
        printf("MININC: %d\n", minute);
        __delay_cycles(300000);



      }
    }

    if (!((P5->IN & BIT2) == BIT2))
    {
      if (minute == 0)
      {
        minute = 59;
        printf("MINDEC: %d\n", minute);
        sprintf(minutedisplay, "%d:%2d:%2d %cM", hour, minute, second, daynight);
        commandWrite(0xC0);
        for (i = 0; i < 10; i++)
          dataWrite(minutedisplay[i]);
        __delay_cycles(300000);

      }
      else if (minute != 0)
      {
        minute -= 1;
        printf("MINDEC: %d\n", minute);
        sprintf(minutedisplay, "%d:%2d:%2d %cM", hour, minute, second, daynight);
        commandWrite(0xC0);
        for (i = 0; i < 10; i++)
          dataWrite(minutedisplay[i]);
        __delay_cycles(300000);

      }

    }

//    while (((((P5->IN & BIT1) == BIT1)) && ((P5->IN & BIT2) == BIT2))){
//    commandWrite(0xC0);
//    sprintf(minutedisplay, "%d:%2d:%2d", hour, minute, second);
//
//    for (i = 0; i < 10; i++)
//      dataWrite(minutedisplay[i]);
//    __delay_cycles(300000);
//    commandWrite(0xC0);
//    sprintf(blinkminute, "%d:  :%2d", hour, second);
//    for (i = 0; i < 11; i++)
//      dataWrite(blinkminute[i]);
//    __delay_cycles(300000);
//    }


  }
}
