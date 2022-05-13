//2021-12-23
//为GPS供电增加控制，这样就可以方便的使用usb烧写固件
//修改频率范围为1KHz-200MHz

//2022-03-06
//修改编码器为中断方式工作，编码器变得更好用

// include the library code:
#include <TinyGPS++.h>
#include <LiquidCrystal.h>
#include <string.h>
#include <ctype.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Wire.h>
#include <si5351.h>
#include <EEPROMex.h>

// The TinyGPS++ object
TinyGPSPlus gps;
Si5351 si5351;

#define control	                 2              // 1=button ,2=EP11 ,3=EP12

// Set up MCU pins
#define LDO_Enable               6
#define ppsPin                   2
//#define encoderPinA              A1
//#define encoderPinB              A0
#define przycisk                A2
#define sw_up					          A1
#define sw_down					        A0
#define RS                       7
#define E                        8
#define DB4                      9
#define DB5                     10
#define DB6                     11
#define DB7                     12
#define Freq2					1000000000ULL

volatile byte seqA = 0;
volatile byte seqB = 0;
volatile byte cnt1 = 0;
volatile byte cnt2 = 0;
volatile boolean right = false;
volatile boolean left = false;
volatile boolean button = false;


LiquidCrystal lcd(RS, E, DB4, DB5, DB6, DB7);

// configure variables
unsigned long XtalFreq = 100000000;
unsigned long XtalFreq_old = 100000000;
long stab;
long correction = 0;
byte stab_count = 44;
unsigned long mult = 0, Freq = 10000000;
int second = 0, minute = 0, hour = 0;
int zone = 1;
unsigned int tcount = 0;
unsigned int tcount2 = 0;
int validGPSflag = false;
char c;
boolean newdata = false;
boolean GPSstatus = true;
byte new_freq = 1;
unsigned long freq_step = 1000;
byte encoderOLD, menu = 0, band = 1, f_step = 1;
boolean time_enable = true;
unsigned long pps_correct;
byte pps_valid = 1;
byte correct_byte;
float stab_float;
//*************************************************************************************
//                                    SETUP
//*************************************************************************************
void setup()
{
	
  pinMode(LDO_Enable, OUTPUT); // GPS电源控制引脚使能
  digitalWrite(LDO_Enable, HIGH); //GPS电源控制引脚输出高电平
//  pinMode(encoderPinA, INPUT);                    // Set up rotary encoder
//  digitalWrite(encoderPinA, HIGH);
//  pinMode(encoderPinB, INPUT);
//  digitalWrite(encoderPinB, HIGH);
  pinMode(sw_up, INPUT);                    // Set up rotary encoder
  digitalWrite(sw_up, HIGH);
  pinMode(sw_down, INPUT);
  digitalWrite(sw_down, HIGH);
  
  pinMode(przycisk, INPUT);                      // Set up push buttons
  digitalWrite(przycisk, HIGH);

  PCICR =  0b00000010; // 1. PCIE1: Pin Change Interrupt Enable 1
  PCMSK1 = 0b00000111; // Enable Pin Change Interrupt for A0, A1, A2

  TCCR1B = 0;                                    //Disable Timer5 during setup
  TCCR1A = 0;                                    //Reset
  TCNT1  = 0;                                    //Reset counter to zero
  TIFR1  = 1;                                    //Reset overflow
  TIMSK1 = 1;                                    //Turn on overflow flag
  pinMode(ppsPin, INPUT);                        // Inititalize GPS 1pps input
  digitalWrite(ppsPin, HIGH);


  lcd.begin(16, 2);
  lcd.display();
  if (digitalRead(przycisk) == 0) {
	  lcd.print("Initialization");
	  EEPROM.writeLong(1 * 4, 10000000);
	  EEPROM.writeLong(2 * 4, 16700000);
	  EEPROM.writeLong(3 * 4, 37200000);
	  EEPROM.writeLong(4 * 4, 64800000);
	  EEPROM.writeLong(5 * 4, 145000000);
	  EEPROM.writeByte(82, 1);
	  EEPROM.writeInt(80, 1);
	  EEPROM.writeLong(90, 0);
	  EEPROM.writeByte(99, 1);
	  delay(3000);
  }
	band = EEPROM.readByte(82);
	Freq = EEPROM.readLong(band * 4);
	zone = EEPROM.readInt(80);
	correct_byte = EEPROM.readByte(99);
	correction = EEPROM.readLong(90);
	
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);

  Serial.begin(9600);

  // Set CLK0 to output 2,5MHz
  si5351.set_ms_source(SI5351_CLK0, SI5351_PLLA);
  si5351.set_freq(250000000ULL, SI5351_CLK0);
  si5351.set_ms_source(SI5351_CLK1, SI5351_PLLB);
  si5351.set_ms_source(SI5351_CLK2, SI5351_PLLA);
  si5351.set_freq(Freq * SI5351_FREQ_MULT, SI5351_CLK1);
  si5351.set_freq(Freq2, SI5351_CLK2);
  correct_si5351a();
  si5351.update_status();

  lcd.clear();
  lcd.print(" GPS  GENERATOR");
  lcd.setCursor(0, 1);
  lcd.print("Si5351a by SQ1GU");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Waiting for GPS");

  GPSproces(2000);

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    lcd.home();
    lcd.print("No GPS connected");
    lcd.setCursor(0, 1);
    lcd.print (" check wiring!! ");
    delay(5000);
    GPSstatus = false;
  }
  lcd.clear();
  
  
  // if GPS connected then...
  if (GPSstatus == true) {
    lcd.print("Waiting for SAT");
    time_on_lcd();
    sat_on_lcd();
    do {
      GPSproces(1000);
    } while (gps.satellites.value() == 0);

    hour = gps.time.hour() + zone;
    minute = gps.time.minute();
    second = gps.time.second();
    time_on_lcd();
    sat_on_lcd();
    attachInterrupt(0, PPSinterrupt, RISING);
    TCCR1B = 0;
    tcount = 0;
    mult = 0;
    validGPSflag = 1;
  }
 
  
  freq_on_lcd();
  lcd.print(" kHz");
  sat_on_lcd();
  time_on_lcd();
}

//***************************************************************************************
//                                         LOOP
//***************************************************************************************
void loop()
{
  if (tcount2 != tcount) {
    tcount2 = tcount;
    pps_correct = millis();
  }
  if (tcount < 4 ) {
    GPSproces(0);
  }
  if (gps.time.isUpdated()) {
    hour = (gps.time.hour() + zone) % 24;
	if(hour<0) hour = hour * -1;
    minute = gps.time.minute();
    second = gps.time.second();
  }
  if (gps.satellites.isUpdated() && menu == 0) {
    sat_on_lcd();
  }

  if (new_freq == 1) {
    correct_si5351a();
    new_freq = 0;
    lcd.setCursor(15, 0);
    lcd.print("*");
  }
  if (new_freq == 2) {
    update_si5351a();
    freq_on_lcd();
    new_freq = 0;
  }
  if (digitalRead(przycisk) == 0) {
    delay(10);
    if (digitalRead(przycisk) == 0) {
      menu++;
      if (menu > 5) menu = 0;
      lcd.setCursor(0, 1);
      switch (menu) {
        case 0:
          sat_on_lcd();
          time_on_lcd();
          time_enable = true;
          break;
        case 1:
          band_on_lcd();
          break;
        case 2:
          step_on_lcd();
          break;
        case 3:
          freq2_on_lcd();
          break;
        case 4:
          EEPROM.writeLong(band * 4, Freq);
          stab_on_lcd();
          break;
        case 5:
          timezone_on_lcd();
          break;
      }
      delay(200);
    }
  }



  if (millis() > pps_correct + 1200) {
    pps_valid = 0;
    pps_correct = millis();
    time_enable = false;
    lcd.setCursor (15, 0);
    lcd.print(" ");

  }
  
  //ENCread();

  //编码器改为中断方式
  if (left) {
    left = false;
    change_down();     
  }

  if (right) {
    right = false;
    change_up();     
  }

}
/*
//**************************************************************************************
//                       INTERRUPT  ENC
//**************************************************************************************
void ENCread(){
	if(control == 1){
		if(digitalRead(sw_up) == 0){
			delay(200);
			if (digitalRead(sw_up) == 0) change_up(); delay(200);
		}
		if(digitalRead(sw_down) == 0){
			delay(200);
			if (digitalRead(sw_down) == 0) change_down(); delay(200);			
		}
	}
	
  if (control == 2){
    if (digitalRead(encoderPinA) == 0) {
      delay(2);
      if (digitalRead(encoderPinA) == 0) {
        if (digitalRead(encoderPinB) == 0) change_up();
        else change_down();  
        delay(40); 
      }
    }
  } 
  
	if(control == 3){
		if (digitalRead(encoderPinA) != encoderOLD) {
		delay(4);
			if (digitalRead(encoderPinA) != encoderOLD) {
				encoderOLD = digitalRead(encoderPinA);
				if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) change_up();
				else change_down();
       delay(50);
			}
		}
	}
}
*/
//*******************************************
void change_up(){
        switch (menu) {
          case 1: {
              band++;
              if (band > 5) band = 5;
			  EEPROM.writeByte(82, band);
			  band_on_lcd();
            }
            break;
          case 2: {
              f_step++;
              if (f_step > 8)f_step = 8;
              step_on_lcd();
            }
            break;
          case 3: {
              Freq += freq_step;
              if (Freq > 200000000) Freq -= freq_step;
              new_freq = 2;
            }
            break;
          case 5: {
              zone++;
              if (zone > 12)zone = 12;
              EEPROM.writeInt(80, zone);
              timezone_on_lcd();
            }
            break;
        }
}
void change_down(){
        switch (menu) {
          case 1: {
              band--;
              if (band == 0) band = 1;
              EEPROM.writeByte(82, band);
			  band_on_lcd();
			  }
            break;
          case 2: {
              f_step--;
              if (f_step == 0 )f_step = 1;
              step_on_lcd();
            }
            break;
          case 3: {
              Freq -= freq_step;
              if (Freq > 200000000 || Freq < 1000) Freq += freq_step;
              new_freq = 2;
            }
            break;
          case 5: {
              zone--;
              if (zone < -12)zone = -12;
              EEPROM.writeInt(80, zone);
              timezone_on_lcd();
            }
            break;
        }   
}

//**************************************************************************************
//                       INTERRUPT  1PPS
//**************************************************************************************
//******************************************************************
void PPSinterrupt()
{
  tcount++;
  stab_count--;
  if (tcount == 4)                               // Start counting the 2.5 MHz signal from Si5351A CLK0
  {
    TCCR1B = 7;                                  //Clock on rising edge of pin 5
    // loop();
  }
  if (tcount == 44)                              //The 40 second gate time elapsed - stop counting
  {
    TCCR1B = 0;                                  //Turn off counter
    if (pps_valid == 1) {
      XtalFreq_old = XtalFreq;
      XtalFreq = mult * 0x10000 + TCNT1;           //Calculate correction factor
      new_freq = 1;
    }
    TCNT1 = 0;                                   //Reset count to zero
    mult = 0;
    tcount = 0;                                  //Reset the seconds counter
    pps_valid = 1;
    Serial.begin(9600);
    stab_count = 44;
	correct_freq();
    if (menu == 4)stab_on_lcd();
  }
  if (validGPSflag == 1)                      //Start the UTC timekeeping process
  {
    second++;
    if (second == 60)                            //Set time using GPS NMEA data
    {
      minute++ ;
      second = 0 ;
    }
    if (minute == 60)
    {
      hour++;
      minute = 0 ;
    }
    if (hour == 24) hour = 0 ;
    if (time_enable) time_on_lcd();
  }
  if (menu == 4) {
    lcd.setCursor(14, 1);
    if (stab_count < 10) lcd.print(" ");
    lcd.print(stab_count);
  }
}
//*******************************************************************************
// Timer 1 overflow intrrupt vector.
ISR(TIMER1_OVF_vect)
{
  mult++;                                          //Increment multiplier
  TIFR1 = (1 << TOV1);                             //Clear overlow flag
}
//********************************************************************************
//                                TIMEZONE on LCD <>
//********************************************************************************
void timezone_on_lcd()
{
  time_enable = false;
  lcd.setCursor(0, 1);
  lcd.print("TIME zone ");
  if (zone > 0) lcd.print("+");
  lcd.print(zone);
  lcd.print(" < > ");
}
//********************************************************************************
//                                STAB on LCD  stabilnośc częstotliwości
//********************************************************************************
void stab_on_lcd() {

  lcd.setCursor(7, 1);
  lcd.print("         ");
  lcd.setCursor(0, 1);
  lcd.print("Fstab ");

  lcd.print(stab_float);
  lcd.print("Hz");
}
//********************************************************************************
//   correct ferquency
//********************************************************************************
void correct_freq()
{
  long pomocna;
  time_enable = false;

  stab = XtalFreq - 100000000;
  stab = stab * 10 ;
  if (stab > 100 || stab < -100) {
	  correction = correction + stab;
  }
  else if (stab > 20 || stab < -20) {
	  correction = correction + stab / 2;
  }
  else correction = correction + stab / 4;
  
  if(stab < 5 || stab > -5) {
	  if(correct_byte != 0){
		  EEPROM.writeLong(90, correction);
		  EEPROM.writeByte(99, 0);
	  }
  }
  
  pomocna = (10000 / (Freq / 1000000));
  stab = stab * 100;
  stab = stab / pomocna;
  stab_float = float(stab);
  stab_float = stab_float / 10;
	
}
//********************************************************************************
//                                FREQ_2 on LCD <>
//********************************************************************************
void freq2_on_lcd()
{
  time_enable = false;
  lcd.setCursor(0, 1);
  lcd.print("FREQ Bank ");
  lcd.print(band);
  lcd.print(" < > ");
}
//********************************************************************************
//                                STEP on LCD
//********************************************************************************
void step_on_lcd()
{
  time_enable = false;
  lcd.setCursor(0, 1);
  lcd.print("STEP ");
  switch (f_step) {
    case 1: freq_step = 1, lcd.print("   1Hz");
      break;
    case 2: freq_step = 10, lcd.print("  10Hz");
      break;
    case 3: freq_step = 100, lcd.print(" 100Hz");
      break;
    case 4: freq_step = 1000, lcd.print("  1kHz");
      break;
    case 5: freq_step = 10000, lcd.print(" 10kHz");
      break;
    case 6: freq_step = 100000, lcd.print("100kHz");
      break;
    case 7: freq_step = 1000000 , lcd.print("  1MHz");
      break;
    case 8: freq_step = 10000000, lcd.print(" 10Mhz");
      break;
  }
}
//********************************************************************************
//                                BAND on LCD
//********************************************************************************
void band_on_lcd()
{
  time_enable = false;
  lcd.setCursor(0, 1);
  lcd.print("BANK ");
  lcd.print(band);
  lcd.print("      < > ");
  Freq = EEPROM.readLong(band * 4);
  freq_on_lcd();
  update_si5351a();
}
//********************************************************************************
//                                TIME on LCD
//********************************************************************************
void time_on_lcd()
{
  char sz[32];
  sprintf(sz, "%02d:%02d:%02d ", hour, minute, second);
  lcd.setCursor(7, 1);
  lcd.print(" ");
  lcd.print(sz);
}
//********************************************************************************
//                                SAT nr. on LCD
//********************************************************************************
void sat_on_lcd()
{
  time_enable = false;
  lcd.setCursor(0, 1);
  lcd.print("SAT ");
  lcd.print(gps.satellites.value());
  lcd.print("  ");
  time_enable = true;
}

//*********************************************************************************
//                             Freq on LCD
//*********************************************************************************
void freq_on_lcd() {
  char buf[10];

  // Print frequency to the LCD
  ltoa(Freq, buf, 10);
  time_enable = false;
  lcd.home();
  if (Freq < 1000000)
  {
    lcd.print(" ");
    lcd.print(" ");
    lcd.print(" ");
    lcd.print(" ");
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print('.');
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print(buf[5]);
  }

  if (Freq >= 1000000 && Freq < 10000000)
  {
    lcd.print(" ");
    lcd.print(" ");
    lcd.print(buf[0]);
    lcd.print(',');
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print('.');
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print(buf[6]);
  }

  if (Freq >= 10000000 && Freq < 100000000)
  {
    lcd.print(" ");
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(',');
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print('.');
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(buf[7]);
  }

  if (Freq >= 100000000)
  {
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print(',');
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print('.');
    lcd.print(buf[6]);
    lcd.print(buf[7]);
    lcd.print(buf[8]);
  }
}
//********************************************************************
//             NEW frequency
//********************************************************************
void update_si5351a()
{
  si5351.set_freq(Freq * SI5351_FREQ_MULT, SI5351_CLK1);

}
//********************************************************************
//             NEW frequency correction
//********************************************************************
void correct_si5351a()
{
  si5351.set_correction(correction, SI5351_PLL_INPUT_XO);

  //update_si5351a();

}
//*********************************************************************
//                    Odczyt danych z GPS
//**********************************************************************
static void GPSproces(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial.available())
      gps.encode(Serial.read());
  } while (millis() - start < ms);
}
//*********************************************************************


ISR (PCINT1_vect) {

// If interrupt is triggered by the button
  if (!digitalRead(przycisk)) {
    
    button = true;}

// Else if interrupt is triggered by encoder signals
  else {
    
    // Read A and B signals
    //boolean A_val = digitalRead(encoderPinB);
    //boolean B_val = digitalRead(encoderPinA);
    boolean A_val = digitalRead(sw_down);
    boolean B_val = digitalRead(sw_up);
    
    // Record the A and B signals in seperate sequences
    seqA <<= 1;
    seqA |= A_val;
    
    seqB <<= 1;
    seqB |= B_val;
    
    // Mask the MSB four bits
    seqA &= 0b00001111;
    seqB &= 0b00001111;
    
    // Compare the recorded sequence with the expected sequence
    if (seqA == 0b00001001 && seqB == 0b00000011) {
      cnt1++;
      left = true;
      }
     
    if (seqA == 0b00000011 && seqB == 0b00001001) {
      cnt2++;
      right = true;
      }
  }

}  
