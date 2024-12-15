#include <RTClib.h>
#include <LiquidCrystal.h>
#include <string.h>
#include <Stepper.h>
#include <DHT.h>


#define WATER_LEVEL_CHANNEL 0 // ANALOG PIN 0. used by ADC for checking water level
const int WATER_LEVEL_THRESHOLD = 10;
const int TEMPERATURE_THRESHOLD = 40;

#define DHT_PIN 6   // PIN 6. sensors for humidity and temperature.
#define DHT_TYPE DHT11


#define RECEIVER_DATA_AVAILABLE 0x80  // 0b1000_0000
#define TRANSMIT_BUFFER_EMPTY 0x20  // 0b0010_0000  

#define STEPPER_PIN1 10
#define STEPPER_PIN2 9
#define STEPPER_PIN3 8
#define STEPPER_PIN4 7

#define RED_LED 0x01  // mask used to select one pin of 22-25 for individual LED // 22
#define GREEN_LED 0x02 // 23  
#define BLUE_LED 0x04 // 24
#define YELLOW_LED 0x8 // 25


#define START_BUTTON 0x01
#define RESET_BUTTON 0x02
#define CONTROL_BUTTON 0x01


#define LCD_REGISTER_SELECT_PIN 12  // PIN 12
#define LCD_ENABLE_PIN 11 // PIN 11
#define DIGITAL_PIN_4 5 // PIN 5
#define DIGITAL_PIN_5 4 // PIN 4
#define DIGITAL_PIN_6 3 // PIN 3
#define DIGITAL_PIN_7 2 // PIN 2


volatile unsigned char *myPORTK = (unsigned char *) 0x108; // Button registers. PIN A8-A15 (analog pins by default, but we configure them as digital for buttons)
volatile unsigned char *myDDRK = (unsigned char *) 0x107; //
volatile unsigned char *myPINK = (unsigned char *) 0x106; //
volatile unsigned char *myDIDR2 = (unsigned char *) 0x7D; // Register 


volatile unsigned char* myPCMSK2 = (unsigned char*) 0x6D; // Registers related to interrupts
volatile unsigned char* myPCICR = (unsigned char*) 0x68;
volatile unsigned char* myPCIFR = (unsigned char*) 0x3B;


volatile unsigned char* myPORTA = (unsigned char*) 0x22; // LED registers. PINS 22-29
volatile unsigned char* myDDRA = (unsigned char*) 0x21; //
volatile unsigned char* myPINA = (unsigned char*) 0x20; //


volatile unsigned char* myADMUX = (unsigned char*) 0x7C; // ADC registers
volatile unsigned char* myADCSRB = (unsigned char*) 0x7B; //
volatile unsigned char* myADCSRA = (unsigned char*) 0x7A; //
volatile unsigned int* ADC_DATA = (unsigned int*) 0x78; //


volatile unsigned char* myUCSR0A = (unsigned char *)0x00C0; // UART registers
volatile unsigned char* myUCSR0B = (unsigned char *)0x00C1; //
volatile unsigned char* myUCSR0C = (unsigned char *)0x00C2; //
volatile unsigned int* myUBRR0  = (unsigned int *) 0x00C4; //
volatile unsigned char* myUDR0   = (unsigned char *)0x00C6; //


RTC_DS1307 rtc;

Stepper step(2048, STEPPER_PIN1, STEPPER_PIN2, STEPPER_PIN3, STEPPER_PIN4);

DHT dht (DHT_PIN, DHT_TYPE);
bool DisplayingTemperatureAndHumidity = false;
bool MonitoringDHT = true;
float Temperature = 0;
float Humidity = 0;

unsigned int WaterLevel = 0;

enum STATE {DISABLED, IDLE, ERROR, RUNNING, START};
enum STATE CurrentState = DISABLED;
enum STATE PreviousState;
bool OnOrOffButon = true;

char BufferLCD[16];

LiquidCrystal lcd(LCD_REGISTER_SELECT_PIN, LCD_ENABLE_PIN, DIGITAL_PIN_4, DIGITAL_PIN_5, DIGITAL_PIN_6, DIGITAL_PIN_7);
bool FanAllowed = false;

void setup() {
  InitializeUART(19200); // initialize the UART to a 19200 baud rate
  //Serial.begin(9600);
  InitializeADC(); // initialize analog to digital converter
  ConfigureRegisters();
  dht.begin();
  rtc.begin();  // initialize communication with the RTC module
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // set RTC date and time to when the sketch was compiled
  lcd.begin(16, 2);
  lcd.clear(); // clear the LCD
  step.setSpeed(2);
  // sei();
}


void loop() {
  DateTime Time = rtc.now();  // get current time
  WaterLevel = ADC_Read(WATER_LEVEL_CHANNEL);
  LoadBuffer();
  lcd.setCursor(0,1);
  if (MonitoringDHT) {lcd.print(BufferLCD);}
  else if (CurrentState == ERROR) {
    lcd.print("WATER LOW!");
  }
  SetLED(CurrentState);

  

  switch (CurrentState) {
    case DISABLED:
      MonitoringDHT = false;
      FanAllowed = false;
      break;
    case ERROR:
      FanAllowed = false;
      MonitoringDHT = false;
      break;
    case RUNNING:
      MonitoringDHT = true;
      FanAllowed = true;
       if ((unsigned int)dht.readTemperature(true) < TEMPERATURE_THRESHOLD) {
        CurrentState = IDLE;
        lcd.clear();
      }
      
       if ((unsigned int)ADC_Read(WATER_LEVEL_CHANNEL) < WATER_LEVEL_THRESHOLD) {
        CurrentState = ERROR;
        lcd.clear();
        lcd.print("WATER LOW");
        SetLED(CurrentState); //
        // create an error message for LCD
      }
      break;
    case IDLE:
      FanAllowed = false;
      MonitoringDHT = true;
      if ((unsigned int)dht.readTemperature(true) >= TEMPERATURE_THRESHOLD) {
        CurrentState = RUNNING;
      }
      if ((unsigned int)ADC_Read(WATER_LEVEL_CHANNEL) < WATER_LEVEL_THRESHOLD) {
        lcd.clear();
        lcd.print("WATER LOW");
        CurrentState = ERROR;
      }
      break;
  }
  SetFanMotor(FanAllowed);

  if (PINL & CONTROL_BUTTON) {  // if the control button is pressed move the vent
    step.step(1);
    unsigned char STEP[100];
    snprintf(STEP, 50, "Vent position moved Time: %02d:%02d:%02d Date: %d/%d/%d\n", Time.hour(), Time.minute(), Time.second(), Time.month(), Time.day(), Time.year());
    PutStringUART(STEP, strlen(STEP));
  }

  if (PreviousState != CurrentState) {
    unsigned char BufferUART[100];
    snprintf(BufferUART, 100, "STATE TRANSITION: %s to %s\n", GetStateName(PreviousState), GetStateName(CurrentState));
    PutStringUART(BufferUART, strlen(BufferUART));
    snprintf(BufferUART, 100, "Time: %02d:%02d:%02d Date: %d/%d/%d\n", Time.hour(), Time.minute(), Time.second(), Time.month(), Time.day(), Time.year());
    PutStringUART(BufferUART, strlen(BufferUART));
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(GetStateName(CurrentState));
  }

PreviousState = CurrentState;

}


void SetFanMotor(bool FanAllowed) { // PIN 30 PC7 
  
  if (FanAllowed) {
    PORTC |= 0b10000000;
  }
  else {
    PORTC &= 0b01111111;
  }
}

void SetLED(STATE CurrentState) {  // use mask definitions to turn on one of the LEDs.
  if (CurrentState == RUNNING) {
    *myPORTA = BLUE_LED;
    return;
  }
  if (CurrentState == IDLE) {
    *myPORTA = GREEN_LED;
    return;
  }
  if (CurrentState == DISABLED) {
    *myPORTA = YELLOW_LED;
    return;
  }
  if (CurrentState == ERROR) {
    *myPORTA = RED_LED;
  }
}


ISR(PCINT2_vect) {
  if (*myPINK & RESET_BUTTON) {
    if (CurrentState == ERROR) {
      CurrentState = IDLE;
    }
  }
  else if (*myPINK & START_BUTTON) {
    if (CurrentState == RUNNING || CurrentState == ERROR || CurrentState == IDLE) {
      CurrentState = DISABLED;
    }
    else if (CurrentState == DISABLED) {
      CurrentState = IDLE;
    }
  }
  // else if (*myPINK & CONTROL_BUTTON) {
  //   step.step(1);
  // }
}

void LoadBuffer() {
  snprintf(BufferLCD, 16, "H:%d T:%dF", (int)dht.readHumidity(), (int)dht.readTemperature(true)); 
} 

void ConfigureRegisters() {
  *myDDRA |= 0b00001111;  // set pins 22-25 for output. these pins are used for setting one of the signal LEDs.
  *myPORTK &= 0b11110000; // set external pull up resistor for start and reset buttons 

  DDRC |= 0b10000000;
  DDRL &= 0b11111110;
  PORTL &= 0b11111110;

  *myDIDR2 |= 0b00000011; // disable analog functionality of pins A8, A9, A10, A11
  *myPCICR |= 0b00000100; // enable interrupts on port K;
  *myPCMSK2 |= 0b00000011;  // enable interrupts on pins A8 and A9, A10
  *myDDRK &= ~(0b00000011); // configure pins A8, A9, A10, A11 as input 
}

const char* GetStateName(STATE state) {
    switch (state) {
        case DISABLED: return "DISABLED";
        case ERROR: return "ERROR";
        case RUNNING: return "RUNNING";
        case IDLE: return "IDLE";
        default: return "UNKNOWN";
    }
}


void InitializeADC() {
  // setup the A register
  *myADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *myADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *myADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *myADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *myADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *myADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *myADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *myADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *myADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *myADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}


unsigned int ADC_Read(unsigned char ADC_ChannelNumber) {    // USE WATER WATER_LEVEL_CHANNEL
  // clear the channel selection bits (MUX 4:0)
  *myADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *myADCSRB &= 0b11110111;
  // set the channel number
  if(ADC_ChannelNumber > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    ADC_ChannelNumber -= 8;
    // set MUX bit 5
    *myADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *myADMUX  += ADC_ChannelNumber;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *myADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*myADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *ADC_DATA;
}


void InitializeUART(int BaudRate) { //
  unsigned long ProcessorFrequency = 16000000;
  unsigned int BaudRateRegisterValue = (ProcessorFrequency / 16 / BaudRate - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = BaudRateRegisterValue;
}


unsigned char CheckDataBufferUART() {
  return *myUCSR0A & RECEIVER_DATA_AVAILABLE;
}


unsigned char GetCharacterUART() {
  return *myUDR0;
}


void PutStringUART(unsigned char* String, int Length) {
  while ((*myUCSR0A & TRANSMIT_BUFFER_EMPTY)==0);
  for (int i = 0; i < Length && String[i] != '\0'; i++) {
    while ((*myUCSR0A & TRANSMIT_BUFFER_EMPTY)==0);
    *myUDR0 = String[i];
  }
}
