/*

  Samsung IR Mapper
  -----------------
  Remaps the Samsung remotes keys 1-4 to HDMI inputs 1-4
  by sending the appropriate IR code for HDMI 1-4 after
  receiving the code for keys 1-4.

  You'll need an IR receiver like VS1838B and an IR diode.
  The cpu will be in deep sleep most of the time and wake
  up by interrupt. It will run fine for months/years powered 
  by an 18650.

  Wiring/Pins:
    2: VS1838B Data
    3: IR Anode/+

  Boards/Tested with:
    - Arduino Nano
    - ProMini 8MHz
    - ProMini 16MHz (even with 3.3V)

*/

#include <Arduino.h>
#include <IRremote.h>
#include <LowPower.h>

#define NUM1 0xE0E020DF
#define NUM2 0xE0E0A05F
#define NUM3 0xE0E0609F
#define NUM4 0xE0E010EF
#define NUM5 0xE0E0906F
#define NUM6 0xE0E050AF
#define NUM7 0xE0E030CF
#define NUM8 0xE0E0B04F
#define NUM9 0xE0E0708F
#define NUM0 0xE0E08877

// When the cpu wakes up from deep sleep, the timing of the first 
// pulse is shorter, resulting in different codes:
#define WAKE_NUM1 0xE13DDA28
#define WAKE_NUM2 0xAD586662
#define WAKE_NUM3 0x273009C4
#define WAKE_NUM4 0xF5999288
#define WAKE_NUM5 0x731A3E02

#define HDMI1 0xE0E09768
#define HDMI2 0xE0E07D82
#define HDMI3 0xE0E043BC
#define HDMI4 0xE0E0A35C

#define TV   0xE0E0D827
#define PC   0xE0E09669
#define USB  0xE0E031CE
#define AV   0xE0E0D728
#define COMP 0xE0E061AE
#define EXT  0xE0E021DE

#define POWER 0xE0E019E6
#define MUTE 0xE0E0F00F

#define STB_MUTE 0x20CFEA15
#define STB_POW 0x20CFFA05
#define STB_UP 0x20CF18E7
#define STB_DOWN 0x20CF629D
#define STB_LEFT 0x20CF827D
#define STB_RIGHT 0x20CF926D
#define STB_OK 0x20CFA25D
#define STB_EXIT 0x20CFB847
#define STB_INFO 0x20CF609F
#define STB_REPEAT 0xFFFFFFFF

#define RECV_PIN 2 // pin 2
#define LOW_BATTERY 3000 // low battery threshold in mV 

IRrecv irrecv(RECV_PIN);
IRsend irsend; // Sends on pin 3
decode_results res,resvalid;
unsigned long lastReceived = 0;
unsigned long lastActive = 0;
unsigned int Vcc = 0;

void dumpRaw(decode_results *results);
void sleep();
long readVcc();

void setup() {
  Serial.begin(115200);
  irrecv.blink13(true);
  irrecv.enableIRIn(); 
}

void loop() {
  // Receive IR and keep last valid code (ignore repeat codes)
  if (irrecv.decode(&res)) {
    Serial.print("Code: ");
    Serial.println(res.value, HEX);
    dumpRaw(&res);
    switch (res.value) {
      case STB_REPEAT: 
        break; // Ignore REPEAT Code(s)
      default: 
        resvalid = res;
      break;
    }  
    irrecv.resume();
    lastReceived = millis();
    lastActive = millis();
  }
  // Wait for 300ms after last code was received, then send code
  if(lastReceived > 0 && lastReceived+300 < millis()) {
    lastReceived = 0;
    Vcc = readVcc();
    Serial.print("Vcc: ");
    Serial.print(Vcc);
    Serial.println(" mV");
    if(Vcc < LOW_BATTERY) {
      // MUTE 3 times to indicate low battery
      for(int i = 0; i<6; i++) {
        irsend.sendSAMSUNG(MUTE,32); 
        delay(300);
      }
    }
    Serial.print("Final: ");
    Serial.println(resvalid.value, HEX);
    switch (resvalid.value) {
      case STB_INFO:
      case STB_POW:
      case NUM1: 
      case WAKE_NUM1:
        irsend.sendSAMSUNG(HDMI1,32); 
        break;
      case NUM2: 
      case WAKE_NUM2:
        irsend.sendSAMSUNG(HDMI2,32); 
        break;
      case NUM3: 
      case WAKE_NUM3:
        irsend.sendSAMSUNG(HDMI3,32); 
        break;
      case NUM4: 
      case WAKE_NUM4:
        irsend.sendSAMSUNG(HDMI4,32); 
        break;
      break;
    }
    delay(100);
    irrecv.enableIRIn(); // required after sending
  }  
  // Reset lastActive on millis() overflow
  if(lastActive == 0 || lastActive > millis()) lastActive = millis();
  // Sleep if not active for 5s
  if(lastActive > 0 && lastActive+5e3 < millis() && millis()>5e3) {
    sleep();
    lastActive = millis();
  }
}



void dumpRaw(decode_results *results) {
  // Print Raw data
  Serial.print("Type:");
  Serial.println(results->decode_type);
  Serial.print("Timing[");
  Serial.print(results->rawlen-1, DEC);
  Serial.println("]: ");

  for (int i = 1;  i < results->rawlen;  i++) {
    unsigned long  x = results->rawbuf[i] * USECPERTICK;
    if (!(i & 1)) {  // even
      Serial.print("-");
      if (x < 1000)  Serial.print(" ") ;
      if (x < 100)   Serial.print(" ") ;
      Serial.print(x, DEC);
    } else {  // odd
      Serial.print("     ");
      Serial.print("+");
      if (x < 1000)  Serial.print(" ") ;
      if (x < 100)   Serial.print(" ") ;
      Serial.print(x, DEC);
      if (i < results->rawlen-1) Serial.print(", "); //',' not needed for last one
    }
    if (!(i % 8))  Serial.println("");
  }
  Serial.println("");                    // Newline
}

void sleep() {
  Serial.println("Sleep...");
  delay(20);
  PCICR |= 1 << PCIE2;
  PCMSK2 |= (1 << PCINT18);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
  PCICR ^= 1 << PCIE2;
  PCMSK2 ^= (1 << PCINT18);
}

ISR(PCINT2_vect)
{
} 


long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif   
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both 
  long result = (high<<8) | low; 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}