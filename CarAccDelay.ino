#define DBG_SERIAL

#ifdef DBG_SERIAL
#include <SoftwareSerial.h>
#endif

#include "CarAccDelay.h"



#include "avr/sleep.h"
#include "avr/wdt.h"

int acc_out = 0;
int drc_out = 0;
int batSW = 0;
int batST = 0;
int lowBat_cnt = 0;
int batVol;

// sleep time definitions for Watch-dog-timer

#define sT16	B00000000     // 16m second
#define sT32	B00000001     // 32m second
#define sT64	B00000010     // 64m second
#define sT125	B00000011     // 125m seconds
#define sT250	B00000100     // 250m seconds
#define st500	B00000101     // 500m seconds
#define sT1	B00000110     // 1 second
#define sT2	B00000111     // 2 seconds
#define sT4	B00100000     // 4 seconds
#define sT8	B00100001     // 8 seconds

#define sleepTime sT125		// select sleep-time from above 1/8Sec
#define	delayTicks	240	// 8*30 30Sec


void setup() {
  // put your setup code here, to run once:
  byte u= MCUSR;
  cli();
  MCUSR = 0;
  WDTCSR =_BV(WDCE)|_BV(WDIF);
  WDTCSR =_BV(WDIE)| sT125;
  
  sei();
  pinMode(acOutPin, OUTPUT);
  pinMode(drOutPin, OUTPUT);
  pinMode(CancelLEDPin, OUTPUT);
  pinMode(BattryLEDPin, OUTPUT);
  pinMode(accInPin, INPUT);
  pinMode(CancelSWPin, INPUT_PULLUP);
  pinMode(BattrySWPin, INPUT_PULLUP);
  ADMUX  = 0b10000000;    // VREF設定 Internal 1.1V
  ADCSRB = 0b00000000;
#ifdef DBG_SERIAL
  mySerial.begin(19200);
     if(u & PORF) 
      mySerial.print("Power on");
    if (u& WDRF)
      mySerial.print("WDT RESET");
#endif
}

const uint16_t L = (12.3 / (1.1 / (8.2 / (100 + 8.2)) / 1023));

void loop() {
  uint16_t u;
#ifndef DBG_SERIAL
  if (batST && !acc_out) {
    adcSleep();
    u = ADC;
    if (u < L ) {
      if ( ++lowBat_cnt > 10) {
        batST = 0;
      } else {
        lowBat_cnt = 0;
      }
    }
  }
#else
  deepSleep();            // sleep + ADCpower-off
  if ((acc_out&7)==0) {
  mySerial.print(digitalRead(accInPin)?"A":"a");
  mySerial.print(digitalRead(CancelSWPin)?"C":"c");
  mySerial.println(digitalRead(BattrySWPin)?"B":"b");
//  mySerial.println(drc_out);
  }
#if 1
adcSleep();
  u = ADC;
  mySerial.print(" Adc:");
  mySerial.println(batVol);
#endif
  if(drc_out&4) {
  switch (drc_out&3) {
    case 1:
         digitalWrite( BattryLEDPin, LOW );
          digitalWrite( drOutPin, HIGH );
          break;
    case 2:
         digitalWrite( drOutPin,LOW );
         digitalWrite( acOutPin, HIGH );
          break;
    case 3:
         digitalWrite( acOutPin, LOW );
         digitalWrite( CancelLEDPin, HIGH );
          break;
    case 0:
         digitalWrite( CancelLEDPin, LOW );
         digitalWrite( BattryLEDPin, HIGH );
        break;
   }
   }
#endif

}
ISR(ADC_vect)// *** Intrpt svc rtn for  ISR(vect) *****
{
  int u;
   u=batVol*9+ADC;
   batVol=u/10;
}

ISR(WDT_vect) { // *** Intrpt svc rtn for WDT ISR(vect) *****
#ifndef DBG_SERIAL
  if (!digitalRead(CancelSWPin)) {
    if ( acc_out ) {
      acc_out = 0;
      drc_out = 0;
      digitalWrite( acOutPin, LOW );
      digitalWrite( drOutPin, LOW );
      digitalWrite( CancelLEDPin, LOW );
      if (batST) {
        batST = 0;
        digitalWrite( BattryLEDPin, LOW );
      }
    }
  } else {
    if (digitalRead(accInPin)) {
      if (!acc_out) {
        if (!drc_out) {
          drc_out = 1;
          digitalWrite( drOutPin, LOW );
        }
        digitalWrite( acOutPin, HIGH );
        digitalWrite( CancelLEDPin, HIGH );

      }
      acc_out = delayTicks;
    } else {
      if (acc_out > 0) {
        acc_out--;
        if ((acc_out & 7) == 7) {
          digitalWrite( CancelLEDPin, HIGH );
        }
        if ((acc_out & 7) == 4) {
          digitalWrite( CancelLEDPin, LOW );
        }
        if (acc_out == 0) {
          digitalWrite( acOutPin, LOW );
        }
      }
    }
    if (!digitalRead(BattrySWPin)) {
      if (!batSW) {
        batSW = 1;
        if (!batST) {
          batST = 1;
          digitalWrite( BattryLEDPin, HIGH );
        } else {
          digitalWrite( BattryLEDPin, LOW );
          batST = 0;
        }
      }
    } else {
      if (batSW) {
        batSW = 0;
      }
    }
    if (batST) {
      if (!drc_out) {
        drc_out = 1;
        digitalWrite( drOutPin, HIGH );
        digitalWrite( BattryLEDPin, HIGH );
      }
    } else {
      if (drc_out && !acc_out) {
        drc_out = 0;
        digitalWrite( drOutPin, LOW );
      }
    }
  }
#else
  acc_out++;
  if ((acc_out&7)==7) {
    drc_out++;
  }
#endif
}
void deepSleep(void) {
  ADCSRA = 0b00000111;    // disable ADC to save power
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();            // sleep until WDT-interrup
  sleep_disable();
  ADCSRA |= 0b10000111;    // enable ADC again
}

void adcSleep(void) {
  ADCSRA = 0b11001111;  // 128kHz ADC Start ADC INT EN
  set_sleep_mode(SLEEP_MODE_ADC);
  sleep_enable();
  sleep_cpu();            // sleep until WDT-interrup
  ADCSRA = 0b00000111;  // 128kHz ADCDIS ADC INT DIS
  sleep_disable();
}
