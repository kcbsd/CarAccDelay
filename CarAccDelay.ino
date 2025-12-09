#define USE_DIMMER
//#define PCB_DIMMER
//#define DBG_SERIAL
#define USE_LATCHING
//#define TEST_WIRING
//#define TEST_LATCHING
#if defined(DBG_SERIAL)
#include <SoftwareSerial.h>
#define DBG_MSG(m) mySerial.print(m)
#else
#define DBG_MSG(m)
#endif

#include "CarAccDelay.h"

#define PS_OFF 0
#define PS_NORM 1
#define PS_DELAY_OFF 2
#define PS_WAIT_OFF 3
#define PS_FORCE_ON 4
#define PS_FORCE_OFF 5

#include "avr/sleep.h"
#include "avr/wdt.h"

int pState = PS_OFF;

int ticks = 0;
int acc_cnt = 0;
bool acc_out = false;
bool drc_out = 0;
bool batSW = false;
bool batLast = false;
int powSWcnt = 0;
bool batLED = false;
bool powLED = false;
int batVol;
#ifdef USE_DIMMER
bool last_illumination = false;
#endif
#ifdef USE_LATCHING
int latchST = 0;
#endif
// sleep time definitions for Watch-dog-timer

#define sT16 B00000000	 // 16m second
#define sT32 B00000001	 // 32m second
#define sT64 B00000010	 // 64m second
#define sT125 B00000011  // 125m seconds
#define sT250 B00000100  // 250m seconds
#define st500 B00000101  // 500m seconds
#define sT1 B00000110 	 // 1 second
#define sT2 B00000111 	 // 2 seconds
#define sT4 B00100000 	 // 4 seconds
#define sT8 B00100001 	 // 8 seconds

#define sleepTime sT125 		// select sleep-time from above 1/8Sec
#define delayTicks 120			// 8*15 15Sec
const uint16_t VolL = 921;	// 12.5V 実測値
#define DIMMER_DUTY			180


#if defined(USE_LATCHING)
void LRelay(bool on) {
	if (on) {
		if (latchST < -1) {
			digitalWrite(drOutPin, LOW);
		}
		digitalWrite(dsOutPin, HIGH);
		latchST = 3;
	} else {
		if (latchST > 1) {
			digitalWrite(dsOutPin, LOW);
		}
		digitalWrite(drOutPin, HIGH);
		latchST = -3;
	}
}
#endif()
void powled(bool on,bool init) {
	powLED = on;
#if defined(USE_DIMMER)
	if (on) {
			if (init) {
				TCCR1A = 0xf2;		//高速PWM 一致でHigh
				TCCR1B = 0x1b;		//64 分周
				ICR1	= 255;
				DBG_MSG("setup PWM power\r\n");
				OCR1A = 256;
			}
			OCR1B = (last_illumination || acc_cnt < delayTicks )? DIMMER_DUTY : 0;
	} else {
			if (init) {
				TCCR1A = 0x02;
				TCCR1B = 0x18;
				digitalWrite(powerLEDPin, powLED);
				DBG_MSG("cleanup PWM power\r\n");
				digitalWrite(powerLEDPin, powLED);
			} else
				OCR1B = 256;
	}
#else
	digitalWrite(powerLEDPin, powLED);
#endif
}
void batled(bool on,bool init) {
	batLED = on;
#if defined(USE_DIMMER)
	if (on) {
			if (init) {
				TCCR1A = 0xf2;		//高速PWM 一致でHigh
				TCCR1B = 0x1b;		//64 分周
				ICR1	= 255;
				DBG_MSG("setup PWM battery\r\n");
				OCR1B = 256;
			}
			OCR1A = (last_illumination || acc_cnt < delayTicks )? DIMMER_DUTY : 0;
	} else {
			if (init) {
				TCCR1A = 0x02;
				TCCR1B = 0x18;
				digitalWrite(batteryLEDPin, batLED);
				DBG_MSG("cleanup PWM battery\r\n");
				digitalWrite(batteryLEDPin, batLED);
			} else
				OCR1A = 256;
	}
#else
	digitalWrite(batteryLEDPin, batLED);
#endif
}
void setup() {
	// put your setup code here, to run once:
	byte u = MCUSR;
	cli();
	MCUSR = 0;
	WDTCSR = _BV(WDCE) | _BV(WDIF);
	WDTCSR = _BV(WDIE) | sT125;
#if defined(USE_DIMMER)
	pinMode(illuminationPin, INPUT);
#ifdef TEST_WIRING
					powled(true,true);
					batled(true,true);
#endif
#endif
	pinMode(acOutPin, OUTPUT);
	pinMode(drOutPin, OUTPUT);
#ifdef USE_LATCHING
	pinMode(dsOutPin, OUTPUT);
#endif
	pinMode(powerLEDPin, OUTPUT);
	pinMode(batteryLEDPin, OUTPUT);
	pinMode(accInPin, INPUT);
	pinMode(powerSWPin, INPUT_PULLUP);
	pinMode(BatterySWPin, INPUT_PULLUP);
	ADMUX = 0b10000000;  // VREF設定 Internal 1.1V
	ADCSRB = 0b00000000;
	sei();
#if defined(DBG_SERIAL)
	mySerial.begin(19200);
	if (u & PORF)
		DBG_MSG("Power on\r\n");
	if (u & WDRF)
		DBG_MSG("WDT RESET\r\n");
#if defined(TEST_LATCHING)
	DBG_MSG("TEST LATCHING\r\n");
#endif
#if defined(TEST_WIRING)
	DBG_MSG("TEST WERING\r\n");
	powled(true,false);
	batled(false,false);
#endif
#endif
#if defined(USE_LATCHING)
	LRelay(false);
	DBG_MSG("latchST:");
	DBG_MSG(latchST);
	DBG_MSG("\r\n");
#endif
}


void loop() {
#if defined(TEST_LATCHING)
	idleWait();	// sleep + ADCpower-off
	adcSleep(false);
	digitalWrite(drOutPin, !digitalRead(powerSWPin));
	digitalWrite(dsOutPin, !digitalRead(BatterySWPin));
	digitalWrite(powerLEDPin, !digitalRead(powerSWPin));
	digitalWrite(batteryLEDPin, !digitalRead(BatterySWPin));
#endif
#if defined(TEST_WIRING)
	idleWait();	// sleep + ADCpower-off
	adcSleep(false);
	#ifdef USE_DIMMER
	if (last_illumination != digitalRead(illuminationPin)) {
		last_illumination = !last_illumination;
				DBG_MSG(last_illumination?"dimmer on\r\m":"dimmer off\r\n");
				if(powLED)
					powled(powLED,false);
				if(batLED)
					powled(batLED,false);
	}
#endif
	if ((ticks & 7) == 0) {
		DBG_MSG(ticks);
		DBG_MSG("<=ticks ");
		DBG_MSG(batVol);
		DBG_MSG(digitalRead(accInPin) ? " A" : " a");
	#ifdef USE_DIMMER
		DBG_MSG(digitalRead(illuminationPin) ? "D" : "d");
#endif
		DBG_MSG(digitalRead(powerSWPin) ? "P":"p");
		DBG_MSG(digitalRead(BatterySWPin) ? "B\r\n" : "b\r\n");
	}
	if ((ticks & 31) == 0) {
#if defined(USE_LATCHING)
		switch ((ticks / 32) % 5) {
			case 4:
				digitalWrite(dsOutPin, LOW);
				digitalWrite(drOutPin, HIGH);
				DBG_MSG("drOUT\r\n");
				break;
#else
		switch ((ticks / 32) % 4) {
#endif
			default:
				digitalWrite(drOutPin, LOW);
				digitalWrite(acOutPin, HIGH);
				DBG_MSG("acOUT\r\n");
				break;
			case 1:
				digitalWrite(acOutPin, LOW);
#ifdef USE_DIMMER
				powled(true,false);
#else
				digitalWrite(powerLEDPin, HIGH);
#endif
				DBG_MSG("power LED\r\n");
				break;
			case 2:
#ifdef USE_DIMMER
				powled(false,false);
				batled(true,false);
#else
				digitalWrite(powerLEDPin, LOW);
				digitalWrite(batteryLEDPin, HIGH);
#endif
				DBG_MSG("battery LED\r\n");
				break;
			case 3:
#ifdef USE_DIMMER
				batled(false,false);
#else
				digitalWrite(batteryLEDPin, LOW);
#endif
#if defined(USE_LATCHING)
				DBG_MSG("dsOUT\r\n");
				digitalWrite(dsOutPin, HIGH);
#else
				DBG_MSG("drOUT\r\n");
				digitalWrite(drOutPin, HIGH);
#endif
				break;
		}
	}
#endif
#if !defined(TEST_LATCHING) && !defined(TEST_WIRING)
	if(pState==PS_OFF && !drc_out) {
		deepSleep();	// sleep + ADCpower-off
		adcSleep(true);
	} else {
		idleWait();
		adcSleep(false);
	}
	DBG_MSG(pState);
	DBG_MSG(":pState ");
	DBG_MSG(batVol);
	DBG_MSG(":batVol\r\n");
	switch (pState) {
		default:	// off force on
			if (digitalRead(accInPin)) {
				acc_cnt = delayTicks;
				if (!acc_out) {
					digitalWrite(acOutPin, HIGH);
					acc_out = true;
					DBG_MSG("AccIn On\r\n");
					if(!batSW) {
						powled(true,true);
					} else {
						batSW=false;
						powled(true,false);
					}
					batled(false,false);
				}
				pState = PS_NORM;
			}
			break;
		case PS_FORCE_OFF:
			if (!digitalRead(accInPin)) {
				pState = PS_OFF;
					if(!batSW)
						powled(false,true);

			}
			break;
		case PS_DELAY_OFF:
			if (digitalRead(accInPin)) {
				pState = PS_NORM;
				acc_cnt = delayTicks;
				powled(powLED,false);
			} else {
				if (--acc_cnt == 0) {
					pState = PS_OFF;
					digitalWrite(acOutPin, LOW);
					acc_out = false;
					if(!batSW)
						powled(false,true);
				}
			}
			break;
		case PS_WAIT_OFF:
		case PS_NORM:
			if (digitalRead(accInPin)) {
				acc_cnt = delayTicks;
			} else {
				if (pState == PS_NORM) {
					pState = PS_DELAY_OFF;
				} else {
					pState = PS_FORCE_ON;
					acc_cnt = -1;
				}
			}
			break;
	}
#ifdef USE_DIMMER
	if (last_illumination != digitalRead(illuminationPin)) {
		last_illumination = !last_illumination;
		DBG_MSG(last_illumination?"dimmer on\r\nm":"dimmer off\r\n");
		if (powLED)
			powled(powLED,false);
		if (batLED)
			batled(batLED,false);
	}
#endif
	if (!digitalRead(BatterySWPin)) {
		if (!batLast) {
			batLast = true;
			batSW = !batSW;
			DBG_MSG(batSW ? "batSW On\r\n" : "batSW off\r\n");
		}
	} else
		batLast = false;
	if (!digitalRead(powerSWPin)) {
		++powSWcnt;
		DBG_MSG(powSWcnt);
		DBG_MSG(":powSWcnt\r\n");
	} else if (powSWcnt > 0) {
		if (pState == PS_DELAY_OFF) {
			acc_cnt = 0;
			acc_out = false;
			digitalWrite(acOutPin, LOW);
			if(!batSW)
				powled(false,true);
			pState = PS_OFF;
		} else if (pState == PS_FORCE_OFF) {
			pState = PS_NORM;
			digitalWrite(acOutPin, HIGH);
			acc_out = true;
		} else if (pState == PS_WAIT_OFF) {
			pState = PS_NORM;
			powled(powLED,false);
		} else if (pState == PS_FORCE_ON) {
			digitalWrite(acOutPin, LOW);
			acc_out = false;
			pState = PS_OFF;
			if(!batSW)
				powled(false,true);
		} else if (powSWcnt > 6) {
			if (pState == PS_NORM || pState == PS_FORCE_ON) {
				if (acc_out) {
					digitalWrite(acOutPin, LOW);
					acc_out = false;
				}
				DBG_MSG("Acc Force Off\r\n");
				acc_cnt = -1;
				pState = PS_FORCE_OFF;
			} else if (pState == PS_FORCE_OFF) {
				if (!acc_out) {
					digitalWrite(acOutPin, HIGH);
					acc_out = true;
				}
				acc_cnt = -1;
				pState = PS_FORCE_ON;
				DBG_MSG("Acc Force On\r\n");
			} else if (pState == PS_OFF && batVol >= VolL) {
				if (!acc_out) {
					digitalWrite(acOutPin, HIGH);
					acc_out = true;
				}
				if(!batSW)
					powled(true,true);
				acc_cnt = -1;
				pState = PS_FORCE_ON;
				DBG_MSG("Acc Force On\r\n");
			}
		} else {
			if (pState == PS_NORM) {
				pState = PS_WAIT_OFF;
			}
		}
		powSWcnt = 0;
	}
	if (batVol < VolL) {
		batSW = false;
		if (pState == PS_FORCE_ON || (pState== PS_OFF && drc_out) ) {
			if (acc_out) {
				acc_out = false;
				digitalWrite(acOutPin, LOW);
			}
			if (drc_out) {
				drc_out = false;
				LRelay(true);

			}
			powled(false,true);
			batled(false,true);
			pState = PS_OFF;
			acc_cnt = 0;
		}
	}
	if (batSW) {
		if (!drc_out) {
			drc_out = 1;
			batled(true,true);
#if defined(USE_LATCHING)
			LRelay(true);
#else
			digitalWrite(drOutPin, HIGH);
#endif
		}
	} else {
		if (acc_out) {
			if (!drc_out) {
				drc_out = 1;
#if defined(USE_LATCHING)
				LRelay(true);
#else
				digitalWrite(drOutPin, HIGH);
#endif
			}
		} else if (drc_out) {
			drc_out = 0;
			batled(false,true);
#if defined(USE_LATCHING)
			LRelay(false);
#else
			digitalWrite(drOutPin, LOW);
#endif
		}
	}
#endif
}

ISR(ADC_vect)  // *** Intrpt svc rtn for	ISR(vect) *****
{
	int u;
	u = batVol * 9 + ADC;
	batVol = u / 10;
}

ISR(WDT_vect) {  // *** Intrpt svc rtn for WDT ISR(vect) *****
	++ticks;
#if !defined(TEST_WIRING)
#if defined(USE_LATCHING)
	if (latchST > 1) {
		latchST--;
		if (latchST == 1) {
			digitalWrite(dsOutPin, LOW);
			DBG_MSG("LATCH RESET:1\r\n");
		}
	}
	if (latchST < -1) {
		latchST++;
		if (latchST == -1) {
			digitalWrite(drOutPin, LOW);
			DBG_MSG("LATCH RESET:-1\r\n");
		}
	}
#endif
#endif
#if !defined(TEST_LATCHING) && !defined(TEST_WIRING)
	if (batSW) {
		if (!batLED && (ticks & 3) == 0) {
			batled(true,false);
		} else if (batLED && ((acc_out && (ticks & 3) == 2) || (!acc_out && (ticks & 3) == 1))) {
			batled(false,false);
		}
	} else if (batLED) {
		batled(false,false);
	}
	switch (pState) {
		default:
			powled(false,false);
			break;
		case PS_NORM:
			if(!powLED)
				powled(true,false);
			break;
		case PS_DELAY_OFF:
			if ((acc_cnt & 7) == 0) {
				powled(true,false);
			} else if ((acc_cnt & 7) == 4) {
				powled(false,false);
			}
			break;
		case PS_WAIT_OFF:
			if ((ticks & 3) == 0) {
				powled(true,false);
			} else if ((ticks & 3) == 2) {
				powled(false,false);
			}
			break;
		case PS_FORCE_ON:
			if ((ticks & 7) == 0) {
				powled(true,false);
			} else if ((ticks & 7) == 1) {
				powled(false,false);
			}
			break;
		case PS_FORCE_OFF:
			if ((ticks & 7) == 0) {
				powled(true,false);
			} else if ((ticks & 7) == 6) {
				powled(false,false);
			}
			break;
	}
#endif
}
void deepSleep(void) {
	ADCSRA = 0b00000111;	// disable ADC to save power
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	sleep_cpu();	// sleep until WDT-interrup
	sleep_disable();
	ADCSRA |= 0b10000111;  // enable ADC again
}

void adcSleep(bool use_adc_sleep) {
	ADCSRA = 0b11001111;	// 128kHz ADC Start ADC INT EN
	if (use_adc_sleep) {
		set_sleep_mode(SLEEP_MODE_ADC);
	} else {
		set_sleep_mode(SLEEP_MODE_IDLE);
	}
	sleep_enable();
	sleep_cpu();					// sleep until WDT-interrup
	ADCSRA = 0b00000111;	// 128kHz ADCDIS ADC INT DIS
	sleep_disable();
}
void idleWait(void){
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
	sleep_cpu();					// sleep until WDT-interrup
	sleep_disable();
}
