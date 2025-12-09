#if defined(USE_LATCHING)

#if defined(USE_DIMMER) || defined(PCB_DIMMER)
#define acOutPin 3
#define drOutPin 9
#define dsOutPin 10
#define powerLEDPin 5       //0C1B
#define batteryLEDPin 6      //OC1A
#if defined(USE_DIMMER)
#define  illuminationPin 2
#endif
#define powerSWPin 7
#define BatterySWPin 4
#if defined(DBG_SERIAL)
SoftwareSerial mySerial(11,8);
#endif
#else
#define acOutPin 8
#define drOutPin 9
#define dsOutPin 10
#define powerLEDPin 2
#define batteryLEDPin 3
#define powerSWPin 5
#define BatterySWPin 4
#if defined(DBG_SERIAL)
SoftwareSerial mySerial(11,6);
#endif
#endif


#else
#define powerSWPin 4
#define BattrySWPin 5
#define powerLEDPin 2
#define batteryLEDPin 3
#if defined(DBG_SERIAL)
SoftwareSerial mySerial(3,6);
#endif
#endif

#define batADpin A0
#define accInPin 1
