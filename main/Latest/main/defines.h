#ifndef defines_h
#define defines_h

#if ( defined(ARDUINO_RASPBERRY_PI_PICO_W) )
  #if defined(WEBSOCKETS_USE_RP2040W)
    #undef WEBSOCKETS_USE_RP2040W
  #endif
  #define WEBSOCKETS_USE_RP2040W            true
  #define USE_RP2040W_WIFI                  true
  #define USE_WIFI_NINA                     false
#else
  #error This code is intended to run only on the RP2040W boards ! Please check your Tools->Board setting.
#endif

#define DEBUG_WEBSOCKETS_PORT     Serial
// Debug Level from 0 to 4
#define _WEBSOCKETS_LOGLEVEL_     3

#endif      //defines_h