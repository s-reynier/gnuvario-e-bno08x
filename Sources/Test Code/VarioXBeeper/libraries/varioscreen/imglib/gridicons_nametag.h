#if defined(ESP8266) || defined(ESP32)
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif
// 24 x 24 gridicons_nametag
const unsigned char gridicons_nametag[] PROGMEM = { /* 0X01,0X01,0XB4,0X00,0X40,0X00, */
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 
0x3C, 0xFF, 0xE0, 0x24, 0x07, 0xC0, 0x24, 0x03, 
0xC0, 0x3C, 0x03, 0xC0, 0x3C, 0x03, 0xCF, 0x00, 
0xF3, 0xCF, 0x81, 0xF3, 0xCF, 0xFF, 0xF3, 0xCF, 
0xFF, 0xF3, 0xCC, 0x00, 0x33, 0xCC, 0x00, 0x33, 
0xCC, 0x00, 0x33, 0xCF, 0xFF, 0xF3, 0xCF, 0xFF, 
0xF3, 0xC0, 0x00, 0x03, 0xE0, 0x00, 0x07, 0xFF, 
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};