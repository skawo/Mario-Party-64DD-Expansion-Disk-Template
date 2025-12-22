#ifndef DD_RTC_H
#define DD_RTC_H

#include <stdint.h>

//64DD RTC
extern unsigned char year;
extern unsigned char month;
extern unsigned char day;
extern unsigned char hour;
extern unsigned char min;
extern unsigned char sec;

//RTC function
void getRTC_64dd(void);

#endif