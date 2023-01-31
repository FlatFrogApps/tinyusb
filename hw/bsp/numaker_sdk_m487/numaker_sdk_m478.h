#ifndef NUMAKER_SDK_M478_H
#define NUMAKER_SDK_M478_H

#include "NuMicro.h"

// Core clock from PLL
#define CCLOCKSPEED   FREQ_192MHZ

#define FSUSB_CLK_FREQ  48000000ul
#define FSUSB_CLK_DIV   (CCLOCKSPEED / FSUSB_CLK_FREQ)

#endif // NUMAKER_SDK_M478_H
