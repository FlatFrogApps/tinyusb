/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include <stdint.h>
#include <stdbool.h>

#include "bsp/board.h"
#include "numaker_sdk_m478.h"
#include "nuv_usbd.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
#define LED_PORT     PC
#define LED_PIN      3
#define LED_STATE_ON 0

/**
 * @brief       Routine to send a char
 * @param[in]   ch Character to send to debug port.
 * @returns     Send value from UART debug port
 * @details     Send a target char to UART debug port .
 */
static void SendChar_ToUART(uint8_t ch)
{
    while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);

    UART0->DAT = ch;
    if(ch == '\n')
    {
        while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);
        UART0->DAT = '\r';
    }
}

#if 0
static void PutString(char *str)
{
    while (*str != '\0')
    {
        SendChar_ToUART(*str++);
    }
}
#endif


/*  Must be called with protected registers unlocked */
// TODO: Revise this
static void setup_usb_fs(void)
{
    /* Enable USB IP clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select USBD                 00 -> USB device         PHY enable             SB0 must be 1*/
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_USBEN_Msk | SYS_USBPHY_SBO_Msk;

    /* Select USB IP clock source - Use PLL */
    CLK->CLKSEL0 |= CLK_CLKSEL0_USBSEL_Msk;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_USBDIV_Msk) | CLK_CLKDIV0_USB(FSUSB_CLK_DIV);

    /* BYTE-mode, USBEN, PHYEN, pullup on D+ disconnected */
    USBD->ATTR = USBD_ATTR_BYTEM_Msk |  USBD_ATTR_USBEN_Msk | USBD_ATTR_PHYEN_Msk;

    /* Force SE0, and then clear it to connect*/
    USBD_SET_SE0();
}

/*  Must be called with protected registers unlocked */
static void setup_usb_hs(void)
{
    uint32_t volatile i;

    /* Enable IP clock */
    CLK_EnableModuleClock(HSUSBD_MODULE);

    SYS->USBPHY &= ~SYS_USBPHY_HSUSBROLE_Msk;    /* select HSUSBD */

    /* Enable USB PHY */
    SYS->USBPHY = (SYS->USBPHY & ~(SYS_USBPHY_HSUSBROLE_Msk | SYS_USBPHY_HSUSBACT_Msk)) | SYS_USBPHY_HSUSBEN_Msk;
    for (i=0; i<0x1000; i++);      // delay > 10 us
    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

    /* Initial USB engine */
    HSUSBD_ENABLE_PHY();

    /* wait PHY clock ready */
    while (1)
    {
        HSUSBD->EP[EPA].EPMPS = 0x20ul;
        if (HSUSBD->EP[EPA].EPMPS == 0x20ul) {
            HSUSBD->EP[EPA].EPMPS = 0x0ul;
            break;
        }
    }
    HSUSBD->OPER &= ~HSUSBD_OPER_HISPDEN_Msk;   /* full-speed */
}

void board_init(void)
{


    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT,CLK_CLKDIV0_HCLK(1));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(CCLOCKSPEED);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);  // Timer0

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF6MFP_Msk|SYS_GPF_MFPL_PF7MFP_Msk)) |
                    (SYS_GPF_MFPL_PF6MFP_I2S0_LRCK|SYS_GPF_MFPL_PF7MFP_I2S0_DO);
    SYS->GPF_MFPH = (SYS->GPF_MFPH & ~(SYS_GPF_MFPH_PF8MFP_Msk|SYS_GPF_MFPH_PF9MFP_Msk|SYS_GPF_MFPH_PF10MFP_Msk)) |
                    (SYS_GPF_MFPH_PF8MFP_I2S0_DI|SYS_GPF_MFPH_PF9MFP_I2S0_MCLK|SYS_GPF_MFPH_PF10MFP_I2S0_BCLK );

    PF->SMTEN |= GPIO_SMTEN_SMTEN10_Msk;
    PD->SMTEN |= GPIO_SMTEN_SMTEN1_Msk;

#if CFG_TUSB_OS  == OPT_OS_NONE
    // 1ms tick timer
    SysTick_Config(96000000 / 1000);
#endif

    // Setup USB
    setup_usb_fs();
    setup_usb_hs();

    GPIO_SetMode(PH, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PH, BIT1, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PH, BIT2, GPIO_MODE_OUTPUT);
    // LEDs are active-low
    PH0 = 1;  // LED_R
    PH1 = 1;  // LED_Y
    PH2 = 0;  // LED_G

    UART_Open(UART0, 115200);
    PH1 = 1;  // LED_Y
    SYS_LockReg();
    puts("\nBoard initialized\n");

}

#if CFG_TUSB_OS  == OPT_OS_NONE
volatile uint32_t system_ticks = 0;
void SysTick_Handler (void)
{
    PH2 ^= 1;
    system_ticks++;
}

uint32_t board_millis(void)
{
  return system_ticks;
}
#endif


//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
    (void) state;
    PH0 = state;

#if 0
    PH1 = state;
    PH2 = !state;
#endif
}

uint32_t board_button_read(void)
{
  return 0;
}

int board_uart_read(uint8_t* buf, int len)
{
  (void) buf; (void) len;
  return 0;
}

int board_uart_write(void const * buf, int len)
{
    const char* ibuf = buf;
    for(int i=0; i<len; i++)
    {
        SendChar_ToUART(ibuf[i]);
    }

    return 0;
}
