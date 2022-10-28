// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2021, Alex Taradov <alex@taradov.com>. All rights reserved.

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "rp2040.h"
#include "hal_gpio.h"

//-----------------------------------------------------------------------------
#define PERIOD_FAST    100000
#define PERIOD_SLOW    500000

#define F_REF          12000000
#define F_CPU          120000000
#define F_PER          120000000
#define F_RTC          (F_REF / 256)
#define F_TICK         1000000

HAL_GPIO_PIN(PINX,      0, 6, sio_6)
HAL_GPIO_PIN(LED,      0, 25, sio_25)
HAL_GPIO_PIN(BUTTON,   0, 18, sio_18)
HAL_GPIO_PIN(UART_TX,  0,  0, uart0_tx)
HAL_GPIO_PIN(UART_RX,  0,  1, uart0_rx)
HAL_GPIO_PIN(I2C0_SDA,  0,  4, i2c0_sda)
HAL_GPIO_PIN(I2C0_SCL,  0,  5, i2c0_scl)

//-----------------------------------------------------------------------------
static volatile uint32_t timer_period;

//-----------------------------------------------------------------------------
static void timer_set_period(int period)
{
  timer_period = period;
  TIMER->ALARM0 = TIMER->TIMELR + timer_period;
}

//-----------------------------------------------------------------------------
void irq_handler_timer_0(void)
{
  TIMER->INTR = TIMER_INTR_ALARM_0_Msk;
  TIMER->ALARM0 = TIMER->TIMELR + timer_period;
  HAL_GPIO_LED_toggle();
}

//-----------------------------------------------------------------------------
static void timer_init(void)
{
  RESETS_CLR->RESET = RESETS_RESET_timer_Msk;
  while (0 == RESETS->RESET_DONE_b.timer);

  timer_set_period(PERIOD_SLOW);

  TIMER->INTE = TIMER_INTE_ALARM_0_Msk;

  NVIC_EnableIRQ(TIMER_IRQ_0_IRQn);
}

//-----------------------------------------------------------------------------
static void uart_init(uint32_t baud)
{
  RESETS_CLR->RESET = RESETS_RESET_uart0_Msk;
  while (0 == RESETS->RESET_DONE_b.uart0);

  baud = (F_PER * 4) / baud;

  UART0->UARTIBRD = baud / 64;
  UART0->UARTFBRD = baud % 64;

  UART0->UARTLCR_H = (3/*8 bits*/ << UART0_UARTLCR_H_WLEN_Pos) | UART0_UARTLCR_H_FEN_Msk;
  UART0->UARTCR = UART0_UARTCR_UARTEN_Msk | UART0_UARTCR_RXE_Msk | UART0_UARTCR_TXE_Msk;

  HAL_GPIO_UART_TX_init();
  HAL_GPIO_UART_RX_init();
}

//-----------------------------------------------------------------------------
static void i2c0_init(void)
{
  RESETS_CLR->RESET = RESETS_RESET_i2c0_Msk;
  while (0 == RESETS->RESET_DONE_b.i2c0);
  
  RESETS_CLR->RESET = RESETS_RESET_i2c0_Msk;
  while (0 == RESETS->RESET_DONE_b.i2c0);

  I2C0->IC_ENABLE = 0;

  I2C0->IC_CON_b.MASTER_MODE = I2C0_IC_CON_MASTER_MODE_ENABLED;
  I2C0->IC_CON_b.SPEED = I2C0_IC_CON_SPEED_FAST;
  I2C0->IC_CON_b.IC_10BITADDR_SLAVE = I2C0_IC_CON_IC_10BITADDR_MASTER_ADDR_7BITS;
  I2C0->IC_CON_b.IC_10BITADDR_MASTER = I2C0_IC_CON_IC_10BITADDR_SLAVE_ADDR_7BITS;
  I2C0->IC_CON_b.IC_RESTART_EN = I2C0_IC_CON_IC_RESTART_EN_DISABLED;
  I2C0->IC_CON_b.IC_SLAVE_DISABLE = I2C0_IC_CON_IC_SLAVE_DISABLE_SLAVE_DISABLED;
  I2C0->IC_CON_b.STOP_DET_IFADDRESSED = I2C0_IC_CON_STOP_DET_IFADDRESSED_DISABLED;
  I2C0->IC_CON_b.TX_EMPTY_CTRL = I2C0_IC_CON_TX_EMPTY_CTRL_ENABLED;
  I2C0->IC_CON_b.RX_FIFO_FULL_HLD_CTRL = I2C0_IC_CON_RX_FIFO_FULL_HLD_CTRL_DISABLED;

  I2C0->IC_FS_SCL_HCNT = 9601;
  I2C0->IC_FS_SCL_LCNT = 14400;
  I2C0->IC_FS_SPKLEN = 900;
  I2C0->IC_SDA_HOLD = 37;

  I2C0->IC_ENABLE = 1;
  
  HAL_GPIO_I2C0_SDA_init();
  HAL_GPIO_I2C0_SCL_init();
}

//-----------------------------------------------------------------------------
static void uart_putc(char c)
{
  while (UART0->UARTFR_b.TXFF);
  UART0->UARTDR = c;
}

//-----------------------------------------------------------------------------
static bool uart_getc(char *c)
{
  if (0 == UART0->UARTFR_b.RXFE)
  {
    *c = UART0->UARTDR;
    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
static void uart_puts(char *s)
{
  while (*s)
    uart_putc(*s++);
}

//-----------------------------------------------------------------------------
static char invert_case(char c)
{
  if ('a' <= c && c <= 'z')
    return c + ('A' - 'a');
  else if ('A' <= c && c <= 'Z')
    return c - ('A' - 'a');
  return c;
}

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  // Enable XOSC
  XOSC->CTRL     = (XOSC_CTRL_FREQ_RANGE_1_15MHZ << XOSC_CTRL_FREQ_RANGE_Pos);
  XOSC->STARTUP  = 47; // ~1 ms @ 12 MHz
  XOSC_SET->CTRL = (XOSC_CTRL_ENABLE_ENABLE << XOSC_CTRL_ENABLE_Pos);
  while (0 == (XOSC->STATUS & XOSC_STATUS_STABLE_Msk));

  // Setup SYS PLL for 12 MHz * 40 / 4 / 1 = 120 MHz
  RESETS_CLR->RESET = RESETS_RESET_pll_sys_Msk;
  while (0 == RESETS->RESET_DONE_b.pll_sys);

  PLL_SYS->CS = (1 << PLL_SYS_CS_REFDIV_Pos);
  PLL_SYS->FBDIV_INT = 40;
  PLL_SYS->PRIM = (4 << PLL_SYS_PRIM_POSTDIV1_Pos) | (1 << PLL_SYS_PRIM_POSTDIV2_Pos);

  PLL_SYS_CLR->PWR = PLL_SYS_PWR_VCOPD_Msk | PLL_SYS_PWR_PD_Msk;
  while (0 == PLL_SYS->CS_b.LOCK);

  PLL_SYS_CLR->PWR = PLL_SYS_PWR_POSTDIVPD_Msk;

  // Setup USB PLL for 12 MHz * 36 / 3 / 3 = 48 MHz
  RESETS_CLR->RESET = RESETS_RESET_pll_usb_Msk;
  while (0 == RESETS->RESET_DONE_b.pll_usb);

  PLL_USB->CS = (1 << PLL_SYS_CS_REFDIV_Pos);
  PLL_USB->FBDIV_INT = 36;
  PLL_USB->PRIM = (3 << PLL_SYS_PRIM_POSTDIV1_Pos) | (3 << PLL_SYS_PRIM_POSTDIV2_Pos);

  PLL_USB_CLR->PWR = PLL_SYS_PWR_VCOPD_Msk | PLL_SYS_PWR_PD_Msk;
  while (0 == PLL_USB->CS_b.LOCK);

  PLL_USB_CLR->PWR = PLL_SYS_PWR_POSTDIVPD_Msk;

  // Switch clocks to their final socurces
  CLOCKS->CLK_REF_CTRL = (CLOCKS_CLK_REF_CTRL_SRC_xosc_clksrc << CLOCKS_CLK_REF_CTRL_SRC_Pos);

  CLOCKS->CLK_SYS_CTRL = (CLOCKS_CLK_SYS_CTRL_AUXSRC_clksrc_pll_sys << CLOCKS_CLK_SYS_CTRL_AUXSRC_Pos);
  CLOCKS_SET->CLK_SYS_CTRL = (CLOCKS_CLK_SYS_CTRL_SRC_clksrc_clk_sys_aux << CLOCKS_CLK_SYS_CTRL_SRC_Pos);

  CLOCKS->CLK_PERI_CTRL = CLOCKS_CLK_PERI_CTRL_ENABLE_Msk |
      (CLOCKS_CLK_PERI_CTRL_AUXSRC_clk_sys << CLOCKS_CLK_PERI_CTRL_AUXSRC_Pos);

  CLOCKS->CLK_USB_CTRL = CLOCKS_CLK_USB_CTRL_ENABLE_Msk |
      (CLOCKS_CLK_USB_CTRL_AUXSRC_clksrc_pll_usb << CLOCKS_CLK_USB_CTRL_AUXSRC_Pos);

  CLOCKS->CLK_ADC_CTRL = CLOCKS_CLK_ADC_CTRL_ENABLE_Msk |
      (CLOCKS_CLK_ADC_CTRL_AUXSRC_clksrc_pll_usb << CLOCKS_CLK_ADC_CTRL_AUXSRC_Pos);

  CLOCKS->CLK_RTC_DIV = (256 << CLOCKS_CLK_RTC_DIV_INT_Pos); // 12MHz / 256 = 46875 Hz
  CLOCKS->CLK_RTC_CTRL = CLOCKS_CLK_RTC_CTRL_ENABLE_Msk |
      (CLOCKS_CLK_RTC_CTRL_AUXSRC_xosc_clksrc << CLOCKS_CLK_ADC_CTRL_AUXSRC_Pos);

  // Configure 1 us tick for watchdog and timer
  WATCHDOG->TICK = ((F_REF/F_TICK) << WATCHDOG_TICK_CYCLES_Pos) | WATCHDOG_TICK_ENABLE_Msk;

  // Enable GPIOs
  RESETS_CLR->RESET = RESETS_RESET_io_bank0_Msk | RESETS_RESET_pads_bank0_Msk;
  while (0 == RESETS->RESET_DONE_b.io_bank0 || 0 == RESETS->RESET_DONE_b.pads_bank0);
}

//-----------------------------------------------------------------------------
void i2cRead( uint8_t slvAdd, uint8_t regAdd, uint8_t* data, uint32_t len )
{
    (void)len;  // ignore it for now.
    I2C0->IC_ENABLE = 0;
    I2C0->IC_TAR = slvAdd;
    I2C0->IC_ENABLE = 1;

    I2C0->IC_DATA_CMD = ( ( I2C0_IC_DATA_CMD_RESTART_DISABLE << I2C0_IC_DATA_CMD_RESTART_Pos ) |
                          ( I2C0_IC_DATA_CMD_STOP_ENABLE << I2C0_IC_DATA_CMD_STOP_Pos ) |
                          ( I2C0_IC_DATA_CMD_CMD_WRITE << I2C0_IC_DATA_CMD_CMD_Pos ) |
                          regAdd );

    while ( I2C0->IC_RAW_INTR_STAT_b.TX_EMPTY != I2C0_IC_INTR_STAT_R_TX_EMPTY_ACTIVE );
    while ( I2C0->IC_RAW_INTR_STAT_b.STOP_DET != I2C0_IC_INTR_STAT_R_STOP_DET_ACTIVE );

    I2C0->IC_DATA_CMD = ( ( I2C0_IC_DATA_CMD_RESTART_DISABLE << I2C0_IC_DATA_CMD_RESTART_Pos ) |
                          ( I2C0_IC_DATA_CMD_STOP_ENABLE << I2C0_IC_DATA_CMD_STOP_Pos ) |
                          ( I2C0_IC_DATA_CMD_CMD_READ << I2C0_IC_DATA_CMD_CMD_Pos ) );

    while ( !( I2C0->IC_RXFLR) );
    //while ( I2C0->IC_RAW_INTR_STAT_b.RX_DONE );

    data[0] = I2C0->IC_DATA_CMD_b.DAT;
}

//-----------------------------------------------------------------------------
void i2cWrite( uint8_t slvAdd, uint8_t* data, uint8_t len )
{
    I2C0->IC_ENABLE = 0;
    I2C0->IC_TAR = slvAdd;
    I2C0->IC_ENABLE = 1;

    for (uint8_t cnt = 0; cnt < len; cnt++ )
    {
    //bool firstByte = ( cnt == 0 ? true : false );
    bool lastByte  = ( ( len - cnt ) == 1 ? true : false );
    I2C0->IC_DATA_CMD = ( ( I2C0_IC_DATA_CMD_RESTART_DISABLE << I2C0_IC_DATA_CMD_RESTART_Pos ) |
                          ( ( lastByte ? I2C0_IC_DATA_CMD_STOP_ENABLE : I2C0_IC_DATA_CMD_STOP_DISABLE ) << I2C0_IC_DATA_CMD_STOP_Pos ) |
                          //( I2C0_IC_DATA_CMD_CMD_WRITE << I2C0_IC_DATA_CMD_CMD_Pos ) |
                          *data++ );

    while ( I2C0->IC_RAW_INTR_STAT_b.TX_EMPTY != I2C0_IC_INTR_STAT_R_TX_EMPTY_ACTIVE );
    }
}

//-----------------------------------------------------------------------------
uint8_t convBin2hex(uint8_t input)
{
    uint8_t retVal = 'q';

    if ( ( input & 0x0F ) <= 9 )
    {
      retVal = ( input & 0x0F ) + '0';
    }
    else if ( ( input & 0x0F ) >= 0x0a )
    {
      retVal = ( ( input & 0x0F ) - 10 ) + 'a';
    }

    return retVal;
}

//-----------------------------------------------------------------------------
void printByte( uint8_t data )
{
    uart_putc('[');
    uart_putc( convBin2hex( data >> 4 ));
    uart_putc( convBin2hex( data ));
    uart_puts("]\n\r");
}

//-----------------------------------------------------------------------------
int main(void)
{
  uint32_t cnt = 0;
  bool fast = false;
  char rxch;
  uint8_t devSlvAdd = 0x76;  // BMP280 sensor i2c address

  sys_init();
  timer_init();
  uart_init(115200);
  i2c0_init();

  uart_puts("\r\n ------ Testing RP2400 Communication ports ------ \r\n");

  //HAL_GPIO_PINX_out();
  //HAL_GPIO_PINX_set();
  
  HAL_GPIO_LED_out();
  HAL_GPIO_LED_clr();

  HAL_GPIO_BUTTON_in();
  HAL_GPIO_BUTTON_pullup();

  HAL_GPIO_I2C0_SCL_pullup();
  HAL_GPIO_I2C0_SDA_pullup();


  uint32_t x = 0;
  uart_puts("\r\nWait..\r\n");
  while( x++ < 5000000 );

  // BMP280 sensor basic cfg data
  uint8_t cfgData[3] = { 0xf4,
                         0x27,
                         0x00 };

  uart_puts("\r\nWriting cfg\r\n");
  i2cWrite( devSlvAdd, &cfgData[0], 3);

  uint32_t timeCnt = 0;
  while (1)
  {
    if (HAL_GPIO_BUTTON_read())
      cnt = 0;
    else if (cnt < 5001)
      cnt++;

    if (5000 == cnt)
    {
      fast = !fast;
      timer_set_period(fast ? PERIOD_FAST : PERIOD_SLOW);
      uart_putc('.');
    }

    if (uart_getc(&rxch))
    {
      uart_putc(fast ? invert_case(rxch) : rxch);
    }

    if ( !( timeCnt % ( 1024 * 4000 ) ) )
    {
      uart_puts("\r\nRead I2C\r\n");
    
      uint8_t data;

      uart_puts("DevId\r\n");
      i2cRead( devSlvAdd, 0xd0, &data, 1 );
      printByte( data );

      uart_puts("Press\r\n");
      i2cRead( devSlvAdd, 0xf9, &data, 1 );
      printByte( data );

      i2cRead( devSlvAdd, 0xf8, &data, 1 );
      printByte( data );

      i2cRead( devSlvAdd, 0xf7, &data, 1 );
      printByte( data );

      uart_puts("Temp\r\n");
      i2cRead( devSlvAdd, 0xfc, &data, 1 );
      printByte( data );

      i2cRead( devSlvAdd, 0xfb, &data, 1 );
      printByte( data );

      i2cRead( devSlvAdd, 0xfa, &data, 1 );
      printByte( data );
    }
    timeCnt++;
  }

  return 0;
}


