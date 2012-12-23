/*
 * STM32 Microcontroller RCC (Reset and Clock Control) module
 *
 * Copyright (C) 2010 Andre Beckus
 * Copyright (C) 2012
 *
 * Source code based on omap_clk.c
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "stm32f2xx.h"
#include "clktree.h"
#include <stdio.h>


/* DEFINITIONS*/

/* See README for DEBUG details. */
//#define DEBUG_STM32_RCC

#ifdef DEBUG_STM32_RCC
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32_RCC: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define HSI_FREQ 8000000
#define LSI_FREQ 40000

#define RCC_CR_OFFSET         0x00
#define RCC_PLLCFGR_OFFSET    0x04
#define RCC_CFGR_OFFSET       0x08
#define RCC_CIR_OFFSET        0x0c
#define RCC_AHB1RSTR_OFFSET   0x10
#define RCC_AHB2RSTR_OFFSET   0x14
#define RCC_AHB3RSTR_OFFSET   0x18
#define RCC_APB1RSTR_OFFSET   0x20
#define RCC_APB2RSTR_OFFSET   0x24
#define RCC_AHB1ENR_OFFSET    0x30
#define RCC_AHB2ENR_OFFSET    0x34
#define RCC_AHB3ENR_OFFSET    0x38
#define RCC_APB1ENR_OFFSET    0x40
#define RCC_APB2ENR_OFFSET    0x44
#define RCC_AHB1LPENR_OFFSET  0x50
#define RCC_AHB2LPENR_OFFSET  0x54
#define RCC_AHB3LPENR_OFFSET  0x58
#define RCC_APB1LPENR_OFFSET  0x60
#define RCC_APB2LPENR_OFFSET  0x64
#define RCC_BDCR_OFFSET       0x70
#define RCC_CSR_OFFSET        0x74
#define RCC_SSCGR_OFFSET      0x80
#define RCC_PLLI2SCFGR_OFFSET 0x84

/* CR */
#define RCC_CR_PLLI2SRDY_BIT    27
#define RCC_CR_PLLI2SON_BIT     26
#define RCC_CR_PLLRDY_BIT       25
#define RCC_CR_PLLON_BIT        24
#define RCC_CR_CSSON_BIT        19
#define RCC_CR_HSEBYP_BIT       18
#define RCC_CR_HSERDY_BIT       17
#define RCC_CR_HSEON_BIT        16
#define RCC_CR_HSICAL_START     8
#define RCC_CR_HSICAL_MASK      0x0000ff00
#define RCC_CR_HSITRIM_START    3
#define RCC_CR_HSITRIM_MASK     0x000000f8
#define RCC_CR_HSIRDY_BIT       1
#define RCC_CR_HSION_BIT        0

/* PLLCFGR */
#define RCC_PLLCFGR_PLLQ_START  24
#define RCC_PLLCFGR_PLLQ_MASK   0x0F000000
#define RCC_PLLCFGR_PLLSRC_BIT  22
#define RCC_PLLCFGR_PLLP_START  16
#define RCC_PLLCFGR_PLLP_MASK   0x00030000
#define RCC_PLLCFGR_PLLN_START  6
#define RCC_PLLCFGR_PLLN_MASK   0x00007FC0
#define RCC_PLLCFGR_PLMM_START  0
#define RCC_PLLCFGR_PLMM_MASK   0x0000003F

/* CFGR */
#define RCC_CFGR_MCO2_START     30
#define RCC_CFGR_MCO2_MASK      0xC0000000
#define RCC_CFGR_MCO2PRE_START  27
#define RCC_CFGR_MCO2PRE_MASK   0x38000000
#define RCC_CFGR_MCO1PRE_START  24
#define RCC_CFGR_MCO1PRE_MASK   0x07000000
#define RCC_CFGR_I2SSRC_BIT     23
#define RCC_CFGR_MCO1_START     21
#define RCC_CFGR_MCO1_MASK      0x00600000
#define RCC_CFGR_RTCPRE_START   16
#define RCC_CFGR_RTCPRE_MASK    0x001F0000
#define RCC_CFGR_PPRE2_START    13
#define RCC_CFGR_PPRE2_MASK     0x0000E000
#define RCC_CFGR_PPRE1_START    10
#define RCC_CFGR_PPRE1_MASK     0x00001C00
#define RCC_CFGR_HPRE_START     4
#define RCC_CFGR_HPRE_MASK      0x000000F0
#define RCC_CFGR_SWS_START      2
#define RCC_CFGR_SWS_MASK       0x0000000C
#define RCC_CFGR_SW_START       0
#define RCC_CFGR_SW_MASK        0x00000003

/* CIR */
#define RCC_CIR_CSSC_BIT        23
#define RCC_CIR_PLLI2SRDYC_BIT  21
#define RCC_CIR_PLLRDYC_BIT     20
#define RCC_CIR_HSERDYC_BIT     19
#define RCC_CIR_HSIRDYC_BIT     18
#define RCC_CIR_LSERDYC_BIT     17
#define RCC_CIR_LSIRDYC_BIT     16
#define RCC_CIR_PLLI2SRDYIE_BIT 13
#define RCC_CIR_PLLRDYIE_BIT    12
#define RCC_CIR_HSERDYIE_BIT    11
#define RCC_CIR_HSIRDYIE_BIT    10
#define RCC_CIR_LSERDYIE_BIT     9
#define RCC_CIR_LSIRDYIE_BIT     8
#define RCC_CIR_CSSF_BIT         7
#define RCC_CIR_PLLI2SRDYF_BIT   5
#define RCC_CIR_PLLRDYF_BIT      4
#define RCC_CIR_HSERDYF_BIT      3
#define RCC_CIR_HSIRDYF_BIT      2
#define RCC_CIR_LSERDYF_BIT      1
#define RCC_CIR_LSIRDYF_BIT      0

/* AHB1RSTR */
#define RCC_AHB1RSTR_OTGHSRST_BIT  29
#define RCC_AHB1RSTR_ETHMACRST_BIT 25
#define RCC_AHB1RSTR_DMA2RST_BIT   22
#define RCC_AHB1RSTR_DMA1RST_BIT   21
#define RCC_AHB1RSTR_CRCRST_BIT    12
#define RCC_AHB1RSTR_GPIOIRST_BIT   8
#define RCC_AHB1RSTR_GPIOHRST_BIT   7
#define RCC_AHB1RSTR_GPIOGRST_BIT   6
#define RCC_AHB1RSTR_GPIOFRST_BIT   5
#define RCC_AHB1RSTR_GPIOERST_BIT   4
#define RCC_AHB1RSTR_GPIODRST_BIT   3
#define RCC_AHB1RSTR_GPIOCRST_BIT   2
#define RCC_AHB1RSTR_GPIOBRST_BIT   1
#define RCC_AHB1RSTR_GPIOARST_BIT   0

/* AHB2RSTR */
#define RCC_AHB2RSTR_OTGFSRST_BIT 7
#define RCC_AHB2RSTR_RNGRST_BIT   6
#define RCC_AHB2RSTR_HASHRST_BIT  5
#define RCC_AHB2RSTR_CRYPRST_BIT  4
#define RCC_AHB2RSTR_DCMIRST_BIT  0

/* AHB3RSTR */
#define RCC_AHB3RSTR_FSMCRST_BIT  0

/* APB1RSTR */
#define RCC_APB1RSTR_DACRST_BIT   29
#define RCC_APB1RSTR_PWRRST_BIT   28
#define RCC_APB1RSTR_CAN2RST_BIT  26
#define RCC_APB1RSTR_CAN1RST_BIT  25
#define RCC_APB1RSTR_I2C3RST_BIT  23
#define RCC_APB1RSTR_I2C2RST_BIT  22
#define RCC_APB1RSTR_I2C1RST_BIT  21
#define RCC_APB1RSTR_UART5RST_BIT 20
#define RCC_APB1RSTR_UART4RST_BIT 19
#define RCC_APB1RSTR_UART3RST_BIT 18
#define RCC_APB1RSTR_UART2RST_BIT 17
#define RCC_APB1RSTR_SPI3RST_BIT  15
#define RCC_APB1RSTR_SPI2RST_BIT  14
#define RCC_APB1RSTR_WWDGRST_BIT  11
#define RCC_APB1RSTR_TIM14RST_BIT  8
#define RCC_APB1RSTR_TIM13RST_BIT  7
#define RCC_APB1RSTR_TIM12RST_BIT  6
#define RCC_APB1RSTR_TIM7RST_BIT   5
#define RCC_APB1RSTR_TIM6RST_BIT   4
#define RCC_APB1RSTR_TIM5RST_BIT   3
#define RCC_APB1RSTR_TIM4RST_BIT   2
#define RCC_APB1RSTR_TIM3RST_BIT   1
#define RCC_APB1RSTR_TIM2RST_BIT   0

/* APB2RSTR */
#define RCC_APB2RSTR_TIM11RST_BIT  18
#define RCC_APB2RSTR_TIM10RST_BIT  17
#define RCC_APB2RSTR_TIM9RST_BIT   16
#define RCC_APB2RSTR_SYSCFGRST_BIT 14
#define RCC_APB2RSTR_SPI1RST_BIT   12
#define RCC_APB2RSTR_SDIORST_BIT   11
#define RCC_APB2RSTR_ADCRST_BIT     8
#define RCC_APB2RSTR_USART6RST_BIT  5
#define RCC_APB2RSTR_USART1RST_BIT  4
#define RCC_APB2RSTR_TIM8RST_BIT    1
#define RCC_APB2RSTR_TIM1RST_BIT    0

/* AHB1ENR */
#define RCC_AHB1ENR_OTGHSULPIEN_BIT 30
#define RCC_AHB1ENR_OTGHSEN_BIT     29
#define RCC_AHB1ENR_ETHMACPTPEN_BIT 28
#define RCC_AHB1ENR_ETHMACRXEN_BIT  27
#define RCC_AHB1ENR_ETHMACTXEN_BIT  26
#define RCC_AHB1ENR_ETHMACEN_BIT    25
#define RCC_AHB1ENR_DMA2EN_BIT      22
#define RCC_AHB1ENR_DMA1EN_BIT      21
#define RCC_AHB1ENR_BKPSRAMEN_BIT   12
#define RCC_AHB1ENR_CRCEN_BIT       12
#define RCC_AHB1ENR_GPIOIEN_BIT      8
#define RCC_AHB1ENR_GPIOHEN_BIT      7
#define RCC_AHB1ENR_GPIOGEN_BIT      6
#define RCC_AHB1ENR_GPIOFEN_BIT      5
#define RCC_AHB1ENR_GPIOEEN_BIT      4
#define RCC_AHB1ENR_GPIODEN_BIT      3
#define RCC_AHB1ENR_GPIOCEN_BIT      2
#define RCC_AHB1ENR_GPIOBEN_BIT      1
#define RCC_AHB1ENR_GPIOAEN_BIT      0

/* AHB2ENR  */
#define RCC_AHB2ENR_OTGFSEN_BIT 7
#define RCC_AHB2ENR_RNGEN_BIT   6
#define RCC_AHB2ENR_HASHEN_BIT  5
#define RCC_AHB2ENR_CRYPEN_BIT  4
#define RCC_AHB2ENR_DCMIEN_BIT  0

/* AHB3ENR */
#define RCC_AHB3ENR_BIT 0

/* APB1ENR */
#define RCC_APB1ENR_DACEN_BIT    29
#define RCC_APB1ENR_PWREN_BIT    28
#define RCC_APB1ENR_CAN2EN_BIT   26
#define RCC_APB1ENR_CAN1EN_BIT   25
#define RCC_APB1ENR_I2C3EN_BIT   23
#define RCC_APB1ENR_I2C2EN_BIT   22
#define RCC_APB1ENR_I2C1EN_BIT   21
#define RCC_APB1ENR_UART5EN_BIT  20
#define RCC_APB1ENR_UART4EN_BIT  19
#define RCC_APB1ENR_USART3EN_BIT 18
#define RCC_APB1ENR_USART2EN_BIT 17
#define RCC_APB1ENR_SPI3EN_BIT   15
#define RCC_APB1ENR_SPI2EN_BIT   14
#define RCC_APB1ENR_WWDGEN_BIT   11
#define RCC_APB1ENR_TIM14EN_BIT   8
#define RCC_APB1ENR_TIM13EN_BIT   7
#define RCC_APB1ENR_TIM12EN_BIT   6
#define RCC_APB1ENR_TIM7EN_BIT    5
#define RCC_APB1ENR_TIM6EN_BIT    4
#define RCC_APB1ENR_TIM5EN_BIT    3
#define RCC_APB1ENR_TIM4EN_BIT    2
#define RCC_APB1ENR_TIM3EN_BIT    1
#define RCC_APB1ENR_TIM2EN_BIT    0

/* APB2ENR */
#define RCC_APB2ENR_TIM11EN_BIT  18
#define RCC_APB2ENR_TIM10EN_BIT  17
#define RCC_APB2ENR_TIM9EN_BIT   16
#define RCC_APB2ENR_SYSCFGEN_BIT 14
#define RCC_APB2ENR_SPI1EN_BIT   12
#define RCC_APB2ENR_SDIOEN_BIT   11
#define RCC_APB2ENR_ADC3EN_BIT   10
#define RCC_APB2ENR_ADC2EN_BIT    9
#define RCC_APB2ENR_ADC1EN_BIT    8
#define RCC_APB2ENR_USART6EN_BIT  5
#define RCC_APB2ENR_USART1EN_BIT  4
#define RCC_APB2ENR_TIM8EN_BIT    1
#define RCC_APB2ENR_TIM1EN_BIT    0

/* BDCR */
#define RCC_BDCR_BDRST_BIT    16
#define RCC_BDCR_RTCEN_BIT    15
#define RCC_BDCR_RTCSEL_START 8
#define RCC_BDCR_RTCSEL_MASK  0x00000300
#define RCC_BDCR_LSEBYP_BIT   2
#define RCC_BDCR_LSERDY_BIT   1
#define RCC_BDCR_LSEON_BIT    0

/* CSR */
#define RCC_CSR_LPWR_RSTF_BIT 31
#define RCC_CSR_WWDGRSTF_BIT  30
#define RCC_CSR_IWDGRSTF_BIT  29
#define RCC_CSR_SFTRSTF_BIT   28
#define RCC_CSR_PORRSTF_BIT   27
#define RCC_CSR_PINRSTF_BIT   26
#define RCC_CSR_BORRSTF_BIT   25
#define RCC_CSR_RMVF_BIT      24
#define RCC_CSR_LSIRDY_BIT     1
#define RCC_CSR_LSION_BIT      0

/* SSCGR */
#define RCC_SSCGR_SSCGEN_BIT              31
#define RCC_SSCGR_SPREADSEL_BIT           30
#define RCC_SSCGR_INCSTEP_START           13
#define RCC_SSCGR_INCSTEP_MASK            0x0FFFE000
#define RCC_SSCGR_MODPER_START            0
#define RCC_SSCGR_MODPER_MASK             0x00001FFF
#define RCC_SSCGR_PLLI2SCFGR_PLLI2SR2_BIT 30
#define RCC_SSCGR_PLLI2SCFGR_PLLI2SR1_BIT 29
#define RCC_SSCGR_PLLI2SCFGR_PLLI2SR0_BIT 28
#define RCC_SSCGR_PLLI2SCFGR_PLLI2SN8_BIT 14 
#define RCC_SSCGR_PLLI2SCFGR_PLLI2SN7_BIT 13
#define RCC_SSCGR_PLLI2SCFGR_PLLI2SN6_BIT 12
#define RCC_SSCGR_PLLI2SCFGR_PLLI2SN5_BIT 11
#define RCC_SSCGR_PLLI2SCFGR_PLLI2SN4_BIT 10
#define RCC_SSCGR_PLLI2SCFGR_PLLI2SN3_BIT  9
#define RCC_SSCGR_PLLI2SCFGR_PLLI2SN2_BIT  8
#define RCC_SSCGR_PLLI2SCFGR_PLLI2SN1_BIT  7
#define RCC_SSCGR_PLLI2SCFGR_PLLI2SN0_BIT  6

#define PLLSRC_HSI_SELECTED 0 /* add code that use these defines, or remove */
#define PLLSRC_HSE_SELECTED 1

#define SW_HSI_SELECTED 0
#define SW_HSE_SELECTED 1
#define SW_PLL_SELECTED 2

struct Stm32Rcc {
   /* Inherited */
   SysBusDevice busdev;

   /* Properties */
   uint32_t osc_freq;
   uint32_t osc32_freq;

   /* Private */
   MemoryRegion iomem;

   /* Register Values */
   uint32_t RCC_AHB1ENR;
   uint32_t RCC_APB1ENR;
   uint32_t RCC_APB2ENR;
   uint32_t RCC_PLLCFGR;
   uint32_t RCC_APB1RSTR;

    /* Register Field Values */

   /* CFGR */
   uint32_t RCC_CFGR_MCO2;
   uint32_t RCC_CFGR_MCO2PRE;
   uint32_t RCC_CFGR_MCO1PRE;
   uint32_t RCC_CFGR_I2SSRC;
   uint32_t RCC_CFGR_MCO1;
   uint32_t RCC_CFGR_RTCPRE;
   uint32_t RCC_CFGR_PPRE2;
   uint32_t RCC_CFGR_PPRE1;
   uint32_t RCC_CFGR_HPRE;
   uint32_t RCC_CFGR_SW;


    Clk HSICLK;
    Clk HSECLK;
    Clk LSECLK;
    Clk LSICLK;
    Clk SYSCLK;
    Clk PLLXTPRECLK;
    Clk PLLCLK;
    Clk HCLK;  /* Output from AHB Prescaler */
    Clk PCLK1; /* Output from APB1 Prescaler */
    Clk PCLK2; /* Output from APB2 Prescaler */
    Clk PERIPHCLK[STM32_PERIPH_COUNT];

    qemu_irq irq;
};

/*-----------------------------------------------------------------------------
  RCC - enable peripheral clock
-----------------------------------------------------------------------------*/
static void stm32_rcc_periph_enable(Stm32Rcc* s, uint32_t new_value, bool init,
                                    int periph, uint32_t bit_mask)
{
    clktree_set_enabled(s->PERIPHCLK[periph], IS_BIT_SET(new_value, bit_mask));
    return;
}

/*-----------------------------------------------------------------------------
  RCC CR - CONFIGURATION REGISTER
-----------------------------------------------------------------------------*/
static uint32_t stm32_rcc_RCC_CR_read(Stm32Rcc *s)
{
   bool PLLON = clktree_is_enabled(s->PLLCLK);
   bool HSEON = clktree_is_enabled(s->HSECLK);
   bool HSION = clktree_is_enabled(s->HSICLK);

   /* not included: CS,HSICAL,HSITRIM */
   return GET_BIT_MASK(RCC_CR_PLLRDY_BIT, PLLON) |
      GET_BIT_MASK(RCC_CR_PLLON_BIT, PLLON) |
      GET_BIT_MASK(RCC_CR_HSERDY_BIT, HSEON) |
      GET_BIT_MASK(RCC_CR_HSEON_BIT, HSEON) |
      GET_BIT_MASK(RCC_CR_HSIRDY_BIT, HSION) |
      GET_BIT_MASK(RCC_CR_HSION_BIT, HSION);
}

static void stm32_rcc_RCC_CR_write(Stm32Rcc *s, uint32_t new_value, bool init)
{
    bool new_PLLON;
    bool new_HSEON;
    bool new_HSION;

    new_PLLON = IS_BIT_SET(new_value, RCC_CR_PLLON_BIT);
    if ((clktree_is_enabled(s->PLLCLK) && !new_PLLON) &&
       s->RCC_CFGR_SW == SW_PLL_SELECTED) {
        stm32f2xx_hw_warn("PLL cannot disable, selected as the system clock.");
    }
    clktree_set_enabled(s->PLLCLK, new_PLLON);

    new_HSEON = IS_BIT_SET(new_value, RCC_CR_HSEON_BIT);
    if ((clktree_is_enabled(s->HSECLK) && !new_HSEON) &&
       (s->RCC_CFGR_SW == SW_HSE_SELECTED ||
        (s->RCC_CFGR_SW == SW_PLL_SELECTED)
       )
      ) {
        stm32f2xx_hw_warn("HSE osc. cannot disable, driving the system clock.");
    }
    clktree_set_enabled(s->HSECLK, new_HSEON);

    new_HSION = IS_BIT_SET(new_value, RCC_CR_HSION_BIT);
    if((clktree_is_enabled(s->HSECLK) && !new_HSEON) &&
       (s->RCC_CFGR_SW == SW_HSI_SELECTED ||
        (s->RCC_CFGR_SW == SW_PLL_SELECTED)
       )
      ) {
        stm32f2xx_hw_warn("HSI osc. cannot disable, driving the system clock.");
    }
    clktree_set_enabled(s->HSICLK, new_HSION);
}

/*-----------------------------------------------------------------------------
  RCC PLLCFGR - PLL CONFIGURATION REGISTER
-----------------------------------------------------------------------------*/
static uint32_t stm32_rcc_RCC_PLLCFGR_read(Stm32Rcc* s)
{
   return s->RCC_PLLCFGR;
}

static void stm32_rcc_RCC_PLLCFGR_write(Stm32Rcc *s, uint32_t value, bool init)
{
   s->RCC_PLLCFGR = value;
   /* todo: add configuration of pll clocks */
   return;
}

/*-----------------------------------------------------------------------------
  RCC CFGR - CLOCK CONFIGURATION REGISTER
-----------------------------------------------------------------------------*/
static uint32_t stm32_rcc_RCC_CFGR_read(Stm32Rcc *s)
{
   uint32_t rcc_cfgr;
   rcc_cfgr = (s->RCC_CFGR_MCO2 << RCC_CFGR_MCO2_START) |
      (s->RCC_CFGR_MCO2PRE << RCC_CFGR_MCO2PRE_START) |
      (s->RCC_CFGR_MCO1PRE << RCC_CFGR_MCO1PRE_START) |
      (s->RCC_CFGR_I2SSRC << RCC_CFGR_I2SSRC_BIT) |
      (s->RCC_CFGR_MCO1 << RCC_CFGR_MCO1_START) |
      (s->RCC_CFGR_RTCPRE << RCC_CFGR_RTCPRE_START) |
      (s->RCC_CFGR_PPRE2 << RCC_CFGR_PPRE2_START) |
      (s->RCC_CFGR_PPRE1 << RCC_CFGR_PPRE1_START) |
      (s->RCC_CFGR_HPRE << RCC_CFGR_HPRE_START) |
      (s->RCC_CFGR_SW << RCC_CFGR_SWS_START) |
      (s->RCC_CFGR_SW << RCC_CFGR_SW_START);

   /* printf("CFGR read 0x%x\n", rcc_cfgr); */
   return rcc_cfgr;
}


static void stm32_rcc_RCC_CFGR_write(Stm32Rcc *s, uint32_t new_value, bool init)
{
   uint32_t new_PPRE2;
   uint32_t new_PPRE1;
   uint32_t new_HPRE;
   uint32_t apb1_prescaler;
   uint32_t apb2_prescaler;
   uint32_t ahb_prescaler;

   s->RCC_CFGR_MCO2 = (new_value & RCC_CFGR_MCO2_MASK) >> RCC_CFGR_MCO2_START;
   s->RCC_CFGR_MCO2PRE = (new_value & RCC_CFGR_MCO2PRE_MASK) >> RCC_CFGR_MCO2PRE_START;
   s->RCC_CFGR_MCO1PRE = (new_value & RCC_CFGR_MCO1PRE_MASK) >> RCC_CFGR_MCO1PRE_START;
   s->RCC_CFGR_I2SSRC = GET_BIT_VALUE(new_value, RCC_CFGR_I2SSRC_BIT);
   s->RCC_CFGR_MCO1 = (new_value & RCC_CFGR_MCO1_MASK) >> RCC_CFGR_MCO1_START;
   s->RCC_CFGR_RTCPRE = (new_value & RCC_CFGR_RTCPRE_MASK) >> RCC_CFGR_RTCPRE_START;
   
   /* PPRE2 - APB2 high-speed prescaler */
   new_PPRE2 = (new_value & RCC_CFGR_PPRE2_MASK) >> RCC_CFGR_PPRE2_START;
   switch (new_PPRE2) {
   case 4:
      apb2_prescaler = 2;
      break;
   case 5:
      apb2_prescaler = 4;
      break;
   case 6:
      apb2_prescaler = 8;
      break;
   case 7:
      apb2_prescaler = 16;
      break;
   default:
      /* assume 0xx -> no division */
      apb2_prescaler = 1;
      break;
   }
   clktree_set_scale(s->PCLK2, 1, apb2_prescaler);
   
   /* PPRE1 - APB1 low-speed prescaler */
   new_PPRE1 = (new_value & RCC_CFGR_PPRE1_MASK) >> RCC_CFGR_PPRE1_START;
   switch (new_PPRE1) {
   case 4:
      apb1_prescaler = 2;
      break;
   case 5:
      apb1_prescaler = 4;
      break;
   case 6:
      apb1_prescaler = 8;
      break;
   case 7:
      apb1_prescaler = 16;
      break;
   default:
      /* assume 0xx -> no division */
      apb1_prescaler = 1;
      break;
   }
   clktree_set_scale(s->PCLK1, 1, apb1_prescaler);

   /* HPRE - AHB prescaler */
   new_HPRE = (new_value & RCC_CFGR_HPRE_MASK) >> RCC_CFGR_HPRE_START;
   switch (new_HPRE) {
   case 8:
      ahb_prescaler = 2;
      break;
   case 9:
      ahb_prescaler = 4;
      break;
   case 10:
      ahb_prescaler = 8;
      break;
   case 11:
      ahb_prescaler = 16;
      break;
   case 12:
      ahb_prescaler = 64;
      break;
   case 13:
      ahb_prescaler = 128;
      break;
   case 14:
      ahb_prescaler = 256;
      break;
   case 15:
      ahb_prescaler = 512;
      break;
   default:
      /* assume 0xx -> no division */
      ahb_prescaler = 1;
      break;
   }
   clktree_set_scale(s->HCLK, 1, ahb_prescaler);
   
   /* SW - system clock switch */
   s->RCC_CFGR_SW = (new_value & RCC_CFGR_SW_MASK) >> RCC_CFGR_SW_START;
   /* printf("CFGR_SW write: %u\n", s->RCC_CFGR_SW); */
   switch (s->RCC_CFGR_SW) {
   case 0:
   case 1:
   case 2:
      clktree_set_selected_input(s->SYSCLK, s->RCC_CFGR_SW);
      break;
   default:
      hw_error("invalid SYSCLK selection");
      break;
   }
}

/*-----------------------------------------------------------------------------
  RCC APB1RSTR - Peripheral reset register
  DUMMY
-----------------------------------------------------------------------------*/
static uint32_t stm32_rcc_RCC_APB1RSTR_read(Stm32Rcc* s)
{
   return s->RCC_APB1RSTR;
}

static void stm32_rcc_RCC_APB1RSTR_write(Stm32Rcc* s, uint32_t new_value,
                                         bool init)
{
   s->RCC_APB1RSTR = new_value;
}

/*-----------------------------------------------------------------------------
  RCC AHB1ENR - Peripheral clock enable register
  DUMMY - ONLY GPIOs added 
-----------------------------------------------------------------------------*/
static uint32_t stm32_rcc_RCC_AHB1ENR_read(Stm32Rcc* s)
{
   return s->RCC_AHB1ENR;
}

static void stm32_rcc_RCC_AHB1ENR_write(Stm32Rcc* s, uint32_t new_value,
                                        bool init)
{
    stm32_rcc_periph_enable(s, new_value, init, STM32_GPIOA,
                            RCC_AHB1ENR_GPIOAEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_GPIOB,
                            RCC_AHB1ENR_GPIOBEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_GPIOC,
                            RCC_AHB1ENR_GPIOCEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_GPIOD,
                            RCC_AHB1ENR_GPIODEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_GPIOE,
                            RCC_AHB1ENR_GPIOEEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_GPIOF,
                            RCC_AHB1ENR_GPIOFEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_GPIOG,
                            RCC_AHB1ENR_GPIOGEN_BIT);

    s->RCC_AHB1ENR = new_value & 0x3F3208FF;
}

/*-----------------------------------------------------------------------------
  RCC APB1ENR - Peripheral clock enable register
-----------------------------------------------------------------------------*/
static void stm32_rcc_RCC_APB1ENR_write(Stm32Rcc *s, uint32_t new_value,
                    bool init)
{
   /*   stm32_rcc_periph_enable(s, new_value, init, STM32_DAC,
                           RCC_APB1ENR_DACEN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_PWR,
                           RCC_APB1ENR_PWREN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_CAN2,
                           RCC_APB1ENR_CAN2EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_CAN1,
                           RCC_APB1ENR_CAN1EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_I2C3,
                           RCC_APB1ENR_I2C3EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_I2C2,
                           RCC_APB1ENR_I2C2EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_I2C1,
                           RCC_APB1ENR_I2C1EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_UART5,
                           RCC_APB1ENR_UART5EN_BIT);*/
   stm32_rcc_periph_enable(s, new_value, init, STM32_UART4,
                           RCC_APB1ENR_UART4EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_UART3,
                           RCC_APB1ENR_USART3EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_UART2,
                           RCC_APB1ENR_USART2EN_BIT);
   /*   stm32_rcc_periph_enable(s, new_value, init, STM32_SPI3,
                           RCC_APB1ENR_SPI3EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_SPI2,
                           RCC_APB1ENR_USART2EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_WWDG,
                           RCC_APB1ENR_WWDGEN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_TIM14,
                           RCC_APB1ENR_TIM14EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_TIM13,
                           RCC_APB1ENR_TIM13EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_TIM12,
                           RCC_APB1ENR_TIM12EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_TIM7,
                           RCC_APB1ENR_TIM7EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_TIM6,
                           RCC_APB1ENR_TIM6EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_TIM5,
                           RCC_APB1ENR_TIM5EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_TIM4,
                           RCC_APB1ENR_TIM4EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_TIM3,
                           RCC_APB1ENR_TIM3EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_TIM2,
   RCC_APB1ENR_TIM2EN_BIT);*/

   s->RCC_APB1ENR = new_value & 0x36FEC9FF;
}

/*-----------------------------------------------------------------------------
  RCC APB2ENR - Peripheral clock enable register
-----------------------------------------------------------------------------*/
static void stm32_rcc_RCC_APB2ENR_write(Stm32Rcc *s, uint32_t new_value,
                                        bool init)
{
   /* suggestion, add the different pheripherals to tables and just loop */

   /*   stm32_rcc_periph_enable(s, new_value, init, STM32_TIM11,
                           RCC_APB2ENR_TIM11EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_TIM10,
                           RCC_APB2ENR_TIM10EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_TIM9,
                           RCC_APB2ENR_TIM9EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_SYSCFG,
                           RCC_APB2ENR_SYSCFGEN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_SPI1,
                           RCC_APB2ENR_SPI1EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_SDIO,
                           RCC_APB2ENR_SDIOEN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_ADC3,
                           RCC_APB2ENR_ADC3EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_ADC2,
                           RCC_APB2ENR_ADC2EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_ADC1,
                           RCC_APB2ENR_ADC1EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_UART6,
                           RCC_APB2ENR_USART6EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_UART1,
                           RCC_APB2ENR_USART1EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_TIM8,
                           RCC_APB2ENR_TIM8EN_BIT);
   stm32_rcc_periph_enable(s, new_value, init, STM32_TIM1,
   RCC_APB2ENR_TIM1EN_BIT);*/

    s->RCC_APB2ENR = new_value & 0x00035f33;
}


/*-----------------------------------------------------------------------------
  RCC BDCR - Backup domain control register
-----------------------------------------------------------------------------*/
static uint32_t stm32_rcc_RCC_BDCR_read(Stm32Rcc *s)
{
    bool lseon = clktree_is_enabled(s->LSECLK);

    return GET_BIT_MASK(RCC_BDCR_LSERDY_BIT, lseon) |
           GET_BIT_MASK(RCC_BDCR_LSEON_BIT, lseon);
}

static void stm32_rcc_RCC_BDCR_write(Stm32Rcc *s, uint32_t new_value, bool init)
{
    clktree_set_enabled(s->LSECLK, IS_BIT_SET(new_value, RCC_BDCR_LSEON_BIT));
}

/*-----------------------------------------------------------------------------
  RCC CSR - Clock control & status register
-----------------------------------------------------------------------------*/
static uint32_t stm32_rcc_RCC_CSR_read(Stm32Rcc *s)
{
    bool lseon = clktree_is_enabled(s->LSICLK);

    return GET_BIT_MASK(RCC_CSR_LSIRDY_BIT, lseon) |
           GET_BIT_MASK(RCC_CSR_LSION_BIT, lseon);
}

static void stm32_rcc_RCC_CSR_write(Stm32Rcc *s, uint32_t new_value, bool init)
{
    clktree_set_enabled(s->LSICLK, IS_BIT_SET(new_value, RCC_CSR_LSION_BIT));
}

/*-----------------------------------------------------------------------------
  RCC Register read mapping
-----------------------------------------------------------------------------*/
static uint64_t stm32_rcc_readw(void *opaque, target_phys_addr_t offset)
{
    Stm32Rcc* s = (Stm32Rcc*) opaque;

    switch (offset) {
    case RCC_CR_OFFSET:
       return stm32_rcc_RCC_CR_read(s);
    case RCC_PLLCFGR_OFFSET:
       return stm32_rcc_RCC_PLLCFGR_read(s);
    case RCC_CFGR_OFFSET:
       return stm32_rcc_RCC_CFGR_read(s);
    case RCC_CIR_OFFSET:
       return 0;
    case RCC_AHB1RSTR_OFFSET:
       STM32_NOT_IMPL_REG(offset, 4);
       break;
    case RCC_AHB2RSTR_OFFSET:
       STM32_NOT_IMPL_REG(offset, 4);
       break;
    case RCC_AHB3RSTR_OFFSET:
       STM32_NOT_IMPL_REG(offset, 4);
       break;
    case RCC_APB1RSTR_OFFSET:
       return stm32_rcc_RCC_APB1RSTR_read(s);
       STM32_NOT_IMPL_REG(offset, 4);
       break;
    case RCC_APB2RSTR_OFFSET:
       STM32_NOT_IMPL_REG(offset, 4);
       break;
    case RCC_AHB1ENR_OFFSET:
       return stm32_rcc_RCC_AHB1ENR_read(s);
    case RCC_AHB2ENR_OFFSET:
       STM32_NOT_IMPL_REG(offset, 4);
       break;
    case RCC_AHB3ENR_OFFSET:
       STM32_NOT_IMPL_REG(offset, 4);
       break;
    case RCC_APB1ENR_OFFSET:
       return s->RCC_APB1ENR;
    case RCC_APB2ENR_OFFSET:
       return s->RCC_APB2ENR;
    case RCC_AHB1LPENR_OFFSET:
       STM32_NOT_IMPL_REG(offset, 4);
       break;
    case RCC_AHB2LPENR_OFFSET:
       STM32_NOT_IMPL_REG(offset, 4);
       break;
    case RCC_AHB3LPENR_OFFSET:
       STM32_NOT_IMPL_REG(offset, 4);
       break;
    case RCC_APB1LPENR_OFFSET:
       STM32_NOT_IMPL_REG(offset, 4);
       break;
    case RCC_APB2LPENR_OFFSET:
       STM32_NOT_IMPL_REG(offset, 4);
       break;
    case RCC_BDCR_OFFSET:
       return stm32_rcc_RCC_BDCR_read(s);
    case RCC_CSR_OFFSET:
       return stm32_rcc_RCC_CSR_read(s);
    case RCC_SSCGR_OFFSET:
       STM32_NOT_IMPL_REG(offset, 4);
       break;
    case RCC_PLLI2SCFGR_OFFSET:
       STM32_NOT_IMPL_REG(offset, 4);
       break;
    case 0x400:
       STM32_NOT_IMPL_REG(offset, 4);
       return 0;
    default:
       STM32_BAD_REG(offset, 4);
       break;
    }
}

/*-----------------------------------------------------------------------------
  RCC Register write mapping
-----------------------------------------------------------------------------*/
static void stm32_rcc_writew(void *opaque, target_phys_addr_t offset,
                          uint64_t value)
{
    Stm32Rcc *s = (Stm32Rcc *)opaque;

    switch(offset) {
    case RCC_CR_OFFSET:
       stm32_rcc_RCC_CR_write(s, value, false);
       break;
    case RCC_PLLCFGR_OFFSET:
       return stm32_rcc_RCC_PLLCFGR_write(s, value, false);
       break;
    case RCC_CFGR_OFFSET:
       stm32_rcc_RCC_CFGR_write(s, value, false);
       break;
    case RCC_CIR_OFFSET:
       /* Allow a write but don't take any action */
       break;
    case RCC_APB1RSTR_OFFSET:
       stm32_rcc_RCC_APB1RSTR_write(s, value, false);
       break;
    case RCC_APB2RSTR_OFFSET:
       STM32_NOT_IMPL_REG(offset, 4);
       break;
    case RCC_AHB1ENR_OFFSET:
       stm32_rcc_RCC_AHB1ENR_write(s, value, false);
       break;
    case RCC_APB2ENR_OFFSET:
       stm32_rcc_RCC_APB2ENR_write(s, value, false);
       break;
    case RCC_APB1ENR_OFFSET:
       stm32_rcc_RCC_APB1ENR_write(s, value, false);
       break;
    case RCC_BDCR_OFFSET:
       stm32_rcc_RCC_BDCR_write(s, value, false);
       break;
    case RCC_CSR_OFFSET:
       stm32_rcc_RCC_CSR_write(s, value, false);
       break;
    case RCC_AHB1RSTR_OFFSET:
       STM32_NOT_IMPL_REG(offset, 4);
       break;
    case 0x400:
       break;
    default:
       STM32_BAD_REG(offset, 4);
       break;
    }
}

static uint64_t stm32_rcc_read(void *opaque, target_phys_addr_t offset,
                          unsigned size)
{
    switch(size) {
        case 4:
            return stm32_rcc_readw(opaque, offset);
        default:
            STM32_NOT_IMPL_REG(offset, size);
            return 0;
    }
}

static void stm32_rcc_write(void *opaque, target_phys_addr_t offset,
                       uint64_t value, unsigned size)
{
    switch(size) {
        case 4:
            stm32_rcc_writew(opaque, offset, value);
            break;
        default:
            STM32_NOT_IMPL_REG(offset, size);
            break;
    }
}

static const MemoryRegionOps stm32_rcc_ops = {
   .read = stm32_rcc_read,
   .write = stm32_rcc_write,
   .endianness = DEVICE_NATIVE_ENDIAN
};


static void stm32_rcc_reset(DeviceState *dev)
{
    Stm32Rcc *s = FROM_SYSBUS(Stm32Rcc, sysbus_from_qdev(dev));

    stm32_rcc_RCC_CR_write(s, 0x00000083, true);
    stm32_rcc_RCC_CFGR_write(s, 0x00000000, true);
    stm32_rcc_RCC_APB2ENR_write(s, 0x00000000, true);
    stm32_rcc_RCC_APB1ENR_write(s, 0x00000000, true);
    stm32_rcc_RCC_BDCR_write(s, 0x00000000, true);
    stm32_rcc_RCC_CSR_write(s, 0x0c000000, true);
}

/* IRQ handler to handle updates to the HCLK frequency.
 * This updates the SysTick scales. */
static void stm32_rcc_hclk_upd_irq_handler(void *opaque, int n, int level)
{
    Stm32Rcc* s = (Stm32Rcc*) opaque;
    uint32_t hclk_freq;
    uint32_t ext_ref_freq;

    hclk_freq = clktree_get_output_freq(s->HCLK);

    /* Only update the scales if the frequency is not zero. */
    if(hclk_freq > 0) {
        ext_ref_freq = hclk_freq / 8;

        /* Update the scales - these are the ratio of QEMU clock ticks
         * (which is an unchanging number independent of the CPU frequency) to
         * system/external clock ticks.
         */
        system_clock_scale = get_ticks_per_sec() / hclk_freq;
        external_ref_clock_scale = get_ticks_per_sec() / ext_ref_freq;
    }

#ifdef DEBUG_STM32_RCC
    DPRINTF("Cortex SYSTICK frequency set to %lu Hz (scale set to %d).\n",
                (unsigned long)hclk_freq, system_clock_scale);
    DPRINTF("Cortex SYSTICK ext ref frequency set to %lu Hz "
              "(scale set to %d).\n",
              (unsigned long)ext_ref_freq, external_ref_clock_scale);
#endif
}







/* PUBLIC FUNCTIONS */

void stm32f2xx_rcc_check_periph_clk(Stm32Rcc *s, stm32_periph_t periph)
{
    Clk clk = s->PERIPHCLK[periph];
    assert(clk != NULL);

    if (!clktree_is_enabled(clk)) {
        /* I assume writing to a peripheral register while the peripheral clock
         * is disabled is a bug and give a warning to unsuspecting programmers.
         * When I made this mistake on real hardware the write had no effect.
         */
        hw_error("Warning: You are attempting to use the %s peripheral while "
                 "its clock is disabled.\n", stm32f2xx_periph_name(periph));
    }
}

void stm32f2xx_rcc_set_periph_clk_irq(Stm32Rcc *s, stm32_periph_t periph,
                                      qemu_irq periph_irq)
{
    Clk clk = s->PERIPHCLK[periph];
    assert(clk != NULL);
    clktree_adduser(clk, periph_irq);
}

uint32_t stm32f2xx_rcc_get_periph_freq(Stm32Rcc *s, stm32_periph_t periph)
{
    Clk clk;
    clk = s->PERIPHCLK[periph];
    assert(clk != NULL);
    return clktree_get_output_freq(clk);
}

/*-----------------------------------------------------------------------------
  RCC INITs
-----------------------------------------------------------------------------*/

/* rcc clock init */
static void stm32_rcc_init_clk(Stm32Rcc *s)
{
    int i;
    qemu_irq* hclk_upd_irq;
    Clk HSI_DIV2;
    Clk HSE_DIV2;

    hclk_upd_irq = qemu_allocate_irqs(stm32_rcc_hclk_upd_irq_handler, s, 1);

    /* Make sure all the peripheral clocks are null initially.
     * This will be used for error checking to make sure
     * an invalid clock is not referenced (not all of the
     * indexes will be used).
     */
    for(i = 0; i < STM32_PERIPH_COUNT; i++) {
        s->PERIPHCLK[i] = NULL;
    }

    /* Initialize clocks */
    /* Source clocks are initially disabled, which represents
     * a disabled oscillator.  Enabling the clock represents
     * turning the clock on.
     */
    s->HSICLK = clktree_create_src_clk("HSI", HSI_FREQ, false);
    s->LSICLK = clktree_create_src_clk("LSI", LSI_FREQ, false);
    s->HSECLK = clktree_create_src_clk("HSE", s->osc_freq, false);
    s->LSECLK = clktree_create_src_clk("LSE", s->osc32_freq, false);

    HSI_DIV2 = clktree_create_clk("HSI/2",
                                  1,
                                  2,
                                  true,
                                  CLKTREE_NO_MAX_FREQ,
                                  0,
                                  s->HSICLK,
                                  NULL);

    HSE_DIV2 = clktree_create_clk("HSE/2",
                                  1,
                                  2,
                                  true,
                                  CLKTREE_NO_MAX_FREQ,
                                  0,
                                  s->HSECLK,
                                  NULL);

    s->PLLXTPRECLK = clktree_create_clk("PLLXTPRE",
                                        1,
                                        1,
                                        true,
                                        CLKTREE_NO_MAX_FREQ,
                                        CLKTREE_NO_INPUT,
                                        s->HSECLK,
                                        HSE_DIV2,
                                        NULL);

    /* PLLCLK contains both the switch and the multiplier, which are shown as
     * two separate components in the clock tree diagram.
     */
    s->PLLCLK = clktree_create_clk("PLLCLK",
                                   0,
                                   1,
                                   false,
                                   72000000,
                                   CLKTREE_NO_INPUT,
                                   HSI_DIV2,
                                   s->PLLXTPRECLK,
                                   NULL);

    s->SYSCLK = clktree_create_clk("SYSCLK",
                                   1,
                                   1,
                                   true,
                                   72000000,
                                   CLKTREE_NO_INPUT,
                                   s->HSICLK,
                                   s->HSECLK,
                                   s->PLLCLK,
                                   NULL);

    s->HCLK = clktree_create_clk("HCLK",
                                 0,
                                 1,
                                 true,
                                 72000000,
                                 0,
                                 s->SYSCLK,
                                 NULL);

    clktree_adduser(s->HCLK, hclk_upd_irq[0]);

    s->PCLK1 = clktree_create_clk("PCLK1",
                                  0,
                                  1,
                                  true,
                                  36000000,
                                  0,
                                  s->HCLK,
                                  NULL);

    s->PCLK2 = clktree_create_clk("PCLK2",
                                  0,
                                  1,
                                  true,
                                  72000000,
                                  0,
                                  s->HCLK,
                                  NULL);

    /* Peripheral clocks */
    s->PERIPHCLK[STM32_GPIOA] = clktree_create_clk("GPIOA",
                                                   1,
                                                   1,
                                                   false,
                                                   CLKTREE_NO_MAX_FREQ,
                                                   0,
                                                   s->HCLK,
                                                   NULL);
    s->PERIPHCLK[STM32_GPIOB] = clktree_create_clk("GPIOB",
                                                   1,
                                                   1,
                                                   false,
                                                   CLKTREE_NO_MAX_FREQ,
                                                   0,
                                                   s->HCLK,
                                                   NULL);
    s->PERIPHCLK[STM32_GPIOC] = clktree_create_clk("GPIOC",
                                                   1,
                                                   1,
                                                   false,
                                                   CLKTREE_NO_MAX_FREQ,
                                                   0,
                                                   s->HCLK,
                                                   NULL);
    s->PERIPHCLK[STM32_GPIOD] = clktree_create_clk("GPIOD",
                                                   1,
                                                   1,
                                                   false,
                                                   CLKTREE_NO_MAX_FREQ,
                                                   0,
                                                   s->HCLK,
                                                   NULL);
    s->PERIPHCLK[STM32_GPIOE] = clktree_create_clk("GPIOE",
                                                   1,
                                                   1,
                                                   false,
                                                   CLKTREE_NO_MAX_FREQ,
                                                   0,
                                                   s->HCLK,
                                                   NULL);
    s->PERIPHCLK[STM32_GPIOF] = clktree_create_clk("GPIOF",
                                                   1,
                                                   1,
                                                   false,
                                                   CLKTREE_NO_MAX_FREQ,
                                                   0,
                                                   s->HCLK,
                                                   NULL);
    s->PERIPHCLK[STM32_GPIOG] = clktree_create_clk("GPIOG",
                                                   1,
                                                   1,
                                                   false,
                                                   CLKTREE_NO_MAX_FREQ,
                                                   0,
                                                   s->HCLK,
                                                   NULL);
    /* todo: remove AFIO module */
    s->PERIPHCLK[STM32_AFIO] = clktree_create_clk("AFIO",
                                                  1,
                                                  1,
                                                  false,
                                                  CLKTREE_NO_MAX_FREQ,
                                                  0,
                                                  s->PCLK2,
                                                  NULL);

    s->PERIPHCLK[STM32_UART1] = clktree_create_clk("UART1",
                                                   1,
                                                   1,
                                                   false,
                                                   CLKTREE_NO_MAX_FREQ,
                                                   0,
                                                   s->PCLK2,
                                                   NULL);
    s->PERIPHCLK[STM32_UART2] = clktree_create_clk("UART2",
                                                   1,
                                                   1,
                                                   false,
                                                   CLKTREE_NO_MAX_FREQ,
                                                   0,
                                                   s->PCLK1,
                                                   NULL);
    s->PERIPHCLK[STM32_UART3] = clktree_create_clk("UART3",
                                                   1,
                                                   1,
                                                   false,
                                                   CLKTREE_NO_MAX_FREQ,
                                                   0,
                                                   s->PCLK1,
                                                   NULL);
    s->PERIPHCLK[STM32_UART4] = clktree_create_clk("UART4",
                                                   1,
                                                   1,
                                                   false,
                                                   CLKTREE_NO_MAX_FREQ,
                                                   0,
                                                   s->PCLK1,
                                                   NULL);
    s->PERIPHCLK[STM32_UART5] = clktree_create_clk("UART5",
                                                   1,
                                                   1,
                                                   false,
                                                   CLKTREE_NO_MAX_FREQ,
                                                   0,
                                                   s->PCLK1,
                                                   NULL);
}

/* rcc init */
static int stm32_rcc_init(SysBusDevice *dev)
{
   Stm32Rcc* s = FROM_SYSBUS(Stm32Rcc, dev);
   memory_region_init_io(&s->iomem, &stm32_rcc_ops, s, "rcc", 0x400);
   sysbus_init_mmio(dev, &s->iomem);
   sysbus_init_irq(dev, &s->irq);
   stm32_rcc_init_clk(s);
   return 0;
}

static Property stm32_rcc_properties[] = {
   DEFINE_PROP_UINT32("osc_freq", Stm32Rcc, osc_freq, 0),
   DEFINE_PROP_UINT32("osc32_freq", Stm32Rcc, osc32_freq, 0),
   DEFINE_PROP_END_OF_LIST()
};

/* rcc class init */
static void stm32_rcc_class_init(ObjectClass* klass, void* data)
{
   DeviceClass* dc = DEVICE_CLASS(klass);
   SysBusDeviceClass* k = SYS_BUS_DEVICE_CLASS(klass);

   k->init = stm32_rcc_init;
   dc->reset = stm32_rcc_reset;
   dc->props = stm32_rcc_properties;
}

static TypeInfo stm32_rcc_info = {
   .name  = "stm32f2xx_rcc",
   .parent = TYPE_SYS_BUS_DEVICE,
   .instance_size  = sizeof(Stm32Rcc),
   .class_init = stm32_rcc_class_init
};

/* rcc register types */
static void stm32_rcc_register_types(void)
{
   type_register_static(&stm32_rcc_info);
}
type_init(stm32_rcc_register_types)
