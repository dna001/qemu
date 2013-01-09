/*
 * STM32 Microcontroller TIM module
 *
 * Copyright (C) 2013 Daniel Nygren
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

#include "sysbus.h"
#include "stm32f2xx.h"


/* DEFINITIONS*/
#define TIM_CR1_CEN_BIT 1
#define TIM_CR1_UDIS_BIT 2

/* See the README file for details on these settings. */
//#define DEBUG_STM32_TIM

#ifdef DEBUG_STM32_TIM
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32_TIM: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

struct Stm32Tim {
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    stm32_periph_t periph;
    void *stm32_rcc_prop;

    /* Private */
    MemoryRegion iomem;

    Stm32Rcc *stm32_rcc;

    int timer_index;

    int64_t load_val;
    int64_t time;
    int64_t rate;
    int64_t ticks_per_sec;

    /* Register Values */
    uint32_t
        TIM_CR1,
        TIM_CR2,
        TIM_SR,
        TIM_CNT,
        TIM_PSC,
        TIM_ARR;

    /* Qemu Timer. */
    struct QEMUTimer *timer;

    qemu_irq irq;
    int curr_irq_level;
};


/* HELPER FUNCTIONS */


/* TIMER HANDLERS */
/* Once the receive delay is finished, indicate the USART is finished receiving.
 * This will allow it to receive the next character.  The current character was
 * already received before starting the delay.
 */
static void stm32_timer_tick(void *opaque)
{
    struct Stm32Tim *timer = (struct Stm32Tim *) opaque;

    if (!timer->TIM_ARR) {
        timer->TIM_SR = 0;
        timer->TIM_CNT = 0;
    } else {
        timer->TIM_CNT = timer->load_val;
        timer->time = qemu_get_clock_ns(vm_clock);
    }

    //omap_gp_timer_intr(timer, GPT_OVF_IT);
    //stm32_timer_update(timer);
}


/* REGISTER IMPLEMENTATION */

static void stm32_tim_reset(DeviceState *dev)
{
    Stm32Tim *s = FROM_SYSBUS(Stm32Tim, sysbus_from_qdev(dev));

    s->time = 0;
    s->rate = 0;
    s->ticks_per_sec = 0;
    s->TIM_CR1 = 0;
    s->TIM_CR2 = 0;
    s->TIM_SR = 0;
    s->TIM_CNT = 0;
    s->TIM_PSC = 0;
}


static uint64_t stm32_tim_readw(Stm32Tim *s, target_phys_addr_t offset)
{

    switch (offset) {
        case 0x00: /* CR1 */
            return s->TIM_CR1;
        case 0x04: /* CR2 */
            return s->TIM_CR2;
        case 0x10: /* SR */
            return s->TIM_SR;
        case 0x24: /* CNT */
            return s->TIM_CNT;
        case 0x28: /* PSC */
            return s->TIM_PSC;
        case 0x2C: /* ARR */
            return s->TIM_ARR;
        default:
            STM32_BAD_REG(offset, 4);
            return 0;
    }
}

static void stm32_tim_writew(
        Stm32Tim *s,
        target_phys_addr_t offset,
        uint64_t value)
{
    switch (offset) {
        case 0x00: /* CR1 */
            s->TIM_CR1 = value;
            break;
        case 0x04: /* CR2 */
            s->TIM_CR2 = value;
            break;
        case 0x10: /* SR */
            s->TIM_SR = value;
            break;
        case 0x24: /* CNT */
            s->TIM_CNT = value;
            break;
        case 0x28: /* PSC */
            s->TIM_PSC = value;
            break;
        case 0x2C: /* ARR */
            s->TIM_ARR = value;
            break;
        default:
            STM32_BAD_REG(offset, 4);
            break;
    }
}

static uint64_t stm32_tim_readh(Stm32Tim *s, target_phys_addr_t offset)
{

    switch (offset) {
        case 0x00: /* CR1 */
            return STM32_REG_READH_VALUE(offset, s->TIM_CR1);
        case 0x04: /* CR2 */
            return STM32_REG_READH_VALUE(offset, s->TIM_CR2);
        case 0x10: /* SR */
            return STM32_REG_READH_VALUE(offset, s->TIM_SR);
        case 0x24: /* CNT */
            return STM32_REG_READH_VALUE(offset, s->TIM_CNT);
        case 0x28: /* PSC */
            return STM32_REG_READH_VALUE(offset, s->TIM_PSC);
        case 0x2C: /* ARR */
            return STM32_REG_READH_VALUE(offset, s->TIM_ARR);
        default:
            STM32_BAD_REG(offset, 2);
            return 0;
    }
}

static void stm32_tim_writeh(
        Stm32Tim *s,
        target_phys_addr_t offset,
        uint64_t value)
{
    switch (offset) {
        case 0x00: /* CR1 */
            s->TIM_CR1 = STM32_REG_WRITEH_VALUE(offset, 0, value);
            break;
        case 0x04: /* CR2 */
            s->TIM_CR2 = STM32_REG_WRITEH_VALUE(offset, 0, value);
            break;
        case 0x10: /* SR */
            s->TIM_SR = STM32_REG_WRITEH_VALUE(offset, 0, value);
            break;
        case 0x24: /* CNT */
            s->TIM_CNT = STM32_REG_WRITEH_VALUE(offset, 0, value);
            break;
        case 0x28: /* PSC */
            s->TIM_PSC = STM32_REG_WRITEH_VALUE(offset, 0, value);
            break;
        case 0x2C: /* ARR */
            s->TIM_ARR = STM32_REG_WRITEH_VALUE(offset, 0, value);
            break;
        default:
            STM32_BAD_REG(offset, 2);
            break;
    }
}

static uint64_t stm32_tim_read(void *opaque, target_phys_addr_t offset,
                          unsigned size)
{
    Stm32Tim *s = (Stm32Tim *)opaque;

    switch(size) {
        case HALFWORD_ACCESS_SIZE:
            return stm32_tim_readh(s, offset);
        case WORD_ACCESS_SIZE:
            return stm32_tim_readw(s, offset);
        default:
            STM32_BAD_REG(offset, size);
            return 0;
    }
}

static void stm32_tim_write(void *opaque, target_phys_addr_t offset,
                       uint64_t value, unsigned size)
{
    Stm32Tim *s = (Stm32Tim *)opaque;

    stm32f2xx_rcc_check_periph_clk((Stm32Rcc *)s->stm32_rcc, s->periph);

    switch(size) {
        case HALFWORD_ACCESS_SIZE:
            stm32_tim_writeh(s, offset, value);
            break;
        case WORD_ACCESS_SIZE:
            stm32_tim_writew(s, offset, value);
            break;
        default:
            STM32_BAD_REG(offset, size);
            break;
    }
}

static const MemoryRegionOps stm32_tim_ops = {
    .read = stm32_tim_read,
    .write = stm32_tim_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};


/* DEVICE INITIALIZATION */

static int stm32_tim_init(SysBusDevice *dev)
{
    //qemu_irq *clk_irq;
    Stm32Tim *s = FROM_SYSBUS(Stm32Tim, dev);

    s->stm32_rcc = (Stm32Rcc *)s->stm32_rcc_prop;

    memory_region_init_io(&s->iomem, &stm32_tim_ops, s,
                          "tim", 0x03ff);
    sysbus_init_mmio(dev, &s->iomem);

    sysbus_init_irq(dev, &s->irq);

    s->timer =
          qemu_new_timer_ns(vm_clock,(QEMUTimerCB *)stm32_timer_tick, s);

    /* Register handlers to handle updates to the USART's peripheral clock. */
    /*clk_irq =
          qemu_allocate_irqs(stm32_tim_clk_irq_handler, (void *)s, 1);
    stm32f2xx_rcc_set_periph_clk_irq(s->stm32_rcc, s->periph, clk_irq[0]);*/

    stm32_tim_reset((DeviceState *)s);

    return 0;
}

static Property stm32_tim_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32Tim, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_PTR("stm32f2xx_rcc", Stm32Tim, stm32_rcc_prop),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32_tim_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32_tim_init;
    dc->reset = stm32_tim_reset;
    dc->props = stm32_tim_properties;
}

static TypeInfo stm32_tim_info = {
    .name  = "stm32f2xx_tim",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32Tim),
    .class_init = stm32_tim_class_init
};

static void stm32_tim_register_types(void)
{
    type_register_static(&stm32_tim_info);
}

type_init(stm32_tim_register_types)
