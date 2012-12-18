/*
 * STM32 Microcontroller GPIO (General Purpose I/O) module
 *
 * Copyright (C) 2010 Andre Beckus
 * Copyright (C) 2012
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

#define GPIOx_MODE_OFFSET  0x00
#define GPIOx_TYPE_OFFSET  0x04
#define GPIOx_SPEED_OFFSET 0x08
#define GPIOx_PUPD_OFFSET  0x0C
#define GPIOx_ID_OFFSET    0x10
#define GPIOx_OD_OFFSET    0x14
#define GPIOx_BSR_OFFSET   0x18
#define GPIOx_LOCK_OFFSET  0x1C
#define GPIOx_AFL_OFFSET   0x20
#define GPIOx_AFH_OFFSET   0x24

#define GPIOx_CRL_INDEX 0
#define GPIOx_CRH_INDEX 1



struct Stm32Gpio {
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    stm32_periph_t periph;
    void *stm32_rcc_prop;

    /* Private */
    MemoryRegion iomem;

    Stm32Rcc *stm32_rcc;

   uint32_t mode;   /* mode register */
   uint32_t otype;  /* output type register */
   uint32_t ospeed; /* output speed register */
   uint32_t pupd;   /* pull-up/down register */
   uint32_t id;     /* input data register */
   uint32_t od;     /* output data register */
   uint32_t bsr;    /* bit set/reset register */
   uint32_t lock;   /* lock register */
   uint32_t afl;    /* alternate function low */
   uint32_t afh;    /* alternate function high*/

    /* CRL = 0
     * CRH = 1
     */
   uint32_t GPIOx_CRy[2];

    /* 0 = input
     * 1 = output
     */
    uint16_t dir_mask;

    uint32_t GPIOx_ODR;

    /* IRQs used to communicate with the machine implementation.
     * There is one IRQ for each pin.  Note that for pins configured
     * as inputs, the output IRQ state has no meaning.  Perhaps
     * the output should be updated to match the input in this case....
     */
    qemu_irq out_irq[STM32_GPIO_PIN_COUNT];

    uint16_t in;

    /* EXTI IRQ to notify on input change - there is one EXTI IRQ per pin. */
    qemu_irq exti_irq[STM32_GPIO_PIN_COUNT];
};



/* CALLBACKs */

/* Trigger fired when a GPIO input pin changes state (based
 * on an external stimulus from the machine).
 */
static void stm32_gpio_in_trigger(void *opaque, int irq, int level)
{
    Stm32Gpio *s = opaque;
    unsigned pin = irq;

    assert(pin < STM32_GPIO_PIN_COUNT);

    /* Only proceed if the pin has actually changed value (the trigger
     * will fire when the IRQ is set, even if it set to the same level). */
    if(GET_BIT_VALUE(s->in, pin) != level) {
        /* Update internal pin state. */
        CHANGE_BIT(s->in, pin, level);

        /* Propagate the trigger to the EXTI module. */
        qemu_set_irq(s->exti_irq[pin], level);
    }
}



/* HELPER FUNCTIONS */

/* Gets the four configuration bits for the pin from the CRL or CRH
 * register.
 */
static uint8_t stm32f2xx_gpio_get_pin_config(Stm32Gpio *s, unsigned pin) {
    unsigned reg_index, reg_pin;
    unsigned reg_start_bit;

    /* BIG FIXME! */

    assert(pin < STM32_GPIO_PIN_COUNT);

    /* Determine the register (CRL or CRH). */
    reg_index = pin / 8;
    assert((reg_index == GPIOx_CRL_INDEX) || (reg_index == GPIOx_CRH_INDEX));

    /* Get the pin within the register. */
    reg_pin = pin % 8;

    /* Get the start position of the config bits in the register (each
     * pin config takes 4 bits). */
    reg_start_bit = reg_pin * 4;

    return (s->GPIOx_CRy[reg_index] >> reg_start_bit) & 0xf;
}





/* REGISTER IMPLEMENTATION */
#if 0
/* we don't have any CRL or CRH register !!! */

/* Update the CRL or CRH Configuration Register */
static void stm32_gpio_GPIOx_CRy_write(Stm32Gpio *s, int cr_index,
                                        uint32_t new_value, bool init)
{
    unsigned pin, pin_dir;

    assert((cr_index == GPIOx_CRL_INDEX) || (cr_index == GPIOx_CRH_INDEX));

    s->GPIOx_CRy[cr_index] = new_value;

    /* Update the direction mask */
    for(pin=0; pin < STM32_GPIO_PIN_COUNT; pin++) {
        pin_dir = stm32f2xx_gpio_get_mode_bits(s, pin);
        /* If the mode is 0, the pin is input.  Otherwise, it
         * is output.
         */
        CHANGE_BIT(s->dir_mask, pin, pin_dir);
    }
}
#endif

/* Write the Output Data Register.
 * Propagates the changes to the output IRQs.
 * Perhaps we should also update the input to match the output for
 * pins configured as outputs... */
static void stm32_gpio_GPIOx_ODR_write(Stm32Gpio *s,
                                       uint32_t new_value, bool init)
{
    uint32_t old_value;
    uint16_t changed, changed_out;
    unsigned pin;

    old_value = s->od;

    /* Update register value.  Per documentation, the upper 16 bits
     * always read as 0. */
    s->od = new_value & 0x0000ffff;

    /* Get pins that changed value */
    changed = old_value ^ new_value;

    /* Get changed pins that are outputs - we will not touch input pins */
    changed_out = changed & s->dir_mask;

    if (changed_out) {
        for (pin = 0; pin < STM32_GPIO_PIN_COUNT; pin++) {
            /* If the value of this pin has changed, then update
             * the output IRQ.
             */
            if (IS_BIT_SET(changed_out, pin)) {
                qemu_set_irq(
                        s->out_irq[pin],
                        IS_BIT_SET(s->od, pin) ? 1 : 0);
            }
        }
    }
}

/* Write the Bit Set/Reset Register.
 * Setting a bit sets or resets the corresponding bit in the output
 * register.  The lower 16 bits perform resets, and the upper 16 bits
 * perform sets.  Register is write-only and so does not need to store
 * a value.
 */
static void stm32_gpio_GPIOx_BSRR_write(Stm32Gpio *s, uint32_t new_value)
{
    uint32_t new_ODR;

    new_ODR = s->GPIOx_ODR;

    /* Perform sets with upper halfword. */
    new_ODR |= new_value & 0x0000ffff;

    /* Perform resets. */
    new_ODR &= ~(new_value >> 16) & 0x0000ffff;

    stm32_gpio_GPIOx_ODR_write(s, new_value, false);
}

static uint64_t stm32_gpio_readw(Stm32Gpio *s, target_phys_addr_t offset)
{

   /*
     port 0-8 (a-i)
     int port;
     port = (int) ((addr & 0x0000FF00) << 10);
     offset within gpio block - each block registered separately,
     so no need to calculate offset.
     target_phys_addr_t offset;
     offset = (addr & 0x000000FF);
   */

    switch (offset) {
    case GPIOx_MODE_OFFSET:
       return s->mode;
       break;
    case GPIOx_TYPE_OFFSET:
       return s->otype;
       break;
    case GPIOx_SPEED_OFFSET:
       return s->ospeed;
       break;
    case GPIOx_PUPD_OFFSET:
       return s->pupd;
       break;
    case GPIOx_ID_OFFSET:
       return s->id;
       break;
    case GPIOx_OD_OFFSET:
       return s->od;
       break;
    case GPIOx_BSR_OFFSET:
       return 0x00000000; /* read returns zero */
       break;
    case GPIOx_LOCK_OFFSET:
       return s->lock;
       break;
    case GPIOx_AFL_OFFSET:
       return s->afl;
       break;
    case GPIOx_AFH_OFFSET:
       return s->afh;
       break;
    default:
       STM32_BAD_REG(offset, WORD_ACCESS_SIZE);
       break;
    }
#if 0

        case GPIOx_CRL_OFFSET: /* GPIOx_CRL */
            return s->GPIOx_CRy[GPIOx_CRL_INDEX];
        case GPIOx_CRH_OFFSET: /* GPIOx_CRH */
            return s->GPIOx_CRy[GPIOx_CRH_INDEX];
        case GPIOx_IDR_OFFSET:
            return s->in;
        case GPIOx_ODR_OFFSET:
            return s->GPIOx_ODR;
        /* Note that documentation says BSRR and BRR are write-only, but reads
         * work on real hardware.  We follow the documentation.*/
        case GPIOx_BSRR_OFFSET: /* GPIOC_BSRR */
            STM32_WO_REG(offset);
            return 0;
        case GPIOx_BRR_OFFSET: /* GPIOC_BRR */
            STM32_WO_REG(offset);
            return 0;
        case GPIOx_LCKR_OFFSET: /* GPIOx_LCKR */
            /* Locking is not yet implemented */
            return 0;
        default:
            STM32_BAD_REG(offset, WORD_ACCESS_SIZE);
            return 0;
    }
#endif
}

static void stm32_gpio_writew(Stm32Gpio *s, target_phys_addr_t offset,
                          uint64_t value)
{
   /*
     port 0-8 (a-i)
     int port;
     port = ((addr & 0x0000FF00) >> 10);
     offset within gpio block
     target_phys_addr_t offset;
     offset = (addr & 0x000000FF);
   */
   switch (offset) {
   case GPIOx_MODE_OFFSET:
      s->mode = value;
      break;
   case GPIOx_TYPE_OFFSET:
      s->otype = value;
      break;
   case GPIOx_SPEED_OFFSET:
      s->ospeed = value;
      break;
   case GPIOx_PUPD_OFFSET:
      s->pupd = value;
      break;
   case GPIOx_ID_OFFSET:
      /* this register is read-only, but let's keep writes for now. */
      s->id = value;
      break;
   case GPIOx_OD_OFFSET:
      stm32_gpio_GPIOx_ODR_write(s, value, false);
      //s->port[port].od = value;
      break;
   case GPIOx_BSR_OFFSET:
      stm32_gpio_GPIOx_BSRR_write(s, value);
      //s->port[port].bsr = value;
      break;
   case GPIOx_LOCK_OFFSET:
      s->lock = value;
      break;
   case GPIOx_AFL_OFFSET:
      s->afl = value;
      break;
   case GPIOx_AFH_OFFSET:
      s->afh = value;
      break;
   default:
      STM32_BAD_REG(offset, WORD_ACCESS_SIZE);
      break;
   }
# if 0
        case GPIOx_CRL_OFFSET: /* GPIOx_CRL */
            stm32_gpio_GPIOx_CRy_write(s, GPIOx_CRL_INDEX, value, false);
            break;
        case GPIOx_CRH_OFFSET: /* GPIOx_CRH */
            stm32_gpio_GPIOx_CRy_write(s, GPIOx_CRH_INDEX, value, false);
            break;
        case GPIOx_IDR_OFFSET:
            STM32_RO_REG(offset);
            break;
        case GPIOx_ODR_OFFSET: /* GPIOx_ODR */
            stm32_gpio_GPIOx_ODR_write(s, value, false);
            break;
        case GPIOx_BSRR_OFFSET: /* GPIOx_BSRR */
            stm32_gpio_GPIOx_BSRR_write(s, value);
            break;
        case GPIOx_BRR_OFFSET: /* GPIOx_BRR */
            stm32_gpio_GPIOx_BRR_write(s, value);
            break;
        case GPIOx_LCKR_OFFSET: /* GPIOx_LCKR */
            /* Locking is not implemented */
            STM32_NOT_IMPL_REG(offset, 4);
            break;
        default:
            STM32_BAD_REG(offset, 4);
            break;
#endif
}



static uint64_t stm32_gpio_read(void *opaque, target_phys_addr_t offset,
                          unsigned size)
{
    Stm32Gpio *s = (Stm32Gpio *)opaque;

    switch(size) {
        case WORD_ACCESS_SIZE:
            return stm32_gpio_readw(s, offset);
        default:
            STM32_BAD_REG(offset, size);
            return 0;
    }
}

static void stm32_gpio_write(void *opaque, target_phys_addr_t offset,
                       uint64_t value, unsigned size)
{
    Stm32Gpio *s = (Stm32Gpio *)opaque;

    printf("gpio write\n");
    stm32f2xx_rcc_check_periph_clk((Stm32Rcc *)s->stm32_rcc, s->periph);

    switch(size) {
        case WORD_ACCESS_SIZE:
            stm32_gpio_writew(s, offset, value);
            break;
        default:
            STM32_BAD_REG(offset, size);
            break;
    }
}

static const MemoryRegionOps stm32_gpio_ops = {
    .read = stm32_gpio_read,
    .write = stm32_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32_gpio_reset(DeviceState *dev)
{
    Stm32Gpio *s = FROM_SYSBUS(Stm32Gpio, sysbus_from_qdev(dev));

    //stm32_gpio_GPIOx_CRy_write(s, GPIOx_CRL_INDEX, 0x44444444, true);
    //stm32_gpio_GPIOx_CRy_write(s, GPIOx_CRH_INDEX, 0x44444444, true);
    //stm32_gpio_GPIOx_ODR_write(s, 0x00000000, true);

    stm32_gpio_GPIOx_ODR_write(s, 0x00000000, true);
    s->mode = 0;
    s->otype = 0;
    s->ospeed = 0;
    s->pupd = 0;
    s->id = 0;
    s->od = 0;
    s->bsr = 0;
    s->lock = 0;
    s->afl = 0;
    s->afh = 0;

    /* reset values */
    s->mode   = 0xA8000000; /* only for port A, how? */ 
    s->mode   = 0x00000280; /* only for port B, how? */ 
    s->ospeed = 0x000000C0; /* only for port B, how? */
    s->pupd   = 0x64000000; /* only for port A, how? */
    s->pupd   = 0x00000100; /* only for port B, how? */
}






/* PUBLIC FUNCTIONS */

uint8_t stm32f2xx_gpio_get_config_bits(Stm32Gpio *s, unsigned pin) {
    return (stm32f2xx_gpio_get_pin_config(s, pin) >> 2) & 0x3;
}

uint8_t stm32f2xx_gpio_get_mode_bits(Stm32Gpio *s, unsigned pin) {
    return stm32f2xx_gpio_get_pin_config(s, pin) & 0x3;
}

void stm32f2xx_gpio_set_exti_irq(Stm32Gpio *s, unsigned pin, qemu_irq exti_irq)
{
    assert(pin < STM32_GPIO_PIN_COUNT);

    s->exti_irq[pin] = exti_irq;
}






/* DEVICE INITIALIZATION */

static int stm32_gpio_init(SysBusDevice *dev)
{
    unsigned pin;
    Stm32Gpio *s = FROM_SYSBUS(Stm32Gpio, dev);

    s->stm32_rcc = (Stm32Rcc *)s->stm32_rcc_prop;

    memory_region_init_io(&s->iomem, &stm32_gpio_ops, s,
                          "gpio", 0x03ff);
    sysbus_init_mmio(dev, &s->iomem);

    qdev_init_gpio_in(&dev->qdev, stm32_gpio_in_trigger, STM32_GPIO_PIN_COUNT);
    qdev_init_gpio_out(&dev->qdev, s->out_irq, STM32_GPIO_PIN_COUNT);

    for(pin = 0; pin < STM32_GPIO_PIN_COUNT; pin++) {
        stm32f2xx_gpio_set_exti_irq(s, pin, NULL);
    }

    return 0;
}

static Property stm32_gpio_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32Gpio, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_PTR("stm32f2xx_rcc", Stm32Gpio, stm32_rcc_prop),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32_gpio_init;
    dc->reset = stm32_gpio_reset;
    dc->props = stm32_gpio_properties;
}

static TypeInfo stm32_gpio_info = {
    .name  = "stm32f2xx_gpio",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32Gpio),
    .class_init = stm32_gpio_class_init
};

static void stm32_gpio_register_types(void)
{
    type_register_static(&stm32_gpio_info);
}

type_init(stm32_gpio_register_types)
