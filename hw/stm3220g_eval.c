/*
 * STM 3220G-EVAL
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

#include "stm32f2xx.h"
#include "sysbus.h"
#include "arm-misc.h"
#include "devices.h"
#include "console.h"
#include "sysemu.h"
#include "boards.h"


struct stm3220g_eval {
    Stm32 *stm32;
    bool last_button_pressed;
    qemu_irq button_irq;
};




static void led_irq_handler(void *opaque, int n, int level)
{
    /* There should only be one IRQ for the LED */
    assert(n == 0);

    /* Assume that the IRQ is only triggered if the LED has changed state.
     * If this is not correct, we may get multiple LED Offs or Ons in a row.
     */
    switch (level) {
        case 0:
            printf("LED Off\n");
            break;
        case 1:
            printf("LED On\n");
            break;
    }
}

static void stm3220g_eval_key_event(void *opaque, int keycode)
{
    struct stm3220g_eval* s;
    bool make;
    int core_keycode;

    s = (struct stm3220g_eval*) opaque;

    if((keycode & 0x80) == 0) {
        make = true;
        core_keycode = keycode;
    } else {
        make = false;
        core_keycode = keycode & 0x7f;
    }

    /* Responds when a "B" key press is received.
     * Inside the monitor, you can type "sendkey b"
     */
    if(core_keycode == 0x30) {
        if(make) {
            if(!s->last_button_pressed) {
                qemu_irq_raise(s->button_irq);
                s->last_button_pressed = true;
            }
        } else {
            if(s->last_button_pressed) {
                qemu_irq_lower(s->button_irq);
                s->last_button_pressed = false;
            }
        }
    }
    return;

}


static void stm3220g_eval_init(ram_addr_t ram_size,
                     const char *boot_device,
                     const char *kernel_filename, const char *kernel_cmdline,
                     const char *initrd_filename, const char *cpu_model)
{
    qemu_irq *led_irq;
    struct stm3220g_eval* s;
    Stm32Gpio *stm32_gpio[STM32_GPIO_COUNT];
    Stm32Uart *stm32_uart[STM32_UART_COUNT];
    Stm32Tim  *stm32_tim[STM32_TIM_COUNT];

    s = (struct stm3220g_eval*) g_malloc0(sizeof(struct stm3220g_eval));

    stm32f2xx_init(/*flash_size*/0x0001ffff,
               /*ram_size*/0x00004fff,
               kernel_filename,
               stm32_gpio,
               stm32_uart,
               stm32_tim,
               8000000,
               32768);

    /* Connect LED to GPIO C pin 12 */
    led_irq = qemu_allocate_irqs(led_irq_handler, NULL, 1);
    qdev_connect_gpio_out((DeviceState *)stm32_gpio[STM32_GPIOC_INDEX], 12, led_irq[0]);

    /* Connect button to GPIO A pin 0 */
    s->button_irq = qdev_get_gpio_in((DeviceState *)stm32_gpio[STM32_GPIOA_INDEX], 0);
    qemu_add_kbd_event_handler(stm3220g_eval_key_event, s);

    /* Connect RS232 to UART */
    stm32f2xx_uart_connect(
            stm32_uart[STM32_UART3_INDEX],
            serial_hds[0],
            STM32_USART3_NO_REMAP);
 }

static QEMUMachine stm3220g_eval_machine = {
    .name = "stm3220g-eval",
    .desc = "STM 3220G-EVAL",
    .init = stm3220g_eval_init,
};


static void stm3220g_eval_machine_init(void)
{
    qemu_register_machine(&stm3220g_eval_machine);
}

machine_init(stm3220g_eval_machine_init);
