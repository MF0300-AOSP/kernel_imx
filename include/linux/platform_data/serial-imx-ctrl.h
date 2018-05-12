/*
 * Copyright (C) 2008 Google, Inc.
 * Author: Nick Pelly <npelly@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_ARCH_IMX_SERIAL_HS_H
#define __ASM_ARCH_IMX_SERIAL_HS_H

#include<linux/serial_core.h>

struct uart_port *imx_uart_get_uart_port(int port_index);
unsigned int imx_uart_tx_empty(struct uart_port *uport);
void imx_uart_request_clock_off(struct uart_port *uport);
void imx_uart_request_clock_on(struct uart_port *uport);
void imx_uart_set_mctrl(struct uart_port *uport,
				    unsigned int mctrl);
#endif
