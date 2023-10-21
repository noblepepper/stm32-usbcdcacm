/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Provides main entry point.  Initialise subsystems and enter GDB
 * protocol loop.
 */

#include "general.h"
#include "cdcacmwrap.h"

int main(int argc, char **argv)
{
	(void) argc;
	(void) argv;

	clock_setup();
	gpio_setup();
	cdcacm_init();
	usbuart_init();
	while (true) {
		if (usb_data_waiting())
			usart_send_blocking(USART3, usb_recv_blocking());
		if (usart_data_waiting(USART3))
			usb_send_blocking(usart_recv(USART3));
        asm("nop");
	}

	/* Should never get here */
	return 0;
}

