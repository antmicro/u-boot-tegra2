/*
 *  (C) Copyright 2011
 *  Ant Micro <www.antmicro.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef _BOARD_H_
#define _BOARD_H_

void tegra2_start(void);
static void pllp_init(void);
static void uart_init(void);
static void enable_usb_power(void);
static void smsc_ulpi_init(void);
static void asix_init(void);
void usb_periph_init(void);

#endif	/* BOARD_H */
