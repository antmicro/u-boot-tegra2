/*
 * ulpi_linux.c
 *
 * this file is ulpi_phy_power_on() function taken from
 * arch/arm/mach-tegra/usb_phy.c linux kernel code, slightly 
 * modified for U-Boot by Ant Micro <www.antmicro.com>
 *
 * Original arch/arm/mach-tegra/usb_phy.c Copyrights:
 *   Copyright (C) 2010 Google, Inc.
 *   Copyright (C) 2010 - 2011 NVIDIA Corporation
 *   Erik Gilling <konkers@google.com>
 *   Benoit Goby <benoit@android.com>
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

// ulpi phy power on
static void ulpi_phy_power_on(void) {
        uint32_t val;
        uint32_t base = USBADDR2;
        udelay(1000);

	usb_reset(base);

        val = readl(base + ULPI_TIMING_CTRL_0);
        val |= ULPI_OUTPUT_PINMUX_BYP | ULPI_CLKOUT_PINMUX_BYP;
        writel(val, base + ULPI_TIMING_CTRL_0);

        val = readl(base + USB_SUSP_CTRL);
        val |= ULPI_PHY_ENABLE;
        writel(val, base + USB_SUSP_CTRL);

        val = 0;
        writel(val, base + ULPI_TIMING_CTRL_1);

        val |= ULPI_DATA_TRIMMER_SEL(4);
        val |= ULPI_STPDIRNXT_TRIMMER_SEL(4);
        val |= ULPI_DIR_TRIMMER_SEL(4);
        writel(val, base + ULPI_TIMING_CTRL_1);

        udelay(10);

        val |= ULPI_DATA_TRIMMER_LOAD;
        val |= ULPI_STPDIRNXT_TRIMMER_LOAD;
        val |= ULPI_DIR_TRIMMER_LOAD;
        writel(val, base + ULPI_TIMING_CTRL_1);

        val = ULPI_WAKEUP | ULPI_RD_RW_WRITE | ULPI_PORT(0);
        writel(val, base + ULPI_VIEWPORT);

        if (wait_for_register(base + ULPI_VIEWPORT, ULPI_WAKEUP, 0, 1000)) {
                printf("%s: timeout waiting for ulpi phy wakeup\n", __func__);
                return;
        }

        /* Fix VbusInvalid due to floating VBUS */
	val = ULPI_RUN | ULPI_RD_RW_WRITE | ULPI_PORT(0) | ULPI_ADDR(0x08) | ULPI_DATA_WR(0x40);
        writel(val, base + ULPI_VIEWPORT);
        if (wait_for_register(base + ULPI_VIEWPORT, ULPI_RUN, 0, 1000)) {
		printf("%s: timeout accessing ulpi phy\n", __func__);        
		return;
	}
        val = ULPI_RUN | ULPI_RD_RW_WRITE | ULPI_PORT(0) | ULPI_ADDR(0x0B) | ULPI_DATA_WR(0x80);
        writel(val, base + ULPI_VIEWPORT);
        if (wait_for_register(base + ULPI_VIEWPORT, ULPI_RUN, 0, 1000)) {
                printf("%s: timeout accessing ulpi phy\n", __func__);
                return;
        }

        val = readl(base + USB_PORTSC1);
        val |= USB_PORTSC1_WKOC | USB_PORTSC1_WKDS | USB_PORTSC1_WKCN;
        writel(val, base + USB_PORTSC1);

        val = readl(base + USB_SUSP_CTRL);
        val |= USB_SUSP_CLR;
        writel(val, base + USB_SUSP_CTRL);
        udelay(100);

        val = readl(base + USB_SUSP_CTRL);
        val &= ~USB_SUSP_CLR;
        writel(val, base + USB_SUSP_CTRL);
}
