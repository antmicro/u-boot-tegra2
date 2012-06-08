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

#include <common.h>
#include <ns16550.h>
#include <asm/io.h>
#include <asm/arch/tegra2.h>
#include <asm/arch/sys_proto.h>

#include <asm/arch/clk_rst.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/uart.h>
#include "colibri_tegra2.h"

#include "tegra2_regs.h"

DECLARE_GLOBAL_DATA_PTR;

const struct tegra2_sysinfo sysinfo = {
	CONFIG_TEGRA2_BOARD_STRING
};


static void pllp_init(void) 
{

	uint32_t reg;
        reg = nv_readreg(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_PLLP_BASE_0);

        if (!(reg & PLL_BASE_OVRRIDE)) {

                reg = (PLL_BYPASS | PLL_BASE_OVRRIDE | PLL_DIVP);
                reg |= (((NVRM_PLLP_FIXED_FREQ_KHZ/500) << 8) | PLL_DIVM);
                nv_writereg(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_PLLP_BASE_0, reg);

                reg |= PLL_ENABLE;
                nv_writereg(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_PLLP_BASE_0, reg);

                reg &= ~PLL_BYPASS;
                nv_writereg(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_PLLP_BASE_0, reg);
        }
}

static void uart_init(void)
{
	uint32_t reg;

	reg = nv_readreg(APB_BASE + APB_MISC_PP_TRISTATE_REG_A_0);
	reg = reg & ~(1 << TRISTATE_SDIO1);
	nv_writereg(APB_BASE + APB_MISC_PP_TRISTATE_REG_A_0,reg);

	// mux SDB/SDC to PWM
	reg = nv_readreg(APB_BASE + APB_MISC_PP_PIN_MUX_CTL_D_0);
	reg = reg & ~(SDB_PINMUX_MASK | SDC_PINMUX_MASK);
	reg = reg | (1 << SDB_PINMUX) | (1 << SDC_PINMUX);
	nv_writereg(APB_BASE + APB_MISC_PP_PIN_MUX_CTL_D_0, reg);

	// mux SDIO1 to UARTA ***
	reg = nv_readreg(APB_BASE + APB_MISC_PP_PIN_MUX_CTL_A_0); 
	reg = reg & ~(SDIO1_PINMUX_MASK);
	reg = reg | (3 << SDIO1_PINMUX);
	nv_writereg(APB_BASE + APB_MISC_PP_PIN_MUX_CTL_A_0, reg);

	// disable uart
	reg = nv_readreg(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_RST_DEVICES_L_0);
	reg |= SWR_UARTA_RST;
	nv_writereg(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_RST_DEVICES_L_0, reg);

	// enable clock
	reg = nv_readreg(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_ENB_L_SET_0); 
	reg |= CLK_ENB_UARTA;
	nv_writereg(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_ENB_L_SET_0, reg);

	reg = nv_readreg(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_UART1_0); 
	reg &= 0x3FFFFFFF;
	nv_writereg(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_UART1_0, reg);

	udelay(2);

	// enable uart
	reg = nv_readreg(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_RST_DEVICES_L_0); 
	reg &= ~SWR_UARTA_RST;
	nv_writereg(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_RST_DEVICES_L_0, reg);

}

static void enable_usb_power(void) 
{
	uint32_t reg;

        reg = nv_readreg(APB_MISC_PP_TRISTATE_REG_B);
        reg &= ~(1 << TRISTATE_SPIG);
        nv_writereg(APB_MISC_PP_TRISTATE_REG_B, reg);
	nv_gpio_out(GPIO_PORT_W, 2, 0);
}

static void smsc_ulpi_init(void) 
{
	uint32_t reg;

	//Apply reset to SMSC ULPI chip
        reg = nv_readreg(APB_MISC_PP_TRISTATE_REG_B);
        reg &= ~(1 << TRISTATE_UAC);
        nv_writereg(APB_MISC_PP_TRISTATE_REG_B, reg);
	
	nv_gpio_out(GPIO_PORT_V, 1, 0);
	udelay(5000);
	nv_gpio_out(GPIO_PORT_V, 1, 1);

	//Set up PLLP OUT4 output
        reg = nv_readreg(APB_MISC_PP_TRISTATE_REG_A);
        reg &= ~(1 << TRISTATE_CDEV2);
        nv_writereg(APB_MISC_PP_TRISTATE_REG_A, reg);
        
	reg = nv_readreg(GPIO_PORT_W + GPIO_CNF);
        reg &= ~(1 << 5);
        nv_writereg(GPIO_PORT_W + GPIO_CNF, reg);

        reg = nv_readreg(APB_MISC_PP_PIN_MUX_CTL_C);
        reg &= ~(3 << 4);
        reg |= (3 << 4);
        nv_writereg(APB_MISC_PP_PIN_MUX_CTL_C, reg);
	
	//Configure 24MHz clock for SMSC ULPI
        reg = nv_readreg(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_CLK_OUT_ENB_U_0); 
        reg |= (1 << 29);
        nv_writereg(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_CLK_OUT_ENB_U_0, reg);

        reg = nv_readreg(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_PLLP_OUTB_0); 
        reg |= (8 << 25) | (0 << 24) | (1 << 18);
        nv_writereg(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_PLLP_OUTB_0, reg);
}

static void asix_init(void)
{
	uint32_t reg;

	//Enable ASIX
        reg = nv_readreg(APB_MISC_PP_TRISTATE_REG_A);
        reg &= ~(1 << TRISTATE_DTE);
        nv_writereg(APB_MISC_PP_TRISTATE_REG_A, reg);
	nv_gpio_out(GPIO_PORT_BB, 1, 1);
}

// USB timing
#define USB_PLL_ENABLE_DLY 0x02
#define USB_PLL_STABLE_CNT 0x33
#define USB_PLL_ACTIVE_DLY 0x05
#define USB_PLL_XTAL_FREQ_CNT 0x7f
#define USB_PLL_DIVN 0x3C0
#define USB_PLL_DIVM 0x0D
#define USB_PLL_DIVP 0x00
#define USB_PLL_CPCON 0xC
#define USB_PLL_LFCON 0
#define USB_DEBOUNCE 0x7EF4
#define USB_TRACK_LENGTH 5
#define UTMIP_IDLE_WAIT_DELAY 17
#define UTMIP_ELASTIC_LIMIT 16
#define UTMIP_HS_SYNC_START_DELAY 9

static int wait_for_register(uint32_t addr, uint32_t mask, uint32_t result, int timeout_)
{
        int timeout = timeout_;
        while (timeout-- > 0) {
                if ((nv_readreg(addr) & mask) == result) return 0;
                udelay(2);
        }
        return -1;
}

void usb_reset(uint32_t UsbBase)
{
        uint32_t RegVal = 0;
        if(UsbBase == USBADDR2)
        {

                // enable USB clock
                RegVal= nv_readreg(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0+4);
                RegVal |= (1 << 26);
                nv_writereg(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0+4, RegVal);

                // reset USB
                RegVal= nv_readreg(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_RST_DEVICES_L_0+4);
                if (RegVal & (1 << 26))
                {
                        nv_writereg(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_RST_DEVICES_L_0+4, RegVal); 

                        udelay(2);

                        RegVal = nv_readreg(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_RST_DEVICES_L_0+4);
                        RegVal &= ~(1 << 26);
                        nv_writereg(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_RST_DEVICES_L_0+4, RegVal);

                        udelay(2);
                }

        }
        else if(UsbBase == USBADDR3)
        {
                // enable USB clock
                RegVal = nv_readreg(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0+4);
                RegVal |= (1 << 27);
                nv_writereg(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0+4, RegVal);

                // reset USB
                RegVal = nv_readreg(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_RST_DEVICES_L_0+4);
                if (RegVal & (1 << 27))
                {
                        nv_writereg(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_RST_DEVICES_L_0+4, RegVal);

                        udelay(2);

                        //RegVal= readl(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_RST_DEVICES_L_0+4);
                        RegVal = nv_readreg(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_RST_DEVICES_L_0+4);
                        RegVal &= ~(1 << 27);
                        nv_writereg(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_RST_DEVICES_L_0+4, RegVal);

                        udelay(2);
                }
        }

        // set UTMIP_RESET/UHSIC_RESET
        RegVal = nv_readreg(UsbBase+USB_SUSP_CTRL);
        RegVal |= (1 << 11);
        nv_writereg(UsbBase+USB_SUSP_CTRL, RegVal);

        if(UsbBase == USBADDR3)
        {
                // set UTMIP_PHY_ENB to 1.
                RegVal = nv_readreg(UsbBase+USB_SUSP_CTRL);
                RegVal |= (1 << 12);
                nv_writereg(UsbBase+USB_SUSP_CTRL, RegVal);
        }

}

#include "ulpi_linux.c"

void usb_controller_init(uint32_t addr) {
		int RegVal;
                uint32_t UsbBase = addr;

                /* Stop crystal clock by setting UTMIP_PHY_XTAL_CLOCKEN low */
                RegVal= readl(UsbBase+UTMIP_MISC_CFG1);
                RegVal &= ~(1 << 30);
                writel(RegVal, UsbBase+UTMIP_MISC_CFG1);

                /* Follow the crystal clock disable by >100ns delay. */
                udelay(1);

                /* PLL Delay CONFIGURATION settings
                 * The following parameters control the bring up of the plls:
                 */
                RegVal= readl(UsbBase+UTMIP_MISC_CFG1);
                RegVal &= 0xFFFC003F;
                RegVal |= (USB_PLL_STABLE_CNT << 6);

                RegVal &= 0xFF83FFFF;
                RegVal |= (USB_PLL_ACTIVE_DLY << 18);
                writel(RegVal, UsbBase+UTMIP_MISC_CFG1);

                /* Set PLL enable delay count and Crystal frequency count */
                RegVal= readl(UsbBase+UTMIP_PLL_CFG1);
                RegVal &= 0x08FFFFFF;
                RegVal |= (USB_PLL_ENABLE_DLY << 27);

                RegVal &= 0xFFFFF000;
                RegVal |= (USB_PLL_XTAL_FREQ_CNT);
                writel(RegVal, UsbBase+UTMIP_PLL_CFG1);

                /* Setting the tracking length time. */
                RegVal= readl(UsbBase+UTMIP_BIAS_CFG1);
                RegVal &= 0xFFFFFF07;
                RegVal |= (USB_TRACK_LENGTH << 3);
                writel(RegVal, UsbBase+UTMIP_BIAS_CFG1);

                /* Program Debounce time for VBUS to become valid. */
                RegVal= readl(UsbBase+UTMIP_DEBOUNCE_CFG0);
                RegVal &= 0xFFFF0000;
                RegVal |= USB_DEBOUNCE;
                writel(RegVal, UsbBase+UTMIP_DEBOUNCE_CFG0);

                /* Set UTMIP_FS_PREAMBLE_J to 1 */
                RegVal= readl(UsbBase+UTMIP_TX_CFG0);
                RegVal |= (1 << 19);
                writel(RegVal, UsbBase+UTMIP_TX_CFG0);

                /* Disable Batery charge enabling bit set to '1' for disable */
                RegVal= readl(UsbBase+UTMIP_BAT_CHRG_CFG0);
                RegVal |= (1 << 0);
                writel(RegVal, UsbBase+UTMIP_BAT_CHRG_CFG0);

                /* Set UTMIP_XCVR_LSBIAS_SEL to 0 */
                /* Set UTMIP_XCVR_LSBIAS_SEL to 0 */
                RegVal= readl(UsbBase+UTMIP_XCVR_CFG0);
                RegVal &= ~(1 << 21);
                writel(RegVal, UsbBase+UTMIP_XCVR_CFG0);

                /* Set bit 3 of UTMIP_SPARE_CFG0 to 1 */
                RegVal= readl(UsbBase+UTMIP_SPARE_CFG0);
                RegVal |= (1 << 3);
                writel(RegVal, UsbBase+UTMIP_SPARE_CFG0);

                /* Configure the UTMIP_IDLE_WAIT and UTMIP_ELASTIC_LIMIT
                 * Setting these fields, together with default values of the other
                 * fields, results in programming the registers below as follows:
                 *         UTMIP_HSRX_CFG0 = 0x9168c000
                 *         UTMIP_HSRX_CFG1 = 0x13
                 */

                /* Set PLL enable delay count and Crystal frequency count */
                RegVal= readl(UsbBase+UTMIP_HSRX_CFG0);
                RegVal &= 0xFFF07FFF;
                RegVal |= UTMIP_IDLE_WAIT_DELAY << 15;

                RegVal &= 0xFFFF83FF;
                RegVal |= UTMIP_ELASTIC_LIMIT << 10;
                writel(RegVal, UsbBase+UTMIP_HSRX_CFG0);

                /* Configure the UTMIP_HS_SYNC_START_DLY */
                RegVal= readl(UsbBase+UTMIP_HSRX_CFG1);
                RegVal &= 0xFFFFFFC1;
                RegVal |= UTMIP_HS_SYNC_START_DELAY << 1;
                writel(RegVal, UsbBase+UTMIP_HSRX_CFG1);

                /* Preceed  the crystal clock disable by >100ns delay. */
                udelay(1);

                /* Resuscitate  crystal clock by setting UTMIP_PHY_XTAL_CLOCKEN */
                RegVal= readl(UsbBase+UTMIP_MISC_CFG1);
                RegVal |= (1 << 30);
                writel(RegVal, UsbBase+UTMIP_MISC_CFG1);
}


void usb3_init(void)
{

        int RegVal;
        int loop_count;
        int PhyClkValid;


	uint32_t RegData;
	RegData = (USB_PLL_CPCON << 8) | (USB_PLL_LFCON << 4);
	nv_writereg(NV_PA_CLK_RST_BASE+0x0CC, RegData);
	RegData = (USB_PLL_DIVM << 0) | (USB_PLL_DIVN << 8) | (USB_PLL_DIVP << 20) | (0 << 31) | (1 << 30);
	nv_writereg(NV_PA_CLK_RST_BASE+0x0C0, RegData);

	// initialize USB3 controller
	usb_reset(USBADDR3);
	usb_controller_init(USBADDR3);

        RegVal= readl(USBADDR3+USB_SUSP_CTRL);
        RegVal &= ~(1 << 11);
        writel(RegVal, USBADDR3+USB_SUSP_CTRL);
        loop_count = 100000;
        while (loop_count-- > 0)
        {
	        PhyClkValid = readl(USBADDR3+USB_SUSP_CTRL) & (1 << 7);
	        if (PhyClkValid)
	                break;
	        udelay(1);
        }

        RegVal= readl(USBADDR3+ICUSB_CTRL);
        RegVal &= ~(1 << 3);
        writel(RegVal, USBADDR3+ICUSB_CTRL);

        RegVal= readl(USBADDR3+PORTSC1);
        RegVal &= ~((1 << 31)+(1 << 30)+(1 << 29));
        writel(RegVal, USBADDR3+PORTSC1);

        RegVal= readl(USBADDR3+UTMIP_XCVR_CFG0);
        RegVal &= ~((1 << 18)+(1 << 16)+(1 << 14));
        writel(RegVal, USBADDR3+UTMIP_XCVR_CFG0);

        RegVal= readl(USBADDR3+UTMIP_XCVR_CFG1);
        RegVal &= ~((1 << 4)+(1 << 2)+(1 << 0));
        writel(RegVal, USBADDR3+UTMIP_XCVR_CFG1);

}

int timer_init(void)
{
	reset_timer();
	return 0;
}

#ifdef CONFIG_BOARD_EARLY_INIT_F
int board_early_init_f(void)
{
	pllp_init();

	#if defined(CONFIG_TEGRA2_ENABLE_UARTA)
	uart_init();
	#endif

        tegra2_start();

        return 0;
}
#endif

#define usb2_init() ulpi_phy_power_on()

int board_init(void)
{

	#ifdef CONFIG_USB_EHCI_TEGRA
	if (USB_EHCI_TEGRA_BASE_ADDR == USBADDR2) {
		smsc_ulpi_init();
		asix_init();
		usb2_init();
	} else {
		enable_usb_power();
		usb3_init();
	}
	#endif

	gd->bd->bi_boot_params = (NV_PA_SDRAM_BASE + 0x100);
	gd->bd->bi_arch_number = CONFIG_MACH_TYPE;

	return 0;
}
