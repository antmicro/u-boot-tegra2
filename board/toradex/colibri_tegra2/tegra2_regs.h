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

#ifndef __TEGRA2_REGS_H__
#define __TEGRA2_REGS_H__

#define nv_readreg(a)           *((const volatile uint32_t *)(a))
#define nv_writereg(a,val)      *((volatile uint32_t *)(a)) = (val)

#define CLK_RST_CONTROLLER_RST_DEVICES_L_0_SWR_UARTA_RST 		(1 << 6)
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0_CLK_ENB_UARTA 		(1 << 6)
#define CLK_RST_CONTROLLER_CLK_SOURCE_UARTA_0_UARTA_CLK_SRC_MASK 	(0x3 << 30)
#define CLK_RST_CONTROLLER_CLK_SOURCE_UARTA_0_UARTA_CLK_SRC_PLLP_OUT0 	(0 << 30)
#define CLK_RST_CONTROLLER_PLLP_BASE_0_PLLP_ENABLE 			(1 << 30)
#define CLK_RST_CONTROLLER_PLLP_BASE_0_PLLP_BYPASS 			(1 << 31)
#define CLK_RST_CONTROLLER_PLLP_BASE_0_PLLP_REF_DIS_REF_ENABLE 		0
#define CLK_RST_CONTROLLER_PLLP_BASE_0_PLLP_REF_DIS_SHIFT 		29
#define CLK_RST_CONTROLLER_PLLP_BASE_0_PLLP_BASE_OVRRIDE  		(1<< 28)
#define CLK_RST_CONTROLLER_PLLP_BASE_0_PLLP_DIVP_SHIFT                  20 // 3 bits
#define CLK_RST_CONTROLLER_PLLP_BASE_0_PLLP_DIVN_SHIFT                  8  // 10 bits
#define CLK_RST_CONTROLLER_PLLP_BASE_0_PLLP_DIVM_SHIFT                  0  // 5 bits
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0           			0x10
#define CLK_RST_CONTROLLER_RST_DEVICES_L_0          			0x4
#define CLK_RST_CONTROLLER_OSC_CTRL_0                			0x50

#define CLK_RST_CONTROLLER_CLK_SOURCE_UART1_0 	0x178
#define CLK_RST_CONTROLLER_CLK_ENB_L_SET_0 	0x320
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_U_0 	0x18
#define CLK_RST_CONTROLLER_PLLP_OUTB_0 		0xA8
#define CLK_RST_CONTROLLER_PLLP_BASE_0 		0xA0

#define APB_MISC_PP_TRISTATE_REG_A_0 		0x14
#define APB_MISC_PP_PIN_MUX_CTL_D_0 		0x8C
#define APB_MISC_PP_PIN_MUX_CTL_A_0 		0x80


#define FLOW_CTLR_HALT_COP_EVENTS_uSEC   	(1 << 25)
#define FLOW_CTLR_HALT_COP_EVENTS_MODE_SHIFT 	29
#define FLOW_CTLR_HALT_COP_EVENTS_MODE_MASK 	(0x7 << FLOW_CTLR_HALT_COP_EVENTS_0_MODE_SHIFT)
#define FLOW_CTLR_HALT_COP_EVENTS_ZERO_SHIFT 	0
#define FLOW_CTRL_HALT_COP_EVENTS_ZERO_MASK 	0xFF
#define FLOW_PA_BASE        			0x60007000 
#define FLOW_CTLR_HALT_COP_EVENTS               0x04

#define APB_MISC_PP_PULLUPDOWN_REG_E_0          0xb0

#define APB_MISC_PP_PULLUPDOWN_REG_E_0_SDIO1_PU_PD_NORMAL       0x0
#define APB_MISC_PP_PULLUPDOWN_REG_E_0_SDIO1_PU_PD_PULL_DOWN	0x1
#define APB_MISC_PP_PULLUPDOWN_REG_E_0_SDIO1_PU_PD_PULL_UP      0x2
#define APB_MISC_PP_PULLUPDOWN_REG_E_0_SDIO1_PU_PD_RSVD         0x3

#define BIT(x) 			(1 << x)

#define SDB_PINMUX 		10
#define SDC_PINMUX 		14
#define SDB_PINMUX_MASK 	(0x3 << SDB_PINMUX)
#define SDC_PINMUX_MASK 	(0x3 << SDC_PINMUX)

#define SDIO1_PINMUX 		30
#define SDIO1_PINMUX_MASK 	(0x3 << SDIO1_PINMUX)

#define APB_BASE 		0x70000000
#define GPIO_BASE	 	0x6000D000

#define APB_MISC_PP_TRISTATE_REG_A 	APB_BASE+0x0014
#define APB_MISC_PP_TRISTATE_REG_B 	APB_BASE+0x0018
#define APB_MISC_PP_TRISTATE_REG_C 	APB_BASE+0x001C
#define APB_MISC_PP_TRISTATE_REG_D 	APB_BASE+0x0020
#define APB_MISC_PP_PIN_MUX_CTL_A	APB_BASE+0x0080
#define APB_MISC_PP_PIN_MUX_CTL_B	APB_BASE+0x0084
#define APB_MISC_PP_PIN_MUX_CTL_C	APB_BASE+0x0088
#define APB_MISC_PP_PIN_MUX_CTL_D	APB_BASE+0x008C
#define APB_MISC_PP_PIN_MUX_CTL_E	APB_BASE+0x0090
#define APB_MISC_PP_PIN_MUX_CTL_F	APB_BASE+0x0094
#define APB_MISC_PP_PIN_MUX_CTL_G	APB_BASE+0x0098
#define APB_MISC_PP_PIN_MUX_CTL_H	APB_BASE+0x009C

//TRISTATE_REG_A_GROUP
#define TRISTATE_SDIO1 	30
#define TRISTATE_DTE 	15
#define TRISTATE_CDEV2 	5

//TRISTATE_REG_B_GROUP
#define TRISTATE_UAC 	20
#define TRISTATE_SPIH 	17
#define TRISTATE_SPIG 	16

#define GPIO_CONTROLLER_0 GPIO_BASE+0x0000
#define GPIO_CONTROLLER_1 GPIO_BASE+0x0080
#define GPIO_CONTROLLER_2 GPIO_BASE+0x0100
#define GPIO_CONTROLLER_3 GPIO_BASE+0x0180
#define GPIO_CONTROLLER_4 GPIO_BASE+0x0200
#define GPIO_CONTROLLER_5 GPIO_BASE+0x0280
#define GPIO_CONTROLLER_6 GPIO_BASE+0x0300

#define GPIO_PORT_A GPIO_CONTROLLER_0+0x0000
#define GPIO_PORT_B GPIO_CONTROLLER_0+0x0004
#define GPIO_PORT_C GPIO_CONTROLLER_0+0x0008
#define GPIO_PORT_D GPIO_CONTROLLER_0+0x000C
#define GPIO_PORT_E GPIO_CONTROLLER_1+0x0000
#define GPIO_PORT_F GPIO_CONTROLLER_1+0x0004
#define GPIO_PORT_G GPIO_CONTROLLER_1+0x0008
#define GPIO_PORT_H GPIO_CONTROLLER_1+0x000C
#define GPIO_PORT_I GPIO_CONTROLLER_2+0x0000
#define GPIO_PORT_J GPIO_CONTROLLER_2+0x0004
#define GPIO_PORT_K GPIO_CONTROLLER_2+0x0008
#define GPIO_PORT_L GPIO_CONTROLLER_2+0x000C
#define GPIO_PORT_M GPIO_CONTROLLER_3+0x0000
#define GPIO_PORT_N GPIO_CONTROLLER_3+0x0004
#define GPIO_PORT_O GPIO_CONTROLLER_3+0x0008
#define GPIO_PORT_P GPIO_CONTROLLER_3+0x000C
#define GPIO_PORT_Q GPIO_CONTROLLER_4+0x0000
#define GPIO_PORT_R GPIO_CONTROLLER_4+0x0004
#define GPIO_PORT_S GPIO_CONTROLLER_4+0x0008
#define GPIO_PORT_T GPIO_CONTROLLER_4+0x000C
#define GPIO_PORT_U GPIO_CONTROLLER_5+0x0000
#define GPIO_PORT_V GPIO_CONTROLLER_5+0x0004
#define GPIO_PORT_W GPIO_CONTROLLER_5+0x0008
#define GPIO_PORT_X GPIO_CONTROLLER_5+0x000C
#define GPIO_PORT_Y GPIO_CONTROLLER_6+0x0000
#define GPIO_PORT_Z GPIO_CONTROLLER_6+0x0004
#define GPIO_PORT_AA GPIO_CONTROLLER_6+0x0008
#define GPIO_PORT_BB GPIO_CONTROLLER_6+0x000C

#define GPIO_CNF 	0x00
#define GPIO_OE 	0x10
#define GPIO_OUT 	0x20
#define GPIO_IN 	0x30


#define USB1_LEGACY_CTRL        0x410
#define USB_SUSP_CTRL           0x400

#define ULPI_VIEWPORT           0x170
#define   ULPI_WAKEUP           (1 << 31)
#define   ULPI_RUN              (1 << 30)
#define   ULPI_RD_RW_WRITE      (1 << 29)
#define   ULPI_RD_RW_READ       (0 << 29)
#define   ULPI_PORT(x)          (((x) & 0x7) << 24)
#define   ULPI_ADDR(x)          (((x) & 0xff) << 16)
#define   ULPI_DATA_RD(x)       (((x) & 0xff) << 8)
#define   ULPI_DATA_WR(x)       (((x) & 0xff) << 0)

#define USB_PORTSC1             0x184
#define   USB_PORTSC1_PTS(x)    (((x) & 0x3) << 30)
#define   USB_PORTSC1_PSPD(x)   (((x) & 0x3) << 26)
#define   USB_PORTSC1_PHCD      (1 << 23)
#define   USB_PORTSC1_WKOC      (1 << 22)
#define   USB_PORTSC1_WKDS      (1 << 21)
#define   USB_PORTSC1_WKCN      (1 << 20)
#define   USB_PORTSC1_PTC(x)    (((x) & 0xf) << 16)
#define   USB_PORTSC1_PP        (1 << 12)
#define   USB_PORTSC1_LS(x)     (((x) & 0x3) << 10)
#define   USB_PORTSC1_SUSP      (1 << 7)
#define   USB_PORTSC1_PE        (1 << 2)
#define   USB_PORTSC1_CCS       (1 << 0)

#define USB_SUSP_CTRL           0x400
#define   USB_WAKE_ON_CNNT_EN_DEV       (1 << 3)
#define   USB_WAKE_ON_DISCON_EN_DEV     (1 << 4)
#define   USB_SUSP_CLR          (1 << 5)
#define   USB_PHY_CLK_VALID     (1 << 7)
#define   UTMIP_RESET           (1 << 11)
#define   UHSIC_RESET           (1 << 11)
#define   UTMIP_PHY_ENABLE      (1 << 12)
#define   UHSIC_PHY_ENABLE      (1 << 12)
#define   ULPI_PHY_ENABLE       (1 << 13)
#define   USB_SUSP_SET          (1 << 14)
#define   USB_WAKEUP_DEBOUNCE_COUNT(x)  (((x) & 0x7) << 16)

#define ULPI_TIMING_CTRL_0      0x424
#define   ULPI_CLOCK_OUT_DELAY(x)       ((x) & 0x1F)
#define   ULPI_OUTPUT_PINMUX_BYP        (1 << 10)
#define   ULPI_CLKOUT_PINMUX_BYP        (1 << 11)
#define   ULPI_SHADOW_CLK_LOOPBACK_EN   (1 << 12)
#define   ULPI_SHADOW_CLK_SEL           (1 << 13)
#define   ULPI_CORE_CLK_SEL             (1 << 14)
#define   ULPI_SHADOW_CLK_DELAY(x)      (((x) & 0x1F) << 16)
#define   ULPI_LBK_PAD_EN               (1 << 26)
#define   ULPI_LBK_PAD_E_INPUT_OR       (1 << 27)
#define   ULPI_CLK_OUT_ENA              (1 << 28)
#define   ULPI_CLK_PADOUT_ENA           (1 << 29)

#define ULPI_TIMING_CTRL_1      0x428
#define   ULPI_DATA_TRIMMER_LOAD        (1 << 0)
#define   ULPI_DATA_TRIMMER_SEL(x)      (((x) & 0x7) << 1)
#define   ULPI_STPDIRNXT_TRIMMER_LOAD   (1 << 16)
#define   ULPI_STPDIRNXT_TRIMMER_SEL(x) (((x) & 0x7) << 17)
#define   ULPI_DIR_TRIMMER_LOAD         (1 << 24)
#define   ULPI_DIR_TRIMMER_SEL(x)       (((x) & 0x7) << 25)

#define ICUSB_CTRL              0x16C
#define PORTSC1                 0x184
#define USB_SUSP_CTRL           0x400
#define USB1_LEGACY_CTRL        0x410
#define UTMIP_PLL_CFG1          0x804
#define UTMIP_XCVR_CFG0         0x808
#define UTMIP_HSRX_CFG0         0x810
#define UTMIP_HSRX_CFG1         0x814
#define UTMIP_TX_CFG0           0x820
#define UTMIP_MISC_CFG1         0x828
#define UTMIP_DEBOUNCE_CFG0     0x82C
#define UTMIP_BAT_CHRG_CFG0     0x830
#define UTMIP_SPARE_CFG0        0x834
#define UTMIP_XCVR_CFG1         0x838
#define UTMIP_BIAS_CFG1         0x83C

#define nv_gpio_out(port, pin, val)     \
        nv_writereg(port + GPIO_CNF, (nv_readreg(port + GPIO_CNF) | (1 << pin))); \
        nv_writereg(port + GPIO_OE, (nv_readreg(port + GPIO_OE) | (1 << pin))); \
        nv_writereg(port + GPIO_OUT, ((nv_readreg(port + GPIO_OUT) & ~(1 << pin)) | (val << pin)));
    
#endif
