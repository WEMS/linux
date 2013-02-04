/*
 * Copyright (c) 2012 Wireless Energy Management Systems International Ltd.
 * Author: Guy Thouret <guythouret@wems.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/physmap.h>
#include <linux/gpio.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <mach/iomux-mx21.h>

#include "devices-imx21.h"

/* Map Additional IO for RS232 Signalling */
/* TODO: Need to correctly define some of these as out pins TBC */
#define WEMS_ASD01_UART2_DCD	(GPIO_PORTB | GPIO_GPIO | GPIO_IN | 19)
#define WEMS_ASD01_UART2_RI		PB18_PF_CSI_D6
#define WEMS_ASD01_UART2_DSR	PB17_PF_CSI_D5
#define WEMS_ASD01_UART2_DTR	PB14_PF_CSI_D4

#define WEMS_ASD01_UART3_DCD	PB13_PF_CSI_D3
#define WEMS_ASD01_UART3_RI		PB12_PF_CSI_D2
#define WEMS_ASD01_UART3_DSR	PB11_PF_CSI_D1
#define WEMS_ASD01_UART3_DTR	PB10_PF_CSI_D0

/* Map Additional IO for SD */
#define WEMS_ASD01_SD1_VEN		(GPIO_PORTD | GPIO_GPIO | GPIO_OUT | 27)
#define WEMS_ASD01_SD1_WP		(GPIO_PORTD | GPIO_GPIO | GPIO_IN | 26)
#define WEMS_ASD01_SD1_CD		(GPIO_PORTD | GPIO_GPIO | GPIO_IN | 25)

/* Map Additional IO for USB */
#define WEMS_ASD01_USB1_MODE		(GPIO_PORTC | GPIO_GPIO | GPIO_OUT | 6)
#define WEMS_ASD01_USB1_SUSPEND		(GPIO_PORTC | 7)
#define WEMS_ASD01_USB1_FS			(GPIO_PORTC | 8)
#define WEMS_ASD01_USB1_OE			(GPIO_PORTC | 9)
#define WEMS_ASD01_USB1_TXDM		(GPIO_PORTC | 10)
#define WEMS_ASD01_USB1_TXDP		(GPIO_PORTC | 11)
#define WEMS_ASD01_USB1_RXDM		(GPIO_PORTC | 12)
#define WEMS_ASD01_USB1_RXDP		(GPIO_PORTC | 13)

#define WEMS_ASD01_USBH_MODE		(GPIO_PORTB | GPIO_GPIO | GPIO_OUT | 22)
#define WEMS_ASD01_USBH_SUSPEND		(GPIO_PORTB | 25)
#define WEMS_ASD01_USBH_FS			(GPIO_PORTB | 26)
#define WEMS_ASD01_USBH_OE			(GPIO_PORTB | 27)
#define WEMS_ASD01_USBH_TXDM		(GPIO_PORTB | 28)
#define WEMS_ASD01_USBH_TXDP		(GPIO_PORTB | 29)
#define WEMS_ASD01_USBH_RXDM		(GPIO_PORTB | 30)
#define WEMS_ASD01_USBH_RXDP		(GPIO_PORTB | 31)

#define GPIO_GIUS					(1 << 20)

#define WEMS_ASD01_USBH1_MODE		(GPIO_PORTB | GPIO_GPIO | GPIO_OUT | 21)
#define WEMS_ASD01_USBH1_FS			(GPIO_PORTD | GPIO_AIN | GPIO_OUT | GPIO_GIUS | 21)
#define WEMS_ASD01_USBH1_OE			(GPIO_PORTD | GPIO_AIN | GPIO_OUT | GPIO_GIUS | 22)
#define WEMS_ASD01_USBH1_TXDM		(GPIO_PORTD | GPIO_AIN | GPIO_OUT | GPIO_GIUS | 23)
#define WEMS_ASD01_USBH1_TXDP		(GPIO_PORTD | GPIO_AIN | GPIO_OUT | GPIO_GIUS | 24)
#define WEMS_ASD01_USBH1_RXDM		(GPIO_PORTD | GPIO_AOUT | GPIO_IN | GPIO_GIUS | 19)
#define WEMS_ASD01_USBH1_RXDP		(GPIO_PORTD | GPIO_AOUT | GPIO_IN | GPIO_GIUS | 20)

#define WEMS_ASD01_I2C_INT		(GPIO_PORTE | GPIO_GPIO | GPIO_IN | 0)

#define WEMS_ASD01_MMIO_BASE_ADDR   0xf5000000
#define WEMS_ASD01_MMIO_SIZE        0x20

    /*DR(3) &= ~(1 << 27); Turn off MMC_VEN */

static const int wems_asd01_pins[] __initconst = {
	/* Ethernet - CS8900 IRQ */
	(GPIO_PORTE | GPIO_GPIO | GPIO_IN | 2),

	/* Console - UART1 */
	PE12_PF_UART1_TXD,
	PE13_PF_UART1_RXD,
	
	/* Serial1 - UART2 */
	PE6_PF_UART2_TXD,
	PE7_PF_UART2_RXD,
	PE3_PF_UART2_CTS,
	PE4_PF_UART2_RTS,

	/* Serial2 - UART3 */
	PE8_PF_UART3_TXD,
	PE9_PF_UART3_RXD,
	PE10_PF_UART3_CTS,
	PE11_PF_UART3_RTS,

	/* SD - SD1 */
	PE18_PF_SD1_D0,
	PE19_PF_SD1_D1,
	PE20_PF_SD1_D2,
	PE21_PF_SD1_D3,
	PE22_PF_SD1_CMD,
	PE23_PF_SD1_CLK,
	WEMS_ASD01_SD1_VEN,
	WEMS_ASD01_SD1_WP,
	WEMS_ASD01_SD1_CD,


	/* Micro SD - SD2 */
	PB4_PF_SD2_D0,
	PB5_PF_SD2_D1,
	PB6_PF_SD2_D2,
	PB7_PF_SD2_D3,
	PB8_PF_SD2_CMD,
	PB9_PF_SD2_CLK,

	/* Audio Codec - Wolfson WMB8731LSEFL */
	PD17_PF_I2C_DATA,
	PD18_PF_I2C_CLK,
	PC20_PF_SSI1_FS,
	PC23_PF_SSI1_CLK,
	PC21_PF_SSI1_RXD,
	PC24_PF_SSI2_FS,
	PC26_PF_SSI2_TXD,
	PC27_PF_SSI2_CLK, /* Check - don't think this is needed */

	/* Status LEDs 0-3 (Fault, Wireless Green, Wireless Red, Alarm) */
	PC14_PF_TOUT,
	PC15_PF_TIN,
	PC16_PF_SAP_FS,
	PC17_PF_SAP_RXD,

	/* USB Host Port 1-1 (USBD) */
	WEMS_ASD01_USB1_MODE,
	WEMS_ASD01_USB1_SUSPEND,
	WEMS_ASD01_USB1_FS,
	WEMS_ASD01_USB1_OE,
	WEMS_ASD01_USB1_TXDM,
	WEMS_ASD01_USB1_TXDP,
	WEMS_ASD01_USB1_RXDM,
	WEMS_ASD01_USB1_RXDP,

	/* USB Host Port 1-2 (USBH) */
	WEMS_ASD01_USBH_MODE,
	WEMS_ASD01_USBH_SUSPEND,
	WEMS_ASD01_USBH_FS,
	WEMS_ASD01_USBH_OE,
	WEMS_ASD01_USBH_TXDM,
	WEMS_ASD01_USBH_TXDP,
	WEMS_ASD01_USBH_RXDM,
	WEMS_ASD01_USBH_RXDP,

	/* USB Host Port 1-3 (USBH1) */
	WEMS_ASD01_USBH1_MODE,
	WEMS_ASD01_USBH1_FS,
	WEMS_ASD01_USBH1_OE,
	WEMS_ASD01_USBH1_TXDM,
	WEMS_ASD01_USBH1_TXDP,
	WEMS_ASD01_USBH1_RXDM,
	WEMS_ASD01_USBH1_RXDP,


	/* I2C */
	PD17_PF_I2C_DATA,
	PD18_PF_I2C_CLK,
	WEMS_ASD01_I2C_INT, /* I2C INT */

};


/* Ethernet: CS8900@0xd3000000 (CS5) */
#define WEMS_ASD01_CS8900A_IRQ_GPIO		IMX_GPIO_NR(5, 2)

static struct resource wems_asd01_cs8900_resources[] __initdata = {
	DEFINE_RES_MEM(MX21_CS5_BASE_ADDR, 0x01000000),
	DEFINE_RES_IRQ(-1),
};

static const struct platform_device_info wems_asd01_cs8900_devinfo __initconst = {
	.name = "cs89x0",
	.id = 0,
	.res = wems_asd01_cs8900_resources,
	.num_res = ARRAY_SIZE(wems_asd01_cs8900_resources),
};


/* NOR Flash (2x Intel JS28F128J3D 128Mbit)
 * 0: 16M@0xc8000000 (CS0)
 * 1: 16M@0xcc000000 (CS1)
 */
static struct physmap_flash_data wems_asd01_flash_data = {
		.width = 2, /* 2 byte data width */
};

static struct resource wems_asd01_flash_resource[] = {
		[0] = {
			.start = MX21_CS0_BASE_ADDR,
			.end = MX21_CS0_BASE_ADDR + SZ_16M - 1,
			.flags = IORESOURCE_MEM,
		},

		[1] = {
			.start = MX21_CS1_BASE_ADDR,
			.end = MX21_CS1_BASE_ADDR + SZ_16M - 1,
			.flags = IORESOURCE_MEM,
		},
};

static struct platform_device wems_asd01_mtd_device = {
		.name = "physmap-flash",
		.id = 0,
		.dev = {
			.platform_data = &wems_asd01_flash_data,
		},
		.num_resources = 2,
		.resource = &wems_asd01_flash_resource,
};

static struct platform_device *platform_devices[] __initdata = {
	&wems_asd01_mtd_device,
};


/*
 * UART2 and UART3 Use RTS/CTS, UART1 does not
 */
static const struct imxuart_platform_data uart_pdata_rts __initconst = {
	.flags = IMXUART_HAVE_RTSCTS,
};

static const struct imxuart_platform_data uart_pdata_norts __initconst = {
};


/* SD Slot 1 */
static int wems_asd01_sd1_get_ro(struct device *dev)
{
	/* return (__raw_readw(MX21ADS_IO_REG) & MX21ADS_IO_SD_WP) ? 1 : 0; */
	return 0;
}

static int wems_asd01_sd1_init(struct device *dev, irq_handler_t detect_irq, void *data)
{
	return request_irq(gpio_to_irq(IMX_GPIO_NR(4, 25)), detect_irq,
		IRQF_TRIGGER_FALLING, "mmc-detect", data);
}

static void wems_asd01_sd1_exit(struct device *dev, void *data)
{
	free_irq(gpio_to_irq(IMX_GPIO_NR(4, 25)), data);
}

static void wems_asd01_sd1_setpower(struct device *dev, unsigned int vdd)
{
    if (vdd) {
    	gpio_set_value(GPIO_PORTD + 27, 1); /* Set SD_VEN high */
    } else {
    	gpio_set_value(GPIO_PORTD + 27, 0); /* Set SD_VEN low */
    }
}

static const struct imxmmc_platform_data wems_asd01_sd1_pdata __initconst = {
	.get_ro		= wems_asd01_sd1_get_ro,
	.init		= wems_asd01_sd1_init,
	.exit		= wems_asd01_sd1_exit,
	.setpower	= wems_asd01_sd1_setpower,
};


/* USB */
static const struct mx21_usbh_platform_data wems_asd01_usbh_pdata __initconst = {
	.host_xcvr	= MX21_USBXCVR_TXDIF_RXDIF,
};


/* I2C */
static const struct imxi2c_platform_data wems_asd01_i2c_pdata __initconst = {
	.bitrate = 100000,
};


/* Inititalise Function */
static void __init wems_asd01_board_init(void)
{
	imx21_soc_init();

	mxc_gpio_setup_multiple_pins(wems_asd01_pins, ARRAY_SIZE(wems_asd01_pins),"wems_asd01");

	wems_asd01_cs8900_resources[1].start	= gpio_to_irq(WEMS_ASD01_CS8900A_IRQ_GPIO);
	wems_asd01_cs8900_resources[1].end		= gpio_to_irq(WEMS_ASD01_CS8900A_IRQ_GPIO);
	platform_device_register_full(&wems_asd01_cs8900_devinfo);

	imx21_add_imx_uart0(&uart_pdata_norts);
	imx21_add_imx_uart1(&uart_pdata_rts);
	imx21_add_imx_uart2(&uart_pdata_rts);

	imx21_add_mxc_mmc(0,&wems_asd01_sd1_pdata);

	imx21_add_imx21_hcd(&wems_asd01_usbh_pdata);

	imx21_add_imx_i2c(&wems_asd01_i2c_pdata);

	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));
}

static void __init wems_asd01_timer_init(void)
{
	mx21_clocks_init(32768, 26000000);
}

static struct sys_timer wems_asd01_timer = {
	.init	= wems_asd01_timer_init,
};

MACHINE_START(WEMS_ASD01, "ASD01 WEMSprogrammer")
	.atag_offset = 0x100,
	.map_io = mx21_map_io,
	.init_early = imx21_init_early,
	.init_irq = mx21_init_irq,
	.handle_irq = imx21_handle_irq,
	.timer = &wems_asd01_timer,
	.init_machine = wems_asd01_board_init,
	.restart	= mxc_restart,
MACHINE_END
