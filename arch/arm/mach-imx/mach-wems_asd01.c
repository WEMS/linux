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

/* Map Additional IO for RS232 Signalling
 *	TX	->
 *	RX	<-
 *	RTS	->
 *	CTS	<-
 *	DTR	->
 *	DSR	<-
 *	DCD	<-
 *	RI	<-
 */
#define WEMS_ASD01_UART2_DCD	(GPIO_PORTB | GPIO_GPIO | GPIO_IN | 19)
#define WEMS_ASD01_UART2_RI		(GPIO_PORTB | GPIO_GPIO | GPIO_IN | 18)
#define WEMS_ASD01_UART2_DSR	(GPIO_PORTB | GPIO_GPIO | GPIO_IN | 17)
#define WEMS_ASD01_UART2_DTR	(GPIO_PORTB | GPIO_GPIO | GPIO_OUT | 14)
#define WEMS_ASD01_UART2_CTS	(GPIO_PORTE | GPIO_GPIO | GPIO_IN | 3)
#define WEMS_ASD01_UART2_RTS	(GPIO_PORTE | GPIO_GPIO | GPIO_OUT | 4)

#define WEMS_ASD01_UART3_DCD	(GPIO_PORTB | GPIO_GPIO | GPIO_IN | 13)
#define WEMS_ASD01_UART3_RI		(GPIO_PORTB | GPIO_GPIO | GPIO_IN | 12)
#define WEMS_ASD01_UART3_DSR	(GPIO_PORTB | GPIO_GPIO | GPIO_IN | 11)
#define WEMS_ASD01_UART3_DTR	(GPIO_PORTB | GPIO_GPIO | GPIO_OUT | 10)
#define WEMS_ASD01_UART3_CTS	(GPIO_PORTE | GPIO_GPIO | GPIO_IN | 10)
#define WEMS_ASD01_UART3_RTS	(GPIO_PORTE | GPIO_GPIO | GPIO_OUT | 11)

/* Map Additional IO for SD */
#define WEMS_ASD01_SD1_VEN		(GPIO_PORTD | GPIO_GPIO | GPIO_OUT | 27)
#define WEMS_ASD01_SD1_WP		(GPIO_PORTD | GPIO_GPIO | GPIO_IN | 26)
#define WEMS_ASD01_SD1_CD		(GPIO_PORTD | GPIO_GPIO | GPIO_IN | 25)

/* Map Additional IO for USB */
/* usb 1-1 */
#define WEMS_ASD01_USB1_MODE		(GPIO_PORTC | GPIO_GPIO | GPIO_OUT | 6)
#define WEMS_ASD01_USB1_SUSPEND		(GPIO_PORTC | GPIO_PF | GPIO_OUT | 7)
#define WEMS_ASD01_USB1_FS			(GPIO_PORTC | GPIO_PF | GPIO_OUT | 8)
#define WEMS_ASD01_USB1_OE			(GPIO_PORTC | GPIO_PF | GPIO_OUT | 9)
#define WEMS_ASD01_USB1_TXDM		(GPIO_PORTC | GPIO_PF | GPIO_OUT | 10)
#define WEMS_ASD01_USB1_TXDP		(GPIO_PORTC | GPIO_PF | GPIO_OUT | 11)
#define WEMS_ASD01_USB1_RXDM		(GPIO_PORTC | GPIO_PF | GPIO_IN | 12)
#define WEMS_ASD01_USB1_RXDP		(GPIO_PORTC | GPIO_PF | GPIO_IN | 13)

/* usb 1-2 */
#define WEMS_ASD01_USBH_MODE		(GPIO_PORTB | GPIO_GPIO | GPIO_OUT | 22)
#define WEMS_ASD01_USBH_SUSPEND		(GPIO_PORTB | GPIO_PF | GPIO_OUT | 25)
#define WEMS_ASD01_USBH_FS			(GPIO_PORTB | GPIO_PF | GPIO_OUT | 26)
#define WEMS_ASD01_USBH_OE			(GPIO_PORTB | GPIO_PF | GPIO_OUT | 27)
#define WEMS_ASD01_USBH_TXDM		(GPIO_PORTB | GPIO_PF | GPIO_OUT | 28)
#define WEMS_ASD01_USBH_TXDP		(GPIO_PORTB | GPIO_PF | GPIO_OUT | 29)
#define WEMS_ASD01_USBH_RXDM		(GPIO_PORTB | GPIO_PF | GPIO_IN | 30)
#define WEMS_ASD01_USBH_RXDP		(GPIO_PORTB | GPIO_PF | GPIO_IN | 31)

/* usb 1-3 */
#define WEMS_ASD01_USBH1_MODE		(GPIO_PORTB | GPIO_GPIO | GPIO_OUT | 21)
#define WEMS_ASD01_USBH1_FS			(GPIO_PORTD | GPIO_AIN | GPIO_OUT | 21)
#define WEMS_ASD01_USBH1_OE			(GPIO_PORTD | GPIO_AIN | GPIO_OUT | 22)
#define WEMS_ASD01_USBH1_TXDM		(GPIO_PORTD | GPIO_AIN | GPIO_OUT | 23)
#define WEMS_ASD01_USBH1_TXDP		(GPIO_PORTD | GPIO_AIN | GPIO_OUT | 24)
#define WEMS_ASD01_USBH1_RXDM		(GPIO_PORTD | GPIO_AOUT | GPIO_IN | 19)
#define WEMS_ASD01_USBH1_RXDP		(GPIO_PORTD | GPIO_AOUT | GPIO_IN | 20)

#define WEMS_ASD01_I2C_INT			(GPIO_PORTE | GPIO_GPIO | GPIO_IN | 0)

/* Status LEDs */
#define WEMS_ASD01_LED_FAULT		(GPIO_PORTC | GPIO_GPIO | GPIO_OUT | 14)
#define WEMS_ASD01_LED_RF_RED		(GPIO_PORTC | GPIO_GPIO | GPIO_OUT | 15)
#define WEMS_ASD01_LED_RF_GREEN		(GPIO_PORTC | GPIO_GPIO | GPIO_OUT | 16)
#define WEMS_ASD01_LED_ALARM		(GPIO_PORTC | GPIO_GPIO | GPIO_OUT | 17)


#define WEMS_ASD01_MMIO_BASE_ADDR   0xf5000000
#define WEMS_ASD01_MMIO_SIZE        0x20


static const int wems_asd01_pins[] __initconst = {
	/* Ethernet - CS8900 IRQ */
	(GPIO_PORTE | GPIO_GPIO | GPIO_IN | 2),

	/* Console - UART1 */
	PE12_PF_UART1_TXD,
	PE13_PF_UART1_RXD,
	
	/* Serial1 - UART2 */
	PE6_PF_UART2_TXD,
	PE7_PF_UART2_RXD,
	WEMS_ASD01_UART2_DCD,
	WEMS_ASD01_UART2_RI,
	WEMS_ASD01_UART2_DSR,
	WEMS_ASD01_UART2_DTR,
	WEMS_ASD01_UART2_CTS,
	WEMS_ASD01_UART2_RTS,

	/* Serial2 - UART3 */
	PE8_PF_UART3_TXD,
	PE9_PF_UART3_RXD,
	WEMS_ASD01_UART3_DCD,
	WEMS_ASD01_UART3_RI,
	WEMS_ASD01_UART3_DSR,
	WEMS_ASD01_UART3_DTR,
	WEMS_ASD01_UART3_CTS,
	WEMS_ASD01_UART3_RTS,

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
	WEMS_ASD01_LED_FAULT,
	WEMS_ASD01_LED_RF_RED,
	WEMS_ASD01_LED_RF_GREEN,
	WEMS_ASD01_LED_ALARM,

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
 * UART2 and UART3 Use RS232 DTE, UART1 does not
 */
static const struct imxuart_platform_data uart_pdata_uart2 __initconst = {
	.flags = IMXUART_HAVE_RTSCTS | IMXUART_HAVE_DTRDSR | IMXUART_HAVE_DCD | IMXUART_HAVE_RI | IMXUART_IS_DTE,
	/*.gpio_rts = WEMS_ASD01_UART2_RTS,
	.gpio_cts = WEMS_ASD01_UART2_CTS,
	.gpio_dtr = WEMS_ASD01_UART2_DTR,
	.gpio_dsr = WEMS_ASD01_UART2_DSR,
	.gpio_dcd = WEMS_ASD01_UART2_DCD,
	.gpio_ri = WEMS_ASD01_UART2_RI,*/
	.gpio_rts = IMX_GPIO_NR(5,4),
	.gpio_cts = IMX_GPIO_NR(5,3),
	.gpio_dtr = IMX_GPIO_NR(2,14),
	.gpio_dsr = IMX_GPIO_NR(2,17),
	.gpio_dcd = IMX_GPIO_NR(2,19),
	.gpio_ri = IMX_GPIO_NR(2,18),
};

static const struct imxuart_platform_data uart_pdata_uart3 __initconst = {
	.flags = IMXUART_HAVE_RTSCTS | IMXUART_HAVE_DTRDSR | IMXUART_HAVE_DCD | IMXUART_HAVE_RI | IMXUART_IS_DTE,
	/*.gpio_rts = WEMS_ASD01_UART3_RTS,
	.gpio_cts = WEMS_ASD01_UART3_CTS,
	.gpio_dtr = WEMS_ASD01_UART3_DTR,
	.gpio_dsr = WEMS_ASD01_UART3_DSR,
	.gpio_dcd = WEMS_ASD01_UART3_DCD,
	.gpio_ri = WEMS_ASD01_UART3_RI,*/
	.gpio_rts = IMX_GPIO_NR(5,11),
	.gpio_cts = IMX_GPIO_NR(5,10),
	.gpio_dtr = IMX_GPIO_NR(2,10),
	.gpio_dsr = IMX_GPIO_NR(2,11),
	.gpio_dcd = IMX_GPIO_NR(2,13),
	.gpio_ri = IMX_GPIO_NR(2,12),
};

static const struct imxuart_platform_data uart_pdata_uart1 __initconst = {
};


/* SD Slot 1 */
static int wems_asd01_sd1_get_ro(struct device *dev)
{
	return gpio_get_value(IMX_GPIO_NR(4, 26));
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
    	gpio_set_value(IMX_GPIO_NR(4, 27), 1); /* Set SD_VEN high */
    } else {
    	gpio_set_value(IMX_GPIO_NR(4, 27), 0); /* Set SD_VEN low */
    }
}

static const struct imxmmc_platform_data wems_asd01_sd1_pdata __initconst = {
	.get_ro		= wems_asd01_sd1_get_ro,
	.init		= wems_asd01_sd1_init,
	.exit		= wems_asd01_sd1_exit,
	.setpower	= wems_asd01_sd1_setpower,
};

/* SD Slot 2 */
static const struct imxmmc_platform_data wems_asd01_sd2_pdata __initconst = {
	.dat3_card_detect = 1,
};


/* USB */
static const struct mx21_usbh_platform_data wems_asd01_usbh_pdata __initconst = {
	.host_xcvr	= MX21_USBXCVR_TXDIF_RXDIF,
	.otg_xcvr = MX21_USBXCVR_TXDIF_RXDIF,
	.enable_host1 = 1,
	.enable_host2 = 1,
	.enable_otg_host = 1,
	.host1_txenoe = 1,
};


/* I2C */
static const struct imxi2c_platform_data wems_asd01_i2c_pdata __initconst = {
	.bitrate = 100000,
};


/* I2C RTC */
static struct i2c_board_info m41t00_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("m41t00", 0x68),
	},
};


/* Inititalise Function */
static void __init wems_asd01_board_init(void)
{
	imx21_soc_init();

	mxc_gpio_setup_multiple_pins(wems_asd01_pins, ARRAY_SIZE(wems_asd01_pins),"wems_asd01");

	/* Initialise Ethernet */
	wems_asd01_cs8900_resources[1].start	= gpio_to_irq(WEMS_ASD01_CS8900A_IRQ_GPIO);
	wems_asd01_cs8900_resources[1].end		= gpio_to_irq(WEMS_ASD01_CS8900A_IRQ_GPIO);
	platform_device_register_full(&wems_asd01_cs8900_devinfo);

	/* Initialise UARTs */
	imx21_add_imx_uart0(&uart_pdata_uart1);
	imx21_add_imx_uart1(&uart_pdata_uart2);
	imx21_add_imx_uart2(&uart_pdata_uart3);

	/* MMC Power Off */
	gpio_set_value(IMX_GPIO_NR(4, 27), 0);
	/* Initialise MMC */
	imx21_add_mxc_mmc(0,&wems_asd01_sd1_pdata);
	/*imx21_add_mxc_mmc(1,&wems_asd01_sd2_pdata);*/

	/* Test - Set Host xcvr intodiff mode */
	gpio_set_value(IMX_GPIO_NR(3, 6),1);
	gpio_set_value(IMX_GPIO_NR(2, 22),1);
	gpio_set_value(IMX_GPIO_NR(2, 21),1);

	/* Initialise USB */
	imx21_add_imx21_hcd(&wems_asd01_usbh_pdata);

	/* Initialise I2C */
	imx21_add_imx_i2c(&wems_asd01_i2c_pdata);

	/* Initialise RTC */
	i2c_register_board_info(0, m41t00_i2c_board_info, ARRAY_SIZE(m41t00_i2c_board_info));

	/* Initialise Watchdog */
	imx21_add_imx2_wdt();

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
