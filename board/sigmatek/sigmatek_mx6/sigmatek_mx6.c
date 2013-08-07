/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>

#include <asm/imx-common/mxc_i2c.h>
#include <i2c.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define ETH_PHY_RESET		IMX_GPIO_NR(3, 29)

#ifdef CONFIG_DETECT_HZS
#define I2C_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* I2C3, lasal eeprom */
static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO_5__I2C3_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO_5__GPIO_1_5 | PC,
		.gp = IMX_GPIO_NR(1, 5)
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO_6__I2C3_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO_6__GPIO_1_6 | PC,
		.gp = IMX_GPIO_NR(1, 6)
	}
};
#endif /* CONFIG_DETECT_HZS */

static iomux_v3_cfg_t const led_pads[] = {
	MX6_PAD_SD2_DAT1__GPIO_1_14 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX6_PAD_SD1_CLK__USDHC1_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT0__USDHC1_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT1__USDHC1_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT2__USDHC1_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT3__USDHC1_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_KEY_ROW1__ENET_COL        | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_KEY_COL3__ENET_CRS        | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC        | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDIO__ENET_MDIO      | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO_18__ENET_RX_CLK      | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RXD0__ENET_RDATA_0   | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RXD1__ENET_RDATA_1   | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_KEY_COL2__ENET_RDATA_2    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_KEY_COL0__ENET_RDATA_3    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_CRS_DV__ENET_RX_EN   | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RX_ER__ENET_RX_ER    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TXD0__ENET_TDATA_0   | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TXD1__ENET_TDATA_1   | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_KEY_ROW2__ENET_TDATA_2    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_KEY_ROW0__ENET_TDATA_3    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TX_EN__ENET_TX_EN    | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static void setup_iomux_leds(void)
{
	imx_iomux_v3_setup_multiple_pads(led_pads, ARRAY_SIZE(led_pads));
	gpio_direction_output(IMX_GPIO_NR(1, 14), 0);
}

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
}

static struct fsl_esdhc_cfg usdhc_cfg[] = {
	{USDHC1_BASE_ADDR},
};


int dram_init(void)
{
	gd->ram_size = CONFIG_DDR_MB * SZ_1M;

	return 0;
}

int board_mmc_getcd(struct mmc *mmc)
{
	/*
	 * There is no card detect line on the hardwarei. For now we just assume
	 * the card is there.
	 */
	return 1;
};

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;	
	
	imx_iomux_v3_setup_multiple_pads(usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
	usdhc_cfg[0].max_bus_width = 4;

	status |= fsl_esdhc_initialize(bis, &usdhc_cfg[0]);

	return status;
}

static int i2c_read_mac(uchar *buffer)
{
	if (i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0x33,
			CONFIG_SYS_I2C_EEPROM_ADDR_LEN, buffer, 6)) {
		puts("eeprom: read failed\n");
		return -1;
	}
	printf("eeprom: read mac %pM\n", buffer);
	return 0;
}

int board_eth_init(bd_t *bis)
{
	int ret;
	unsigned char mac_addr[6];

	if (!eth_getenv_enetaddr("ethaddr", mac_addr) && !i2c_read_mac(mac_addr))
		eth_setenv_enetaddr("ethaddr", mac_addr);

	setup_iomux_enet();

	ret = cpu_eth_init(bis);
	if (ret)
		printf("FEC MXC: %s:failed\n", __func__);

	return 0;
}

int board_early_init_f(void)
{
	setup_iomux_uart();
	return 0;
}

int board_init(void)
{
	uchar cpuName[0x0F];
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_DETECT_HZS
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

	if (i2c_probe(CONFIG_SYS_I2C_EEPROM_ADDR) != 0) {
		/* No eeprom: pretend to be a wandboard */
		puts("Detect: no eeprom. Switch to Wandboard\n");
		gd->bd->bi_arch_number = 4412;
		return 0;
	} else {
		i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0x70, 1, cpuName, 0x0F);
		if (!strcmp((char*)cpuName, "HZS 558-H")) /* Old name for HZS731 */
			gd->bd->bi_arch_number = MACH_TYPE_SIGMATEK_HZS731H;
		else if (!strcmp((char*)cpuName, "HZS 731-H"))
			gd->bd->bi_arch_number = MACH_TYPE_SIGMATEK_HZS731H;
		else if (!strcmp((char*)cpuName, "HGT 1035-H"))
			gd->bd->bi_arch_number = MACH_TYPE_SIGMATEK_HGT1035H;
		else
			gd->bd->bi_arch_number = MACH_TYPE_SIGMATEK_HZS731H;

		printf("Detect: found sigmatek board: %s\n", cpuName);
	}
#endif
	setup_iomux_leds();

	return 0;
}

int checkboard(void)
{
	puts("Board: Sigmatek mx6\n");

	return 0;
}
