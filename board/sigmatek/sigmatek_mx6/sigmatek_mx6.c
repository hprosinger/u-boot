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
#include <asm/arch/crm_regs.h>
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
static int eeprom_address_len;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define WEIM_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST)

#define ETH_PHY_RESET		IMX_GPIO_NR(3, 29)

#define GPMI_PAD_CTRL0 (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP)
#define GPMI_PAD_CTRL1 (PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_MED | \
			PAD_CTL_SRE_FAST)
#define GPMI_PAD_CTRL2 (GPMI_PAD_CTRL0 | GPMI_PAD_CTRL1)

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


#ifdef CONFIG_SYS_USE_NAND
static iomux_v3_cfg_t const gpmi_pads[] = {
	MX6_PAD_NANDF_CLE__RAWNAND_CLE		| MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NANDF_ALE__RAWNAND_ALE		| MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NANDF_WP_B__RAWNAND_RESETN	| MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NANDF_RB0__RAWNAND_READY0	| MUX_PAD_CTRL(GPMI_PAD_CTRL0),
	MX6_PAD_NANDF_CS0__RAWNAND_CE0N		| MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_SD4_CMD__RAWNAND_RDN		| MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_SD4_CLK__RAWNAND_WRN		| MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NANDF_D0__RAWNAND_D0		| MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NANDF_D1__RAWNAND_D1		| MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NANDF_D2__RAWNAND_D2		| MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NANDF_D3__RAWNAND_D3		| MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NANDF_D4__RAWNAND_D4		| MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NANDF_D5__RAWNAND_D5		| MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NANDF_D6__RAWNAND_D6		| MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NANDF_D7__RAWNAND_D7		| MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_SD4_DAT0__RAWNAND_DQS		| MUX_PAD_CTRL(GPMI_PAD_CTRL1),
};

static void setup_gpmi_nand(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	/* config gpmi nand iomux */
	imx_iomux_v3_setup_multiple_pads(gpmi_pads, ARRAY_SIZE(gpmi_pads));

	/* gate ENFC_CLK_ROOT clock first,before clk source switch */
	clrbits_le32(&mxc_ccm->CCGR2, MXC_CCM_CCGR2_IOMUX_IPT_CLK_IO_MASK);

	/* config gpmi and bch clock to 100 MHz */
	clrsetbits_le32(&mxc_ccm->cs2cdr,
			MXC_CCM_CS2CDR_ENFC_CLK_PODF_MASK |
			MXC_CCM_CS2CDR_ENFC_CLK_PRED_MASK |
			MXC_CCM_CS2CDR_ENFC_CLK_SEL_MASK,
			MXC_CCM_CS2CDR_ENFC_CLK_PODF(0) |
			MXC_CCM_CS2CDR_ENFC_CLK_PRED(3) |
			MXC_CCM_CS2CDR_ENFC_CLK_SEL(3));

	/* enable ENFC_CLK_ROOT clock */
	setbits_le32(&mxc_ccm->CCGR2, MXC_CCM_CCGR2_IOMUX_IPT_CLK_IO_MASK);

	/* enable gpmi and bch clock gating */
	setbits_le32(&mxc_ccm->CCGR4,
		     MXC_CCM_CCGR4_RAWNAND_U_BCH_INPUT_APB_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_BCH_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_GPMI_IO_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_INPUT_APB_MASK |
		     MXC_CCM_CCGR4_PL301_MX6QPER1_BCH_OFFSET);

	/* enable apbh clock gating */
	setbits_le32(&mxc_ccm->CCGR0, MXC_CCM_CCGR0_APBHDMA_MASK);
}
#endif

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
	int status = 0;
	
	imx_iomux_v3_setup_multiple_pads(usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
	usdhc_cfg[0].max_bus_width = 4;

	status |= fsl_esdhc_initialize(bis, &usdhc_cfg[0]);

	return status;
}

static int i2c_read_mac(uchar *buffer)
{
	if (i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0x33,
			eeprom_address_len, buffer, 6)) {
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
#ifdef CONFIG_SYS_USE_NAND
	setup_gpmi_nand();
#endif
	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

	if (i2c_probe(CONFIG_SYS_I2C_EEPROM_ADDR) != 0) {
		/* No eeprom: pretend to be a wandboard */
		puts("Detect: no eeprom. Switch to Wandboard\n");
		gd->bd->bi_arch_number = 4412;
		return 0;
	} else {
		uchar cpuName[0x10];

		cpuName[0xf] = '\0';

		eeprom_address_len = 1;
		i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0x70, eeprom_address_len, cpuName, 0x0F);
		if (!strcmp((char*)cpuName, "HGT 1035-H")) {
			gd->bd->bi_arch_number = MACH_TYPE_SIGMATEK_HGT1035H_PROTO;
		} else {
			eeprom_address_len = 2;
			i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0x70, eeprom_address_len, cpuName, 0x0F);
			if (!strcmp((char*)cpuName, "HGT 1035-H"))
				gd->bd->bi_arch_number = MACH_TYPE_SIGMATEK_HGT1035H;
			else /* "HZS 731-H", "HZS 558-H"(Old name for HZS731) */
				gd->bd->bi_arch_number = MACH_TYPE_SIGMATEK_HZS731H;
		}
		printf("Detect: found sigmatek board: %s\n", cpuName);
	}

	setup_iomux_leds();

	return 0;
}

int checkboard(void)
{
	puts("Board: Sigmatek mx6\n");

	return 0;
}

/*-------------------------------------------------------------------------------------------------------*/

/****************************************************************************
 *
 * EIM
 *
 ****************************************************************************/
static const iomux_v3_cfg_t hgt1035h_eim_pads[] = {
		// Control signals
		MX6_PAD_EIM_LBA__WEIM_WEIM_LBA      | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_EIM_WAIT__WEIM_WEIM_DTACK_B | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_EIM_OE__WEIM_WEIM_OE        | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_EIM_RW__WEIM_WEIM_RW        | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_EIM_CS0__WEIM_WEIM_CS_0     | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_EIM_CS1__WEIM_WEIM_CS_1     | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_SD2_DAT2__WEIM_WEIM_CS_3    | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_SD4_DAT3__GPIO_2_11         | MUX_PAD_CTRL(NO_PAD_CTRL),			// EIM_RDY_INT
		MX6_PAD_SD4_DAT4__GPIO_2_12         | MUX_PAD_CTRL(NO_PAD_CTRL),			// FPGA_RDY_BOOT
		// Addresses 16:25
		MX6_PAD_EIM_A16__WEIM_WEIM_A_16     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_A17__WEIM_WEIM_A_17     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_A18__WEIM_WEIM_A_18     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_A19__WEIM_WEIM_A_19     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_A20__WEIM_WEIM_A_20     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_A21__WEIM_WEIM_A_21     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_A22__WEIM_WEIM_A_22     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_A23__WEIM_WEIM_A_23     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_A24__WEIM_WEIM_A_24     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_A25__WEIM_WEIM_A_25     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		// Data 0:15
		MX6_PAD_CSI0_DATA_EN__WEIM_WEIM_D_0 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_CSI0_VSYNC__WEIM_WEIM_D_1   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_CSI0_DAT4__WEIM_WEIM_D_2    | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_CSI0_DAT5__WEIM_WEIM_D_3    | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_CSI0_DAT6__WEIM_WEIM_D_4    | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_CSI0_DAT7__WEIM_WEIM_D_5    | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_CSI0_DAT8__WEIM_WEIM_D_6    | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_CSI0_DAT9__WEIM_WEIM_D_7    | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_CSI0_DAT12__WEIM_WEIM_D_8   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_CSI0_DAT13__WEIM_WEIM_D_9   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_CSI0_DAT14__WEIM_WEIM_D_10  | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_CSI0_DAT15__WEIM_WEIM_D_11  | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_CSI0_DAT16__WEIM_WEIM_D_12  | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_CSI0_DAT17__WEIM_WEIM_D_13  | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_CSI0_DAT18__WEIM_WEIM_D_14  | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_CSI0_DAT19__WEIM_WEIM_D_15  | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		// Data 16:31
		MX6_PAD_EIM_D16__WEIM_WEIM_D_16     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_D17__WEIM_WEIM_D_17     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_D18__WEIM_WEIM_D_18     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_D19__WEIM_WEIM_D_19     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_D20__WEIM_WEIM_D_20     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_D21__WEIM_WEIM_D_21     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_D22__WEIM_WEIM_D_22     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_D23__WEIM_WEIM_D_23     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_D24__WEIM_WEIM_D_24     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_D25__WEIM_WEIM_D_25     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_D26__WEIM_WEIM_D_26     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_D27__WEIM_WEIM_D_27     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_D28__WEIM_WEIM_D_28     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_D29__WEIM_WEIM_D_29     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_D30__WEIM_WEIM_D_30     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_D31__WEIM_WEIM_D_31     | MUX_PAD_CTRL(WEIM_PAD_CTRL),

		MX6_PAD_EIM_EB0__WEIM_WEIM_EB_0     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_EB1__WEIM_WEIM_EB_1     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_EB2__WEIM_WEIM_EB_2     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_EB3__WEIM_WEIM_EB_3     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_BCLK__WEIM_WEIM_BCLK    | MUX_PAD_CTRL(WEIM_PAD_CTRL),

		MX6_PAD_EIM_DA0__WEIM_WEIM_DA_A_0   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_DA1__WEIM_WEIM_DA_A_1   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_DA2__WEIM_WEIM_DA_A_2   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_DA3__WEIM_WEIM_DA_A_3   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_DA4__WEIM_WEIM_DA_A_4   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_DA5__WEIM_WEIM_DA_A_5   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_DA6__WEIM_WEIM_DA_A_6   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_DA7__WEIM_WEIM_DA_A_7   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_DA8__WEIM_WEIM_DA_A_8   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_DA9__WEIM_WEIM_DA_A_9   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_DA10__WEIM_WEIM_DA_A_10 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_DA11__WEIM_WEIM_DA_A_11 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_DA12__WEIM_WEIM_DA_A_12 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_DA13__WEIM_WEIM_DA_A_13 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_DA14__WEIM_WEIM_DA_A_14 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
		MX6_PAD_EIM_DA15__WEIM_WEIM_DA_A_15 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
};

static int hgt1035h_eim_clks(void)
{
	struct clk *eim_clock;
	struct clk *pll2_400;	// thats the clock we have to use!!!!!
	struct clk *axi;

	// clock with 99MHz
	// get the handles of the clocks
	eim_clock = clk_get(NULL, "emi_slow_clk");
	pll2_400 = clk_get(NULL, "pll2_pfd_400M");
	axi = clk_get(NULL, "axi_clk");

	if (IS_ERR(eim_clock) || IS_ERR(pll2_400) || IS_ERR(axi)) {
		printf("%s: couldn't get required clock-handle for EIM \n", __func__);
		return -1;
	}

	// change the parent of axi to 400M
	if (clk_set_parent(axi, pll2_400)) {
		printf("Error: Couldn't set the right parent for the axi clock\n");
		return -1;
	}
	// AXI should be the parent of EIM
	if (clk_set_parent(eim_clock, axi)) {
		printf("Error: Couldn't set the right parent for the clock of EIM\n");
		return -1;
	}
	// change the divider of AXI to 2		--> AXI is 198MHz
	clk_set_rate(axi, 198000000);
	// and finally another divider to be close to 100 MHz
	// divider of eim_clk to 2  			--> EIM clk = 99MHz
	clk_set_rate(eim_clock, 99000000);

	// enable all those clocks...
	clk_enable(eim_clock);
	clk_enable(axi);
	clk_enable(pll2_400);
	return 0;
}

#define REGS_EIM_BASE			0x021b8000
#define REGS_EIM_SIZE			0xA4
#define REGS_IOMUXC_BASE		0x020E0000
#define REGS_IOMUXC_SIZE		0x08		// size of IOMUXC is in fact much bigger but we write anyway only GPR1 in the moment so mapping 8Bytes is enough

// Addresses of the GPIO data registers
#define REGS_GPIO1_DR			0x0209C000
#define REGS_GPIO2_DR			0x020A0000
#define REGS_GPIO3_DR			0x020A4000
#define REGS_GPIO4_DR			0x020A8000
#define REGS_GPIO5_DR			0x020AC000
#define REGS_GPIO6_DR			0x020B0000
#define REGS_GPIO7_DR			0x020B4000


/*************************************************************************
 * Registers of the IOMUXC
 *************************************************************************/
#define REGS_IOMUXC_GPR1				0x04

#define REGS_IOMUXC_GPR1_SHFT_ACT_CS0	0
#define REGS_IOMUXC_GPR1_SHFT_ADDRS0	1
#define REGS_IOMUXC_GPR1_SHFT_ACT_CS1	3
#define REGS_IOMUXC_GPR1_SHFT_ADDRS1	4
#define REGS_IOMUXC_GPR1_SHFT_ACT_CS2	6
#define REGS_IOMUXC_GPR1_SHFT_ADDRS2	7
#define REGS_IOMUXC_GPR1_SHFT_ACT_CS3	9
#define REGS_IOMUXC_GPR1_SHFT_ADDRS3	10

#define EIM_ADDRSIZE_32M	0
#define EIM_ADDRSIZE_64M	1
#define EIM_ADDRSIZE_128M	2
#define EIM_ADDRSIZE_INV	3

/*************************************************************************
 * CS specific Registers of the EIM
 *************************************************************************/
#define REGS_EIM_CSnGCR1(n)		(0x00 + (n * 0x18))
#define REGS_EIM_CSnGCR2(n)		(0x04 + (n * 0x18))
#define REGS_EIM_CSnRCR1(n)		(0x08 + (n * 0x18))
#define REGS_EIM_CSnRCR2(n)		(0x0C + (n * 0x18))
#define REGS_EIM_CSnWCR1(n)		(0x10 + (n * 0x18))
#define REGS_EIM_CSnWCR2(n)		(0x14 + (n * 0x18))

// Shift values of the Registers
#define REGS_EIM_CSnGCR1_SHFT_CSEN		0
#define REGS_EIM_CSnGCR1_SHFT_SRW		1
#define REGS_EIM_CSnGCR1_SHFT_SRD		2
#define REGS_EIM_CSnGCR1_SHFT_WFL		4
#define REGS_EIM_CSnGCR1_SHFT_RFL		5
#define REGS_EIM_CSnGCR1_SHFT_BL		8
#define REGS_EIM_CSnGCR1_SHFT_BCD		12
#define REGS_EIM_CSnGCR1_SHFT_DSZ		16
#define REGS_EIM_CSnGCR1_SHFT_CSREC		20
#define REGS_EIM_CSnGCR1_SHFT_AUS		23

#define REGS_EIM_CSnGCR2_SHFT_DAPS		4
#define REGS_EIM_CSnGCR2_SHFT_DAE		8
#define REGS_EIM_CSnGCR2_SHFT_DAP		9

#define REGS_EIM_CSnRCR1_SHFT_RCSN		0
#define REGS_EIM_CSnRCR1_SHFT_RCSA		4
#define REGS_EIM_CSnRCR1_SHFT_OEN		8
#define REGS_EIM_CSnRCR1_SHFT_OEA		12
#define REGS_EIM_CSnRCR1_SHFT_RADVN		16
#define REGS_EIM_CSnRCR1_SHFT_RAL		19
#define REGS_EIM_CSnRCR1_SHFT_RADVA		20
#define REGS_EIM_CSnRCR1_SHFT_RWSC		24

#define REGS_EIM_CSnRCR2_SHFT_RBE		3

#define REGS_EIM_CSnWCR1_SHFT_WCSN		0
#define REGS_EIM_CSnWCR1_SHFT_WCSA		3
#define REGS_EIM_CSnWCR1_SHFT_WEN		6
#define REGS_EIM_CSnWCR1_SHFT_WEA		9
#define REGS_EIM_CSnWCR1_SHFT_WBEN		12
#define REGS_EIM_CSnWCR1_SHFT_WBEA		15
#define REGS_EIM_CSnWCR1_SHFT_WADVN		18
#define REGS_EIM_CSnWCR1_SHFT_WADVA		21
#define REGS_EIM_CSnWCR1_SHFT_WWSC		24
#define REGS_EIM_CSnWCR1_SHFT_WBED		30
#define REGS_EIM_CSnWCR1_SHFT_WAL		31

/*************************************************************************
 * Global Registers of the EIM
 *************************************************************************/
#define REGS_EIM_WCR			0x90
#define REGS_EIM_DCR			0x94
#define REGS_EIM_DSR			0x98
#define REGS_EIM_WIAR			0x9C
#define REGS_EIM_EAR			0xA0

// Shift values of the Registers
#define REGS_EIM_WCR_SHFT_BCM			0
#define REGS_EIM_WCR_SHFT_GBCD			1
#define REGS_EIM_WCR_MASK_GBCD			(0x03 << REGS_EIM_WCR_SHFT_GBCD)
#define REGS_EIM_WCR_SHFT_CONT_BCLK		3


#define DSZ_8_LOW_23_16					0x06	// Testaufbau
#define DSZ_8_LOW_15_8					0x05
#define DSZ_32_LOW_31_0					0x03
#define DSZ_16_LOW_15_0					0x01

#define EIM_MAX_CS				4

#define EIM_MEM_PHYS_START		0x08000000

static int hgt1035h_eim_config(void)
{
	int i;
	u32 wcr = 0;
	u32 gcr1[4] = {0, 0, 0, 0};
	u32 gcr2[4] = {0, 0, 0, 0};
	u32 rcr1[4] = {0, 0, 0, 0};
	u32 rcr2[4] = {0, 0, 0, 0};
	u32 wcr1[4] = {0, 0, 0, 0};
	u32 wcr2[4] = {0, 0, 0, 0};
	u32 gpr1 = 0;	// register in IOMUXC to configure address ranges

	gpr1 = readl(REGS_IOMUXC_BASE + REGS_IOMUXC_GPR1);	// read the value of the register
	gpr1 &= 0xFFFFf000;
	gpr1 |= (0x01 << REGS_IOMUXC_GPR1_SHFT_ACT_CS0);	// activate CS0
	gpr1 |= (EIM_ADDRSIZE_32M << REGS_IOMUXC_GPR1_SHFT_ADDRS0);		// 32M for FPGA are enough
	gpr1 |= (0x01 << REGS_IOMUXC_GPR1_SHFT_ACT_CS1);	// activate CS1
	gpr1 |= (EIM_ADDRSIZE_32M << REGS_IOMUXC_GPR1_SHFT_ADDRS1);		// 32M for FPGA are enough
	gpr1 |= (0x01 << REGS_IOMUXC_GPR1_SHFT_ACT_CS2);	// activate CS2
	gpr1 |= (EIM_ADDRSIZE_32M << REGS_IOMUXC_GPR1_SHFT_ADDRS2);		// 32M for FPGA are enough
	gpr1 |= (0x01 << REGS_IOMUXC_GPR1_SHFT_ACT_CS3);	// activate CS3
	gpr1 |= (EIM_ADDRSIZE_32M << REGS_IOMUXC_GPR1_SHFT_ADDRS3);		// 32M for MRAM are enough


	// Values for CS0 and CS1 worked out from FPGA Team, Wolfgang Marik, 09.08.2013
	wcr |= (0x01 << REGS_EIM_WCR_SHFT_GBCD);		// BC = EIM divided by 2
	wcr |= (0x00 << REGS_EIM_WCR_SHFT_CONT_BCLK);

	// Chip select settings for asynchronous FPGA access
	gcr1[0] |= (0x01 << REGS_EIM_CSnGCR1_SHFT_CSEN);		// enable this CS
	gcr1[0] |= (DSZ_32_LOW_31_0 << REGS_EIM_CSnGCR1_SHFT_DSZ);		// Configure the Data Port Size 32bit
	gcr1[0] |= (0x03 << REGS_EIM_CSnGCR1_SHFT_CSREC);		// time between 2 chipselects
	gcr1[0] |= (0x01 << REGS_EIM_CSnGCR1_SHFT_AUS);

	gcr2[0] |= (0x07 << REGS_EIM_CSnGCR2_SHFT_DAPS);
	gcr2[0] |= (0x01 << REGS_EIM_CSnGCR2_SHFT_DAE);
	gcr2[0] |= (0x01 << REGS_EIM_CSnGCR2_SHFT_DAP);

	rcr1[0] |= (0x01 << REGS_EIM_CSnRCR1_SHFT_RADVN);
	rcr1[0] |= (0x01 << REGS_EIM_CSnRCR1_SHFT_RWSC);

	rcr2[0] |= (0x01 << REGS_EIM_CSnRCR2_SHFT_RBE);

	wcr1[0] |= (0x01 << REGS_EIM_CSnWCR1_SHFT_WADVN);
	wcr1[0] |= (0x01 << REGS_EIM_CSnWCR1_SHFT_WWSC);


	// Chip select settings for synchronous FPGA access (DPRAM)
	gcr1[1] |= (0x01 << REGS_EIM_CSnGCR1_SHFT_CSEN);		// enable this CS
	gcr1[1]	|= (0x01 << REGS_EIM_CSnGCR1_SHFT_SRW);
	gcr1[1]	|= (0x01 << REGS_EIM_CSnGCR1_SHFT_SRD);
	gcr1[1]	|= (0x01 << REGS_EIM_CSnGCR1_SHFT_WFL);
	gcr1[1]	|= (0x01 << REGS_EIM_CSnGCR1_SHFT_RFL);
	gcr1[1]	|= (0x04 << REGS_EIM_CSnGCR1_SHFT_BL);
	gcr1[1]	|= (0x01 << REGS_EIM_CSnGCR1_SHFT_BCD);
	gcr1[1] |= (DSZ_32_LOW_31_0 << REGS_EIM_CSnGCR1_SHFT_DSZ);		// Configure the Data Port Size 32bit
	gcr1[1] |= (0x02 << REGS_EIM_CSnGCR1_SHFT_CSREC);		// time between 2 chipselects
	gcr1[1] |= (0x01 << REGS_EIM_CSnGCR1_SHFT_AUS);

	rcr1[1] |= (0x03 << REGS_EIM_CSnRCR1_SHFT_RWSC);

	rcr2[1] |= (0x01 << REGS_EIM_CSnRCR2_SHFT_RBE);

	wcr1[1] |= (0x01 << REGS_EIM_CSnWCR1_SHFT_WWSC);

	// Configuration taken from SRAM of HGT
	// We have a clock of 99MHz --> one EIM cycle lasts 10.1ns
	gcr1[3] |= (1 << REGS_EIM_CSnGCR1_SHFT_CSEN);		// enable this CS
	gcr1[3] |= (DSZ_16_LOW_15_0 << REGS_EIM_CSnGCR1_SHFT_DSZ);		// Configure the Data Port Size
	gcr1[3] |= (0 << REGS_EIM_CSnGCR1_SHFT_CSREC);		// time between 2 chipselects

	rcr1[3] |= (0x00 << REGS_EIM_CSnRCR1_SHFT_RCSN);
	rcr1[3] |= (0x00 << REGS_EIM_CSnRCR1_SHFT_RCSA);
	rcr1[3] |= (0x00 << REGS_EIM_CSnRCR1_SHFT_OEN);
	rcr1[3] |= (0x01 << REGS_EIM_CSnRCR1_SHFT_OEA);

	rcr1[3] |= (0x01 << REGS_EIM_CSnRCR1_SHFT_RAL);	// Address is valid until the end of the read cycle
	rcr1[3] |= (0x00 << REGS_EIM_CSnRCR1_SHFT_RADVA);	// time after start when the address is valid 10.1ns
	rcr1[3] |= (0x04 << REGS_EIM_CSnRCR1_SHFT_RWSC);	// time of a read cycle	= 40.4ns

	rcr2[3] |= (0x01 << REGS_EIM_CSnRCR2_SHFT_RBE);

	wcr1[3] |= (0x00 << REGS_EIM_CSnWCR1_SHFT_WCSN);
	wcr1[3] |= (0x00 << REGS_EIM_CSnWCR1_SHFT_WCSA);
	wcr1[3] |= (0x00 << REGS_EIM_CSnWCR1_SHFT_WEN);
	wcr1[3] |= (0x01 << REGS_EIM_CSnWCR1_SHFT_WEA);

	wcr1[3] |= (0x00 << REGS_EIM_CSnWCR1_SHFT_WADVA);
	wcr1[3] |= (0x04 << REGS_EIM_CSnWCR1_SHFT_WWSC);
	wcr1[3] |= (0x01 << REGS_EIM_CSnWCR1_SHFT_WAL);


	writel(wcr, REGS_EIM_BASE + REGS_EIM_WCR);

	for (i=0; i<EIM_MAX_CS; i++) {
		writel(gcr1[i], REGS_EIM_BASE + REGS_EIM_CSnGCR1(i));
		writel(gcr2[i], REGS_EIM_BASE + REGS_EIM_CSnGCR2(i));
		writel(rcr1[i], REGS_EIM_BASE + REGS_EIM_CSnRCR1(i));
		writel(rcr2[i], REGS_EIM_BASE + REGS_EIM_CSnRCR2(i));
		writel(wcr1[i], REGS_EIM_BASE + REGS_EIM_CSnWCR1(i));
		writel(wcr2[i], REGS_EIM_BASE + REGS_EIM_CSnWCR2(i));
	}

	// write the IOMUXC Configuration
	writel(gpr1, REGS_IOMUXC_BASE + REGS_IOMUXC_GPR1);

	return 0;
}

static void hgt1035h_init_eim(void)
{
	imx_iomux_v3_setup_multiple_pads(hgt1035h_eim_pads, ARRAY_SIZE(hgt1035h_eim_pads));
/*
	if (hgt1035h_eim_clks()) {
		puts("couldn't initialize EIM Clock");
		return;
	}
*/
	if (hgt1035h_eim_config()) {
		puts("couldn't initialize EIM");
		return;
	}
}

#define START_CS0       0x08000000
#define START_CS1       (0x08000000 + (32*1024*1024))

#define PHYS_ADDR       START_CS0

static void allow_unaligned(void)
{
	uint32_t reg = get_cr();

	reg &= ~0x2;
	set_cr(reg);
}

static int eim_setup(void)
{
	allow_unaligned();
	hgt1035h_init_eim();

	return 0;
}

static int eim_w4(char * const s)
{
	uint32_t addr;
	uint32_t *t;

	addr = PHYS_ADDR + simple_strtol (s, NULL, 10);
	t = (uint32_t *)addr;
	printf("write %d to %p\n", sizeof(uint16_t), t);
	*t = 0xaaaaaaaa;
	return 0;
}

static int eim_w2(char * const s)
{
	uint32_t addr;
	uint16_t *t;

	addr = PHYS_ADDR + simple_strtol (s, NULL, 10);
	t = (uint16_t *)addr;
	printf("write %d to %p\n", sizeof(uint16_t), t);
	*t = 0xaaaa;
	return 0;
}

static int eim_w1(char * const s)
{
	uint32_t addr;
	uint8_t *t;

	addr = PHYS_ADDR + simple_strtol (s, NULL, 10);
	t = (uint8_t *)addr;
	printf("write %d to %p\n", sizeof(uint8_t), t);
	*t = 0xaa;
	return 0;
}

int do_eim(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char *cmd;

	if (argc != 2 && argc != 3)
		return cmd_usage(cmdtp);

	eim_setup();

	cmd = argv[1];
	switch (cmd[0]) {
	case 's':
		eim_setup();
		break;
	case 'w':	/* pld or pci */
		if (cmd[1] == '4')
			eim_w4(argv[2]);
		else if (cmd[1] == '2')
			eim_w2(argv[2]);
		else
			eim_w1(argv[2]);
		break;
	default:
		return cmd_usage(cmdtp);
	}
	return 0;
}

U_BOOT_CMD(
	eim,	3,	1,	do_eim,
	"eim commands",
	"\n"
	"eim s         - setup eim interface\n"
	"eim w4 offset - write to offset [0-3]"
);
