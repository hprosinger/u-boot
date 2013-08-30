/*
 * Copyright (C) 2013 Sigmatek
 *
 * Configuration settings for the Sigmatek mx6 based boards
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_SYS_USE_NAND

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>
#include <asm/sizes.h>

#define CONFIG_MX6
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define MACH_TYPE_SIGMATEK_HZS731H		4373
#define MACH_TYPE_SIGMATEK_HGT1035H		4374
#define MACH_TYPE_SIGMATEK_HGT1035H_PROTO		4375	// Todo: remove one time
//#define CONFIG_MACH_TYPE		MACH_TYPE_SIGMATEK_MX6

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(3 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

/* Command definition */
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY		1

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 500 * SZ_1M)
#define CONFIG_LOADADDR			0x12000000
#define CONFIG_SYS_TEXT_BASE		0x17800000

#ifdef CONFIG_SYS_USE_NAND
#define CONFIG_CMD_NAND
#define CONFIG_CMD_NAND_TRIMFFS

/* NAND stuff */
#define CONFIG_NAND_MXS
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION

/* DMA stuff, needed for GPMI/MXS NAND support */
#define CONFIG_APBH_DMA
#define CONFIG_APBH_DMA_BURST
#define CONFIG_APBH_DMA_BURST8

/* MTD stuff */
#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS
#define CONFIG_CMD_MTDPARTS     /* Enable MTD parts commands */
#define MTDIDS_DEFAULT          "nand0=gpmi-nand"
#define MTDPARTS_DEFAULT        "mtdparts=gpmi-nand:16m(uboot),-(ubisystem)"

/* UBIFS stuff */
#define CONFIG_CMD_UBI          /* UBI commands */
#define CONFIG_CMD_UBIFS        /* UBIFS commands */
#define CONFIG_RBTREE
#define CONFIG_LZO              /* LZO is needed for UBIFS */

#endif /* CONFIG_SYS_USE_NAND */

/* MMC Configuration */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

/* Ethernet Configuration */
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		MII100
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		1
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL

/* I2C Configs */
#define CONFIG_DETECT_HZS
#define CONFIG_HARD_I2C
#define CONFIG_I2C_MXC
#define CONFIG_SYS_I2C_BASE                 I2C3_BASE_ADDR
#define CONFIG_SYS_I2C_SPEED                100000
#define CONFIG_SYS_I2C_EEPROM_ADDR          0x50
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN      2

/* Device tree config */
#if defined(CONFIG_MX6DL)
#define CONFIG_DEFAULT_FDT_FILE		"imx6dl-sigmatek_mx6.dtb"
#elif defined(CONFIG_MX6S)
#define CONFIG_DEFAULT_FDT_FILE		"imx6s-sigmatek_mx6.dtb"
#endif

#ifdef CONFIG_SYS_BOOT_NAND
	/*
	 * The partions' layout for NAND is:
	 *     mtd0: 16M      (uboot)
	 *     mtd1: left     (ubisystem)
	 */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"mtdparts=" MTDPARTS_DEFAULT "\0" \
	"bootargs=console=ttymxc0,115200 video=mxcfb0 " \
		"ubi.mtd=1 root=ubi0:system rootfstype=ubifs " MTDPARTS_DEFAULT "\0" \
	"bootcmd=ubi part ubisystem;ubifsmount ubi0:bootldr;ubifsload ${loadaddr} uImage;" \
		"bootm\0"

#elif CONFIG_SYS_BOOT_UPDATER
#define CONFIG_BOOTARGS \
	"console=ttymxc0,115200 rdinit=/linuxrc"
#define CONFIG_BOOTCOMMAND \
	"bootm 0x10800000 0x10c00000"
#undef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY 0
#undef CONFIG_EXTRA_ENV_SETTINGS

#else
#define CONFIG_EXTRA_ENV_SETTINGS \
	"script=boot.scr\0" \
	"uimage=uImage\0" \
	"console=ttymxc0\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"fdt_addr=0x11000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"mmcdev=0\0" \
	"mmcpart=1\0" \
	"mmcroot=/dev/mmcblk0p3 rootwait rw\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"root=${mmcroot}\0" \
	"loadbootscript=" \
		"if ext2load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script}; then " \
			"true; " \
		"else " \
			"ext2load mmc ${mmcdev}:${mmcpart} ${loadaddr} backup/${script}; " \
		"fi;\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loaduimage=ext2load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${uimage}\0" \
	"loadfdt=ext2load mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootm ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootm; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootm; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
	"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
		"netboot=echo Booting from net ...; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${uimage}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
				"bootm ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootm; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootm; " \
		"fi;\0"

/* check more devices that may contain the bootscript (SD1, and SD3)
 * this code should be kicked for the final product!!!!!
*/
#define CONFIG_BOOTCOMMAND \
	   "mmc dev ${mmcdev}; " \
	   "if mmc rescan; then " \
		"true; " \
	   "else " \
		"setenv mmcdev 1; " \
		"mmc dev ${mmcdev}; " \
		"mmc rescan; " \
	   "fi; " \
	   "if run loadbootscript; then " \
	   	"run bootscript; " \
	   "else " \
	 	"if run loaduimage; then " \
			"run mmcboot; " \
		"else " \
			"run netboot; " \
		"fi; " \
	   "fi; " 
#endif /* CONFIG_SYS_BOOT_NAND */

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT	       "=> "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE		256

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS	       16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

#define CONFIG_CMDLINE_EDITING

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(8 * 1024)

#ifdef CONFIG_SYS_BOOT_NAND
#define CONFIG_SYS_USE_NAND
#define CONFIG_ENV_IS_IN_NAND
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_OFFSET		(8 << 20)
#define CONFIG_ENV_SECT_SIZE		(128 << 10)
#define CONFIG_ENV_SIZE			CONFIG_ENV_SECT_SIZE
#elif CONFIG_SYS_BOOT_UPDATER
#define CONFIG_ENV_IS_NOWHERE
#else
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0
#endif

#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_BOOTZ

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

#endif			       /* __CONFIG_H * */
