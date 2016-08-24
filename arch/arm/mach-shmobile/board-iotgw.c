/*
 * Balboa IoT Gateway board support
 *
 * Based on Farwater-Geo board.
 * Based on RSKRZA1 board.
 *
 * Copyright (C) 2016  PortalCo
 * Copyright (C) 2015  Andrey P. Vasilyev
 * Copyright (C) 2013  Renesas Solutions Corp.
 * Copyright (C) 2013  Magnus Damm
 * Copyright (C) 2014  Chris Brandt
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/sh_eth.h>
#include <asm/mach/map.h>
#include "common.h"
#include "irqs.h"
#include "r7s72100.h"
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/cache-l2x0.h>
#include <linux/spi/rspi.h>
#include <linux/spi/sh_spibsc.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/serial_sci.h>
#include <linux/i2c.h>
#include <linux/i2c-riic.h>
#include <linux/mfd/tmio.h>
#include <linux/pwm.h>
#include <linux/platform_data/at24.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/usb/r8a66597.h>
#include <linux/platform_data/dma-rza1.h>
#include <linux/uio_driver.h>
#include <clocksource/sh_ostm.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <linux/can/platform/rza1_can.h>
#include <linux/pps-gpio.h>
#include <linux/leds.h>


static int usbgs = -1;
static int __init early_usbgs(char *str)
{
	usbgs = 0;
	get_option(&str, &usbgs);
	return 0;
}
early_param("usbgs", early_usbgs);

static struct map_desc rza1_io_desc[] __initdata = {
	/* create a 1:1 entity map for 0xe8xxxxxx
	 * used by INTC.
	 */
	{
		.virtual	= 0xe8000000,
		.pfn		= __phys_to_pfn(0xe8000000),
		.length		= SZ_256M,
		.type		= MT_DEVICE_NONSHARED
	},
	/* create a 1:1 entity map for 0xfcfexxxx
	 * used by MSTP, CPG.
	 */
	{
		.virtual	= 0xfcfe0000,
		.pfn		= __phys_to_pfn(0xfcfe0000),
		.length		= SZ_64K,
		.type		= MT_DEVICE_NONSHARED
	},
#ifdef CONFIG_CACHE_L2X0
	/* create a 1:1 entity map for 0x3ffffxxx
	 * used by L2CC (PL310).
	 */
	{
		.virtual	= 0xfffee000,
		.pfn		= __phys_to_pfn(0x3ffff000),
		.length		= SZ_4K,
		.type		= MT_DEVICE_NONSHARED
	},
#endif
};

void __init rza1_map_io(void)
{
#ifdef CONFIG_DEBUG_LL
	/* Note: Becase we defined a .map_io handler, we must manually set our
	   SCIF2 memory mapping here. see arch/arm/mm/mmu.c */
	debug_ll_io_init();
#endif
	iotable_init(rza1_io_desc, ARRAY_SIZE(rza1_io_desc));
}


/* DMA */
#define CHCFG(reqd_v, loen_v, hien_v, lvl_v, am_v, sds_v, dds_v, tm_v)\
	{								\
		.reqd	=	reqd_v,					\
		.loen	=	loen_v,					\
		.hien	=	hien_v,					\
		.lvl	=	lvl_v,					\
		.am	=	am_v,					\
		.sds	=	sds_v,					\
		.dds	=	dds_v,					\
		.tm	=	tm_v,					\
	}
#define DMARS(rid_v, mid_v)	\
	{								\
		.rid	= rid_v,					\
		.mid	= mid_v,					\
	}

static const struct rza1_dma_slave_config rza1_dma_slaves[] = {
	{
		.slave_id	= RZA1DMA_SLAVE_SDHI0_TX,
		.addr		= 0xe804e030,
		.chcfg		= CHCFG(0x1, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x1, 0x30),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_SDHI0_RX,
		.addr		= 0xe804e030,
		.chcfg		= CHCFG(0x0, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x2, 0x30),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_SDHI1_TX,
		.addr		= 0xe804e830,
		.chcfg		= CHCFG(0x1, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x1, 0x31),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_SDHI1_RX,
		.addr		= 0xe804e830,
		.chcfg		= CHCFG(0x0, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x2, 0x31),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_MMCIF_TX,
		.addr		= 0xe804c834,
		.chcfg		= CHCFG(0x1, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x1, 0x32),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_MMCIF_RX,
		.addr		= 0xe804c834,
		.chcfg		= CHCFG(0x0, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x2, 0x32),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_PCM_MEM_SSI0,
		.addr		= 0xe820b018,		/* SSIFTDR_0 */
		.chcfg		= CHCFG(0x1, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x1, 0x38),
	}, {
		.slave_id	= RZA1DMA_SLAVE_PCM_MEM_SRC1,
		.addr		= 0xe820970c,		/* DMATD1_CIM */
		.chcfg		= CHCFG(0x1, 0x0, 0x1, 0x1, 0x1, 0x1, 0x1, 0x0),
		.dmars		= DMARS(0x1, 0x41),
	}, {
		.slave_id	= RZA1DMA_SLAVE_PCM_SSI0_MEM,
		.addr		= 0xe820b01c,		/* SSIFRDR_0 */
		.chcfg		= CHCFG(0x0, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x2, 0x38),
	}, {
		.slave_id	= RZA1DMA_SLAVE_PCM_SRC0_MEM,
		.addr		= 0xe8209718,		/* DMATU0_CIM */
		.chcfg		= CHCFG(0x0, 0x0, 0x1, 0x1, 0x1, 0x1, 0x1, 0x0),
		.dmars		= DMARS(0x2, 0x40),
	}, {
		.slave_id	= RZA1DMA_SLAVE_SCIF1_TX,
		.addr		= 0xe800780c,		/* SCFTDR of SCIF1 */
		.chcfg		= CHCFG(0x1, 0x0, 0x1, 0x1, 0x2, 0x0, 0x0, 0x0),
		.dmars		= DMARS(0x1, 0x19),
	}, {
		.slave_id	= RZA1DMA_SLAVE_SCIF1_RX,
		.addr		= 0xe8007814,		/* SCFRDR of SCIF1 */
		.chcfg		= CHCFG(0x0, 0x0, 0x1, 0x1, 0x2, 0x0, 0x0, 0x0),
		.dmars		= DMARS(0x2, 0x19),
	},

};

static const struct rza1_dma_pdata dma_pdata __initconst = {
	.slave		= rza1_dma_slaves,
	.slave_num	= ARRAY_SIZE(rza1_dma_slaves),
	.channel_num	= 16,	/* 16 MAX channels */
};

static const struct resource rza1_dma_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8200000, 0x1000),
	DEFINE_RES_MEM(0xfcfe1000, 0x1000),
	DEFINE_RES_NAMED(gic_iid(41), 16, NULL, IORESOURCE_IRQ),
	DEFINE_RES_IRQ(gic_iid(57)),	/* DMAERR */
};

static const struct platform_device_info dma_info  __initconst = {
	.name		= "rza1-dma",
	.id		= -1,
	.res		= rza1_dma_resources,
	.num_res	= ARRAY_SIZE(rza1_dma_resources),
	.data		= &dma_pdata,
	.size_data	= sizeof(dma_pdata),
};

/* Ether */
static const struct sh_eth_plat_data ether_pdata __initconst = {
	.phy			= 0x00, /* PD60610 */
	.edmac_endian		= EDMAC_LITTLE_ENDIAN,
	.phy_interface		= PHY_INTERFACE_MODE_MII,
	.no_ether_link		= 1
};

static const struct resource ether_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8203000, 0x800),
	DEFINE_RES_MEM(0xe8204800, 0x200),
	DEFINE_RES_IRQ(gic_iid(359)),
};

static const struct platform_device_info ether_info __initconst = {
	.parent		= &platform_bus,
	.name		= "r7s72100-ether",
	.id		= -1,
	.res		= ether_resources,
	.num_res	= ARRAY_SIZE(ether_resources),
	.data		= &ether_pdata,
	.size_data	= sizeof(ether_pdata),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* OSTM */
static struct rza1_ostm_pdata ostm_pdata = {
	.clksrc.name = "ostm.0",
	.clksrc.rating = 300,
	.clkevt.name = "ostm.1",
	.clkevt.rating = 300,
};

static const struct resource ostm_resources[] __initconst = {
	[0] = DEFINE_RES_MEM_NAMED(0xfcfec000, 0x030, "ostm.0"),
	[1] = DEFINE_RES_MEM_NAMED(0xfcfec400, 0x030, "ostm.1"),
	[2] = DEFINE_RES_IRQ_NAMED(134, "ostm.0"),
	[3] = DEFINE_RES_IRQ_NAMED(135, "ostm.1"),
};

static const struct platform_device_info ostm_info __initconst = {
	.name		= "ostm",
	.id		= 0,
	.data 		= &ostm_pdata,
	.size_data	= sizeof(ostm_pdata),
	.res		= ostm_resources,
	.num_res	= ARRAY_SIZE(ostm_resources),
};

/* RTC */
static const struct resource rtc_resources[] __initconst = {
	DEFINE_RES_MEM(0xfcff1000, 0x2e),
	DEFINE_RES_IRQ(gic_iid(309)),	/* Period IRQ */
	DEFINE_RES_IRQ(gic_iid(310)),	/* Carry IRQ */
	DEFINE_RES_IRQ(gic_iid(308)),	/* Alarm IRQ */
};

static const struct platform_device_info rtc_info __initconst = {
	.parent		= &platform_bus,
	.name		= "sh-rtc",
	.id		= -1,
	.res		= rtc_resources,
	.num_res	= ARRAY_SIZE(rtc_resources),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* SPI NOR Flash */
/* Single Flash only */
static struct mtd_partition spibsc0_flash_partitions[] = {
	{
		.name		= "spibsc0_loader",
		.offset		= 0x00000000,
		.size		= 0x00080000,
		/* .mask_flags	= MTD_WRITEABLE, */
	},
	{
		.name		= "spibsc0_bootenv",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x00040000,
	},
	{
		.name		= "spibsc0_kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x00400000,
	},
	{
		.name		= "spibsc0_rootfs",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data spibsc0_flash_pdata = {
	.name	= "m25p80",
	.parts	= spibsc0_flash_partitions,
	.nr_parts = ARRAY_SIZE(spibsc0_flash_partitions),
	.type = "s25fl256s",
};

static struct mtd_partition rspi1_flash_partitions[] = {
	{
		.name		= "rspi1_data",
		.offset		= 0x00000000,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= MTD_WRITEABLE,
	},
};

static struct flash_platform_data rspi1_flash_pdata = {
	.name	= "m25p80",
	.parts	= rspi1_flash_partitions,
	.nr_parts = ARRAY_SIZE(rspi1_flash_partitions),
	.type = "n25q00",
};

/* QSPI Flash (Memory Map Mode, read only) */
static struct mtd_partition qspi_flash_partitions[] __initdata = {
	{
		.name		= "qspi_rootfs",
		.offset		= 0x00800000,
		.size		= 32 * SZ_1M - 0x00800000,
	},
};

static const struct physmap_flash_data qspi_flash_data __initconst = {
	.width		= 4,
	.probe_type	= "map_rom",
	.parts		= qspi_flash_partitions,
	.nr_parts	= ARRAY_SIZE(qspi_flash_partitions),
};

static const struct resource qspi_flash_resources[] __initconst = {
	DEFINE_RES_MEM(0x18000000, SZ_32M),
};

static const struct platform_device_info qspi_flash_info __initconst = {
	.parent		= &platform_bus,
	.name		= "physmap-flash",
	.id		= 0,
	.res		= qspi_flash_resources,
	.num_res	= ARRAY_SIZE(qspi_flash_resources),
	.data		= &qspi_flash_data,
	.size_data	= sizeof(qspi_flash_data),
	.dma_mask	= DMA_BIT_MASK(32),
};


/* RSPI */
#define RSPI_RESOURCE(idx, baseaddr, irq)				\
static const struct resource rspi##idx##_resources[] __initconst = {	\
	DEFINE_RES_MEM(baseaddr, 0x24),					\
	DEFINE_RES_IRQ_NAMED(irq, "error"),				\
	DEFINE_RES_IRQ_NAMED(irq + 1, "rx"),				\
	DEFINE_RES_IRQ_NAMED(irq + 2, "tx"),				\
}

//RSPI_RESOURCE(0, 0xe800c800, gic_iid(270));
RSPI_RESOURCE(1, 0xe800d000, gic_iid(273));
//RSPI_RESOURCE(2, 0xe800d800, gic_iid(276));
//RSPI_RESOURCE(3, 0xe800e000, gic_iid(279));
//RSPI_RESOURCE(4, 0xe800e800, gic_iid(282));

static const struct rspi_plat_data rspi_pdata __initconst = {
	.num_chipselect	= 1,
};

#define r7s72100_register_rspi(idx)					   \
	platform_device_register_resndata(&platform_bus, "rspi-rz", idx,   \
					rspi##idx##_resources,		   \
					ARRAY_SIZE(rspi##idx##_resources), \
					&rspi_pdata, sizeof(rspi_pdata))


static struct spi_board_info iotgw_spi_devices[] __initdata = {
	{
		/* SPI Flash0 */
		.modalias = "m25p80",
		.bus_num = 5,
		.chip_select = 0,
		.platform_data = &spibsc0_flash_pdata,
	},
	{
		/* SPI Flash1 */
		.modalias = "m25p80",
		.bus_num = 1,
		.chip_select = 0,
		.platform_data = &rspi1_flash_pdata,
	},
};

/* spibsc0 */
static const struct sh_spibsc_info spibsc0_pdata = {
	.bus_num	= 5,
};

static const struct resource spibsc0_resources[] __initconst = {
	DEFINE_RES_MEM(0x3fefa000, 0x100),
};

static const struct platform_device_info spibsc0_info __initconst = {
	.name		= "spibsc",
	.id		= 0,
	.data 		= &spibsc0_pdata,
	.size_data	= sizeof(spibsc0_pdata),
	.num_res	= ARRAY_SIZE(spibsc0_resources),
	.res		= spibsc0_resources,
};


/* USB Host */
static const struct r8a66597_platdata r8a66597_pdata __initconst = {
	.endian = 0,
	.on_chip = 1,
	.xtal = R8A66597_PLATDATA_XTAL_48MHZ,
};

static const struct resource r8a66597_usb_host0_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8010000, 0x1a0),
	DEFINE_RES_IRQ(gic_iid(73)),
};

static const struct platform_device_info r8a66597_usb_host0_info __initconst= {
	.name		= "r8a66597_hcd",
	.id		= 0,
	.data		= &r8a66597_pdata,
	.size_data	= sizeof(r8a66597_pdata),
	.res		= r8a66597_usb_host0_resources,
	.num_res	= ARRAY_SIZE(r8a66597_usb_host0_resources),
};

static const struct resource r8a66597_usb_host1_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8207000, 0x1a0),
	DEFINE_RES_IRQ(gic_iid(74)),
};

static const struct platform_device_info r8a66597_usb_host1_info __initconst = {
	.name		= "r8a66597_hcd",
	.id		= 1,
	.data		= &r8a66597_pdata,
	.size_data	= sizeof(r8a66597_pdata),
	.res		= r8a66597_usb_host1_resources,
	.num_res	= ARRAY_SIZE(r8a66597_usb_host1_resources),
};

/* USB Gadget */
static const struct r8a66597_platdata r8a66597_usb_gadget0_pdata __initconst = {
	.endian = 0,
	.on_chip = 1,
	.xtal = R8A66597_PLATDATA_XTAL_48MHZ,
};

static const struct resource r8a66597_usb_gadget0_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8010000, 0x1a0),
	DEFINE_RES_IRQ(gic_iid(73)),
};

static const struct platform_device_info r8a66597_usb_gadget0_info __initconst = {
	.name		= "r8a66597_udc",
	.id		= 0,
	.data		= &r8a66597_usb_gadget0_pdata,
	.size_data	= sizeof(r8a66597_usb_gadget0_pdata),
	.res		= r8a66597_usb_gadget0_resources,
	.num_res	= ARRAY_SIZE(r8a66597_usb_gadget0_resources),
};

static const struct r8a66597_platdata r8a66597_usb_gadget1_pdata __initconst = {
	.endian = 0,
	.on_chip = 1,
	.xtal = R8A66597_PLATDATA_XTAL_48MHZ,
};

static const struct resource r8a66597_usb_gadget1_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8207000, 0x1a0),
	DEFINE_RES_IRQ(gic_iid(74)),
};

static const struct platform_device_info r8a66597_usb_gadget1_info __initconst = {
	.name		= "r8a66597_udc",
	.id		= 1,
	.data		= &r8a66597_usb_gadget1_pdata,
	.size_data	= sizeof(r8a66597_usb_gadget1_pdata),
	.res		= r8a66597_usb_gadget1_resources,
	.num_res	= ARRAY_SIZE(r8a66597_usb_gadget1_resources),
};


static struct gpio_led iotgw_leds[] = {
        {
                .name		= "led2",
                .gpio		= 82, // Pin 17 is P6_0, named LED2
		.active_low	= 1,
                .default_state	= LEDS_GPIO_DEFSTATE_ON,
        },
        {
                .name		= "led3",
                .gpio		= 83, // Pin 19 is P6_1, named LED3
		.active_low	= 1,
                .default_state	= LEDS_GPIO_DEFSTATE_ON,
        },
};

static struct gpio_led_platform_data iotgw_leds_pdata = {
        .leds = iotgw_leds,
        .num_leds = ARRAY_SIZE(iotgw_leds),
};

static struct platform_device leds_device = {
	.name = "leds-gpio",
	.id = 0,
	.dev = {
		.platform_data  = &iotgw_leds_pdata,
	},
};



#ifdef CONFIG_CAN_RZA1
static struct resource rz_can_resources[] = {
	[0] = {
		.start	= 0xe803a000,
		.end	= 0xe803b813,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 258,
		.end	= 258,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= 260,
		.end	= 260,
		.flags	= IORESOURCE_IRQ,
	},
	[3] = {
		.start	= 259,
		.end	= 259,
		.flags	= IORESOURCE_IRQ,
	},
	[4] = {
		.start	= 253,
		.end	= 253,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct rz_can_platform_data rz_can_data = {
	.channel	= 1,
	.clock_select	= CLKR_CLKC,
};

static struct platform_device_info rz_can_device = {
	.name		= "rz_can",
	.id		= 1,
	.num_res	= ARRAY_SIZE(rz_can_resources),
	.res		= rz_can_resources,
	.data		= &rz_can_data,
	.size_data	= sizeof(rz_can_data),
};
#endif /* CONFIG_CAN_RZA1 */

/* By default, the Linux ARM code will pre-allocated IRQ descriptors
   based on the size (HW) of the GIC. For this device, that means
   576 possible interrutps sources. This eats up a lot of RAM at
   run-time that is pretty much wasted.
   Therefore, use this 'whitelist' below to delete any pre-allocated
   irq descriptors except those that is in this list to achive up to
   400KB of RAM savings.
*/
struct irq_res {
	int irq;	/* Starting IRQ number */
	int count;	/* The number of consecutive IRQs */
};
struct irq_res const irq_keep_list[] __initconst = {
//	{32, 1},	/* IRQ0 */
//	{33, 1},	/* IRQ1 */
//	{34, 1},	/* IRQ2 */
//	{35, 1},	/* IRQ3 */
//	{36, 1},	/* IRQ4 */
//	{37, 1},	/* IRQ5 */
//	{38, 1},	/* IRQ6 */
//	{39, 1},	/* IRQ7 */
//	{40, 1},	/* PL310ERR (L2 Cache error - not used) */
	{41, 17},	/* RZA1_DMA */
	{73, 1},	/* USB0 (host/device) */
	{74, 1},	/* USB1 (host/device) */
//	{75, 23},	/* VDC0 */
//	{99, 23},	/* VDC1 */
//	{126, 1},	/* JCU */
	{134, 2},	/* OSTM */
	{139, 1},	/* MTU2-TGI0A (Kernel jiffies) */
//	{170, 2}, {146, 2},	/* ADC and MTU2-TGI1A */
//	{189, 8},	/* RIIC0 */
//	{197, 8},	/* RIIC1 */
//	{205, 8},	/* RIIC2 */
//	{213, 8},	/* RIIC3 */
	{221, 4},	/* SCIF0 */
	{225, 4},	/* SCIF1 */
	{229, 4},	/* SCIF2 */
	{233, 4},	/* SCIF3 */
	{237, 4},	/* SCIF4 */
	{241, 4},	/* SCIF5 */
	{245, 4},	/* SCIF6 */
	{249, 4},	/* SCIF7 */
	{253, 2},	/* CAN GERR/GRECC */
//	{255, 3},	/* CAN0 */
//	{258, 3},	/* CAN1 */
//	{261, 3},	/* CAN2 */
	{264, 3},	/* CAN3 */
//	{267, 3},	/* CAN4 */
	{270, 3},	/* RSPI0 */
	{273, 3},	/* RSPI1 */
	{276, 3},	/* RSPI2 */
	{279, 3},	/* RSPI3 */
	{282, 3},	/* RSPI4 */
//	{299, 3},	/* MMC */
//	{302, 3},	/* SDHI0 */
//	{305, 3},	/* SDHI1 */
	{308, 3},	/* RTC */
	{347, 4},	/* SCI0 */
	{351, 4},	/* SCI1 */
	{359, 1},	/* ETH */
};

static int rz_irq_trim = 0;
static int __init early_rz_irq_trim(char *str)
{
	rz_irq_trim = 1;
	return 0;
}
early_param("rz_irq_trim", early_rz_irq_trim);

/* Removes unused pre-allocated IRQ. */
/* This operation only occurs when 'rz_irq_trim' is on the boot command line */
#ifdef CONFIG_XIP_KERNEL
static void remove_irqs(void)
{
	int i,j;
	int max = nr_irqs;
	int keep;

	if( !rz_irq_trim )
		return;		/* Feature disabled if 'rz_irq_trim' not set */

	/* Run through all allocated irq descriptors and delete
	   the ones we are not using */
	/* Skip 0 - 16 because they are the soft irqs */
	for (i=17; i < max; i++) {
		/* Is it one we want to keep? */
		for ( j=0, keep=0 ; j < sizeof(irq_keep_list)/sizeof(struct irq_res); j++) {
			if( i == irq_keep_list[j].irq ) {
				keep = 1;
				break;
			}
		}

		if( keep )
			i += irq_keep_list[j].count - 1; /* skip if multiple */
		else
			irq_free_descs(i, 1);		/* un-used irq */
	}
}
#endif


/* SCIF */
#define R7S72100_SCIF(index, baseaddr, irq, dmatx, dmarx)		\
static const struct plat_sci_port scif##index##_platform_data = {	\
	.type		= PORT_SCIF,					\
	.regtype	= SCIx_SH2_SCIF_FIFODATA_REGTYPE,		\
	.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP,		\
	.scscr		= SCSCR_RIE | SCSCR_TIE | SCSCR_RE | SCSCR_TE |	\
			  SCSCR_REIE,					\
	.dma_slave_tx = dmatx,						\
	.dma_slave_rx = dmarx,						\
};									\
									\
static struct resource scif##index##_resources[] = {			\
	DEFINE_RES_MEM(baseaddr, 0x100),				\
	DEFINE_RES_IRQ(irq + 1),					\
	DEFINE_RES_IRQ(irq + 2),					\
	DEFINE_RES_IRQ(irq + 3),					\
	DEFINE_RES_IRQ(irq),						\
}									\

//R7S72100_SCIF(0, 0xe8007000, gic_iid(221), 0, 0);
R7S72100_SCIF(1, 0xe8007800, gic_iid(225), 0, 0);
R7S72100_SCIF(2, 0xe8008000, gic_iid(229), 0, 0);
//R7S72100_SCIF(3, 0xe8008800, gic_iid(233), 0, 0);
//R7S72100_SCIF(4, 0xe8009000, gic_iid(237), 0, 0);
//R7S72100_SCIF(5, 0xe8009800, gic_iid(241), 0, 0);
//R7S72100_SCIF(6, 0xe800a000, gic_iid(245), 0, 0);
//R7S72100_SCIF(7, 0xe800a800, gic_iid(249), 0, 0);

#define r7s72100_register_scif(index)					\
	do {								\
		struct platform_device *dev =				\
			platform_device_register_resndata (		\
				&platform_bus, "sh-sci", index,		\
				scif##index##_resources,		\
				ARRAY_SIZE(scif##index##_resources),	\
				&scif##index##_platform_data,		\
				sizeof(scif##index##_platform_data));	\
		dma_set_mask_and_coherent (&(dev->dev), DMA_BIT_MASK(32)); \
		dma_set_coherent_mask (&(dev->dev), DMA_BIT_MASK(32)); \
	} while (0)

/* SCI: SCI0 and SCI1 */
#define R7S72100_SCI(index, baseaddr, irq)				\
static const struct plat_sci_port sci##index##_platform_data = {	\
	.type		= PORT_SCI,					\
	.regtype	= SCIx_SCI_REGTYPE,				\
	.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP,		\
	.scscr		= SCSCR_RIE | SCSCR_TIE | SCSCR_RE | SCSCR_TE |	\
			  SCSCR_REIE,					\
};									\
									\
static struct resource sci##index##_resources[] = {			\
	DEFINE_RES_MEM(baseaddr, 0x10),					\
	DEFINE_RES_IRQ(irq + 1),					\
	DEFINE_RES_IRQ(irq + 2),					\
	DEFINE_RES_IRQ(irq + 3),					\
	DEFINE_RES_IRQ(irq),						\
}									\

//R7S72100_SCI(0, 0xe800b000, gic_iid(347));
R7S72100_SCI(1, 0xe800b800, gic_iid(351));

#define r7s72100_register_sci(index)					       \
	platform_device_register_resndata(&platform_bus, "sh-sci", index+8,    \
					  sci##index##_resources,	       \
					  ARRAY_SIZE(sci##index##_resources),  \
					  &sci##index##_platform_data,	       \
					  sizeof(sci##index##_platform_data))


static void __init iotgw_add_standard_devices(void)
{
#ifdef CONFIG_CACHE_L2X0
	/* Early BRESP enable, 16K*8way(defualt) */
	/* NOTES: BRESP can be set for IP version after r2p0 */
	/*        As of linux-3.16, cache-l2x0.c handles this automatically */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,16,0)
	l2x0_init(IOMEM(0xfffee000), 0x40000000, 0xffffffff);	/* Early BRESP enable */
#else
	l2x0_init(IOMEM(0xfffee000), 0x00000000, 0xffffffff);	/* Leave as defaults */
#endif
#endif
#ifdef CONFIG_XIP_KERNEL
	remove_irqs();
#endif

	r7s72100_clock_init();
	r7s72100_pinmux_setup();
	r7s72100_add_dt_devices();

#ifdef CONFIG_CAN_RZA1
	platform_device_register_full(&rz_can_device);
#endif

	platform_device_register_full(&ostm_info);
	platform_device_register_full(&dma_info);
	platform_device_register_full(&ether_info);

	platform_device_register_full(&rtc_info);

	platform_device_register(&leds_device);

#if !defined(CONFIG_XIP_KERNEL) && defined(CONFIG_SPI_SH_SPIBSC)
	platform_device_register_full(&spibsc0_info);
#else
	/* Need to disable spibsc if using memory mapped QSPI */
	platform_device_register_full(&qspi_flash_info);
#endif

	if (usbgs == 0) {
		platform_device_register_full(&r8a66597_usb_gadget0_info);
		platform_device_register_full(&r8a66597_usb_host1_info);
	} else if (usbgs == 1) {
		platform_device_register_full(&r8a66597_usb_host0_info);
		platform_device_register_full(&r8a66597_usb_gadget1_info);
	} else {
		platform_device_register_full(&r8a66597_usb_host0_info);
		platform_device_register_full(&r8a66597_usb_host1_info);
	}

//	r7s72100_register_rspi(0);	/* Not used */
	r7s72100_register_rspi(1);	/* 128MB SPI flash */
//	r7s72100_register_rspi(2);	/* Not used */
//	r7s72100_register_rspi(3);	/* Not used */
//	r7s72100_register_rspi(4);	/* Not used */

	/* Register SPI device information */
	spi_register_board_info(iotgw_spi_devices,
				ARRAY_SIZE(iotgw_spi_devices));

//	r7s72100_register_scif(0);
	r7s72100_register_scif(1);
	r7s72100_register_scif(2);
//	r7s72100_register_scif(3);
//	r7s72100_register_scif(4);

	// Register also SCI0 and SCI1 ports
//	r7s72100_register_sci(0);
	r7s72100_register_sci(1);
}

#define WTCSR 0
#define WTCNT 2
#define WRCSR 4
static void rza1_restart(enum reboot_mode mode, const char *cmd)
{
	void *base = ioremap_nocache(0xFCFE0000, 0x10);

	/* Dummy read (must read WRCSR:WOVF at least once before clearing) */
	*(volatile uint8_t *)(base + WRCSR) = *(uint8_t *)(base + WRCSR);

	*(volatile uint16_t *)(base + WRCSR) = 0xA500;	/* Clear WOVF */
	*(volatile uint16_t *)(base + WRCSR) = 0x5A5F;	/* Reset Enable */
	*(volatile uint16_t *)(base + WTCNT) = 0x5A00;	/* Counter to 00 */
	*(volatile uint16_t *)(base + WTCSR) = 0xA578;	/* Start timer */

	while(1); /* Wait for WDT overflow */
}

void __init iotgw_init_early(void)
{
	shmobile_init_delay();

#ifdef CONFIG_XIP_KERNEL
	/* Set the size of our pre-allocated DMA buffer pool because the
	   default is 256KB */
	init_dma_coherent_pool_size(16 * SZ_1K);
#endif
}

static const char * const iotgw_boards_compat_dt[] __initconst = {
	"renesas,iotgw",
	NULL,
};

DT_MACHINE_START(IOTGW_DT, "iotgw")
	.init_early	= iotgw_init_early,
	.init_machine	= iotgw_add_standard_devices,
	.dt_compat	= iotgw_boards_compat_dt,
	.map_io		= rza1_map_io,
	.restart	= rza1_restart,
MACHINE_END
