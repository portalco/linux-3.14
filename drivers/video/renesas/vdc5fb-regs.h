/*
 * Copyright (C) 2013-2014 Renesas Solutions Corp.
 *
 * Based on drivers/video/ren_vdc4.c
 * Copyright (c) 2012 Renesas Electronics Europe Ltd
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
 *
 */

#ifndef _VDC5FB_REGS_H_
#define _VDC5FB_REGS_H_

/* REGISTER INDEX */
enum {
	/* INPUT CONTROLLER */
	INP_UPDATE,
	INP_SEL_CNT,
	INP_EXT_SYNC_CNT,
	INP_VSYNC_PH_ADJ,
	INP_DLY_ADJ,

	/* IMAGE QUALITY ADJUSTMENT BLOCK */
	IMGCNT_UPDATE,
	IMGCNT_NR_CNT0,
	IMGCNT_NR_CNT1,
	IMGCNT_MTX_MODE,
	IMGCNT_MTX_YG_ADJ0,
	IMGCNT_MTX_YG_ADJ1,
	IMGCNT_MTX_CBB_ADJ0,
	IMGCNT_MTX_CBB_ADJ1,
	IMGCNT_MTX_CRR_ADJ0,
	IMGCNT_MTX_CRR_ADJ1,
	IMGCNT_DRC_REG,

	/* SCALER 0 */
	SC0_SCL0_UPDATE,
	SC0_SCL0_FRC1,
	SC0_SCL0_FRC2,
	SC0_SCL0_FRC3,
	SC0_SCL0_FRC4,
	SC0_SCL0_FRC5,
	SC0_SCL0_FRC6,
	SC0_SCL0_FRC7,
	SC0_SCL0_FRC9,
	SC0_SCL0_MON0,
	SC0_SCL0_INT,
	SC0_SCL0_DS1,
	SC0_SCL0_DS2,
	SC0_SCL0_DS3,
	SC0_SCL0_DS4,
	SC0_SCL0_DS5,
	SC0_SCL0_DS6,
	SC0_SCL0_DS7,
	SC0_SCL0_US1,
	SC0_SCL0_US2,
	SC0_SCL0_US3,
	SC0_SCL0_US4,
	SC0_SCL0_US5,
	SC0_SCL0_US6,
	SC0_SCL0_US7,
	SC0_SCL0_US8,
	SC0_SCL0_OVR1,
	SC0_SCL1_UPDATE,
	SC0_SCL1_WR1,
	SC0_SCL1_WR2,
	SC0_SCL1_WR3,
	SC0_SCL1_WR4,
	SC0_SCL1_WR5,
	SC0_SCL1_WR6,
	SC0_SCL1_WR7,
	SC0_SCL1_WR8,
	SC0_SCL1_WR9,
	SC0_SCL1_WR10,
	SC0_SCL1_WR11,
	SC0_SCL1_MON1,
	SC0_SCL1_PBUF0,
	SC0_SCL1_PBUF1,
	SC0_SCL1_PBUF2,
	SC0_SCL1_PBUF3,
	SC0_SCL1_PBUF_FLD,
	SC0_SCL1_PBUF_CNT,

	/* GRAPHICS 0 */
	GR0_UPDATE,
	GR0_FLM_RD,
	GR0_FLM1,
	GR0_FLM2,
	GR0_FLM3,
	GR0_FLM4,
	GR0_FLM5,
	GR0_FLM6,
	GR0_AB1,
	GR0_AB2,
	GR0_AB3,
	GR0_AB7,
	GR0_AB8,
	GR0_AB9,
	GR0_AB10,
	GR0_AB11,
	GR0_BASE,
	GR0_CLUT,
	GR0_MON,

	/* SCALER 1 */
	SC1_SCL0_UPDATE,
	SC1_SCL0_FRC1,
	SC1_SCL0_FRC2,
	SC1_SCL0_FRC3,
	SC1_SCL0_FRC4,
	SC1_SCL0_FRC5,
	SC1_SCL0_FRC6,
	SC1_SCL0_FRC7,
	SC1_SCL0_FRC9,
	SC1_SCL0_MON0,
	SC1_SCL0_INT,
	SC1_SCL0_DS1,
	SC1_SCL0_DS2,
	SC1_SCL0_DS3,
	SC1_SCL0_DS4,
	SC1_SCL0_DS5,
	SC1_SCL0_DS6,
	SC1_SCL0_DS7,
	SC1_SCL0_US1,
	SC1_SCL0_US2,
	SC1_SCL0_US3,
	SC1_SCL0_US4,
	SC1_SCL0_US5,
	SC1_SCL0_US6,
	SC1_SCL0_US7,
	SC1_SCL0_US8,
	SC1_SCL0_OVR1,
	SC1_SCL1_UPDATE,
	SC1_SCL1_WR1,
	SC1_SCL1_WR2,
	SC1_SCL1_WR3,
	SC1_SCL1_WR4,
	SC1_SCL1_WR5,
	SC1_SCL1_WR6,
	SC1_SCL1_WR7,
	SC1_SCL1_WR8,
	SC1_SCL1_WR9,
	SC1_SCL1_WR10,
	SC1_SCL1_WR11,
	SC1_SCL1_MON1,
	SC1_SCL1_PBUF0,
	SC1_SCL1_PBUF1,
	SC1_SCL1_PBUF2,
	SC1_SCL1_PBUF3,
	SC1_SCL1_PBUF_FLD,
	SC1_SCL1_PBUF_CNT,

	/* GRAPHICS 1 */
	GR1_UPDATE,
	GR1_FLM_RD,
	GR1_FLM1,
	GR1_FLM2,
	GR1_FLM3,
	GR1_FLM4,
	GR1_FLM5,
	GR1_FLM6,
	GR1_AB1,
	GR1_AB2,
	GR1_AB3,
	GR1_AB4,
	GR1_AB5,
	GR1_AB6,
	GR1_AB7,
	GR1_AB8,
	GR1_AB9,
	GR1_AB10,
	GR1_AB11,
	GR1_BASE,
	GR1_CLUT,
	GR1_MON,

	/* IMAGE QUALITY IMPROVER 0 */
	ADJ0_UPDATE,
	ADJ0_BKSTR_SET,
	ADJ0_ENH_TIM1,
	ADJ0_ENH_TIM2,
	ADJ0_ENH_TIM3,
	ADJ0_ENH_SHP1,
	ADJ0_ENH_SHP2,
	ADJ0_ENH_SHP3,
	ADJ0_ENH_SHP4,
	ADJ0_ENH_SHP5,
	ADJ0_ENH_SHP6,
	ADJ0_ENH_LTI1,
	ADJ0_ENH_LTI2,
	ADJ0_MTX_MODE,
	ADJ0_MTX_YG_ADJ0,
	ADJ0_MTX_YG_ADJ1,
	ADJ0_MTX_CBB_ADJ0,
	ADJ0_MTX_CBB_ADJ1,
	ADJ0_MTX_CRR_ADJ0,
	ADJ0_MTX_CRR_ADJ1,

	/* IMAGE QUALITY IMPROVER 1 */
	ADJ1_UPDATE,
	ADJ1_BKSTR_SET,
	ADJ1_ENH_TIM1,
	ADJ1_ENH_TIM2,
	ADJ1_ENH_TIM3,
	ADJ1_ENH_SHP1,
	ADJ1_ENH_SHP2,
	ADJ1_ENH_SHP3,
	ADJ1_ENH_SHP4,
	ADJ1_ENH_SHP5,
	ADJ1_ENH_SHP6,
	ADJ1_ENH_LTI1,
	ADJ1_ENH_LTI2,
	ADJ1_MTX_MODE,
	ADJ1_MTX_YG_ADJ0,
	ADJ1_MTX_YG_ADJ1,
	ADJ1_MTX_CBB_ADJ0,
	ADJ1_MTX_CBB_ADJ1,
	ADJ1_MTX_CRR_ADJ0,
	ADJ1_MTX_CRR_ADJ1,

	/* GRAPHICS 2 */
	GR2_UPDATE,
	GR2_FLM_RD,
	GR2_FLM1,
	GR2_FLM2,
	GR2_FLM3,
	GR2_FLM4,
	GR2_FLM5,
	GR2_FLM6,
	GR2_AB1,
	GR2_AB2,
	GR2_AB3,
	GR2_AB4,
	GR2_AB5,
	GR2_AB6,
	GR2_AB7,
	GR2_AB8,
	GR2_AB9,
	GR2_AB10,
	GR2_AB11,
	GR2_BASE,
	GR2_CLUT,
	GR2_MON,

	/* GRAPHICS 3 */
	GR3_UPDATE,
	GR3_FLM_RD,
	GR3_FLM1,
	GR3_FLM2,
	GR3_FLM3,
	GR3_FLM4,
	GR3_FLM5,
	GR3_FLM6,
	GR3_AB1,
	GR3_AB2,
	GR3_AB3,
	GR3_AB4,
	GR3_AB5,
	GR3_AB6,
	GR3_AB7,
	GR3_AB8,
	GR3_AB9,
	GR3_AB10,
	GR3_AB11,
	GR3_BASE,
	GR3_CLUT_INT,
	GR3_MON,

	/* VIN SYNTHESIZER */
	GR_VIN_UPDATE,
	GR_VIN_AB1,
	GR_VIN_AB2,
	GR_VIN_AB3,
	GR_VIN_AB4,
	GR_VIN_AB5,
	GR_VIN_AB6,
	GR_VIN_AB7,
	GR_VIN_BASE,
	GR_VIN_MON,

	/* OUTPUT IMAGE GENERATOR */
	OIR_SCL0_UPDATE,
	OIR_SCL0_FRC1,
	OIR_SCL0_FRC2,
	OIR_SCL0_FRC3,
	OIR_SCL0_FRC4,
	OIR_SCL0_FRC5,
	OIR_SCL0_FRC6,
	OIR_SCL0_FRC7,
	OIR_SCL0_DS1,
	OIR_SCL0_DS2,
	OIR_SCL0_DS3,
	OIR_SCL0_DS7,
	OIR_SCL0_US1,
	OIR_SCL0_US2,
	OIR_SCL0_US3,
	OIR_SCL0_US8,
	OIR_SCL0_OVR1,
	OIR_SCL1_UPDATE,
	OIR_SCL1_WR1,
	OIR_SCL1_WR2,
	OIR_SCL1_WR3,
	OIR_SCL1_WR4,
	OIR_SCL1_WR5,
	OIR_SCL1_WR6,
	OIR_SCL1_WR7,

	/* GRAPHICS OIR */
	GR_OIR_UPDATE,
	GR_OIR_FLM_RD,
	GR_OIR_FLM1,
	GR_OIR_FLM2,
	GR_OIR_FLM3,
	GR_OIR_FLM4,
	GR_OIR_FLM5,
	GR_OIR_FLM6,
	GR_OIR_AB1,
	GR_OIR_AB2,
	GR_OIR_AB3,
	GR_OIR_AB7,
	GR_OIR_AB8,
	GR_OIR_AB9,
	GR_OIR_AB10,
	GR_OIR_AB11,
	GR_OIR_BASE,
	GR_OIR_CLUT,
	GR_OIR_MON,

	/* GAMMA CORRECTION BLOCK */
	GAM_G_UPDATE,
	GAM_SW,
	GAM_G_LUT1,
	GAM_G_LUT2,
	GAM_G_LUT3,
	GAM_G_LUT4,
	GAM_G_LUT5,
	GAM_G_LUT6,
	GAM_G_LUT7,
	GAM_G_LUT8,
	GAM_G_LUT9,
	GAM_G_LUT10,
	GAM_G_LUT11,
	GAM_G_LUT12,
	GAM_G_LUT13,
	GAM_G_LUT14,
	GAM_G_LUT15,
	GAM_G_LUT16,
	GAM_G_AREA1,
	GAM_G_AREA2,
	GAM_G_AREA3,
	GAM_G_AREA4,
	GAM_G_AREA5,
	GAM_G_AREA6,
	GAM_G_AREA7,
	GAM_G_AREA8,
	GAM_B_UPDATE,
	GAM_B_LUT1,
	GAM_B_LUT2,
	GAM_B_LUT3,
	GAM_B_LUT4,
	GAM_B_LUT5,
	GAM_B_LUT6,
	GAM_B_LUT7,
	GAM_B_LUT8,
	GAM_B_LUT9,
	GAM_B_LUT10,
	GAM_B_LUT11,
	GAM_B_LUT12,
	GAM_B_LUT13,
	GAM_B_LUT14,
	GAM_B_LUT15,
	GAM_B_LUT16,
	GAM_B_AREA1,
	GAM_B_AREA2,
	GAM_B_AREA3,
	GAM_B_AREA4,
	GAM_B_AREA5,
	GAM_B_AREA6,
	GAM_B_AREA7,
	GAM_B_AREA8,
	GAM_R_UPDATE,
	GAM_R_LUT1,
	GAM_R_LUT2,
	GAM_R_LUT3,
	GAM_R_LUT4,
	GAM_R_LUT5,
	GAM_R_LUT6,
	GAM_R_LUT7,
	GAM_R_LUT8,
	GAM_R_LUT9,
	GAM_R_LUT10,
	GAM_R_LUT11,
	GAM_R_LUT12,
	GAM_R_LUT13,
	GAM_R_LUT14,
	GAM_R_LUT15,
	GAM_R_LUT16,
	GAM_R_AREA1,
	GAM_R_AREA2,
	GAM_R_AREA3,
	GAM_R_AREA4,
	GAM_R_AREA5,
	GAM_R_AREA6,
	GAM_R_AREA7,
	GAM_R_AREA8,

	/* TCON BLOCK */
	TCON_UPDATE,
	TCON_TIM,
	TCON_TIM_STVA1,
	TCON_TIM_STVA2,
	TCON_TIM_STVB1,
	TCON_TIM_STVB2,
	TCON_TIM_STH1,
	TCON_TIM_STH2,
	TCON_TIM_STB1,
	TCON_TIM_STB2,
	TCON_TIM_CPV1,
	TCON_TIM_CPV2,
	TCON_TIM_POLA1,
	TCON_TIM_POLA2,
	TCON_TIM_POLB1,
	TCON_TIM_POLB2,
	TCON_TIM_DE,

	/* OUTPUT CONTROLLER */
	OUT_UPDATE,
	OUT_SET,
	OUT_BRIGHT1,
	OUT_BRIGHT2,
	OUT_CONTRAST,
	OUT_PDTHA,
	OUT_CLK_PHASE,

	/* SYSTEM CONTROLLER */
	SYSCNT_INT1,
	SYSCNT_INT2,
	SYSCNT_INT3,
	SYSCNT_INT4,
	SYSCNT_INT5,
	SYSCNT_INT6,
	SYSCNT_PANEL_CLK,
	SYSCNT_CLUT,

	VDC5FB_MAX_REGS,
};

/* IRQ INDEX */
enum {
	S0_VI_VSYNC = 0,
	S0_LO_VSYNC,
	S0_VSYNCERR,
	GR3_VLINE,
	S0_VFIELD,
	IV1_VBUFERR,
	IV3_VBUFERR,
	IV5_VBUFERR,
	IV6_VBUFERR,
	S0_WLINE,
	S1_VI_VSYNC,
	S1_LO_VSYNC,
	S1_VSYNCERR,
	S1_VFIELD,
	IV2_VBUFERR,
	IV4_VBUFERR,
	S1_WLINE,
	OIR_VI_VSYNC,
	OIR_LO_VSYNC,
	OIR_VLINE,
	OIR_VFIELD,
	IV7_VBUFERR,
	IV8_VBUFERR,
	VDC5FB_MAX_IRQS,
};

/* REGISTER ADDRESS OFFSET */
#define	VDC5FB_OFFSET(x)	((x) - VDC5FB_REG_BASE(0))
static unsigned long vdc5fb_offsets[VDC5FB_MAX_REGS] = {
	/* INPUT CONTROLLER */
	[INP_UPDATE]		= VDC5FB_OFFSET(0xFCFF7400),
	[INP_SEL_CNT]		= VDC5FB_OFFSET(0xFCFF7404),
	[INP_EXT_SYNC_CNT]	= VDC5FB_OFFSET(0xFCFF7408),
	[INP_VSYNC_PH_ADJ]	= VDC5FB_OFFSET(0xFCFF740C),
	[INP_DLY_ADJ]		= VDC5FB_OFFSET(0xFCFF7410),

	/* IMAGE QUALITY ADJUSTMENT BLOCK */
	[IMGCNT_UPDATE]		= VDC5FB_OFFSET(0xFCFF7480),
	[IMGCNT_NR_CNT0]	= VDC5FB_OFFSET(0xFCFF7484),
	[IMGCNT_NR_CNT1]	= VDC5FB_OFFSET(0xFCFF7488),
	[IMGCNT_MTX_MODE]	= VDC5FB_OFFSET(0xFCFF74A0),
	[IMGCNT_MTX_YG_ADJ0]	= VDC5FB_OFFSET(0xFCFF74A4),
	[IMGCNT_MTX_YG_ADJ1]	= VDC5FB_OFFSET(0xFCFF74A8),
	[IMGCNT_MTX_CBB_ADJ0]	= VDC5FB_OFFSET(0xFCFF74AC),
	[IMGCNT_MTX_CBB_ADJ1]	= VDC5FB_OFFSET(0xFCFF74B0),
	[IMGCNT_MTX_CRR_ADJ0]	= VDC5FB_OFFSET(0xFCFF74B4),
	[IMGCNT_MTX_CRR_ADJ1]	= VDC5FB_OFFSET(0xFCFF74B8),
	[IMGCNT_DRC_REG]	= VDC5FB_OFFSET(0xFCFF74C0),

	/* SCALER 0 */
	[SC0_SCL0_UPDATE]	= VDC5FB_OFFSET(0xFCFF7500),
	[SC0_SCL0_FRC1]		= VDC5FB_OFFSET(0xFCFF7504),
	[SC0_SCL0_FRC2]		= VDC5FB_OFFSET(0xFCFF7508),
	[SC0_SCL0_FRC3]		= VDC5FB_OFFSET(0xFCFF750C),
	[SC0_SCL0_FRC4]		= VDC5FB_OFFSET(0xFCFF7510),
	[SC0_SCL0_FRC5]		= VDC5FB_OFFSET(0xFCFF7514),
	[SC0_SCL0_FRC6]		= VDC5FB_OFFSET(0xFCFF7518),
	[SC0_SCL0_FRC7]		= VDC5FB_OFFSET(0xFCFF751C),
	[SC0_SCL0_FRC9]		= VDC5FB_OFFSET(0xFCFF7524),
	[SC0_SCL0_MON0]		= VDC5FB_OFFSET(0xFCFF7528),	/* 16bit */
	[SC0_SCL0_INT]		= VDC5FB_OFFSET(0xFCFF752A),	/* 16bit */
	[SC0_SCL0_DS1]		= VDC5FB_OFFSET(0xFCFF752C),
	[SC0_SCL0_DS2]		= VDC5FB_OFFSET(0xFCFF7530),
	[SC0_SCL0_DS3]		= VDC5FB_OFFSET(0xFCFF7534),
	[SC0_SCL0_DS4]		= VDC5FB_OFFSET(0xFCFF7538),
	[SC0_SCL0_DS5]		= VDC5FB_OFFSET(0xFCFF753C),
	[SC0_SCL0_DS6]		= VDC5FB_OFFSET(0xFCFF7540),
	[SC0_SCL0_DS7]		= VDC5FB_OFFSET(0xFCFF7544),
	[SC0_SCL0_US1]		= VDC5FB_OFFSET(0xFCFF7548),
	[SC0_SCL0_US2]		= VDC5FB_OFFSET(0xFCFF754C),
	[SC0_SCL0_US3]		= VDC5FB_OFFSET(0xFCFF7550),
	[SC0_SCL0_US4]		= VDC5FB_OFFSET(0xFCFF7554),
	[SC0_SCL0_US5]		= VDC5FB_OFFSET(0xFCFF7558),
	[SC0_SCL0_US6]		= VDC5FB_OFFSET(0xFCFF755C),
	[SC0_SCL0_US7]		= VDC5FB_OFFSET(0xFCFF7560),
	[SC0_SCL0_US8]		= VDC5FB_OFFSET(0xFCFF7564),
	[SC0_SCL0_OVR1]		= VDC5FB_OFFSET(0xFCFF756C),
	[SC0_SCL1_UPDATE]	= VDC5FB_OFFSET(0xFCFF7580),
	[SC0_SCL1_WR1]		= VDC5FB_OFFSET(0xFCFF7588),
	[SC0_SCL1_WR2]		= VDC5FB_OFFSET(0xFCFF758C),
	[SC0_SCL1_WR3]		= VDC5FB_OFFSET(0xFCFF7590),
	[SC0_SCL1_WR4]		= VDC5FB_OFFSET(0xFCFF7594),
	[SC0_SCL1_WR5]		= VDC5FB_OFFSET(0xFCFF759C),
	[SC0_SCL1_WR6]		= VDC5FB_OFFSET(0xFCFF75A0),
	[SC0_SCL1_WR7]		= VDC5FB_OFFSET(0xFCFF75A4),
	[SC0_SCL1_WR8]		= VDC5FB_OFFSET(0xFCFF75A8),
	[SC0_SCL1_WR9]		= VDC5FB_OFFSET(0xFCFF75AC),
	[SC0_SCL1_WR10]		= VDC5FB_OFFSET(0xFCFF75B0),
	[SC0_SCL1_WR11]		= VDC5FB_OFFSET(0xFCFF75B4),
	[SC0_SCL1_MON1]		= VDC5FB_OFFSET(0xFCFF75B8),
	[SC0_SCL1_PBUF0]	= VDC5FB_OFFSET(0xFCFF75BC),
	[SC0_SCL1_PBUF1]	= VDC5FB_OFFSET(0xFCFF75C0),
	[SC0_SCL1_PBUF2]	= VDC5FB_OFFSET(0xFCFF75C4),
	[SC0_SCL1_PBUF3]	= VDC5FB_OFFSET(0xFCFF75C8),
	[SC0_SCL1_PBUF_FLD]	= VDC5FB_OFFSET(0xFCFF75CC),
	[SC0_SCL1_PBUF_CNT]	= VDC5FB_OFFSET(0xFCFF75D0),

	/* GRAPHICS 0 */
	[GR0_UPDATE]		= VDC5FB_OFFSET(0xFCFF7600),
	[GR0_FLM_RD]		= VDC5FB_OFFSET(0xFCFF7604),
	[GR0_FLM1]		= VDC5FB_OFFSET(0xFCFF7608),
	[GR0_FLM2]		= VDC5FB_OFFSET(0xFCFF760C),
	[GR0_FLM3]		= VDC5FB_OFFSET(0xFCFF7610),
	[GR0_FLM4]		= VDC5FB_OFFSET(0xFCFF7614),
	[GR0_FLM5]		= VDC5FB_OFFSET(0xFCFF7618),
	[GR0_FLM6]		= VDC5FB_OFFSET(0xFCFF761C),
	[GR0_AB1]		= VDC5FB_OFFSET(0xFCFF7620),
	[GR0_AB2]		= VDC5FB_OFFSET(0xFCFF7624),
	[GR0_AB3]		= VDC5FB_OFFSET(0xFCFF7628),
	[GR0_AB7]		= VDC5FB_OFFSET(0xFCFF7638),
	[GR0_AB8]		= VDC5FB_OFFSET(0xFCFF763C),
	[GR0_AB9]		= VDC5FB_OFFSET(0xFCFF7640),
	[GR0_AB10]		= VDC5FB_OFFSET(0xFCFF7644),
	[GR0_AB11]		= VDC5FB_OFFSET(0xFCFF7648),
	[GR0_BASE]		= VDC5FB_OFFSET(0xFCFF764C),
	[GR0_CLUT]		= VDC5FB_OFFSET(0xFCFF7650),
	[GR0_MON]		= VDC5FB_OFFSET(0xFCFF7654),

	/* SCALER 1 */
	[SC1_SCL0_UPDATE]	= VDC5FB_OFFSET(0xFCFF7C00),
	[SC1_SCL0_FRC1]		= VDC5FB_OFFSET(0xFCFF7C04),
	[SC1_SCL0_FRC2]		= VDC5FB_OFFSET(0xFCFF7C08),
	[SC1_SCL0_FRC3]		= VDC5FB_OFFSET(0xFCFF7C0C),
	[SC1_SCL0_FRC4]		= VDC5FB_OFFSET(0xFCFF7C10),
	[SC1_SCL0_FRC5]		= VDC5FB_OFFSET(0xFCFF7C14),
	[SC1_SCL0_FRC6]		= VDC5FB_OFFSET(0xFCFF7C18),
	[SC1_SCL0_FRC7]		= VDC5FB_OFFSET(0xFCFF7C1C),
	[SC1_SCL0_FRC9]		= VDC5FB_OFFSET(0xFCFF7C20),
	[SC1_SCL0_MON0]		= VDC5FB_OFFSET(0xFCFF7C24),
	[SC1_SCL0_INT]		= VDC5FB_OFFSET(0xFCFF7C28),
	[SC1_SCL0_DS1]		= VDC5FB_OFFSET(0xFCFF7C2C),
	[SC1_SCL0_DS2]		= VDC5FB_OFFSET(0xFCFF7C30),
	[SC1_SCL0_DS3]		= VDC5FB_OFFSET(0xFCFF7C34),
	[SC1_SCL0_DS4]		= VDC5FB_OFFSET(0xFCFF7C38),
	[SC1_SCL0_DS5]		= VDC5FB_OFFSET(0xFCFF7C3C),
	[SC1_SCL0_DS6]		= VDC5FB_OFFSET(0xFCFF7C40),
	[SC1_SCL0_DS7]		= VDC5FB_OFFSET(0xFCFF7C44),
	[SC1_SCL0_US1]		= VDC5FB_OFFSET(0xFCFF7C48),
	[SC1_SCL0_US2]		= VDC5FB_OFFSET(0xFCFF7C4C),
	[SC1_SCL0_US3]		= VDC5FB_OFFSET(0xFCFF7C50),
	[SC1_SCL0_US4]		= VDC5FB_OFFSET(0xFCFF7C54),
	[SC1_SCL0_US5]		= VDC5FB_OFFSET(0xFCFF7C58),
	[SC1_SCL0_US6]		= VDC5FB_OFFSET(0xFCFF7C5C),
	[SC1_SCL0_US7]		= VDC5FB_OFFSET(0xFCFF7C60),
	[SC1_SCL0_US8]		= VDC5FB_OFFSET(0xFCFF7C64),
	[SC1_SCL0_OVR1]		= VDC5FB_OFFSET(0xFCFF7C6C),
	[SC1_SCL1_UPDATE]	= VDC5FB_OFFSET(0xFCFF7C80),
	[SC1_SCL1_WR1]		= VDC5FB_OFFSET(0xFCFF7C88),
	[SC1_SCL1_WR2]		= VDC5FB_OFFSET(0xFCFF7C8C),
	[SC1_SCL1_WR3]		= VDC5FB_OFFSET(0xFCFF7C90),
	[SC1_SCL1_WR4]		= VDC5FB_OFFSET(0xFCFF7C94),
	[SC1_SCL1_WR5]		= VDC5FB_OFFSET(0xFCFF7C9C),
	[SC1_SCL1_WR6]		= VDC5FB_OFFSET(0xFCFF7CA0),
	[SC1_SCL1_WR7]		= VDC5FB_OFFSET(0xFCFF7CA4),
	[SC1_SCL1_WR8]		= VDC5FB_OFFSET(0xFCFF7CA8),
	[SC1_SCL1_WR9]		= VDC5FB_OFFSET(0xFCFF7CAC),
	[SC1_SCL1_WR10]		= VDC5FB_OFFSET(0xFCFF7CB0),
	[SC1_SCL1_WR11]		= VDC5FB_OFFSET(0xFCFF7CB4),
	[SC1_SCL1_MON1]		= VDC5FB_OFFSET(0xFCFF7CB8),
	[SC1_SCL1_PBUF0]	= VDC5FB_OFFSET(0xFCFF7CBC),
	[SC1_SCL1_PBUF1]	= VDC5FB_OFFSET(0xFCFF7CC0),
	[SC1_SCL1_PBUF2]	= VDC5FB_OFFSET(0xFCFF7CC4),
	[SC1_SCL1_PBUF3]	= VDC5FB_OFFSET(0xFCFF7CC9),
	[SC1_SCL1_PBUF_FLD]	= VDC5FB_OFFSET(0xFCFF7CCC),
	[SC1_SCL1_PBUF_CNT]	= VDC5FB_OFFSET(0xFCFF7CD0),

	/* GRAPHICS 1 */
	[GR1_UPDATE]		= VDC5FB_OFFSET(0xFCFF7D00),
	[GR1_FLM_RD]		= VDC5FB_OFFSET(0xFCFF7D04),
	[GR1_FLM1]		= VDC5FB_OFFSET(0xFCFF7D08),
	[GR1_FLM2]		= VDC5FB_OFFSET(0xFCFF7D0C),
	[GR1_FLM3]		= VDC5FB_OFFSET(0xFCFF7D10),
	[GR1_FLM4]		= VDC5FB_OFFSET(0xFCFF7D14),
	[GR1_FLM5]		= VDC5FB_OFFSET(0xFCFF7D18),
	[GR1_FLM6]		= VDC5FB_OFFSET(0xFCFF7D1C),
	[GR1_AB1]		= VDC5FB_OFFSET(0xFCFF7D20),
	[GR1_AB2]		= VDC5FB_OFFSET(0xFCFF7D24),
	[GR1_AB3]		= VDC5FB_OFFSET(0xFCFF7D28),
	[GR1_AB4]		= VDC5FB_OFFSET(0xFCFF7D2C),
	[GR1_AB5]		= VDC5FB_OFFSET(0xFCFF7D30),
	[GR1_AB6]		= VDC5FB_OFFSET(0xFCFF7D34),
	[GR1_AB7]		= VDC5FB_OFFSET(0xFCFF7D38),
	[GR1_AB8]		= VDC5FB_OFFSET(0xFCFF7D3C),
	[GR1_AB9]		= VDC5FB_OFFSET(0xFCFF7D40),
	[GR1_AB10]		= VDC5FB_OFFSET(0xFCFF7D44),
	[GR1_AB11]		= VDC5FB_OFFSET(0xFCFF7D48),
	[GR1_BASE]		= VDC5FB_OFFSET(0xFCFF7D4C),
	[GR1_CLUT]		= VDC5FB_OFFSET(0xFCFF7D50),
	[GR1_MON]		= VDC5FB_OFFSET(0xFCFF7D54),

	/* IMAGE QUALITY IMPROVER 0 */
	[ADJ0_UPDATE]		= VDC5FB_OFFSET(0xFCFF7680),
	[ADJ0_BKSTR_SET]	= VDC5FB_OFFSET(0xFCFF7684),
	[ADJ0_ENH_TIM1]		= VDC5FB_OFFSET(0xFCFF7688),
	[ADJ0_ENH_TIM2]		= VDC5FB_OFFSET(0xFCFF768C),
	[ADJ0_ENH_TIM3]		= VDC5FB_OFFSET(0xFCFF7690),
	[ADJ0_ENH_SHP1]		= VDC5FB_OFFSET(0xFCFF7694),
	[ADJ0_ENH_SHP2]		= VDC5FB_OFFSET(0xFCFF7698),
	[ADJ0_ENH_SHP3]		= VDC5FB_OFFSET(0xFCFF769C),
	[ADJ0_ENH_SHP4]		= VDC5FB_OFFSET(0xFCFF76A0),
	[ADJ0_ENH_SHP5]		= VDC5FB_OFFSET(0xFCFF76A4),
	[ADJ0_ENH_SHP6]		= VDC5FB_OFFSET(0xFCFF76A8),
	[ADJ0_ENH_LTI1]		= VDC5FB_OFFSET(0xFCFF76AC),
	[ADJ0_ENH_LTI2]		= VDC5FB_OFFSET(0xFCFF76B0),
	[ADJ0_MTX_MODE]		= VDC5FB_OFFSET(0xFCFF76B4),
	[ADJ0_MTX_YG_ADJ0]	= VDC5FB_OFFSET(0xFCFF76B8),
	[ADJ0_MTX_YG_ADJ1]	= VDC5FB_OFFSET(0xFCFF76BC),
	[ADJ0_MTX_CBB_ADJ0]	= VDC5FB_OFFSET(0xFCFF76C0),
	[ADJ0_MTX_CBB_ADJ1]	= VDC5FB_OFFSET(0xFCFF76C4),
	[ADJ0_MTX_CRR_ADJ0]	= VDC5FB_OFFSET(0xFCFF76C8),
	[ADJ0_MTX_CRR_ADJ1]	= VDC5FB_OFFSET(0xFCFF76CC),

	/* IMAGE QUALITY IMPROVER 1 */
	[ADJ1_UPDATE]		= VDC5FB_OFFSET(0xFCFF7D80),
	[ADJ1_BKSTR_SET]	= VDC5FB_OFFSET(0xFCFF7D84),
	[ADJ1_ENH_TIM1]		= VDC5FB_OFFSET(0xFCFF7D88),
	[ADJ1_ENH_TIM2]		= VDC5FB_OFFSET(0xFCFF7D8C),
	[ADJ1_ENH_TIM3]		= VDC5FB_OFFSET(0xFCFF7D90),
	[ADJ1_ENH_SHP1]		= VDC5FB_OFFSET(0xFCFF7D94),
	[ADJ1_ENH_SHP2]		= VDC5FB_OFFSET(0xFCFF7D98),
	[ADJ1_ENH_SHP3]		= VDC5FB_OFFSET(0xFCFF7D9C),
	[ADJ1_ENH_SHP4]		= VDC5FB_OFFSET(0xFCFF7DA0),
	[ADJ1_ENH_SHP5]		= VDC5FB_OFFSET(0xFCFF7DA4),
	[ADJ1_ENH_SHP6]		= VDC5FB_OFFSET(0xFCFF7DA8),
	[ADJ1_ENH_LTI1]		= VDC5FB_OFFSET(0xFCFF7DA8),
	[ADJ1_ENH_LTI2]		= VDC5FB_OFFSET(0xFCFF7DB0),
	[ADJ1_MTX_MODE]		= VDC5FB_OFFSET(0xFCFF7DB4),
	[ADJ1_MTX_YG_ADJ0]	= VDC5FB_OFFSET(0xFCFF7DB8),
	[ADJ1_MTX_YG_ADJ1]	= VDC5FB_OFFSET(0xFCFF7DBC),
	[ADJ1_MTX_CBB_ADJ0]	= VDC5FB_OFFSET(0xFCFF7DC0),
	[ADJ1_MTX_CBB_ADJ1]	= VDC5FB_OFFSET(0xFCFF7CC4),
	[ADJ1_MTX_CRR_ADJ0]	= VDC5FB_OFFSET(0xFCFF7DC8),
	[ADJ1_MTX_CRR_ADJ1]	= VDC5FB_OFFSET(0xFCFF7DCC),

	/* GRAPHICS 2 */
	[GR2_UPDATE]		= VDC5FB_OFFSET(0xFCFF7700),
	[GR2_FLM_RD]		= VDC5FB_OFFSET(0xFCFF7704),
	[GR2_FLM1]		= VDC5FB_OFFSET(0xFCFF7708),
	[GR2_FLM2]		= VDC5FB_OFFSET(0xFCFF770C),
	[GR2_FLM3]		= VDC5FB_OFFSET(0xFCFF7710),
	[GR2_FLM4]		= VDC5FB_OFFSET(0xFCFF7714),
	[GR2_FLM5]		= VDC5FB_OFFSET(0xFCFF7718),
	[GR2_FLM6]		= VDC5FB_OFFSET(0xFCFF771C),
	[GR2_AB1]		= VDC5FB_OFFSET(0xFCFF7720),
	[GR2_AB2]		= VDC5FB_OFFSET(0xFCFF7724),
	[GR2_AB3]		= VDC5FB_OFFSET(0xFCFF7728),
	[GR2_AB4]		= VDC5FB_OFFSET(0xFCFF772C),
	[GR2_AB5]		= VDC5FB_OFFSET(0xFCFF7730),
	[GR2_AB6]		= VDC5FB_OFFSET(0xFCFF7734),
	[GR2_AB7]		= VDC5FB_OFFSET(0xFCFF7738),
	[GR2_AB8]		= VDC5FB_OFFSET(0xFCFF773C),
	[GR2_AB9]		= VDC5FB_OFFSET(0xFCFF7740),
	[GR2_AB10]		= VDC5FB_OFFSET(0xFCFF7744),
	[GR2_AB11]		= VDC5FB_OFFSET(0xFCFF7748),
	[GR2_BASE]		= VDC5FB_OFFSET(0xFCFF774C),
	[GR2_CLUT]		= VDC5FB_OFFSET(0xFCFF7750),
	[GR2_MON]		= VDC5FB_OFFSET(0xFCFF7754),

	/* GRAPHICS 3 */
	[GR3_UPDATE]		= VDC5FB_OFFSET(0xFCFF7780),
	[GR3_FLM_RD]		= VDC5FB_OFFSET(0xFCFF7784),
	[GR3_FLM1]		= VDC5FB_OFFSET(0xFCFF7788),
	[GR3_FLM2]		= VDC5FB_OFFSET(0xFCFF778C),
	[GR3_FLM3]		= VDC5FB_OFFSET(0xFCFF7790),
	[GR3_FLM4]		= VDC5FB_OFFSET(0xFCFF7794),
	[GR3_FLM5]		= VDC5FB_OFFSET(0xFCFF7798),
	[GR3_FLM6]		= VDC5FB_OFFSET(0xFCFF779C),
	[GR3_AB1]		= VDC5FB_OFFSET(0xFCFF77A0),
	[GR3_AB2]		= VDC5FB_OFFSET(0xFCFF77A4),
	[GR3_AB3]		= VDC5FB_OFFSET(0xFCFF77A8),
	[GR3_AB4]		= VDC5FB_OFFSET(0xFCFF77AC),
	[GR3_AB5]		= VDC5FB_OFFSET(0xFCFF77B0),
	[GR3_AB6]		= VDC5FB_OFFSET(0xFCFF77B4),
	[GR3_AB7]		= VDC5FB_OFFSET(0xFCFF77B8),
	[GR3_AB8]		= VDC5FB_OFFSET(0xFCFF77BC),
	[GR3_AB9]		= VDC5FB_OFFSET(0xFCFF77C0),
	[GR3_AB10]		= VDC5FB_OFFSET(0xFCFF77C4),
	[GR3_AB11]		= VDC5FB_OFFSET(0xFCFF77C8),
	[GR3_BASE]		= VDC5FB_OFFSET(0xFCFF77CC),
	[GR3_CLUT_INT]		= VDC5FB_OFFSET(0xFCFF77D0),
	[GR3_MON]		= VDC5FB_OFFSET(0xFCFF77D4),

	/* VIN SYNTHESIZER */
	[GR_VIN_UPDATE]		= VDC5FB_OFFSET(0xFCFF7E00),
	[GR_VIN_AB1]		= VDC5FB_OFFSET(0xFCFF7E20),
	[GR_VIN_AB2]		= VDC5FB_OFFSET(0xFCFF7E24),
	[GR_VIN_AB3]		= VDC5FB_OFFSET(0xFCFF7E28),
	[GR_VIN_AB4]		= VDC5FB_OFFSET(0xFCFF7E2C),
	[GR_VIN_AB5]		= VDC5FB_OFFSET(0xFCFF7E30),
	[GR_VIN_AB6]		= VDC5FB_OFFSET(0xFCFF7E34),
	[GR_VIN_AB7]		= VDC5FB_OFFSET(0xFCFF7E38),
	[GR_VIN_BASE]		= VDC5FB_OFFSET(0xFCFF7E4C),
	[GR_VIN_MON]		= VDC5FB_OFFSET(0xFCFF7E54),

	/* OUTPUT IMAGE GENERATOR */
	[OIR_SCL0_UPDATE]	= VDC5FB_OFFSET(0xFCFF7E80),
	[OIR_SCL0_FRC1]		= VDC5FB_OFFSET(0xFCFF7E84),
	[OIR_SCL0_FRC2]		= VDC5FB_OFFSET(0xFCFF7E88),
	[OIR_SCL0_FRC3]		= VDC5FB_OFFSET(0xFCFF7E8C),
	[OIR_SCL0_FRC4]		= VDC5FB_OFFSET(0xFCFF7E90),
	[OIR_SCL0_FRC5]		= VDC5FB_OFFSET(0xFCFF7E94),
	[OIR_SCL0_FRC6]		= VDC5FB_OFFSET(0xFCFF7E98),
	[OIR_SCL0_FRC7]		= VDC5FB_OFFSET(0xFCFF7E9C),
	[OIR_SCL0_DS1]		= VDC5FB_OFFSET(0xFCFF7EAC),
	[OIR_SCL0_DS2]		= VDC5FB_OFFSET(0xFCFF7EB0),
	[OIR_SCL0_DS3]		= VDC5FB_OFFSET(0xFCFF7EB4),
	[OIR_SCL0_DS7]		= VDC5FB_OFFSET(0xFCFF7EC4),
	[OIR_SCL0_US1]		= VDC5FB_OFFSET(0xFCFF7EC8),
	[OIR_SCL0_US2]		= VDC5FB_OFFSET(0xFCFF7ECC),
	[OIR_SCL0_US3]		= VDC5FB_OFFSET(0xFCFF7ED0),
	[OIR_SCL0_US8]		= VDC5FB_OFFSET(0xFCFF7EE4),
	[OIR_SCL0_OVR1]		= VDC5FB_OFFSET(0xFCFF7EEC),
	[OIR_SCL1_UPDATE]	= VDC5FB_OFFSET(0xFCFF7F00),
	[OIR_SCL1_WR1]		= VDC5FB_OFFSET(0xFCFF7F08),
	[OIR_SCL1_WR2]		= VDC5FB_OFFSET(0xFCFF7F0C),
	[OIR_SCL1_WR3]		= VDC5FB_OFFSET(0xFCFF7F10),
	[OIR_SCL1_WR4]		= VDC5FB_OFFSET(0xFCFF7F14),
	[OIR_SCL1_WR5]		= VDC5FB_OFFSET(0xFCFF7F1C),
	[OIR_SCL1_WR6]		= VDC5FB_OFFSET(0xFCFF7F20),
	[OIR_SCL1_WR7]		= VDC5FB_OFFSET(0xFCFF7F24),

	/* GRAPHICS OIR */
	[GR_OIR_UPDATE]		= VDC5FB_OFFSET(0xFCFF7F80),
	[GR_OIR_FLM_RD]		= VDC5FB_OFFSET(0xFCFF7F84),
	[GR_OIR_FLM1]		= VDC5FB_OFFSET(0xFCFF7F88),
	[GR_OIR_FLM2]		= VDC5FB_OFFSET(0xFCFF7F8C),
	[GR_OIR_FLM3]		= VDC5FB_OFFSET(0xFCFF7F90),
	[GR_OIR_FLM4]		= VDC5FB_OFFSET(0xFCFF7F94),
	[GR_OIR_FLM5]		= VDC5FB_OFFSET(0xFCFF7F98),
	[GR_OIR_FLM6]		= VDC5FB_OFFSET(0xFCFF7F9C),
	[GR_OIR_AB1]		= VDC5FB_OFFSET(0xFCFF7FA0),
	[GR_OIR_AB2]		= VDC5FB_OFFSET(0xFCFF7FA4),
	[GR_OIR_AB3]		= VDC5FB_OFFSET(0xFCFF7FA8),
	[GR_OIR_AB7]		= VDC5FB_OFFSET(0xFCFF7FB8),
	[GR_OIR_AB8]		= VDC5FB_OFFSET(0xFCFF7FBC),
	[GR_OIR_AB9]		= VDC5FB_OFFSET(0xFCFF7FC0),
	[GR_OIR_AB10]		= VDC5FB_OFFSET(0xFCFF7FC4),
	[GR_OIR_AB11]		= VDC5FB_OFFSET(0xFCFF7FC8),
	[GR_OIR_BASE]		= VDC5FB_OFFSET(0xFCFF7FCC),
	[GR_OIR_CLUT]		= VDC5FB_OFFSET(0xFCFF7FD0),
	[GR_OIR_MON]		= VDC5FB_OFFSET(0xFCFF7FD4),

	/* GAMMA CORRECTION BLOCK */
	[GAM_G_UPDATE]		= VDC5FB_OFFSET(0xFCFF7800),
	[GAM_SW]		= VDC5FB_OFFSET(0xFCFF7804),
	[GAM_G_LUT1]		= VDC5FB_OFFSET(0xFCFF7808),
	[GAM_G_LUT2]		= VDC5FB_OFFSET(0xFCFF780C),
	[GAM_G_LUT3]		= VDC5FB_OFFSET(0xFCFF7810),
	[GAM_G_LUT4]		= VDC5FB_OFFSET(0xFCFF7814),
	[GAM_G_LUT5]		= VDC5FB_OFFSET(0xFCFF7818),
	[GAM_G_LUT6]		= VDC5FB_OFFSET(0xFCFF781C),
	[GAM_G_LUT7]		= VDC5FB_OFFSET(0xFCFF7820),
	[GAM_G_LUT8]		= VDC5FB_OFFSET(0xFCFF7824),
	[GAM_G_LUT9]		= VDC5FB_OFFSET(0xFCFF7828),
	[GAM_G_LUT10]		= VDC5FB_OFFSET(0xFCFF782C),
	[GAM_G_LUT11]		= VDC5FB_OFFSET(0xFCFF7830),
	[GAM_G_LUT12]		= VDC5FB_OFFSET(0xFCFF7834),
	[GAM_G_LUT13]		= VDC5FB_OFFSET(0xFCFF7838),
	[GAM_G_LUT14]		= VDC5FB_OFFSET(0xFCFF783C),
	[GAM_G_LUT15]		= VDC5FB_OFFSET(0xFCFF7840),
	[GAM_G_LUT16]		= VDC5FB_OFFSET(0xFCFF7844),
	[GAM_G_AREA1]		= VDC5FB_OFFSET(0xFCFF7848),
	[GAM_G_AREA2]		= VDC5FB_OFFSET(0xFCFF784C),
	[GAM_G_AREA3]		= VDC5FB_OFFSET(0xFCFF7850),
	[GAM_G_AREA4]		= VDC5FB_OFFSET(0xFCFF7854),
	[GAM_G_AREA5]		= VDC5FB_OFFSET(0xFCFF7858),
	[GAM_G_AREA6]		= VDC5FB_OFFSET(0xFCFF785C),
	[GAM_G_AREA7]		= VDC5FB_OFFSET(0xFCFF7860),
	[GAM_G_AREA8]		= VDC5FB_OFFSET(0xFCFF7864),
	[GAM_B_UPDATE]		= VDC5FB_OFFSET(0xFCFF7880),
	[GAM_B_LUT1]		= VDC5FB_OFFSET(0xFCFF7888),
	[GAM_B_LUT2]		= VDC5FB_OFFSET(0xFCFF788C),
	[GAM_B_LUT3]		= VDC5FB_OFFSET(0xFCFF7890),
	[GAM_B_LUT4]		= VDC5FB_OFFSET(0xFCFF7894),
	[GAM_B_LUT5]		= VDC5FB_OFFSET(0xFCFF7898),
	[GAM_B_LUT6]		= VDC5FB_OFFSET(0xFCFF789C),
	[GAM_B_LUT7]		= VDC5FB_OFFSET(0xFCFF78A0),
	[GAM_B_LUT8]		= VDC5FB_OFFSET(0xFCFF78A4),
	[GAM_B_LUT9]		= VDC5FB_OFFSET(0xFCFF78A8),
	[GAM_B_LUT10]		= VDC5FB_OFFSET(0xFCFF78AC),
	[GAM_B_LUT11]		= VDC5FB_OFFSET(0xFCFF78B0),
	[GAM_B_LUT12]		= VDC5FB_OFFSET(0xFCFF78B4),
	[GAM_B_LUT13]		= VDC5FB_OFFSET(0xFCFF78B8),
	[GAM_B_LUT14]		= VDC5FB_OFFSET(0xFCFF78BC),
	[GAM_B_LUT15]		= VDC5FB_OFFSET(0xFCFF78C0),
	[GAM_B_LUT16]		= VDC5FB_OFFSET(0xFCFF78C4),
	[GAM_B_AREA1]		= VDC5FB_OFFSET(0xFCFF78C8),
	[GAM_B_AREA2]		= VDC5FB_OFFSET(0xFCFF78CC),
	[GAM_B_AREA3]		= VDC5FB_OFFSET(0xFCFF78D0),
	[GAM_B_AREA4]		= VDC5FB_OFFSET(0xFCFF78D4),
	[GAM_B_AREA5]		= VDC5FB_OFFSET(0xFCFF78D8),
	[GAM_B_AREA6]		= VDC5FB_OFFSET(0xFCFF78DC),
	[GAM_B_AREA7]		= VDC5FB_OFFSET(0xFCFF78E0),
	[GAM_B_AREA8]		= VDC5FB_OFFSET(0xFCFF78E4),
	[GAM_R_UPDATE]		= VDC5FB_OFFSET(0xFCFF7900),
	[GAM_R_LUT1]		= VDC5FB_OFFSET(0xFCFF7908),
	[GAM_R_LUT2]		= VDC5FB_OFFSET(0xFCFF790C),
	[GAM_R_LUT3]		= VDC5FB_OFFSET(0xFCFF7910),
	[GAM_R_LUT4]		= VDC5FB_OFFSET(0xFCFF7914),
	[GAM_R_LUT5]		= VDC5FB_OFFSET(0xFCFF7918),
	[GAM_R_LUT6]		= VDC5FB_OFFSET(0xFCFF791C),
	[GAM_R_LUT7]		= VDC5FB_OFFSET(0xFCFF7920),
	[GAM_R_LUT8]		= VDC5FB_OFFSET(0xFCFF7924),
	[GAM_R_LUT9]		= VDC5FB_OFFSET(0xFCFF7928),
	[GAM_R_LUT10]		= VDC5FB_OFFSET(0xFCFF792C),
	[GAM_R_LUT11]		= VDC5FB_OFFSET(0xFCFF7930),
	[GAM_R_LUT12]		= VDC5FB_OFFSET(0xFCFF7934),
	[GAM_R_LUT13]		= VDC5FB_OFFSET(0xFCFF7938),
	[GAM_R_LUT14]		= VDC5FB_OFFSET(0xFCFF793C),
	[GAM_R_LUT15]		= VDC5FB_OFFSET(0xFCFF7940),
	[GAM_R_LUT16]		= VDC5FB_OFFSET(0xFCFF7944),
	[GAM_R_AREA1]		= VDC5FB_OFFSET(0xFCFF7948),
	[GAM_R_AREA2]		= VDC5FB_OFFSET(0xFCFF794C),
	[GAM_R_AREA3]		= VDC5FB_OFFSET(0xFCFF7950),
	[GAM_R_AREA4]		= VDC5FB_OFFSET(0xFCFF7954),
	[GAM_R_AREA5]		= VDC5FB_OFFSET(0xFCFF7958),
	[GAM_R_AREA6]		= VDC5FB_OFFSET(0xFCFF795C),
	[GAM_R_AREA7]		= VDC5FB_OFFSET(0xFCFF7960),
	[GAM_R_AREA8]		= VDC5FB_OFFSET(0xFCFF7964),

	/* TCON BLOCK */
	[TCON_UPDATE]		= VDC5FB_OFFSET(0xFCFF7980),
	[TCON_TIM]		= VDC5FB_OFFSET(0xFCFF7984),
	[TCON_TIM_STVA1]	= VDC5FB_OFFSET(0xFCFF7988),
	[TCON_TIM_STVA2]	= VDC5FB_OFFSET(0xFCFF798C),
	[TCON_TIM_STVB1]	= VDC5FB_OFFSET(0xFCFF7990),
	[TCON_TIM_STVB2]	= VDC5FB_OFFSET(0xFCFF7994),
	[TCON_TIM_STH1]		= VDC5FB_OFFSET(0xFCFF7998),
	[TCON_TIM_STH2]		= VDC5FB_OFFSET(0xFCFF799C),
	[TCON_TIM_STB1]		= VDC5FB_OFFSET(0xFCFF79A0),
	[TCON_TIM_STB2]		= VDC5FB_OFFSET(0xFCFF79A4),
	[TCON_TIM_CPV1]		= VDC5FB_OFFSET(0xFCFF79A8),
	[TCON_TIM_CPV2]		= VDC5FB_OFFSET(0xFCFF79AC),
	[TCON_TIM_POLA1]	= VDC5FB_OFFSET(0xFCFF79B0),
	[TCON_TIM_POLA2]	= VDC5FB_OFFSET(0xFCFF79B4),
	[TCON_TIM_POLB1]	= VDC5FB_OFFSET(0xFCFF79B8),
	[TCON_TIM_POLB2]	= VDC5FB_OFFSET(0xFCFF79BC),
	[TCON_TIM_DE]		= VDC5FB_OFFSET(0xFCFF79C0),

	/* OUTPUT CONTROLLER */
	[OUT_UPDATE]		= VDC5FB_OFFSET(0xFCFF7A00),
	[OUT_SET]		= VDC5FB_OFFSET(0xFCFF7A04),
	[OUT_BRIGHT1]		= VDC5FB_OFFSET(0xFCFF7A08),
	[OUT_BRIGHT2]		= VDC5FB_OFFSET(0xFCFF7A0C),
	[OUT_CONTRAST]		= VDC5FB_OFFSET(0xFCFF7A10),
	[OUT_PDTHA]		= VDC5FB_OFFSET(0xFCFF7A14),
	[OUT_CLK_PHASE]		= VDC5FB_OFFSET(0xFCFF7A24),

	/* SYSTEM CONTROLLER */
	[SYSCNT_INT1]		= VDC5FB_OFFSET(0xFCFF7A80),
	[SYSCNT_INT2]		= VDC5FB_OFFSET(0xFCFF7A84),
	[SYSCNT_INT3]		= VDC5FB_OFFSET(0xFCFF7A88),
	[SYSCNT_INT4]		= VDC5FB_OFFSET(0xFCFF7A8C),
	[SYSCNT_INT5]		= VDC5FB_OFFSET(0xFCFF7A90),
	[SYSCNT_INT6]		= VDC5FB_OFFSET(0xFCFF7A94),
	[SYSCNT_PANEL_CLK]	= VDC5FB_OFFSET(0xFCFF7A98), /* 16-bit */
	[SYSCNT_CLUT]		= VDC5FB_OFFSET(0xFCFF7A9A), /* 16-bit */
};

/* INTERRUPT NAME */
static const char *irq_names[VDC5FB_MAX_IRQS] = {
	[S0_VI_VSYNC]		= "s0_vi_vsync",
	[S0_LO_VSYNC]		= "s0_lo_vsync",
	[S0_VSYNCERR]		= "s0_vsyncerr",
	[GR3_VLINE]		= "gr3_vline",
	[S0_VFIELD]		= "s0_vfield",
	[IV1_VBUFERR]		= "iv1_vbuferr",
	[IV3_VBUFERR]		= "iv3_vbuferr",
	[IV5_VBUFERR]		= "iv5_vbuferr",
	[IV6_VBUFERR]		= "iv6_vbuferr",
	[S0_WLINE]		= "s0_wline",
	[S1_VI_VSYNC]		= "s1_vi_vsync",
	[S1_LO_VSYNC]		= "s1_lo_vsync",
	[S1_VSYNCERR]		= "s1_vsyncerr",
	[S1_VFIELD]		= "s1_vfield",
	[IV2_VBUFERR]		= "iv2_vbuferr",
	[IV4_VBUFERR]		= "iv4_vbuferr",
	[S1_WLINE]		= "s1_wline",
	[OIR_VI_VSYNC]		= "oir_vi_vsync",
	[OIR_LO_VSYNC]		= "oir_lo_vsync",
	[OIR_VLINE]		= "oir_vline",
	[OIR_VFIELD]		= "oir_vfield",
	[IV7_VBUFERR]		= "iv7_vbuferr",
	[IV8_VBUFERR]		= "iv8_vbuferr",
};

/************************************************************************/
/* REGISTER BITS */

#define	RGB888(r, g, b)		\
	((((r) & 0xffu) << 16) | (((g) & 0xffu) << 8) | ((b) & 0xffu))
#define	GBR888(g, b, r)		\
	((((g) & 0xffu) << 16) | (((b) & 0xffu) << 8) | ((r) & 0xffu))
#define	AGBR(a, g, b, r)		\
	((((a) & 0xffu) << 24) | (((g) & 0xffu) << 16) \
	| (((b) & 0xffu) << 8) | ((r) & 0xffu))

/* SYSCNT_PANEL_CLK */
#define	PANEL_DCDR(x)		(((x) & 0x3fu) << 0)
#define PANEL_ICKEN		(1u << 8)
#define PANEL_OCKSEL(x)		(((x) & 0x3u) << 10)
#define PANEL_ICKSEL(x)         (((x) & 0x3u) << 12)

/* SCx_SCL0_FRC1, OIR_SCL0_FRC1 (x=0,1) */
#define	SC_RES_VMASK_ON		(1u << 0)
#define	SC_RES_VMASK(x)		(((x) & 0xffffu) << 16)
/* SCx_SCL0_FRC2, OIR_SCL0_FRC2 */
#define	SC_RES_VLACK_ON		(1u << 0)
#define	SC_RES_VLACK(x)		(((x) & 0xffffu) << 16)
/* SCx_SCL0_FRC3, OIR_SCL0_FRC3 */
#define	SC_RES_VS_SEL		(1u << 0)
#define	SC_RES_VS_IN_SEL	(1u << 8)		/* SC0, SC1 only */
#define	OIR_RES_EN		(1u << 16)		/* OIR only */
/* SCx_SCL0_FRC4, OIR_SCL0_FRC4 */
#define	SC_RES_FH(x)		(((x) & 0x7ffu) << 0)
#define	SC_RES_FV(x)		(((x) & 0x7ffu) << 16)
/* SCx_SCL0_FRC5, OIR_SCL0_FRC5 */
#define	SC_RES_VSDLY(x)		(((x) & 0xffu) << 0)
#define	SC_RES_FLD_DLY_SEL	(1u << 8)		/* SC0, SC1 only */
/* SCx_SCL0_FRC6, OIR_SCL0_FRC6 */
#define	SC_RES_F_VW(x)		(((x) & 0x7ffu) << 0)
#define	SC_RES_F_VS(x)		(((x) & 0x7ffu) << 16)
/* SCx_SCL0_FRC7, OIR_SCL0_FRC7 */
#define	SC_RES_F_HW(x)		(((x) & 0x7ffu) << 0)
#define	SC_RES_F_HS(x)		(((x) & 0x7ffu) << 16)
/* SCx_SCL0_OVR1, OIR_SCL0_OVR1 */
#define	D_SC_RES_BK_COL		RGB888(0, 0, 0)
/* SCx_SCL0_US8, OIR_SCL0_US8 */
#define	SC_RES_IBUS_SYNC_SEL	(1u << 4)

/* GRx_FLM_RD, GR_OIR_FLM_RD (x=0,1,2,3) */
#define	GR_R_ENB		(1u << 0)
/* GRx_FLM1, GR_OIR_FLM1 */
#define	GR_BST_MD		(1u << 0)
#define	GR_OIR_IMR_FLM_INV	(1u << 4)		/* GR_OIR only */
#define	GR_FLM_SEL(x)		(((x) & 0x3u) << 8)
#define	GR_LN_OFF_DIR		(1u << 16)
/* GRx_FLM3, GR_OIR_FLM3 */
#define	GR_FLM_NUM(x)		(((x) & 0x3ffu) << 0)
#define	GR_LN_OFF(x)		(((x) & 0x7fffu) << 16)
/* GRx_FLM4, GR_OIR_FLM4 */
#define	GR_FLM_OFF(x)		(((x) & 0x7fffffu) << 0)
/* GRx_FLM5, GR_OIR_FLM5 */
#define	GR_FLM_LOOP(x)		(((x) & 0x7ffu) << 0)
#define	GR_FLM_LNUM(x)		(((x) & 0x7ffu) << 16)
/* GRx_FLM6, GR_OIR_FLM6 */
#define	GR_RDSWA(x)		(((x) & 0x7u) << 10)
#define	GR_HW(x)		(((x) & 0x3ff) << 16)
#define	GR_FORMAT(x)		(((x) & 0xfu) << 28)
#define	D_GR_FLM6_RGB565	(GR_RDSWA(6) | GR_FORMAT(0))
#define	D_GR_FLM6_RGB888	(GR_RDSWA(4) | GR_FORMAT(1))
#define	D_GR_FLM6_ARGB8888	(GR_RDSWA(4) | GR_FORMAT(4))
#define	D_GR_FLM6_RGBA8888	(GR_RDSWA(4) | GR_FORMAT(11))
/* GRx_AB1, GR_VIN_AB1, GR_OIR_AB1 */
#define	GR_DISP_SEL(x)		(((x) & 0x3u) << 0)
#define	GR_VIN_SCL_UND_SEL	(1u << 2)		/* GR_VIN only */
#define	GR1_CUS_CON_ON		(1u << 28)		/* GR1 only */
#define	GR_AB1_MASK		0xeffffff4u
/* GRx_AB2, GR_VIN_AB2, GR_OIR_AB2 */
#define	GR_GRC_VW(x)		(((x) & 0x7ffu) << 0)
#define	GR_GRC_VS(x)		(((x) & 0x7ffu) << 16)
/* GRx_AB3, GR_VIN_AB3, GR_OIR_AB3 */
#define	GR_GRC_HW(x)		(((x) & 0x7ffu) << 0)
#define	GR_GRC_HS(x)		(((x) & 0x7ffu) << 16)
/* GRx_AB8, GR_VIN_AB8, GR_OIR_AB8 */
/* GRx_AB9, GR_VIN_AB9, GR_OIR_AB9 */
/* GRx_AB10, GR_VIN_AB10, GR_OIR_AB10 */
/* GRx_AB11, GR_VIN_AB11, GR_OIR_AB11 */
#define	D_GR_AB8		AGBR(0, 0, 0, 0)
#define	D_GR_AB9		AGBR(0xffu, 0, 0, 0)
#define	D_GR_AB10		AGBR(0xffu, 0, 0, 0)
#define	D_GR_AB11		AGBR(0xffu, 0, 0, 0)
/* GR0_BASE */
#define	D_GR_BASE		GBR888(0, 0, 0)

/* OUT_CLK_PHASE */
#define	D_OUT_CLK_PHASE		0
/* OUT_BRIGHT1, OUT_BRIGHT2 */
#define	PBRT_G(x)		(((x) & 0x3ffu) << 0)
#define	PBRT_B(x)		(((x) & 0x3ffu) << 16)
#define	PBRT_R(x)		(((x) & 0x3ffu) << 0)
/* OUT_CONTRAST */
#define	CONT_R(x)		(((x) & 0xffu) << 0)
#define	CONT_B(x)		(((x) & 0xffu) << 8)
#define	CONT_G(x)		(((x) & 0xffu) << 16)
/* OUT_PDTHA */
#define	PDTHA_FORMAT(x)		(((x) & 0x3u) << 16)
#define	D_OUT_PDTHA		0x00003021u
/* OUT_SET */
#define	OUT_FORMAT(x)		(((x) & 0x3u) << 12)
#define	D_OUT_SET		0x001f0000u

/* TCON_TIM */
#define	TCON_OFFSET(x)          (((x) & 0x7ffu) << 0)
#define	TCON_HALF(x)            (((x) & 0x7ffu) << 16)
/* TCON_TIM_xxxx */
#define	TCON_VW(x)		(((x) & 0x7ffu) << 0)
#define	TCON_VS(x)		(((x) & 0x7ffu) << 16)
#define	TCON_SEL(x)		(((x) & 0x7u) << 0)
#define	TCON_INV		(0x1u << 4)
#define	TCON_HW(x)		(((x) & 0x7ffu) << 0)
#define	TCON_HS(x)		(((x) & 0x7ffu) << 16)
#define	TCON_SEL(x)		(((x) & 0x7u) << 0)
#define	TCON_INV		(0x1u << 4)
#define	TCON_HS_SEL		(0x1u << 8)
#define	TCON_MD(x)		(((x) & 0x3u) << 12)
#define	TCON_DE_INV		(0x1u << 0)

/* INP_UPDATE */
#define	INP_IMG_UPDATE		(1u << 0)
#define	INP_EXT_UPDATE		(1u << 4)
/* IMGCNT_UPDATE */
#define	IMGCNT_VEN		(1u << 0)
/* SCx_SCLx_UPDATE, GRx_UPADTE, OIR_SCLx_UPDATE (x=0,1) */
#define	SC_SCL_VEN_A		(1u << 0)
#define	SC_SCL_VEN_B		(1u << 4)
#define	SC_SCL_UPDATE		(1u << 8)
#define	SC_SCL_VEN_C		(1u << 12)	/* not OIR_SCL1_UPDATE */
#define	SC_SCL_VEN_D		(1u << 13)	/* not OIR_SCL1_UPDATE */
/* ADJx_UPDATE */
#define	ADJ_VEN			(1u << 0)
/* GRx_UPDATE, GR_OIR_UPDATE (x=2,3) */
#define	GR_IBUS_VEN		(1u << 0)
#define	GR_P_VEN		(1u << 4)
#define	GR_UPDATE		(1u << 8)
/* GAM_x_UPDATE (x=G,B,R) */
#define	GAM_VEN			(1u << 0)
/* TCON_UPDATE */
#define	TCON_VEN		(1u << 0)
/* OUT_UPDATE */
#define	OUTCNT_VEN		(1u << 0)

/************************************************************************/
/* READ / WRITE VDC5 REGISTERS */

static void vdc5fb_write(struct vdc5fb_priv *priv, int reg, u32 data)
{
	if ((SYSCNT_PANEL_CLK == reg) || (SYSCNT_CLUT == reg))
		iowrite16((u16)data, (priv->base + vdc5fb_offsets[reg]));
	else
		iowrite32((u32)data, (priv->base + vdc5fb_offsets[reg]));
}

static unsigned long vdc5fb_read(struct vdc5fb_priv *priv, int reg)
{
	if ((SYSCNT_PANEL_CLK == reg) || (SYSCNT_CLUT == reg))
		return ioread16(priv->base + vdc5fb_offsets[reg]);
	else
		return ioread32(priv->base + vdc5fb_offsets[reg]);
}

static void vdc5fb_setbits(struct vdc5fb_priv *priv, int reg, u32 bits)
{
	u32 tmp;

	tmp = vdc5fb_read(priv, reg);
	tmp |= bits;
	vdc5fb_write(priv, reg, tmp);
}

/************************************************************************/

#endif /* _VDC5FB_REGS_H_ */