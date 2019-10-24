/*
 * Copyright (C) 2018 Spreadtrum Communications Inc.
 *
 * This file is dual-licensed: you can use it either under the terms
 * of the GPL or the X11 license, at your option. Note that this dual
 * licensing only applies to this file, and not this project as a
 * whole.
 *
 * updated at 2018-05-10 14:14:24
 *
 */


#ifndef ANLG_PHY_G2_H
#define ANLG_PHY_G2_H



#define REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL0                 (0x0000)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL1                 (0x0004)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL2                 (0x0008)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL3                 (0x000C)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL4                 (0x0010)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL5                 (0x0014)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL6                 (0x0018)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL7                 (0x001C)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL8                 (0x0020)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_PROMETHEUS_DVFS_INDEX0      (0x0024)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_PROMETHEUS_DVFS_INDEX1      (0x0028)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_PROMETHEUS_DVFS_INDEX2      (0x002C)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_PROMETHEUS_DVFS_INDEX3      (0x0030)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_PROMETHEUS_DVFS_INDEX4      (0x0034)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_PROMETHEUS_DVFS_INDEX5      (0x0038)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_PROMETHEUS_DVFS_INDEX6      (0x003C)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_PROMETHEUS_DVFS_INDEX7      (0x0040)
#define REG_ANLG_PHY_G2_ANALOG_MPLL1_REG_SEL_CFG_0               (0x0044)
#define REG_ANLG_PHY_G2_ANALOG_THM2_THM2_CTL                     (0x0048)
#define REG_ANLG_PHY_G2_ANALOG_THM2_THM2_RESERVED_CTL            (0x004C)
#define REG_ANLG_PHY_G2_ANALOG_THM2_REG_SEL_CFG_0                (0x0050)
#define REG_ANLG_PHY_G2_ANALOG_USB20_USB20_TEST_PIN              (0x0054)
#define REG_ANLG_PHY_G2_ANALOG_USB20_USB20_UTMI_CTL1             (0x0058)
#define REG_ANLG_PHY_G2_ANALOG_USB20_USB20_BATTER_PLL            (0x005C)
#define REG_ANLG_PHY_G2_ANALOG_USB20_USB20_UTMI_CTL2             (0x0060)
#define REG_ANLG_PHY_G2_ANALOG_USB20_USB20_TRIMMING              (0x0064)
#define REG_ANLG_PHY_G2_ANALOG_USB20_USB20_PHY_TUNE_CTL          (0x0068)
#define REG_ANLG_PHY_G2_ANALOG_USB20_USB20_PHY_BIST_TEST         (0x006C)
#define REG_ANLG_PHY_G2_ANALOG_USB20_USB20_ISO_SW                (0x0070)
#define REG_ANLG_PHY_G2_ANALOG_USB20_REG_SEL_CFG_0               (0x0074)

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL0 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_LOCK_DONE             BIT(17)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CLKIN_SEL             BIT(16)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_N(x)                  (((x) & 0x7FF) << 5)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_ICP(x)                (((x) & 0x7) << 2)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_SDM_EN                BIT(1)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_DIV_S                 BIT(0)

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL1 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_NINT(x)               (((x) & 0x7F) << 23)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_KINT(x)               (((x) & 0x7FFFFF))

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL2 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_DIV32_EN              BIT(14)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_POSTDIV               BIT(13)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_FREQ_DOUBLE_EN        BIT(12)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CCS_CTRL(x)           (((x) & 0xFF) << 4)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_MOD_EN                BIT(3)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CLKOUT_EN             BIT(2)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_RST                   BIT(1)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_PD                    BIT(0)

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL3 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_R2_SEL(x)             (((x) & 0x3) << 22)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_R3_SEL(x)             (((x) & 0x3) << 20)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_C1_SEL(x)             (((x) & 0x3) << 18)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_C2_SEL(x)             (((x) & 0x3) << 16)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_KVCO_SEL(x)           (((x) & 0x3) << 14)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_VCO_TEST_INTSEL(x)    (((x) & 0x7) << 11)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_VCO_TEST_INT          BIT(10)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CP_EN                 BIT(9)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_LDO_TRIM(x)           (((x) & 0xF) << 5)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_VCO_TEST_EN           BIT(4)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_FBDIV_EN              BIT(3)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CP_OFFSET(x)          (((x) & 0x3) << 1)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_VCOBUF_EN             BIT(0)

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL4 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_RESERVED(x)           (((x) & 0xFFFF))

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL5 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_BIST_CTRL(x)          (((x) & 0xFF) << 17)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_BIST_EN               BIT(16)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_BIST_CNT(x)           (((x) & 0xFFFF))

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL6 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_26MBUFFER_PD          BIT(0)

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL7 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CALI_MODE(x)          (((x) & 0x3) << 17)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CALI_INI(x)           (((x) & 0x1F) << 12)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CALI_TRIG             BIT(11)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_FREQ_DIFF_EN          BIT(10)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CALI_WAITCNT(x)       (((x) & 0x3) << 8)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CALI_POLARITY         BIT(7)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CALI_DONE             BIT(6)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CALI_OUT(x)           (((x) & 0x1F) << 1)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CALI_CPPD             BIT(0)

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CTRL8 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_VCTRLH_SEL(x)         (((x) & 0x7) << 17)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_VCTRLL_SEL(x)         (((x) & 0x7) << 14)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_RG_CLOSELOOP_EN       BIT(13)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_VCO_BANK_SEL(x)       (((x) & 0x1F) << 8)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CALI_VCTRL_HIGH       BIT(7)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_CALI_VCTRL_LOW        BIT(6)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_VCO_BANK_SEL_OFFSET   BIT(5)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_ADJ_MANUAL_PD         BIT(4)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_ISO_SW_EN             BIT(3)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_TEST_CLK_EN                 BIT(2)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_TEST_SEL                    BIT(1)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_26MBUFFER_CLKOUT_EN   BIT(0)

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_PROMETHEUS_DVFS_INDEX0 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_N_INDEX0(x)           (((x) & 0x7FF) << 4)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_POSTDIV_INDEX0        BIT(3)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_ICP_INDEX0(x)         (((x) & 0x7))

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_PROMETHEUS_DVFS_INDEX1 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_N_INDEX1(x)           (((x) & 0x7FF) << 4)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_POSTDIV_INDEX1        BIT(3)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_ICP_INDEX1(x)         (((x) & 0x7))

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_PROMETHEUS_DVFS_INDEX2 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_N_INDEX2(x)           (((x) & 0x7FF) << 4)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_POSTDIV_INDEX2        BIT(3)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_ICP_INDEX2(x)         (((x) & 0x7))

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_PROMETHEUS_DVFS_INDEX3 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_N_INDEX3(x)           (((x) & 0x7FF) << 4)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_POSTDIV_INDEX3        BIT(3)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_ICP_INDEX3(x)         (((x) & 0x7))

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_PROMETHEUS_DVFS_INDEX4 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_N_INDEX4(x)           (((x) & 0x7FF) << 4)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_POSTDIV_INDEX4        BIT(3)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_ICP_INDEX4(x)         (((x) & 0x7))

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_PROMETHEUS_DVFS_INDEX5 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_N_INDEX5(x)           (((x) & 0x7FF) << 4)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_POSTDIV_INDEX5        BIT(3)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_ICP_INDEX5(x)         (((x) & 0x7))

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_PROMETHEUS_DVFS_INDEX6 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_N_INDEX6(x)           (((x) & 0x7FF) << 4)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_POSTDIV_INDEX6        BIT(3)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_ICP_INDEX6(x)         (((x) & 0x7))

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_PROMETHEUS_DVFS_INDEX7 */

#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_N_INDEX7(x)           (((x) & 0x7FF) << 4)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_POSTDIV_INDEX7        BIT(3)
#define BIT_ANLG_PHY_G2_ANALOG_MPLL1_MPLL1_ICP_INDEX7(x)         (((x) & 0x7))

/* REG_ANLG_PHY_G2_ANALOG_MPLL1_REG_SEL_CFG_0 */

#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_MPLL1_MPLL1_N             BIT(7)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_MPLL1_MPLL1_ICP           BIT(6)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_MPLL1_MPLL1_DIV32_EN      BIT(5)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_MPLL1_MPLL1_POSTDIV       BIT(4)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_MPLL1_MPLL1_CLKOUT_EN     BIT(3)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_MPLL1_MPLL1_RST           BIT(2)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_MPLL1_MPLL1_PD            BIT(1)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_MPLL1_MPLL1_26MBUFFER_PD  BIT(0)

/* REG_ANLG_PHY_G2_ANALOG_THM2_THM2_CTL */

#define BIT_ANLG_PHY_G2_ANALOG_THM2_THM_RSTN                     BIT(27)
#define BIT_ANLG_PHY_G2_ANALOG_THM2_THM_RUN                      BIT(26)
#define BIT_ANLG_PHY_G2_ANALOG_THM2_THM_PD                       BIT(25)
#define BIT_ANLG_PHY_G2_ANALOG_THM2_THM_VALID                    BIT(24)
#define BIT_ANLG_PHY_G2_ANALOG_THM2_THM_BG_RBIAS_MODE            BIT(23)
#define BIT_ANLG_PHY_G2_ANALOG_THM2_THM_TEST_SEL(x)              (((x) & 0x3) << 21)
#define BIT_ANLG_PHY_G2_ANALOG_THM2_THM_BP_MODE                  BIT(20)
#define BIT_ANLG_PHY_G2_ANALOG_THM2_THM_DATA(x)                  (((x) & 0x3FF) << 10)
#define BIT_ANLG_PHY_G2_ANALOG_THM2_THM_BP_DATA(x)               (((x) & 0x3FF))

/* REG_ANLG_PHY_G2_ANALOG_THM2_THM2_RESERVED_CTL */

#define BIT_ANLG_PHY_G2_ANALOG_THM2_THM_RESERVED(x)              (((x) & 0xFFFF))

/* REG_ANLG_PHY_G2_ANALOG_THM2_REG_SEL_CFG_0 */

#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_THM2_THM_RSTN             BIT(3)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_THM2_THM_RUN              BIT(2)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_THM2_THM_PD               BIT(1)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_THM2_THM_RESERVED         BIT(0)

/* REG_ANLG_PHY_G2_ANALOG_USB20_USB20_TEST_PIN */

#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_TESTCLK               BIT(24)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_TESTDATAIN(x)         (((x) & 0xFF) << 16)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_TESTADDR(x)           (((x) & 0xF) << 12)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_TESTDATAOUTSEL        BIT(11)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_TESTDATAOUT(x)        (((x) & 0xF) << 7)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_BIST_MODE(x)          (((x) & 0x1F) << 2)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_T2RCOMP               BIT(1)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_LPBK_END              BIT(0)

/* REG_ANLG_PHY_G2_ANALOG_USB20_USB20_UTMI_CTL1 */

#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_DATABUS16_8           BIT(28)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_SUSPENDM              BIT(27)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_PORN                  BIT(26)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_RESET                 BIT(25)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_RXERROR               BIT(24)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_BYPASS_DRV_DP         BIT(23)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_BYPASS_DRV_DM         BIT(22)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_BYPASS_FS             BIT(21)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_BYPASS_IN_DP          BIT(20)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_BYPASS_IN_DM          BIT(19)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_BYPASS_OUT_DP         BIT(18)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_BYPASS_OUT_DM         BIT(17)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_VBUSVLDEXT            BIT(16)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_RESERVED(x)           (((x) & 0xFFFF))

/* REG_ANLG_PHY_G2_ANALOG_USB20_USB20_BATTER_PLL */

#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_PS_PD_S               BIT(4)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_PS_PD_L               BIT(3)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_REXTENABLE            BIT(2)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_DMPULLUP              BIT(1)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_SAMPLER_SEL           BIT(0)

/* REG_ANLG_PHY_G2_ANALOG_USB20_USB20_UTMI_CTL2 */

#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_DPPULLDOWN            BIT(4)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_DMPULLDOWN            BIT(3)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_TXBITSTUFFENABLE      BIT(2)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_TXBITSTUFFENABLEH     BIT(1)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_SLEEPM                BIT(0)

/* REG_ANLG_PHY_G2_ANALOG_USB20_USB20_TRIMMING */

#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_TUNEHSAMP(x)          (((x) & 0x3) << 25)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_TFREGRES(x)           (((x) & 0x3F) << 19)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_TFHSRES(x)            (((x) & 0x1F) << 14)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_TUNERISE(x)           (((x) & 0x3) << 12)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_TUNEOTG(x)            (((x) & 0x7) << 9)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_TUNEDSC(x)            (((x) & 0x3) << 7)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_TUNESQ(x)             (((x) & 0xF) << 3)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_TUNEEQ(x)             (((x) & 0x7))

/* REG_ANLG_PHY_G2_ANALOG_USB20_USB20_PHY_TUNE_CTL */

#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_TUNEPLLS(x)           (((x) & 0xF) << 10)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_PLL_PFD_DEADZONE(x)   (((x) & 0x3) << 8)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_PLL_PFD_DELAY(x)      (((x) & 0x3) << 6)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_PLL_CP_IOFFSET_EN     BIT(5)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_PLL_CP_IOFFSET(x)     (((x) & 0xF) << 1)
#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_PLL_REF_DOUBLER_EN    BIT(0)

/* REG_ANLG_PHY_G2_ANALOG_USB20_USB20_PHY_BIST_TEST */

#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_BIST_MODE_EN          BIT(0)

/* REG_ANLG_PHY_G2_ANALOG_USB20_USB20_ISO_SW */

#define BIT_ANLG_PHY_G2_ANALOG_USB20_USB20_ISO_SW_EN             BIT(0)

/* REG_ANLG_PHY_G2_ANALOG_USB20_REG_SEL_CFG_0 */

#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_USB20_USB20_SUSPENDM      BIT(8)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_USB20_USB20_PORN          BIT(7)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_USB20_USB20_RESET         BIT(6)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_USB20_USB20_BYPASS_FS     BIT(5)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_USB20_USB20_BYPASS_IN_DM  BIT(4)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_USB20_USB20_SAMPLER_SEL   BIT(3)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_USB20_USB20_DPPULLDOWN    BIT(2)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_USB20_USB20_DMPULLDOWN    BIT(1)
#define BIT_ANLG_PHY_G2_DBG_SEL_ANALOG_USB20_USB20_SLEEPM        BIT(0)


#endif /* ANLG_PHY_G2_H */

