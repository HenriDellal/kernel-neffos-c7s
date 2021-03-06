/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
 *
 * This file is dual-licensed: you can use it either under the terms
 * of the GPL or the X11 license, at your option. Note that this dual
 * licensing only applies to this file, and not this project as a
 * whole.
 *
 * updated at 2017-06-12 20:13:17
 *
 */


#ifndef ANLG_PHY_G2_H
#define ANLG_PHY_G2_H



#define REG_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_CTRL0          (0x0000)
#define REG_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_CTRL1          (0x0004)
#define REG_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_CTRL2          (0x0008)
#define REG_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_BIST_CTRL      (0x000C)
#define REG_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_THM_CTRL       (0x0010)
#define REG_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_THM0_CTRL_0         (0x0014)
#define REG_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_THM0_CTRL_1         (0x0018)
#define REG_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_ANA_DPLL_DUMY       (0x001C)

/* REG_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_CTRL0 */

#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_N(x)                (((x) & 0x7FF) << 17)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_IBIAS(x)            (((x) & 0x3) << 15)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_LPF(x)              (((x) & 0x7) << 12)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_SDM_EN              BIT(11)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_MOD_EN              BIT(10)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_DIV_S               BIT(9)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_RESERVED(x)         (((x) & 0xFF) << 1)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_POSTDIV             BIT(0)

/* REG_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_CTRL1 */

#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_NINT(x)             (((x) & 0x7F) << 23)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_KINT(x)             (((x) & 0x7FFFFF))

/* REG_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_CTRL2 */

#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_CCS_CTRL(x)         (((x) & 0xFF))

/* REG_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_BIST_CTRL */

#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_BIST_EN             BIT(24)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_BIST_CTRL(x)        (((x) & 0xFF) << 16)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_BIST_CNT(x)         (((x) & 0xFFFF))

/* REG_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_DPLL_THM_CTRL */

#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_TEST_CLK_EN              BIT(13)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_TEST_SEL                 BIT(12)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_TEST_THM_SEL             BIT(11)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_ANALOG_PLL_RESERVED(x)   (((x) & 0x7FF))

/* REG_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_THM0_CTRL_0 */

#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_THM_BG_RBIAS_MODE        BIT(2)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_THM_TEST_SEL(x)          (((x) & 0x3))

/* REG_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_THM0_CTRL_1 */

#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_THM_BP_MODE              BIT(24)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_THM_BP_DATA(x)           (((x) & 0xFF) << 16)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_THM_RESERVED(x)          (((x) & 0xFFFF))

/* REG_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_ANA_DPLL_DUMY */

#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_ANALOG_DPLL_DUMY_IN(x)   (((x) & 0xFFFF) << 16)
#define BIT_ANLG_PHY_G2_ANALOG_DPLL_THM_TOP_ANALOG_DPLL_DUMY_OUT(x)  (((x) & 0xFFFF))


#endif /* ANLG_PHY_G2_H */

