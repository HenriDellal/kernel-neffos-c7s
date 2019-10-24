/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _CPP_REG_H_
#define _CPP_REG_H_

#define MM_AHB_RESET                  (0x0004UL)	/* need to check */
#define CPP_AHB_RESET_BIT              (1 << 15)	/* need to check */

#define CPP_BASE                       0x00

#define CPP_PATH_EB                    0x00
#define CPP_DMA_EB_BIT		       BIT(2)
#define CPP_ROT_EB_BIT                 BIT(1)
#define CPP_SCALE_PATH_EB_BIT          BIT(0)

#define CPP_PATH_START                 0x04
#define CPP_DMA_START_BIT              BIT(2)
#define CPP_ROT_START_BIT              BIT(1)
#define CPP_SCALE_START_BIT            BIT(0)

#define CPP_PATH_STS                   0x08
#define CPP_INT_STS                    0x0c
#define CPP_INT_MASK                   0x10
#define CPP_INT_CLR                    0x14
#define CPP_INT_RAW                    0x18
#define CPP_DMA_IRQ_BIT                BIT(2)
#define CPP_ROT_IRQ_BIT                BIT(1)
#define CPP_SCALE_IRQ_BIT              BIT(0)

#define CPP_AXIM_CHN_SET               0x1c
#define CPP_ROT_AXI_WR_ENDIAN_MASK     (7 << 8)
#define CPP_SCALE_DMA_OUTPUT_UV_ENDIAN     (1 << 7)
#define CPP_SCALE_DMA_OUTPUT_Y_ENDIAN      (7 << 4)
#define CPP_SCALE_DMA_INPUT_UV_ENDIAN      (1 << 3)
#define CPP_SCALE_DMA_INPUT_Y_ENDIAN       (7 << 0)
#define CPP_AXIM_CHN_SET_QOS_MASK          (0xf << 16)

#define CPP_PATH0_SRC_ADDR_Y           0x40
#define CPP_PATH0_SRC_ADDR_UV          0x44

#define CPP_PATH0_DES_ADDR_Y           0x48
#define CPP_PATH0_DES_ADDR_UV          0x4c

#define CPP_PATH0_CFG0                 0x50
#define CPP_SCALE_DEC_V_MASK           (3 << 6)
#define CPP_SCALE_DEC_H_MASK           (3 << 4)
#define CPP_SCALE_OUTPUT_FORMAT        (3 << 2)
#define CPP_SCALE_INPUT_FORMAT         (3 << 0)

#define CPP_PATH0_CFG1                 0x54
#define CPP_SCALE_SRC_HEIGHT_MASK      (0x1fff << 16)
#define CPP_SCALE_SRC_WIDTH_MASK       0x1fff

#define CPP_PATH0_CFG2                 0x58
#define CPP_SCALE_DES_HEIGHT_MASK      (0x1fff << 16)
#define CPP_SCALE_DES_WIDTH_MASK       (0x1fff)

#define CPP_PATH0_CFG3                 0x5c
#define CPP_SCALE_DES_PITCH_MASK       (0x1fff << 16)
#define CPP_SCALE_SRC_PITCH_MASK       0x1fff

#define CPP_PATH0_CFG4                 0x60
#define CPP_SCALE_SRC_OFFSET_X_MASK    (0x1fff << 16)
#define CPP_SCALE_SRC_OFFSET_Y_MASK    0x1fff

#define CPP_PATH0_CFG5                 0x64
#define CPP_SCALE_DES_OFFSET_X_MASK    (0x1fff << 16)
#define CPP_SCALE_DES_OFFSET_Y_MASK    0x1fff

#define CPP_ROTATION_SRC_ADDR          0x100
#define CPP_ROTATION_DES_ADDR          0x104
#define CPP_ROTATION_IMG_SIZE          0x108
#define CPP_ROTATION_SRC_PITCH         0x10c
#define CPP_ROTATION_OFFSET_START      0x110
#define CPP_ROTATION_PATH_CFG          0x114
#define CPP_ROT_UV_MODE_BIT            (1 << 4)
#define CPP_ROT_MODE_MASK              (3 << 2)
#define CPP_ROT_PIXEL_FORMAT_BIT       (3 << 0)

#define CPP_DMA_SRC_ADDR          0x180
#define CPP_DMA_DES_ADDR          0x184

#define CPP_DMA_CFG1                 0x188
#define CPP_DMA_HEIGHT_MASK      (0x1fff << 16)
#define CPP_DMA_WIDTH_MASK       0x1fff

#define CPP_DMA_CFG2                 0x18c
#define CPP_DMA_DES_PITCH_MASK       (0x1fff << 16)
#define CPP_DMA_SRC_PITCH_MASK       0x1fff

#define CPP_DMA_CFG3                 0x190
#define CPP_DMA_SRC_OFFSET_X_MASK    (0x1fff << 16)
#define CPP_DMA_SRC_OFFSET_Y_MASK    0x1fff

#define CPP_DMA_CFG4                 0x194
#define CPP_DMA_DES_OFFSET_X_MASK    (0x1fff << 16)
#define CPP_DMA_DES_OFFSET_Y_MASK    0x1fff

#define CPP_END                        0x200	/* need to check */

#define CPP_MMU_EN                     0x0200
#define MMU_PPN_RANGE1		0x0240
#define MMU_PPN_RANGE2		0x0248

#define CPP_MMU_PT_UPDATE_QOS          0x0234
#define CPP_MMU_PT_UPDATE_QOS_MASK     (0xf << 0)
#endif
