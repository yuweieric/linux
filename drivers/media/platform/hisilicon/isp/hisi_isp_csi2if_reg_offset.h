/*
 * Copyright (c) 2016-2017 Linaro Ltd.
 * Copyright (c) 2016-2017 HiSilicon Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef __CSI2IF_REG_OFFSET_H__
#define __CSI2IF_REG_OFFSET_H__

#define CSI2IF_VERSION_REG               0x0
#define CSI2IF_N_LANES_REG               0x4
#define CSI2IF_CSI2_RESETN_REG           0x8
#define CSI2IF_INT_ST_MAIN_REG           0xC
#define CSI2IF_DATA_IDS_1_REG            0x10
#define CSI2IF_PHY_SHUTDOWNZ_REG         0x40
#define CSI2IF_DPHY_RSTZ_REG             0x44
#define CSI2IF_PHY_RX_REG                0x48
#define CSI2IF_PHY_STOPSTATE_REG         0x4C
#define CSI2IF_PHY_TEST_CTRL0_REG        0x50
#define CSI2IF_PHY_TEST_CTRL1_REG        0x54
#define CSI2IF_PHY_CAL_REG               0xCC
#define CSI2IF_INT_ST_PHY_FATAL_REG      0xE0
#define CSI2IF_INT_MSK_PHY_FATAL_REG     0xE4
#define CSI2IF_INT_FORCE_PHY_FATAL_REG   0xE8
#define CSI2IF_INT_ST_PKT_FATAL_REG      0xF0
#define CSI2IF_INT_MSK_PKT_FATAL_REG     0xF4
#define CSI2IF_INT_FORCE_PKT_FATAL_REG   0xF8
#define CSI2IF_INT_ST_FRAME_FATAL_REG    0x100
#define CSI2IF_INT_MSK_FRAME_FATAL_REG   0x104
#define CSI2IF_INT_FORCE_FRAME_FATAL_REG 0x108
#define CSI2IF_INT_ST_PHY_REG            0x110
#define CSI2IF_INT_MSK_PHY_REG           0x114
#define CSI2IF_INT_FORCE_PHY_REG         0x118
#define CSI2IF_INT_ST_PKT_REG            0x120
#define CSI2IF_INT_MSK_PKT_REG           0x124
#define CSI2IF_INT_FORCE_PKT_REG         0x128
#define CSI2IF_INT_ST_LINE_REG           0x130
#define CSI2IF_INT_MSK_LINE_REG          0x134
#define CSI2IF_INT_FORCE_LINE_REG        0x138

#endif
