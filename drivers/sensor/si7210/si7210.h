/*
 * Copyright (c) 2019 Accelerando
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SI7210_H
#define _SI7210_H

/* Si7210 register addresses */
#define SI7210_REG_CHIP_INFO 0xC0
#define SI7210_REG_SIG_HIGH 0xC1
#define SI7210_REG_SIG_LOW 0xC2
#define SI7210_REG_SIG_SEL 0xC3
#define SI7210_REG_CTRL 0xC4
#define SI7210_REG_AUTOINC 0xC5

#define SI7210_CHIP_ID_VALUE 0x01

#define SI7210_CTRL_BIT_SLEEP 0
#define SI7210_CTRL_BIT_STOP 1
#define SI7210_CTRL_BIT_ONEBURST 2
#define SI7210_CTRL_BIT_USESTORE 3	
#define SI7210_CTRL_BIT_MEAS 7

// fine   scale => 0.00125 mT/bit (+-20.47mT full scale) 0.0125G/bit (200.47G fsr)
// coarse scale => 0.01250 mT/bit (+-204.7mT full scale) 0.1250G/bit (2004.7G fsr)
#define SI7210_SCALE_DIVISOR_FINE 80
#define SI7210_SCALE_DIVISOR_COARSE 8
#endif /* _SI7210_H */
