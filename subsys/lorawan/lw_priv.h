/*
 * Copyright (c) 2020 Andreas Sandberg
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_LORAWAN_LW_PRIV_H_
#define ZEPHYR_SUBSYS_LORAWAN_LW_PRIV_H_

const char *lorawan_mcps2str(unsigned int mcps);
const char *lorawan_mlme2str(unsigned int mlme);

const int lorawan_status2errno(unsigned int status);
const char *lorawan_status2str(unsigned int status);

const int lorawan_eventinfo2errno(unsigned int status);
const char *lorawan_eventinfo2str(unsigned int status);

#endif /* ZEPHYR_SUBSYS_LORAWAN_LW_PRIV_H_ */
