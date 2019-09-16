/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2018 Intel Corporation */

#ifndef __PECI_HWMON_H
#define __PECI_HWMON_H

#include <linux/peci.h>

#define TEMP_TYPE_PECI   6 /* Sensor type 6: Intel PECI */
#define UPDATE_INTERVAL  HZ

/**
 * struct temp_data - PECI temperature information
 * @valid: flag to indicate the temperature value is valid
 * @value: temperature value in millidegree Celsius
 * @last_updated: time of the last update in jiffies
 */
struct temp_data {
	uint  valid;
	s32   value;
	ulong last_updated;
};

/**
 * peci_temp_need_update - check whether temperature update is needed or not
 * @temp: pointer to temperature data struct
 *
 * Return: true if update is needed, false if not.
 */
static inline bool peci_temp_need_update(struct temp_data *temp)
{
	if (temp->valid &&
	    time_before(jiffies, temp->last_updated + UPDATE_INTERVAL))
		return false;

	return true;
}

/**
 * peci_temp_mark_updated - mark the temperature is updated
 * @temp: pointer to temperature data struct
 */
static inline void peci_temp_mark_updated(struct temp_data *temp)
{
	temp->valid = 1;
	temp->last_updated = jiffies;
}

#endif /* __PECI_HWMON_H */
