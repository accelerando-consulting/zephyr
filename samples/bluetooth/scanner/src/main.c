/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>


//#define SOUND_PIN 4

#ifdef SOUND_PIN
#include <device.h>
#include <drivers/pwm.h>
static struct device *pwm;

void beep(int period, int duration) 
{
	pwm_pin_set_usec(pwm, SOUND_PIN, period, period / 2, 0);
	k_msleep(duration);
	pwm_pin_set_usec(pwm, SOUND_PIN, 0, 0, 0);
}
#endif

static void scan_start(void);

#define SCAN_SLOTS 15
#define EXPIRE_LIMIT (20*1000)

#define RSSI_OOB_SLOT_UNUSED 127
typedef struct 
{
	bt_addr_t addr;
	uint32_t last_seen;
	int8_t rssi;
} slot_t;

slot_t slots[SCAN_SLOTS];

unsigned long event_count=0;
unsigned long event_start=0;

#define ESC 0x1b

#define ANSI(name,...) static const char ANSI_##name[]={ESC,'[',__VA_ARGS__,'\0'}
	
ANSI(HOME, 'H');
ANSI(CLEAR, '2','J');
ANSI(CLEARRIGHT, '0','K');
ANSI(CLEARDOWN, '0','J');

int cmp_by_rssi(slot_t *table, int a, int b) 
{
	if (table[a].rssi < table[b].rssi) return -1;
	if (table[a].rssi > table[b].rssi) return 1;
	return bt_addr_cmp(&(table[a].addr), &(table[b].addr));
}

void expire_slots(slot_t *table) 
{
	uint32_t now = k_uptime_get();
	
	for (int i=0; i<SCAN_SLOTS; i++) {
		if (table[i].rssi == RSSI_OOB_SLOT_UNUSED) continue;
		if ((table[i].last_seen + EXPIRE_LIMIT) < now) {
			// expire slot i
			table[i].rssi = RSSI_OOB_SLOT_UNUSED;
		}
	}

}

void print_stats(slot_t *table) 
{
	uint32_t now = k_uptime_get();
	uint32_t elapsed_ms = now - event_start;
	uint32_t ratex100 = 100000*event_count/elapsed_ms;
	
	int device_count=0;
	for (int i=0; i<SCAN_SLOTS;i++) {
		if (table[i].rssi != RSSI_OOB_SLOT_UNUSED) ++device_count;
	}
	printk("%d devices%s\n%d.%02d msg/s%s\n",
	       device_count, ANSI_CLEARRIGHT, ratex100/100,ratex100%100, ANSI_CLEARRIGHT);
	printk("%s\n",ANSI_CLEARRIGHT);
	if (elapsed_ms > 10000) {
		event_count = 0;
		event_start = k_uptime_get();
	}
}

void print_slots(slot_t *table) 
{
	char addr_str[BT_ADDR_STR_LEN];
	uint32_t now = k_uptime_get();
	printk("%s", ANSI_HOME);
	print_stats(table);

	for (int i=0; i<SCAN_SLOTS;i++) {
		if (table[i].rssi == RSSI_OOB_SLOT_UNUSED) {
			//printk("%d UNUSED\n", i);
			//continue;
			//break;
		}
		else {
			int age = (now - table[i].last_seen)/1000;
			bt_addr_to_str(&(table[i].addr), addr_str, sizeof(addr_str));
			printk("%3d %s", table[i].rssi, addr_str);
			if (age > 5) {
				printk(" %2ds", age);
			}
			printk("%s\n", ANSI_CLEARRIGHT);
		}
	}
	printk("%s", ANSI_CLEARDOWN);
}

bool sort_slots_by_rssi(slot_t *table) 
{
	// Do a simple selection sort
	int first = 0;
	int best;
	int i;
	bool changed = false;

	for (first=0; first < (SCAN_SLOTS-1); first++) {
		//printk("Scanning for best item from %d candidate rssi=%d\n", first, table[first].rssi);
		for (best=first,i=first+1;i<SCAN_SLOTS;i++) {
			if (table[i].rssi == RSSI_OOB_SLOT_UNUSED) continue;
			if (cmp_by_rssi(table, i, best)>0) {
				//printk("  New best from %d is slot %d rssi=%d\n", first, i, table[i].rssi);
				best=i;
			}
		}
		//printk("  Best from %d is slot %d rssi=%d\n", first, best, table[best].rssi);
		if (best != first) {
			slot_t tmp;
			memcpy(&tmp, table+first, sizeof(slot_t));
			memcpy(table+first, table+best, sizeof(slot_t));
			memcpy(table+best, &tmp, sizeof(slot_t));
			changed = true;
		}
	}
	return changed;
}

static void device_found(const struct bt_le_scan_recv_info *info, struct net_buf_simple *buf)
{
	char addr_str[BT_ADDR_STR_LEN];
	bool changed = false;
	

	bt_addr_to_str(&(info->addr->a), addr_str, sizeof(addr_str));
	++event_count;
	//printk("Device found: %s (RSSI %d)\n", addr_str, info->rssi);
	int empty=-1;
	int found=-1;
	for (int i=0; (found<0)&&(i<SCAN_SLOTS); i++) {
		if (slots[i].rssi == RSSI_OOB_SLOT_UNUSED) {
			if (empty<0) empty = i;
		}
		else if (bt_addr_cmp(&(slots[i].addr), &(info->addr->a))==0) {
			found = i;
		}
	}
	if ((found<0) && (empty<0)) {
		//printk("ALERT slots full\n");
		return;
	}
	
	if (found < 0) {
		//printk("NEW %s RSSI=%d type=%d\n", addr_str, info->rssi,info->adv_type);
		bt_addr_copy(&(slots[empty].addr), &(info->addr->a));
		found = empty;
		changed = true;
	}
	else {
		if (slots[found].rssi != info->rssi) {
			//printk("CHG %s RSSI=%d\n", addr_str, info->rssi);
			changed = true;
		}
	}
	slots[found].rssi = info->rssi;
	slots[found].last_seen = k_uptime_get();

#ifdef SOUND_PIN
	beep(330+info->rssi,20);
#endif

	expire_slots(slots);
	changed |= sort_slots_by_rssi(slots);
	if (changed) print_slots(slots);
	
}

void scan_timeout(void) 
{
	printk("Restart");
	scan_start();
}


static void scan_start(void)
{
	int err;
	static struct bt_le_scan_cb cb = {
		device_found,
		scan_timeout
	};
	event_count = 0;
	event_start = k_uptime_get();
	bt_le_scan_cb_register(&cb);
	err = bt_le_scan_start(BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE,
						BT_LE_SCAN_OPT_NONE,
						BT_GAP_SCAN_FAST_INTERVAL,
						BT_GAP_SCAN_FAST_WINDOW),
			       NULL);
	if (err) {
		printk("Scan err (%d)\n", err);
		return;
	}
	printk("Scan started\n");
}


void main(void)
{
	int err;
	printk("%s%s", ANSI_CLEAR, ANSI_HOME);
	printk("Scanner Go\n");

#ifdef SOUND_PIN
	pwm = device_get_binding(DT_LABEL(DT_INST(0, nordic_nrf_sw_pwm)));
#endif
	
	for (int i=0; i<SCAN_SLOTS; i++) {
		slots[i].rssi = RSSI_OOB_SLOT_UNUSED;
	}
	
	err = bt_enable(NULL);
	if (err) {
		printk("BT init err %d\n", err);
		return;
	}

	printk("BT initialized\n");

	scan_start();
}
