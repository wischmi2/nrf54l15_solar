/*
 * Nordic_Solar — custom BLE GATT service implementation.
 *
 * One characteristic (read + notify) carrying a packed struct solar_status.
 * Read it or subscribe to notifications from the nRF Connect for Mobile app.
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "solar_service.h"

LOG_MODULE_REGISTER(solar_svc, LOG_LEVEL_INF);

static struct bt_uuid_128 solar_svc_uuid = BT_UUID_INIT_128(BT_UUID_SOLAR_SVC_VAL);
static struct bt_uuid_128 solar_status_uuid = BT_UUID_INIT_128(BT_UUID_SOLAR_STATUS_VAL);

static struct solar_status last_status;
static bool notify_enabled;

static ssize_t read_status(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &last_status, sizeof(last_status));
}

static void status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("status notifications %s", notify_enabled ? "enabled" : "disabled");
}

/* Attribute layout: [0]=service, [1]=chrc decl, [2]=chrc value, [3]=CCC */
BT_GATT_SERVICE_DEFINE(solar_svc,
	BT_GATT_PRIMARY_SERVICE(&solar_svc_uuid),
	BT_GATT_CHARACTERISTIC(&solar_status_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_status, NULL, &last_status),
	BT_GATT_CCC(status_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

void solar_service_update(const struct solar_status *s)
{
	memcpy(&last_status, s, sizeof(last_status));

	if (!notify_enabled) {
		return;
	}

	/* Notify on the characteristic value attribute (index 2). */
	(void)bt_gatt_notify(NULL, &solar_svc.attrs[2],
			     &last_status, sizeof(last_status));
}
