#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_ip.h>
#if 0
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#endif
#include <zephyr/net/openthread.h>

#include <openthread/instance.h>
#include <openthread/thread.h>
#include <openthread/ip6.h>
#include <openthread/dataset_ftd.h>

#include <errno.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* =========================
 * Node A configuration
 * ========================= */
#define NODE_NAME                    "NodeA"
#define BLE_DEVICE_NAME              "OT_BLE_A"

/* Replace with actual Node B Thread IPv6 address */
#define PEER_IPV6_ADDR "fdde:ad00:beef:0:67c9:c18e:3dac:4f93"

#define UDP_PEER_PORT                12345
#define FIRST_TX_DELAY_MS            3000
#define TX_PERIOD_MS                 5000

/* =========================
 * Common configuration
 * ========================= */
#define BLE_NOTIFY_PERIOD_MS         1000
#define THREAD_ATTACH_TIMEOUT_MS     30000

#define OT_NETWORK_NAME              "STM_THREAD"
#define OT_CHANNEL                   20
#define OT_PANID                     0x1234

static const uint8_t ot_network_key[16] = {
	0x00, 0x11, 0x22, 0x33,
	0x44, 0x55, 0x66, 0x77,
	0x88, 0x99, 0xaa, 0xbb,
	0xcc, 0xdd, 0xee, 0xff
};

static const uint8_t ot_xpanid[8] = {
	0xde, 0xad, 0x00, 0xbe,
	0xef, 0x00, 0xca, 0xfe
};
#if 0
static struct bt_conn *current_conn;
static bool notify_enabled;
static uint32_t ble_counter;
#endif
static int udp_sock = -1;
static bool thread_ready;
#if 0
/* =========================
 * BLE UUIDs
 * ========================= */
static struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(
	0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 char_uuid = BT_UUID_INIT_128(
	0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

/* =========================
 * BLE GATT
 * ========================= */
static ssize_t read_counter(struct bt_conn *conn,
			    const struct bt_gatt_attr *attr,
			    void *buf, uint16_t len, uint16_t offset)
{
	uint32_t value_le = sys_cpu_to_le32(ble_counter);

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &value_le, sizeof(value_le));
}

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);

	LOG_INF("[%s] BLE notifications %s",
		NODE_NAME, notify_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(dummy_svc,
	BT_GATT_PRIMARY_SERVICE(&service_uuid),
	BT_GATT_CHARACTERISTIC(&char_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_counter, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, BLE_DEVICE_NAME, sizeof(BLE_DEVICE_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("[%s] BLE connection failed: %u", NODE_NAME, err);
		return;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
	}

	current_conn = bt_conn_ref(conn);
	LOG_INF("[%s] BLE connected", NODE_NAME);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);

	LOG_INF("[%s] BLE disconnected (reason 0x%02x)", NODE_NAME, reason);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	notify_enabled = false;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static int ble_init(void)
{
	int err = bt_enable(NULL);
	if (err) {
		LOG_ERR("[%s] bt_enable failed: %d", NODE_NAME, err);
		return err;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("[%s] bt_le_adv_start failed: %d", NODE_NAME, err);
		return err;
	}

	LOG_INF("[%s] BLE advertising started", NODE_NAME);
	return 0;
}

static void ble_notify_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(ble_notify_work, ble_notify_work_fn);

static void ble_notify_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (current_conn && notify_enabled) {
		uint32_t value_le = sys_cpu_to_le32(++ble_counter);
		int err = bt_gatt_notify(NULL, &dummy_svc.attrs[1],
					 &value_le, sizeof(value_le));

		if (err) {
			LOG_WRN("[%s] bt_gatt_notify failed: %d", NODE_NAME, err);
		} else {
			LOG_INF("[%s] BLE notified counter=%u", NODE_NAME, ble_counter);
		}
	}

	k_work_schedule(&ble_notify_work, K_MSEC(BLE_NOTIFY_PERIOD_MS));
}
#endif
/* =========================
 * OpenThread helpers
 * ========================= */
static const char *thread_role_to_str(otDeviceRole role)
{
	switch (role) {
	case OT_DEVICE_ROLE_DISABLED: return "disabled";
	case OT_DEVICE_ROLE_DETACHED: return "detached";
	case OT_DEVICE_ROLE_CHILD:    return "child";
	case OT_DEVICE_ROLE_ROUTER:   return "router";
	case OT_DEVICE_ROLE_LEADER:   return "leader";
	default:                      return "unknown";
	}
}

static bool thread_role_is_attached(otDeviceRole role)
{
	return (role == OT_DEVICE_ROLE_CHILD ||
		role == OT_DEVICE_ROLE_ROUTER ||
		role == OT_DEVICE_ROLE_LEADER);
}

static void thread_dump_unicast_addrs(otInstance *ot)
{
	const otNetifAddress *addr = otIp6GetUnicastAddresses(ot);
	char addr_str[OT_IP6_ADDRESS_STRING_SIZE];

	LOG_INF("[%s] Thread unicast IPv6 addresses:", NODE_NAME);

	while (addr) {
		otIp6AddressToString(&addr->mAddress, addr_str, sizeof(addr_str));
		LOG_INF("[%s]   %s", NODE_NAME, addr_str);
		addr = addr->mNext;
	}
}

static int thread_init_and_start(void)
{
	otInstance *ot;
	otOperationalDataset dataset;
	otError ot_err;

	ot = openthread_get_default_instance();
	if (!ot) {
		LOG_ERR("[%s] OpenThread instance unavailable", NODE_NAME);
		return -ENODEV;
	}

	memset(&dataset, 0, sizeof(dataset));

	dataset.mActiveTimestamp.mSeconds = 1;
	dataset.mComponents.mIsActiveTimestampPresent = true;

	dataset.mChannel = OT_CHANNEL;
	dataset.mComponents.mIsChannelPresent = true;

	dataset.mPanId = OT_PANID;
	dataset.mComponents.mIsPanIdPresent = true;

	memcpy(dataset.mNetworkKey.m8, ot_network_key, sizeof(ot_network_key));
	dataset.mComponents.mIsNetworkKeyPresent = true;

	memcpy(dataset.mExtendedPanId.m8, ot_xpanid, sizeof(ot_xpanid));
	dataset.mComponents.mIsExtendedPanIdPresent = true;

	strncpy(dataset.mNetworkName.m8, OT_NETWORK_NAME,
		sizeof(dataset.mNetworkName.m8) - 1);
	dataset.mNetworkName.m8[sizeof(dataset.mNetworkName.m8) - 1] = '\0';
	dataset.mComponents.mIsNetworkNamePresent = true;

	ot_err = otDatasetSetActive(ot, &dataset);
	if (ot_err != OT_ERROR_NONE) {
		LOG_ERR("[%s] otDatasetSetActive failed: %d", NODE_NAME, ot_err);
		return -EIO;
	}

	ot_err = otIp6SetEnabled(ot, true);
	if (ot_err != OT_ERROR_NONE) {
		LOG_ERR("[%s] otIp6SetEnabled failed: %d", NODE_NAME, ot_err);
		return -EIO;
	}

	ot_err = otThreadSetEnabled(ot, true);
	if (ot_err != OT_ERROR_NONE) {
		LOG_ERR("[%s] otThreadSetEnabled failed: %d", NODE_NAME, ot_err);
		return -EIO;
	}

	LOG_INF("[%s] Thread start requested", NODE_NAME);
	return 0;
}

static int thread_wait_until_attached_and_dump_addrs(void)
{
	otInstance *ot;
	int64_t deadline = k_uptime_get() + THREAD_ATTACH_TIMEOUT_MS;

	ot = openthread_get_default_instance();
	if (!ot) {
		LOG_ERR("[%s] OpenThread instance unavailable", NODE_NAME);
		return -ENODEV;
	}

	while (k_uptime_get() < deadline) {
		otDeviceRole role = otThreadGetDeviceRole(ot);

		LOG_INF("[%s] Thread role: %s", NODE_NAME, thread_role_to_str(role));

		if (thread_role_is_attached(role)) {
			thread_dump_unicast_addrs(ot);
			thread_ready = true;
			return 0;
		}

		k_sleep(K_MSEC(1000));
	}

	LOG_ERR("[%s] Thread attach timeout", NODE_NAME);
	return -ETIMEDOUT;
}

/* =========================
 * UDP TX
 * ========================= */
static int udp_init_socket(void)
{
	udp_sock = zsock_socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	if (udp_sock < 0) {
		LOG_ERR("[%s] socket failed: %d", NODE_NAME, errno);
		return -errno;
	}

	LOG_INF("[%s] UDP socket created", NODE_NAME);
	return 0;
}

static void ot_tx_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(ot_tx_work, ot_tx_work_fn);

static void ot_tx_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (thread_ready && udp_sock >= 0) {
		struct sockaddr_in6 peer = {0};
		char msg[64];
		static uint32_t seq;
		int err;

		peer.sin6_family = AF_INET6;
		peer.sin6_port = htons(UDP_PEER_PORT);

		err = net_addr_pton(AF_INET6, PEER_IPV6_ADDR, &peer.sin6_addr);
		if (err < 0) {
			LOG_ERR("[%s] Invalid peer IPv6 address", NODE_NAME);
			goto out;
		}

		snprintk(msg, sizeof(msg), "%s periodic seq=%u", NODE_NAME, seq++);

		err = zsock_sendto(udp_sock, msg, strlen(msg), 0,
			     	  (struct sockaddr *)&peer, sizeof(peer));
		if (err < 0) {
			LOG_ERR("[%s] sendto failed: %d", NODE_NAME, errno);
		} else {
			LOG_INF("[%s] OT TX -> %s", NODE_NAME, msg);
		}
	}

out:
	k_work_schedule(&ot_tx_work, K_MSEC(TX_PERIOD_MS));
}

int main(void)
{
	int err;

	LOG_INF("[%s] Starting", NODE_NAME);
#if 0
	err = ble_init();
	if (err) {
		LOG_ERR("[%s] BLE init failed", NODE_NAME);
		return 0;
	}
#endif
	err = thread_init_and_start();
	if (err) {
		LOG_ERR("[%s] Thread init/start failed", NODE_NAME);
		return 0;
	}

	err = thread_wait_until_attached_and_dump_addrs();
	if (err) {
		LOG_ERR("[%s] Thread attach failed", NODE_NAME);
		return 0;
	}

	err = udp_init_socket();
	if (err) {
		LOG_ERR("[%s] UDP init failed", NODE_NAME);
		return 0;
	}
#if 0
	k_work_schedule(&ble_notify_work, K_MSEC(BLE_NOTIFY_PERIOD_MS));
#endif
	k_work_schedule(&ot_tx_work, K_MSEC(FIRST_TX_DELAY_MS));

	while (1) {
		k_sleep(K_SECONDS(1));
	}
}