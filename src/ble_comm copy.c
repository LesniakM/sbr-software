#include "ble_comm.h"

extern float kD;
extern float kP;
extern float kI;
extern bool drive;
extern float complementary_angle;

static bool notify_enabled;

// #define DEVICE_NAME CONFIG_BT_DEVICE_NAME
// #define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

// UUIDs for the custom service and characteristics
static struct bt_uuid_128 custom_service_uuid = BT_UUID_INIT_128(0x00000000, 0x0000, 0x0000, 0x0000, 0x000000000001);
static struct bt_uuid_128 kD_uuid = BT_UUID_INIT_128(0x00000000, 0x0000, 0x0000, 0x0000, 0x000000000002);
static struct bt_uuid_128 kP_uuid = BT_UUID_INIT_128(0x00000000, 0x0000, 0x0000, 0x0000, 0x000000000003);
static struct bt_uuid_128 kI_uuid = BT_UUID_INIT_128(0x00000000, 0x0000, 0x0000, 0x0000, 0x000000000004);
static struct bt_uuid_128 drive_uuid = BT_UUID_INIT_128(0x00000000, 0x0000, 0x0000, 0x0000, 0x000000000005);
static struct bt_uuid_128 ca_uuid = BT_UUID_INIT_128(0x00000000, 0x0000, 0x0000, 0x0000, 0x000000000006);

static ssize_t read_k_param(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    float *value = (float *)attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(float));
}

static ssize_t read_b_param(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    bool *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, 1);
}

static ssize_t write_k_param(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    float *value = (float *)attr->user_data;
    const int8_t *byte_buf = (const int8_t *)buf;
    if (len != 2)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    int32_t int_val = (uint8_t)byte_buf[0] + byte_buf[1] * 256;
    *value = ((float)int_val) / 1000.F;
    // memcpy(value, buf, sizeof(float));
    // printk("Updated. D: %d, B: %d, %d, F: %f\n", int_val, byte_buf[0], byte_buf[1], (double)*value);

    return len;
}

static ssize_t write_b_param(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    bool *value = attr->user_data;
    if (len != 1)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    memcpy(value, buf, 1);
    // printk("Updated parameter to %d\n", (bool)*value);
    return 1;
}

static void ccc_angle_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    // printk("UUID: %d\n", attr->uuid);
}

BT_GATT_SERVICE_DEFINE(custom_svc,
                       BT_GATT_PRIMARY_SERVICE(&custom_service_uuid),

                       // kD Characteristic
                       BT_GATT_CHARACTERISTIC(&kD_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              read_k_param, write_k_param, &kD),

                       // kP Characteristic
                       BT_GATT_CHARACTERISTIC(&kP_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              read_k_param, write_k_param, &kP),

                       // kI Characteristic
                       BT_GATT_CHARACTERISTIC(&kI_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              read_k_param, write_k_param, &kI),

                       // drive Characteristic
                       BT_GATT_CHARACTERISTIC(&drive_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              read_b_param, write_b_param, &drive),

                       // kD Characteristic
                       BT_GATT_CHARACTERISTIC(&ca_uuid.uuid, BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_NONE, NULL, NULL,
                                              NULL),

                       BT_GATT_CCC(ccc_angle_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    // BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

// static const struct bt_data sd[] = {
//	BT_DATA_BYTES(BT_DATA_UUID128_ALL, &drive_uuid.uuid),
// };

void init_ble_c()
{
    int err = bt_enable(NULL);
    if (err)
    {
        // printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    // printk("Bluetooth initialized\n");

    // Start advertising
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err)
    {
        // printk("Advertising failed to start (err %d)\n", err);
        return;
    }
    // printk("Advertising started\n");
}

int send_angle_notify(uint32_t sensor_value)
{
    if (!notify_enabled)
    {
        return -EACCES;
    }
    int err = 0;
    err = bt_gatt_notify(NULL, &custom_svc.attrs[10], &sensor_value, sizeof(sensor_value));

    return err;
}