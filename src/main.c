/*
 * Copyright (c) 2021, EVERGREEN FUND 501(c)(3)
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <errno.h>
#include <logging/log.h>
#include <max30102.h>
#include <stddef.h>
#include <string.h>
#include <sys/byteorder.h>
#include <sys/printk.h>
#include <zephyr.h>
#include <zephyr/types.h>

/* Static Functions */
static ssize_t recv(struct bt_conn *conn,
                    const struct bt_gatt_attr *attr, const void *buf,
                    uint16_t len, uint16_t offset, uint8_t flags);

struct Led_sample {
    uint32_t red;
    uint32_t ir;
};

struct __attribute__((__packed__)) Sample_packet {
    struct Led_sample samples[30];
    uint8_t sample_num;
    char end_bytes[2];
};

/* Helper function to reverse endianness */
inline uint32_t reverse_32(uint32_t value) {
    return (((value & 0x000000FF) << 24) |
            ((value & 0x0000FF00) << 8) |
            ((value & 0x00FF0000) >> 8) |
            ((value & 0xFF000000) >> 24));
}

/* For Logging */
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

K_SEM_DEFINE(sem, 0, 1);

#ifdef CONFIG_MAX30102_TRIGGER
/* Handles interrupts */
static void trigger_handler(const struct device *max30102,
                            const struct sensor_trigger *trigger) {
    switch (trigger->type) {
        case SENSOR_TRIG_DATA_READY:
            if (sensor_sample_fetch(max30102)) {
                LOG_ERR("Sample fetch error\n");
                return;
            }
            LOG_INF("Fetched sample\n");
            k_sem_give(&sem);
            break;
        default:
            LOG_ERR("Unknown trigger\n");
    }
}

static void drdy_trigger_mode(const struct device *max30102) {
    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ALL,
    };

    if (sensor_trigger_set(max30102, &trig, trigger_handler)) {
        LOG_ERR("Could not set trigger\n");
        return;
    }
    LOG_INF("Set trigger handler\n");
}
#endif

static uint32_t rand_val = 1;

/* BT832A Custom Service  */
static struct bt_uuid_128 sensor_service_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x0000cafe, 0xb0ba, 0x8bad, 0xf00d, 0xdeadbeef0000));

/* Sensor Control Characteristic */
static struct bt_uuid_128 sensor_ctrl_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x0001cafe, 0xb0ba, 0x8bad, 0xf00d, 0xdeadbeef0000));

/* Sensor Data Characteristic */
static struct bt_uuid_128 sensor_data_notif_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x0002cafe, 0xb0ba, 0x8bad, 0xf00d, 0xdeadbeef0000));

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define ADV_LEN 12

/* Advertising data */
static uint8_t manuf_data[ADV_LEN] = {
    0x01 /*SKD version */,
    0x83 /* STM32WB - P2P Server 1 */,
    0x00 /* GROUP A Feature  */,
    0x00 /* GROUP A Feature */,
    0x00 /* GROUP B Feature */,
    0x00 /* GROUP B Feature */,
    0x00, /* BLE MAC start -MSB */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00, /* BLE MAC stop */
};

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, ADV_LEN)};

/* BLE connection */
struct bt_conn *conn;
/* Notification state */
volatile bool notify_enable;

static void mpu_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    notify_enable = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notification %s", notify_enable ? "enabled" : "disabled");
}

/* The BT832A board is acting as GATT server.
 * The other side is the BLE GATT client.
 * TODO change naming
 */

/* BT832A GATT services and characteristic */

BT_GATT_SERVICE_DEFINE(bt832a_svc,
                       BT_GATT_PRIMARY_SERVICE(&sensor_service_uuid),
                       BT_GATT_CHARACTERISTIC(&sensor_ctrl_char_uuid.uuid,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                                              BT_GATT_PERM_WRITE, NULL, recv, (void *)1),
                       BT_GATT_CHARACTERISTIC(&sensor_data_notif_uuid.uuid,
                                              BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL,
                                              &rand_val),
                       BT_GATT_CCC(mpu_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

static ssize_t recv(struct bt_conn *conn,
                    const struct bt_gatt_attr *attr, const void *buf,
                    uint16_t len, uint16_t offset, uint8_t flags) {
    LOG_INF("Sensor Control Data Received!");

    return 0;
}

static void bt_ready(int err) {
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }
    LOG_INF("Bluetooth initialized");
    /* Start advertising */
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    LOG_INF("Configuration mode: waiting connections...");
}

static void connected(struct bt_conn *connected, uint8_t err) {
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
    } else {
        LOG_INF("Connected");
        if (!conn) {
            conn = bt_conn_ref(connected);
        }
    }
}

static void disconnected(struct bt_conn *disconn, uint8_t reason) {
    if (conn) {
        bt_conn_unref(conn);
        conn = NULL;
    }

    LOG_INF("Disconnected (reason %u)", reason);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

/**
 * @brief IntWorkQueue handler. Used to process interrupts coming from MAX30102 interrupt pin
 * Because all activity is performed on cooperative level no addition protection against data corruption is required
 * @param wrk work object
 * @warning  Called by system scheduled in cooperative level.
 */
/*
static void interrupt_workQueue_handler(struct k_work *wrk) {
    int err;
    static uint8_t sample_num;
    sample_num++;
    LOG_INF("MAX30102 Interrupt: %d!", sample_num);
    gpio_pin_toggle(gpio_0_dev, DEBUG_LED_2);

    // Read interrupt status registers
    uint8_t status_1, status_2;
    err = max30102_get_interrupt_status_1(dev, &status_1);
    err = max30102_get_interrupt_status_2(dev, &status_2);
    if (err)
    {
        LOG_ERR("Failed to read status registers, error code: %d", err);
    }

    // max30102 is ready for temp reading (DIE_TEMP_RDY)
    if (status_2 == 2) {
        struct sensor_value temp;
        sensor_sample_fetch_chan(dev, SENSOR_CHAN_DIE_TEMP);
        sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp);
        float f_temp = sensor_value_to_double(&temp);
        LOG_INF("TEMP=%f", f_temp);
    }
    // FIFO is almost full (A_FULL)
    else if (status_1 == 128) {
        LOG_INF("FIFO full");
        // Read data from MAX30102
        struct sensor_value red;
        struct sensor_value ir;

        struct Sample_packet packet = {.sample_num = sample_num, .end_bytes = {'\r', '\n'}};
        // struct Sample_packet packet = {.end_bytes = {'\r', '\n'}};
        // struct Sample_packet packet;

        for (int i = 0; i < 30; i++) {
            sensor_sample_fetch(dev);
            sensor_channel_get(dev, SENSOR_CHAN_RED, &red);
            sensor_channel_get(dev, SENSOR_CHAN_IR, &ir);

            // Print red LED data
            LOG_INF("RED=%x IR=%x", red.val1, ir.val1);

            packet.samples[i].red = reverse_32(red.val1);
            packet.samples[i].ir = reverse_32(ir.val1);
        }
        if (conn) {
            if (notify_enable) {
                err = bt_gatt_notify(NULL, &bt832a_svc.attrs[4],
                                     &packet, sizeof(packet));
                if (err) {
                    LOG_ERR("Notify error: %d", err);
                } else {
                    LOG_INF("Send notify ok");
                    rand_val++;
                }
            } else {
                LOG_INF("Notify not enabled");
            }
        } else {
            LOG_INF("BLE not connected");
        }
    }
    // Ambient Light Cancellation Overflow (ALC_OVF)
    else {
        // TODO: deal with corrupted FIFO
    }
}
*/

void main(void) {
    const struct device *max30102_dev = DEVICE_DT_GET_ANY(maxim_max30102);

    if (max30102_dev == NULL) {
        LOG_ERR("Could not get max30102 device\n");
        return;
    }
    if (!device_is_ready(max30102_dev)) {
        LOG_ERR("Device %s is not ready\n", max30102_dev->name);
        return;
    }
    int ret;

#ifdef CONFIG_MAX30102_TRIGGER
    drdy_trigger_mode(max30102_dev);
#endif

    /* BLE part */
    bt_conn_cb_register(&conn_callbacks);

    /* Initialize the Bluetooth Subsystem */
    ret = bt_enable(bt_ready);
    if (ret) {
        LOG_ERR("Bluetooth init failed (err %d)", ret);
    }

    while (1) {
#ifdef CONFIG_MAX30102_TRIGGER
        k_sem_take(&sem, K_FOREVER);
#else
        if (sensor_sample_fetch(dev) < 0) {
            LOG_ERR("Sample fetch failed\n");
            return;
        }
        k_msleep(SLEEP_TIME_MS);
#endif
        struct sensor_value red;
        struct sensor_value ir;
        sensor_channel_get(max30102_dev, SENSOR_CHAN_RED, &red);
        sensor_channel_get(max30102_dev, SENSOR_CHAN_IR, &ir);

        LOG_INF("RED=%x IR=%x", red.val1, ir.val1);
    }
}