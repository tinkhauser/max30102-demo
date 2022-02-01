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

#include <errno.h>
#include <logging/log.h>
#include <stddef.h>
#include <string.h>
#include <sys/byteorder.h>
#include <sys/printk.h>

#include <zephyr.h>
#include <zephyr/types.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <max30102.h>

/* 1000 msec = 1 sec */
#define ZEPHYR_OK 0
#define SLEEP_TIME_MS 1000
#define MAX30102_INT_N ((uint8_t)13)  ///< P0.13 pin used for receiving interrupts from MAX30102
#define DEBUG_LED_1 ((uint8_t)17)     ///< P0.17 pin used Debug LED 1
#define DEBUG_LED_2 ((uint8_t)18)     ///< P0.18 pin used Debug LED 2

/* Static Functions */
static int gpio_init(void);
static void max30102_irq_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static void interrupt_workQueue_handler(struct k_work *wrk);
static ssize_t recv(struct bt_conn *conn,
                    const struct bt_gatt_attr *attr, const void *buf,
                    uint16_t len, uint16_t offset, uint8_t flags);

/* Global variables */
const struct device *gpio_0_dev;
struct gpio_callback callback;
struct k_work interrupt_work_item;  ///< interrupt work item
const struct device *dev;

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

void main(void) {
    LOG_ERR("This is a error message!");
    LOG_WRN("This is a warning message!");
    LOG_INF("This is a information message!");
    LOG_DBG("This is a debugging message!");
    int ret;
    bool led_1_is_on = true;

    ret = gpio_init();
    if (ret == ZEPHYR_OK) {
        LOG_INF("GPIOs Int'd!");
    }

    dev = DEVICE_DT_GET_ANY(maxim_max30102);

    if (dev == NULL) {
        LOG_ERR("Could not get max30102 device\n");
        return;
    }
    if (!device_is_ready(dev)) {
        LOG_ERR("max30102 device %s is not ready\n", dev->name);
        return;
    }

    /* BLE part */
    bt_conn_cb_register(&conn_callbacks);

    /* Initialize the Bluetooth Subsystem */
    ret = bt_enable(bt_ready);
    if (ret) {
        LOG_ERR("Bluetooth init failed (err %d)", ret);
    }

    /* Clear any existing interrupts */
    sensor_sample_fetch(dev);

    while (1) {
        gpio_pin_set(gpio_0_dev, DEBUG_LED_1, (int)led_1_is_on);
        led_1_is_on = !led_1_is_on;
        k_msleep(SLEEP_TIME_MS);
    }
}

static int gpio_init(void) {
    int ret = ZEPHYR_OK;

    gpio_0_dev = device_get_binding("GPIO_0");
    if (gpio_0_dev == NULL) {
        LOG_ERR("***ERROR: GPIO device binding!");
        return -1;
    }

    /* LED for debug purposes */
    ret = gpio_pin_configure(gpio_0_dev, DEBUG_LED_1, GPIO_OUTPUT_ACTIVE);
    ret += gpio_pin_configure(gpio_0_dev, DEBUG_LED_2, GPIO_OUTPUT_ACTIVE);
    /* Initialize MAX30102 Interrupt input */
    ret += gpio_pin_configure(gpio_0_dev, MAX30102_INT_N, GPIO_INPUT | GPIO_PULL_UP);
    ret += gpio_pin_interrupt_configure(gpio_0_dev, MAX30102_INT_N, GPIO_INT_EDGE_FALLING);
    gpio_init_callback(&callback, max30102_irq_cb, BIT(MAX30102_INT_N));
    ret += gpio_add_callback(gpio_0_dev, &callback);
    if (ret != 0) {
        LOG_ERR("***ERROR: GPIO initialization\n");
    }
    gpio_pin_set(gpio_0_dev, DEBUG_LED_2, 0);
    k_work_init(&interrupt_work_item, interrupt_workQueue_handler);

    return ret;
}

/**
 * @brief Callback function for handling interrupt coming from the MAX30102 (MAX30102_INT_N pin).
 *
 * @param port Runtime device structure (in ROM) per driver instance.
 * @param cb GPIO callback structure
 * @param pins Identifies a set of pins associated with a port
 * @warning Called at ISR Level, no actual workload should be implemented here
 */
static void max30102_irq_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    k_work_submit(&interrupt_work_item);
}

/**
 * @brief IntWorkQueue handler. Used to process interrupts coming from MAX30102 interrupt pin
 * Because all activity is performed on cooperative level no addition protection against data corruption is required
 * @param wrk work object
 * @warning  Called by system scheduled in cooperative level.
 */
static void interrupt_workQueue_handler(struct k_work *wrk) {
    int err;
    static uint8_t sample_num;
    sample_num++;
    LOG_INF("MAX30102 Interrupt: %d!", sample_num);
    gpio_pin_toggle(gpio_0_dev, DEBUG_LED_2);

    // Read interrupt status registers
    uint8_t status_1 = max30102_get_interrupt_status_1(dev);
    uint8_t status_2 = max30102_get_interrupt_status_2(dev);

    // max30102 is ready for temp reading (DIE_TEMP_RDY)
    if (status_2 == 2) {
        struct max30102_temp temp = {0};
        max30102_get_temp(dev, &temp);
		uint8_t offset = 0;
		//max30102_get_temp_offset(dev, offset)
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
