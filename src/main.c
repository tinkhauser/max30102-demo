/*
 * Copyright (c) 2021 EVERGREEN FUND 501(c)(3)
 * Copyright (c) 2022 Jacob Tinkhauser
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
#include <stdio.h>
#include <sys/byteorder.h>
#include <sys/printk.h>
#include <zephyr.h>
#include <zephyr/types.h>

#include <pb_encode.h>
#include <pb_decode.h>
#include "simple.pb.c" /* TODO: why's does this require *.c and not *.h? */




/* Static Functions */
static ssize_t recv(struct bt_conn *conn,
                    const struct bt_gatt_attr *attr, const void *buf,
                    uint16_t len, uint16_t offset, uint8_t flags);

struct Led_sample {
    uint32_t red;
    uint32_t ir;
};

struct Sample_packet {
    struct Led_sample samples[30];
    uint8_t sample_num;
};

/* For Logging */
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

K_SEM_DEFINE(sem, 0, 1);

/* Nanopb */
bool encode_message(struct Sample_packet *packet, uint8_t *buffer, size_t buffer_size, size_t *message_length) {
    bool status;

    /* Allocate space on the stack to store the message data.
     *
     * It is a good idea to always initialize your structures
     * so that you do not have garbage data from RAM in there.
     */
    DataFrame dataframe_msg = DataFrame_init_zero;

    /* Create a stream that will write to our buffer. */
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);

    for (int i = 0; i < 24; i++) {
        dataframe_msg.ppgs[i].Red = packet->samples[i].red;
        dataframe_msg.ppgs[i].IR = packet->samples[i].ir;
    }

    /* Now we are ready to encode the message! */
    status = pb_encode(&stream, DataFrame_fields, &dataframe_msg);
    *message_length = stream.bytes_written;

    if (!status) {
        printk("Encoding failed: %s\n", PB_GET_ERROR(&stream));
    }

    return status;
}

bool decode_message(uint8_t *buffer, size_t message_length) {
    bool status;

    /* Allocate space for the decoded message. */
    DataFrame message = DataFrame_init_zero;

    /* Create a stream that reads from the buffer. */
    pb_istream_t stream = pb_istream_from_buffer(buffer, message_length);

    /* Now we are ready to decode the message. */
    status = pb_decode(&stream, DataFrame_fields, &message);

    /* Check for errors... */
    if (status) {
        /* Print the data contained in the message. */
        printk("Buffer contains: ");
        printk("Red was %d!\n", (int)message.ppgs[0].Red);
        printk("IR was %d!\n", (int)message.ppgs[0].IR);
        printk("Message length %d\n", message_length);
    } else {
        printk("Decoding failed: %s\n", PB_GET_ERROR(&stream));
    }

    return status;
}

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

#ifdef CONFIG_MAX30102_TRIGGER

/* Handles interrupts */
static void trigger_handler(const struct device *max30102,
                            const struct sensor_trigger *trigger) {
    /* switch (trigger->type) {
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
    */
    static uint8_t sample_num;
    sample_num++;

    struct Sample_packet packet = {.sample_num = sample_num};
    for (int i = 0; i < 30; i++) {
        if (sensor_sample_fetch(max30102)) {
            LOG_ERR("Sample fetch error\n");
            return;
        }
        LOG_INF("Fetched sample\n");
        struct sensor_value red;
        struct sensor_value ir;
        sensor_channel_get(max30102, SENSOR_CHAN_RED, &red);
        sensor_channel_get(max30102, SENSOR_CHAN_IR, &ir);

        packet.samples[i].red = red.val1;
        packet.samples[i].ir = ir.val1;

        // LOG_INF("RED=%x IR=%x", red.val1, ir.val1);
    }

    /* This is the buffer where we will store our message. */
    uint8_t buffer[DataFrame_size];
    size_t message_length;

    /* Encode our message */
    if (!encode_message(&packet, buffer, sizeof(buffer), &message_length)) {
        return;
    }

    //thread_analyzer_print();
    //decode_message(buffer, message_length);

    if (conn) {
        if (notify_enable) {
            /* Since protobuf uses var ints, the message length will change based on input data */
            int err = bt_gatt_notify(NULL, &bt832a_svc.attrs[4], &buffer, message_length);
            if (err) {
                LOG_ERR("Notify error: %d", err);
            } else {
                LOG_INF("Send notify ok");
            }
        } else {
            LOG_INF("Notify not enabled");
        }
    } else {
        LOG_INF("BLE not connected");
    }

    k_sem_give(&sem);
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

static const char *now_str(void)
{
    static char buf[16]; /* ...HH:MM:SS.MMM */
    uint32_t now = k_uptime_get_32();
    unsigned int ms = now % MSEC_PER_SEC;
    unsigned int s;
    unsigned int min;
    unsigned int h;

    now /= MSEC_PER_SEC;
    s = now % 60U;
        now /= 60U;
        min = now % 60U;
        now /= 60U;
        h = now;

        snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u",
                 h, min, s, ms);
        return buf;
}

static int process_mpu6050(const struct device *dev) {
    struct sensor_value temperature;
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    int rc = sensor_sample_fetch(dev);

    if (rc == 0) {
        rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ,
                                accel);
    }
    if (rc == 0) {
        rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,
                                gyro);
    }
    if (rc == 0) {
        rc = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP,
                                &temperature);
    }
    if (rc == 0) {
        LOG_INF(
            "[%s]:%g Cel\n"
            "accel %f %f %f m/s/s\n"
            "gyro  %f %f %f rad/s\n",
            now_str(),
            sensor_value_to_double(&temperature),
            sensor_value_to_double(&accel[0]),
            sensor_value_to_double(&accel[1]),
            sensor_value_to_double(&accel[2]),
            sensor_value_to_double(&gyro[0]),
            sensor_value_to_double(&gyro[1]),
            sensor_value_to_double(&gyro[2]));
    } else {
        LOG_ERR("sample fetch/get failed: %d\n", rc);
    }

    return rc;
}

#ifdef CONFIG_MPU6050_TRIGGER
static struct sensor_trigger trigger;

static void handle_mpu6050_drdy(const struct device *dev,
                const struct sensor_trigger *trig)
{
    int rc = process_mpu6050(dev);

    if (rc != 0) {
        //printf("cancelling trigger due to failure: %d\n", rc);
        //(void)sensor_trigger_set(dev, trig, NULL);
        return;
    }
}
#endif /* CONFIG_MPU6050_TRIGGER */

void main(void) {
    /* MPU 6050 */
#ifdef CONFIG_MPU6050
    const struct device *mpu6050 = DEVICE_DT_GET_ANY(invensense_mpu6050);

    if (!mpu6050) {
        LOG_ERR("Failed to find sensor invesense_mpu6050\n");
        return;
    }
#endif

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

#ifdef CONFIG_MPU6050_TRIGGER
    trigger = (struct sensor_trigger){
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ALL,
    };
    if (sensor_trigger_set(mpu6050, &trigger,
                           handle_mpu6050_drdy) < 0) {
        LOG_ERR("Cannot configure trigger\n");
        return;
    }
    LOG_INF("Configured for triggered sampling.\n");
#endif

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

#ifdef CONFIG_MPU6050
    while (!IS_ENABLED(CONFIG_MPU6050_TRIGGER)) {
        int rc = process_mpu6050(mpu6050);

        if (rc != 0) {
            break;
        }
        k_sleep(K_SECONDS(2));
    }
#endif

    k_sem_take(&sem, K_FOREVER);
}