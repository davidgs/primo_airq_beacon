/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include "sysinit/sysinit.h"
#include "bsp/bsp.h"
#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#include "console/console.h"
#include "senseair/senseair.h"
#include "shell/shell.h"

/* BLE */
#include "nimble/ble.h"
#include "nimble/ble_hci_trans.h"
#include "nimble/hci_common.h"
#include "nimble/hci_vendor.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"

#include "bletest_priv.h"

/* Application-specified header. */
#include "bleprph.h"

/** Log data. */
struct log bleprph_log;

/* CO2 Task settings */
#define CO2_TASK_PRIO           5
#define CO2_STACK_SIZE          (OS_STACK_ALIGN(336))
struct os_eventq co2_evq;
struct os_task co2_task;
bssnz_t os_stack_t co2_stack[CO2_STACK_SIZE];

static int bleprph_gap_event(struct ble_gap_event *event, void *arg);

/* A buffer for host advertising data */
uint8_t g_host_adv_data[BLE_HCI_MAX_ADV_DATA_LEN];
uint8_t g_host_adv_len;

/**
 * Logs information about a connection to the console.
 */
static void
bleprph_print_conn_desc(struct ble_gap_conn_desc *desc)
{
    BLEPRPH_LOG(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=",
                desc->conn_handle, desc->our_ota_addr_type);
    print_addr(desc->our_ota_addr);
    BLEPRPH_LOG(INFO, " our_id_addr_type=%d our_id_addr=",
                desc->our_id_addr_type);
    print_addr(desc->our_id_addr);
    BLEPRPH_LOG(INFO, " peer_ota_addr_type=%d peer_ota_addr=",
                desc->peer_ota_addr_type);
    print_addr(desc->peer_ota_addr);
    BLEPRPH_LOG(INFO, " peer_id_addr_type=%d peer_id_addr=",
                desc->peer_id_addr_type);
    print_addr(desc->peer_id_addr);
    BLEPRPH_LOG(INFO, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                "encrypted=%d authenticated=%d bonded=%d\n",
                desc->conn_itvl, desc->conn_latency,
                desc->supervision_timeout,
                desc->sec_state.encrypted,
                desc->sec_state.authenticated,
                desc->sec_state.bonded);
}

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void
bleprph_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);

    /* Indicate that the flags field should be included; specify a value of 0
     * to instruct the stack to fill the value in for us.
     */
    fields.flags_is_present = 1;
    fields.flags = 0;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this one automatically as well.  This is done by assiging the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids16 = (uint16_t[]){ GATT_SVR_SVC_ALERT_UUID };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        BLEPRPH_LOG(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(BLE_ADDR_TYPE_PUBLIC, 0, NULL, BLE_HS_FOREVER,
                      &adv_params, bleprph_gap_event, NULL);
}

uint8_t
bletest_set_adv_data(uint8_t *dptr, uint8_t *addr)
{
    uint8_t len;

    dptr[0] = 0x03;   /* Length of Service List */
    dptr[1] = 0x03;   /* Param: Service List */
    dptr[2] = 0xAA;   /* Eddystone ID */
    dptr[3] = 0xFE;   /* Eddystone ID */
    dptr[4] = 0x10;   /* Length of Service Data */
    dptr[5] = 0x16;   /* Service Data */
    dptr[6] = 0xAA;   /* Eddystone ID */
    dptr[7] = 0xFE;   /* Eddystone ID */
    dptr[8] = 0x10;   /* Frame type: URL */
    dptr[9] = 0x00;   /* Power */
    dptr[10] = 0x03;  /* https:// */
    dptr[11] = 'r';
    dptr[12] = 'u';
    dptr[13] = 'n';
    dptr[14] = 't';
    dptr[15] = 'i';
    dptr[16] = 'm';
    dptr[17] = 'e';
    dptr[18] = '.';
    dptr[19] = 'i';
    dptr[20] = 'o';

    len = 21;
    addr = NULL;

    g_host_adv_len = len;

    return len;
}

void
bletest_init_adv_instances(void)
{
    uint8_t i;
    int rc;
    uint8_t *addr;
    uint8_t adv_len;
    struct hci_multi_adv_params adv;

    /* Start up all the instances */
        i=1;
        memset(&adv, 0, sizeof(struct hci_multi_adv_params));

        adv.own_addr_type = BLE_HCI_ADV_OWN_ADDR_PUBLIC;
        addr = g_dev_addr;

        adv.adv_type = BLE_HCI_ADV_TYPE_ADV_NONCONN_IND;
        adv.adv_channel_map = 0x07;
        adv.adv_filter_policy = BLE_HCI_ADV_FILT_NONE;
        adv.peer_addr_type = BLE_HCI_ADV_PEER_ADDR_PUBLIC;
        adv_len = bletest_set_adv_data(&g_host_adv_data[0], addr);

        adv.adv_itvl_min = 100000 / BLE_HCI_ADV_ITVL;
        adv.adv_itvl_max = 100000 / BLE_HCI_ADV_ITVL;
        adv.adv_tx_pwr = 0;

        /* Set the advertising parameters */
        rc = bletest_hci_le_set_multi_adv_params(&adv, i);
        assert(rc == 0);

        /* Set advertising data */
        if (adv_len != 0) {
            rc = bletest_hci_le_set_multi_adv_data(&g_host_adv_data[0], adv_len,
                                                   i);
            assert(rc == 0);
        }

        /* Set the advertising parameters */
        rc = bletest_hci_le_set_multi_adv_enable(1, i);
        assert(rc == 0);
    
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleprph uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unuesd by
 *                                  bleprph.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        BLEPRPH_LOG(INFO, "connection %s; status=%d ",
                       event->connect.status == 0 ? "established" : "failed",
                       event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            bleprph_print_conn_desc(&desc);
        }
        BLEPRPH_LOG(INFO, "\n");

        /* Start advertising again. */
        bleprph_advertise();
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        BLEPRPH_LOG(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        bleprph_print_conn_desc(&event->disconnect.conn);
        BLEPRPH_LOG(INFO, "\n");

        /* Connection terminated; resume advertising. */
        bleprph_advertise();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        BLEPRPH_LOG(INFO, "connection updated; status=%d ",
                    event->conn_update.status);
        rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        BLEPRPH_LOG(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        BLEPRPH_LOG(INFO, "encryption change event; status=%d ",
                    event->enc_change.status);
        rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        BLEPRPH_LOG(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        BLEPRPH_LOG(INFO, "subscribe event; conn_handle=%d attr_handle=%d "
                          "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                    event->subscribe.conn_handle,
                    event->subscribe.attr_handle,
                    event->subscribe.reason,
                    event->subscribe.prev_notify,
                    event->subscribe.cur_notify,
                    event->subscribe.prev_indicate,
                    event->subscribe.cur_indicate);
        return 0;

    case BLE_GAP_EVENT_MTU:
        BLEPRPH_LOG(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;
    }

    return 0;
}

static void
bleprph_on_reset(int reason)
{
    BLEPRPH_LOG(ERROR, "Resetting state; reason=%d\n", reason);
}

static void
bleprph_on_sync(void)
{
    bletest_init_adv_instances();
    /* Begin advertising. */
    bleprph_advertise();
}

int
co2_read_event(void)
{
    int value;
    enum senseair_read_type type = SENSEAIR_CO2;
    uint16_t chr_val_handle;
    int rc;

    value = senseair_read(type);
    if (value >= 0) {
        console_printf("Got %d\n", value);
    } else {
        console_printf("Error while reading: %d\n", value);
        goto err;
    }
    gatt_co2_val = value;
    rc = ble_gatts_find_chr(&gatt_svr_svc_co2_uuid.u, BLE_UUID16_DECLARE(CO2_SNS_VAL), NULL, &chr_val_handle);
    assert(rc == 0);
    ble_gatts_chr_updated(chr_val_handle);
    return (0);
err:
    return (rc);
}

/**
 * Event loop for the sensor task.
 */
static void
co2_task_handler(void *unused)
{
    while (1) {
        co2_read_event();
        /* Wait 2 second */
        os_time_delay(OS_TICKS_PER_SEC * 2);
    }
}

/**
 * main
 *
 * The main function for the project. This function initializes the os, calls
 * init_tasks to initialize tasks (and possibly other objects), then starts the
 * OS. We should not return from os start.
 *
 * @return int NOTE: this function should never return!
 */
int
main(int argc, char **argv)
{
    int rc;

    /* Set initial BLE device address. */
    memcpy(g_dev_addr, (uint8_t[6]){0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a}, 6);

    /* Initialize OS */
    sysinit();

    /* Initialize co2 sensor task eventq */
    os_eventq_init(&co2_evq);

    /* Senseair init */
    senseair_init(0);

    /* Create the CO2 reader task.  
     * All sensor operations are performed in this task.
     */
    os_task_init(&co2_task, "sensor", co2_task_handler,
            NULL, CO2_TASK_PRIO, OS_WAIT_FOREVER,
            co2_stack, CO2_STACK_SIZE);

    /* Initialize the bleprph log. */
    log_register("bleprph", &bleprph_log, &log_console_handler, NULL,
                 LOG_SYSLEVEL);

    /* Initialize the NimBLE host configuration. */
    log_register("ble_hs", &ble_hs_log, &log_console_handler, NULL,
                 LOG_SYSLEVEL);
    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;

    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("nimble-cleantech");
    assert(rc == 0);

    /*
     * As the last thing, process events from default event queue.
     */
    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }

    return 0;
}
