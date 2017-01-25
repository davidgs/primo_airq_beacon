/**
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
#include <string.h>

#include <shell/shell.h>
#include <console/console.h>
#include <os/os.h>

#include <hal/hal_uart.h>

#include "senseair/senseair.h"

static const uint8_t cmd_read_co2[] = {
    0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25
};

static int senseair_shell_func(int argc, char **argv);
static struct shell_cmd senseair_cmd = {
    .sc_cmd = "senseair",
    .sc_cmd_func = senseair_shell_func,
};

struct senseair { 
    int uart;
    struct os_sem sema;
    const uint8_t *tx_data;
    int tx_off;
    int tx_len;
    uint8_t rx_data[32]; 
    int rx_off;
    int value;
} senseair;

static int
senseair_tx_char(void *arg)
{
    struct senseair *s = &senseair;
    int rc;

    if (s->tx_off >= s->tx_len) {
        /*
         * Command tx finished.
         */
        s->tx_data = NULL;
        return -1;
    }

    rc = s->tx_data[s->tx_off];
    s->tx_off++;
    return rc;
}

/*
 * CRC for modbus over serial port.
 */
static const uint16_t mb_crc_tbl[] = {
    0x0000, 0xcc01, 0xd801, 0x1400, 0xf001, 0x3c00, 0x2800, 0xe401,
    0xa001, 0x6c00, 0x7800, 0xb401, 0x5000, 0x9c01, 0x8801, 0x4400
};

static uint16_t
mb_crc(const uint8_t *data, int len, uint16_t crc)
{
    while (len-- > 0) {
        crc ^= *data++;
        crc = (crc >> 4) ^ mb_crc_tbl[crc & 0xf];
        crc = (crc >> 4) ^ mb_crc_tbl[crc & 0xf];
    }
    return crc;
}

static int
mb_crc_check(const void *pkt, int len)
{
    uint16_t crc, cmp;
    uint8_t *bp = (uint8_t *)pkt;

    if (len < sizeof(crc) + 1) {
        return -1;
    }
    crc = mb_crc(pkt, len - 2, 0xffff);
    cmp = bp[len - 2] | (bp[len - 1] << 8);
    if (crc != cmp) {
        return -1;
    } else {
        return 0;
    }
}

static int
senseair_rx_char(void *arg, uint8_t data)
{
    struct senseair *s = (struct senseair *)arg;
    int rc;

    if (s->rx_off >= sizeof(s->rx_data)) {
        s->rx_off = 0;
    }
    s->rx_data[s->rx_off] = data;
    s->rx_off++;

    if (s->rx_off == 7) {
        rc = mb_crc_check(s->rx_data, s->rx_off);
        if (rc == 0) {
            s->value = s->rx_data[3] * 256 + s->rx_data[4];
            os_sem_release(&s->sema);
        }
    }
    return 0;
}

void
senseair_tx(struct senseair *s, const uint8_t *tx_data, int data_len)
{
    s->tx_data = tx_data;
    s->tx_len = data_len;
    s->tx_off = 0;
    s->rx_off = 0;

    hal_uart_start_tx(s->uart);
}

int
senseair_read(enum senseair_read_type type)
{
    struct senseair *s = &senseair;
    const uint8_t *cmd;
    int cmd_len;
    int rc;

    if (s->tx_data) {
        /*
         * busy
         */
        return -1;
    }
    switch (type) {
    case SENSEAIR_CO2:
        cmd = cmd_read_co2;
        cmd_len = sizeof(cmd_read_co2);
        break;
    default:
        return -1;
    }
    senseair_tx(s, cmd, cmd_len);
    rc = os_sem_pend(&s->sema, OS_TICKS_PER_SEC / 2);
    if (rc == OS_TIMEOUT) {
        /*
         * timeout
         */
        return -2;
    }
    return s->value;
}

static int
senseair_shell_func(int argc, char **argv)
{
    int value;
    enum senseair_read_type type;

    if (argc < 2) {
usage:
        console_printf("%s co2\n", argv[0]);
        return 0;
    }
    if (!strcmp(argv[1], "co2")) {
        type = SENSEAIR_CO2;
    } else {
        goto usage;
    }
    value = senseair_read(type);
    if (value >= 0) {
        console_printf("Got %d\n", value);
    } else {
        console_printf("Error while reading: %d\n", value);
    }
    return 0;
}

int
senseair_init(int uartno)
{
    int rc;
    struct senseair *s = &senseair;

    rc = shell_cmd_register(&senseair_cmd);
    if (rc) {
        return rc;
    }

    rc = os_sem_init(&s->sema, 1);
    if (rc) {
        return rc;
    }
    rc = hal_uart_init_cbs(uartno, senseair_tx_char, NULL,
      senseair_rx_char, &senseair);
    if (rc) {
        return rc;
    }
    rc = hal_uart_config(uartno, 9600, 8, 1, HAL_UART_PARITY_NONE,
      HAL_UART_FLOW_CTL_NONE);
    if (rc) {
        return rc;
    }
    s->uart = uartno;

    return 0;
}
