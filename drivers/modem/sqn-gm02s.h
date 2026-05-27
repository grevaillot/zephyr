/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SQN_GM02S_H
#define SQN_GM02S_H

#include <zephyr/kernel.h>
#include <ctype.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/init.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/offloaded_netdev.h>
#include <zephyr/net/net_offload.h>
#include <zephyr/net/socket_offload.h>

#include "modem_context.h"
#include "modem_socket.h"
#include "modem_cmd_handler.h"
#include "modem_iface_uart.h"

#define MDM_UART_NODE			DT_INST_BUS(0)
#define MDM_UART_DEV			DEVICE_DT_GET(MDM_UART_NODE)
#define MDM_CMD_TIMEOUT			K_SECONDS(10)
#define MDM_CMD_CONN_TIMEOUT		K_SECONDS(120)
#define MDM_REGISTRATION_TIMEOUT	K_SECONDS(180)
#define MDM_SENDMSG_SLEEP		K_MSEC(1)
#define MDM_MAX_DATA_LENGTH		1500
#define MDM_RECV_MAX_BUF		30
#define MDM_RECV_BUF_SIZE		1500
#define MDM_MAX_SOCKETS			6
#define MDM_BASE_SOCKET_NUM		1 /* Sequans connId range is 1..6 */
#define MDM_NETWORK_RETRY_COUNT		10
#define MDM_INIT_RETRY_COUNT		10
#define MDM_PDP_ACT_RETRY_COUNT		3
#define MDM_WAIT_FOR_RSSI_COUNT		10
#define MDM_WAIT_FOR_RSSI_DELAY		K_SECONDS(2)
#define BUF_ALLOC_TIMEOUT		K_SECONDS(1)
#define MDM_MAX_BOOT_TIME		K_SECONDS(50)

/* Default lengths of certain things. */
#define MDM_MANUFACTURER_LENGTH		10
#define MDM_MODEL_LENGTH		16
#define MDM_REVISION_LENGTH		64
#define MDM_IMEI_LENGTH			16
#define RSSI_TIMEOUT_SECS		30

#define MDM_UNSOL_RDY			CONFIG_MODEM_SQN_GM02S_UNSOL_RDY
#define MDM_APN				CONFIG_MODEM_SQN_GM02S_APN

/* Modem ATOI routine. */
#define ATOI(s_, value_, desc_) modem_atoi(s_, value_, desc_, __func__)

/* pin settings */
enum mdm_control_pins {
	MDM_RESET = 0,
#if DT_INST_NODE_HAS_PROP(0, mdm_power_gpios)
	MDM_POWER,
#endif
};

/* driver data */
struct modem_data {
	struct net_if *net_iface;
	uint8_t mac_addr[6];

	/* modem interface */
	struct modem_iface_uart_data iface_data;
	uint8_t iface_rb_buf[MDM_MAX_DATA_LENGTH];

	/* modem cmds */
	struct modem_cmd_handler_data cmd_handler_data;
	uint8_t cmd_match_buf[MDM_RECV_BUF_SIZE + 1];

	/* socket data */
	struct modem_socket_config socket_config;
	struct modem_socket sockets[MDM_MAX_SOCKETS];

	/* RSSI work */
	struct k_work_delayable rssi_query_work;

	/* modem data */
	char mdm_manufacturer[MDM_MANUFACTURER_LENGTH];
	char mdm_model[MDM_MODEL_LENGTH];
	char mdm_revision[MDM_REVISION_LENGTH];
	char mdm_imei[MDM_IMEI_LENGTH];
	int mdm_rssi;

	/* bytes written to socket in last transaction */
	int sock_written;

	/* Socket from which we are currently reading data. */
	int sock_fd;

	/* Semaphore(s) */
	struct k_sem sem_response;
	struct k_sem sem_tx_ready;
	struct k_sem sem_sock_conn;
};

/* Socket read callback data */
struct socket_read_data {
	char		*recv_buf;
	size_t		recv_buf_len;
	struct sockaddr	*recv_addr;
	uint16_t	recv_read_len;
};

#endif /* SQN_GM02S_H */
