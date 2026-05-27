/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Socket offload driver for the Sequans GM02S (Monarch 2) LTE-M modem.
 * TCP only. Uses AT+SQNSD/SQNSRECV/SQNSSENDEXT for direct socket management.
 */

#define DT_DRV_COMPAT sqn_gm02s

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_sqn_gm02s, CONFIG_MODEM_LOG_LEVEL);

#include "sqn-gm02s.h"

static struct k_thread modem_rx_thread;
static struct k_work_q modem_workq;
static struct modem_data mdata;
static struct modem_context mctx;
static const struct socket_op_vtable offload_socket_fd_op_vtable;

static K_KERNEL_STACK_DEFINE(modem_rx_stack,
			     CONFIG_MODEM_SQN_GM02S_RX_STACK_SIZE);
static K_KERNEL_STACK_DEFINE(modem_workq_stack,
			     CONFIG_MODEM_SQN_GM02S_RX_WORKQ_STACK_SIZE);
NET_BUF_POOL_DEFINE(mdm_recv_pool, MDM_RECV_MAX_BUF, MDM_RECV_BUF_SIZE, 0,
		    NULL);

/* GPIO specs */
static const struct gpio_dt_spec reset_gpio =
	GPIO_DT_SPEC_INST_GET(0, mdm_reset_gpios);
#if DT_INST_NODE_HAS_PROP(0, mdm_power_gpios)
static const struct gpio_dt_spec power_gpio =
	GPIO_DT_SPEC_INST_GET(0, mdm_power_gpios);
#endif

/* ========================================================================
 * Helpers
 * ======================================================================== */

static inline uint32_t hash32(char *str, int len)
{
#define HASH_MULTIPLIER 37

	uint32_t h = 0;
	int i;

	for (i = 0; i < len; ++i) {
		h = (h * HASH_MULTIPLIER) + str[i];
	}

	return h;
}

static inline uint8_t *modem_get_mac(const struct device *dev)
{
	struct modem_data *data = dev->data;
	uint32_t hash_value;

	data->mac_addr[0] = 0x00;
	data->mac_addr[1] = 0x10;

	/* use IMEI for mac_addr */
	hash_value = hash32(mdata.mdm_imei, strlen(mdata.mdm_imei));

	UNALIGNED_PUT(hash_value, (uint32_t *)(data->mac_addr + 2));

	return data->mac_addr;
}

/* Convert string to integer with error handling */
static int modem_atoi(const char *s, const int err_value, const char *desc,
		      const char *func)
{
	int ret;
	char *endptr;

	ret = (int)strtol(s, &endptr, 10);
	if (!endptr || *endptr != '\0') {
		LOG_ERR("bad %s '%s' in %s", desc, s, func);
		return err_value;
	}

	return ret;
}

static inline int digits(int n)
{
	int count = 0;

	if (n == 0) {
		return 1;
	}

	while (n != 0) {
		n /= 10;
		++count;
	}

	return count;
}

static inline int find_len(char *data)
{
	char buf[10] = {0};
	int i;

	for (i = 0; i < 10; i++) {
		if (data[i] == '\r') {
			break;
		}

		buf[i] = data[i];
	}

	return ATOI(buf, 0, "rx_buf");
}

/* ========================================================================
 * AT response handlers
 * ======================================================================== */

/* Handler: OK */
MODEM_CMD_DEFINE(on_cmd_ok)
{
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: ERROR */
MODEM_CMD_DEFINE(on_cmd_error)
{
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: +CME ERROR: <err>[0] */
MODEM_CMD_DEFINE(on_cmd_exterror)
{
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: +CSQ: <signal_power>[0], <qual>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_rssi_csq)
{
	int rssi = ATOI(argv[0], 0, "signal_power");

	if (rssi == 31) {
		mdata.mdm_rssi = -51;
	} else if (rssi >= 0 && rssi <= 31) {
		mdata.mdm_rssi = -114 + ((rssi * 2) + 1);
	} else {
		mdata.mdm_rssi = -1000;
	}

	LOG_INF("RSSI: %d", mdata.mdm_rssi);
	return 0;
}

/* Handler: <manufacturer> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_manufacturer)
{
	size_t out_len =
		net_buf_linearize(mdata.mdm_manufacturer,
				  sizeof(mdata.mdm_manufacturer) - 1,
				  data->rx_buf, 0, len);
	mdata.mdm_manufacturer[out_len] = '\0';
	LOG_INF("Manufacturer: %s", mdata.mdm_manufacturer);
	return 0;
}

/* Handler: <model> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_model)
{
	size_t out_len = net_buf_linearize(
		mdata.mdm_model, sizeof(mdata.mdm_model) - 1, data->rx_buf, 0,
		len);
	mdata.mdm_model[out_len] = '\0';
	LOG_INF("Model: %s", mdata.mdm_model);
	return 0;
}

/* Handler: <rev> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_revision)
{
	size_t out_len = net_buf_linearize(mdata.mdm_revision,
					   sizeof(mdata.mdm_revision) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_revision[out_len] = '\0';
	LOG_INF("Revision: %s", mdata.mdm_revision);
	return 0;
}

/* Handler: <IMEI> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_imei)
{
	size_t out_len =
		net_buf_linearize(mdata.mdm_imei, sizeof(mdata.mdm_imei) - 1,
				  data->rx_buf, 0, len);
	mdata.mdm_imei[out_len] = '\0';
	LOG_INF("IMEI: %s", mdata.mdm_imei);
	return 0;
}

/* Handler: TX Ready (the ">" prompt) */
MODEM_CMD_DIRECT_DEFINE(on_cmd_tx_ready)
{
	k_sem_give(&mdata.sem_tx_ready);
	return len;
}

/* Handler: SEND OK */
MODEM_CMD_DEFINE(on_cmd_send_ok)
{
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: Read data from socket.
 * Response format: +SQNSRECV: <connId>,<len>\r\n<data>
 */
static int on_cmd_sockread_common(int socket_fd,
				  struct modem_cmd_handler_data *data,
				  uint16_t len)
{
	struct modem_socket *sock = NULL;
	struct socket_read_data *sock_data;
	int ret, i;
	int socket_data_length;
	int bytes_to_skip;

	if (!len) {
		LOG_ERR("Invalid length, Aborting!");
		return -EAGAIN;
	}

	/* Make sure we still have buf data */
	if (!data->rx_buf) {
		LOG_ERR("Incorrect format! Ignoring data!");
		return -EINVAL;
	}

	socket_data_length = find_len(data->rx_buf->data);

	/* digits of length + \r\n + \r\nOK\r\n (4 bytes for trailing) */
	bytes_to_skip = digits(socket_data_length) + 2 + 4;
	if (socket_data_length <= 0) {
		LOG_ERR("Length problem (%d). Aborting!", socket_data_length);
		return -EAGAIN;
	}

	/* check to make sure we have all of the data. */
	if (net_buf_frags_len(data->rx_buf) <
	    (socket_data_length + bytes_to_skip)) {
		LOG_DBG("Not enough data -- wait!");
		return -EAGAIN;
	}

	/* Skip "len" and CRLF */
	bytes_to_skip = digits(socket_data_length) + 2;
	for (i = 0; i < bytes_to_skip; i++) {
		net_buf_pull_u8(data->rx_buf);
	}

	if (!data->rx_buf->len) {
		data->rx_buf = net_buf_frag_del(NULL, data->rx_buf);
	}

	sock = modem_socket_from_fd(&mdata.socket_config, socket_fd);
	if (!sock) {
		LOG_ERR("Socket not found! (%d)", socket_fd);
		ret = -EINVAL;
		goto exit;
	}

	sock_data = (struct socket_read_data *)sock->data;
	if (!sock_data) {
		LOG_ERR("Socket data not found! Skip handling (%d)",
			socket_fd);
		ret = -EINVAL;
		goto exit;
	}

	ret = net_buf_linearize(sock_data->recv_buf, sock_data->recv_buf_len,
				data->rx_buf, 0,
				(uint16_t)socket_data_length);
	data->rx_buf = net_buf_skip(data->rx_buf, ret);
	sock_data->recv_read_len = ret;
	if (ret != socket_data_length) {
		LOG_ERR("Total copied data is different then received data!"
			" copied:%d vs. received:%d",
			ret, socket_data_length);
		ret = -EINVAL;
	}

exit:
	/* remove packet from list (ignore errors) */
	(void)modem_socket_packet_size_update(&mdata.socket_config, sock,
					      -socket_data_length);

	return ret;
}

/* Handler: +SQNSRECV: <connId>,<len>\r\n<data> */
MODEM_CMD_DEFINE(on_cmd_sock_readdata)
{
	return on_cmd_sockread_common(mdata.sock_fd, data, len);
}

/* ========================================================================
 * URC handlers (unsolicited)
 * ======================================================================== */

/* Handler: +SQNSRING: <connId>[,<recData>]
 * Data receive indication. In srMode=0, only connId is given.
 * In srMode=1 (data amount mode), connId and recData are given.
 * We configure srMode=0 so we only get connId.
 */
MODEM_CMD_DEFINE(on_cmd_unsol_ring)
{
	struct modem_socket *sock;
	int sock_fd;

	sock_fd = ATOI(argv[0], 0, "connId");

	sock = modem_socket_from_id(&mdata.socket_config, sock_fd);
	if (!sock) {
		LOG_WRN("+SQNSRING: unknown connId %d", sock_fd);
		return 0;
	}

	LOG_DBG("Data Receive Indication for socket: %d (fd=%d)", sock_fd,
		sock->sock_fd);
	modem_socket_data_ready(&mdata.socket_config, sock);

	return 0;
}

/* Handler: +SQNSH: <connId> — Socket closed by remote or timeout */
MODEM_CMD_DEFINE(on_cmd_unsol_close)
{
	struct modem_socket *sock;
	int conn_id;

	conn_id = ATOI(argv[0], 0, "connId");
	sock = modem_socket_from_id(&mdata.socket_config, conn_id);
	if (!sock) {
		return 0;
	}

	LOG_INF("Socket Close Indication for connId: %d", conn_id);
	sock->is_connected = false;
	modem_socket_put(&mdata.socket_config, sock->sock_fd);

	return 0;
}

/* Handler: Modem initialization ready. */
MODEM_CMD_DEFINE(on_cmd_unsol_rdy)
{
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* ========================================================================
 * Socket close
 * ======================================================================== */

static void socket_close(struct modem_socket *sock)
{
	char buf[sizeof("AT+SQNSH=#")] = {0};
	int ret;

	snprintk(buf, sizeof(buf), "AT+SQNSH=%d", sock->id);

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, buf,
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", buf, ret);
	}

	modem_socket_put(&mdata.socket_config, sock->sock_fd);
}

/* ========================================================================
 * Socket operations
 * ======================================================================== */

/*
 * Send binary data over the socket.
 *
 * Sequans AT+SQNSSENDEXT=<connId>,<bytesToSend>
 * -> Modem responds with "> " prompt
 * -> Write exactly <bytesToSend> bytes (NO CTRL+Z needed)
 * -> Modem sends OK or ERROR
 */
static ssize_t send_socket_data(struct modem_socket *sock,
				const struct sockaddr *dst_addr,
				struct modem_cmd *handler_cmds,
				size_t handler_cmds_len, const char *buf,
				size_t buf_len, k_timeout_t timeout)
{
	int ret;
	char send_buf[sizeof("AT+SQNSSENDEXT=#,####")] = {0};

	if (buf_len > MDM_MAX_DATA_LENGTH) {
		buf_len = MDM_MAX_DATA_LENGTH;
	}

	mdata.sock_written = buf_len;
	snprintk(send_buf, sizeof(send_buf), "AT+SQNSSENDEXT=%d,%ld", sock->id,
		 (long)buf_len);

	/* Lock TX */
	(void)k_sem_take(&mdata.cmd_handler_data.sem_tx_lock, K_FOREVER);
	k_sem_reset(&mdata.sem_tx_ready);

	/* Send the AT command */
	ret = modem_cmd_send_nolock(&mctx.iface, &mctx.cmd_handler, NULL, 0U,
				    send_buf, NULL, K_NO_WAIT);
	if (ret < 0) {
		goto exit;
	}

	/* set command handlers */
	ret = modem_cmd_handler_update_cmds(&mdata.cmd_handler_data,
					    handler_cmds, handler_cmds_len,
					    true);
	if (ret < 0) {
		goto exit;
	}

	/* Wait for '>' prompt */
	ret = k_sem_take(&mdata.sem_tx_ready, K_MSEC(5000));
	if (ret < 0) {
		LOG_DBG("Timeout waiting for tx prompt");
		goto exit;
	}

	/* Write data — Sequans counts exactly <bytesToSend> bytes, no CTRL+Z */
	mctx.iface.write(&mctx.iface, buf, buf_len);

	/* Wait for 'OK' or 'ERROR' */
	k_sem_reset(&mdata.sem_response);
	ret = k_sem_take(&mdata.sem_response, timeout);
	if (ret < 0) {
		LOG_DBG("No send response");
		goto exit;
	}

	ret = modem_cmd_handler_get_error(&mdata.cmd_handler_data);
	if (ret != 0) {
		LOG_DBG("Failed to send data");
	}

exit:
	(void)modem_cmd_handler_update_cmds(&mdata.cmd_handler_data, NULL, 0U,
					    false);
	k_sem_give(&mdata.cmd_handler_data.sem_tx_lock);

	if (ret < 0) {
		return ret;
	}

	return mdata.sock_written;
}

static ssize_t offload_sendto(void *obj, const void *buf, size_t len,
			      int flags, const struct sockaddr *to,
			      socklen_t tolen)
{
	int ret;
	struct modem_socket *sock = (struct modem_socket *)obj;

	struct modem_cmd cmd[] = {
		MODEM_CMD_DIRECT(">", on_cmd_tx_ready),
		MODEM_CMD("OK", on_cmd_send_ok, 0, ""),
	};

	if (!buf || len == 0) {
		errno = EINVAL;
		return -1;
	}

	/* UDP is not supported. */
	if (sock->ip_proto == IPPROTO_UDP) {
		errno = ENOTSUP;
		return -1;
	}

	if (!sock->is_connected) {
		errno = ENOTCONN;
		return -1;
	}

	ret = send_socket_data(sock, to, cmd, ARRAY_SIZE(cmd), buf, len,
			       MDM_CMD_TIMEOUT);
	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	errno = 0;
	return ret;
}

static ssize_t offload_recvfrom(void *obj, void *buf, size_t len, int flags,
				struct sockaddr *from, socklen_t *fromlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	char sendbuf[sizeof("AT+SQNSRECV=#,####")] = {0};
	int ret;
	struct socket_read_data sock_data;

	/* +SQNSRECV: <connId>,<len>\r\n<data> */
	struct modem_cmd data_cmd[] = {
		MODEM_CMD("+SQNSRECV: ", on_cmd_sock_readdata, 0U, "")};

	if (!buf || len == 0) {
		errno = EINVAL;
		return -1;
	}

	if (flags & ZSOCK_MSG_PEEK) {
		errno = ENOTSUP;
		return -1;
	}

	/* Clamp to max 1500 as per Sequans spec */
	if (len > 1500) {
		len = 1500;
	}

	snprintk(sendbuf, sizeof(sendbuf), "AT+SQNSRECV=%d,%zd", sock->id,
		 len);

	(void)memset(&sock_data, 0, sizeof(sock_data));
	sock_data.recv_buf = buf;
	sock_data.recv_buf_len = len;
	sock_data.recv_addr = from;
	sock->data = &sock_data;
	mdata.sock_fd = sock->sock_fd;

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd,
			     ARRAY_SIZE(data_cmd), sendbuf,
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		errno = -ret;
		ret = -1;
		goto exit;
	}

	/* HACK: use dst address as from */
	if (from && fromlen) {
		*fromlen = sizeof(sock->dst);
		memcpy(from, &sock->dst, *fromlen);
	}

	errno = 0;
	ret = sock_data.recv_read_len;

exit:
	sock->data = NULL;
	return ret;
}

static ssize_t offload_read(void *obj, void *buffer, size_t count)
{
	return offload_recvfrom(obj, buffer, count, 0, NULL, 0);
}

static ssize_t offload_write(void *obj, const void *buffer, size_t count)
{
	return offload_sendto(obj, buffer, count, 0, NULL, 0);
}

static int offload_ioctl(void *obj, unsigned int request, va_list args)
{
	switch (request) {
	case ZFD_IOCTL_POLL_PREPARE: {
		struct zsock_pollfd *pfd;
		struct k_poll_event **pev;
		struct k_poll_event *pev_end;

		pfd = va_arg(args, struct zsock_pollfd *);
		pev = va_arg(args, struct k_poll_event **);
		pev_end = va_arg(args, struct k_poll_event *);

		return modem_socket_poll_prepare(&mdata.socket_config, obj, pfd,
						 pev, pev_end);
	}
	case ZFD_IOCTL_POLL_UPDATE: {
		struct zsock_pollfd *pfd;
		struct k_poll_event **pev;

		pfd = va_arg(args, struct zsock_pollfd *);
		pev = va_arg(args, struct k_poll_event **);

		return modem_socket_poll_update(obj, pfd, pev);
	}

	default:
		errno = EINVAL;
		return -1;
	}
}

/*
 * Connect to remote TCP host.
 *
 * AT+SQNSD=<connId>,0,<port>,"<ip>",0,0,1
 *   0     = TCP (txProt)
 *   port  = remote port
 *   ip    = remote IP address
 *   0     = closureType (close on timeout)
 *   0     = local port (auto)
 *   1     = connMode (command mode, not online)
 *
 * Response: OK or ERROR/NO CARRIER/+CME ERROR
 */
static int offload_connect(void *obj, const struct sockaddr *addr,
			   socklen_t addrlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	uint16_t dst_port = 0;
	char buf[sizeof("AT+SQNSD=#,0,#####,\""
			"####.####.####.####.####.####.####.####"
			"\",0,0,1")] = {0};
	int ret;
	char ip_str[NET_IPV6_ADDR_LEN];

	/* Verify socket has been allocated */
	if (modem_socket_is_allocated(&mdata.socket_config, sock) == false) {
		LOG_ERR("Invalid socket_id(%d) from fd:%d", sock->id,
			sock->sock_fd);
		errno = EINVAL;
		return -1;
	}

	if (sock->is_connected == true) {
		LOG_ERR("Socket is already connected!! socket_id(%d), "
			"socket_fd:%d",
			sock->id, sock->sock_fd);
		errno = EISCONN;
		return -1;
	}

	/* Find the correct destination port. */
	if (addr->sa_family == AF_INET6) {
		dst_port = ntohs(net_sin6(addr)->sin6_port);
	} else if (addr->sa_family == AF_INET) {
		dst_port = ntohs(net_sin(addr)->sin_port);
	}

	/* UDP is not supported. */
	if (sock->ip_proto == IPPROTO_UDP) {
		errno = ENOTSUP;
		return -1;
	}

	ret = modem_context_sprint_ip_addr(addr, ip_str, sizeof(ip_str));
	if (ret != 0) {
		LOG_ERR("Error formatting IP string %d", ret);
		socket_close(sock);
		errno = -ret;
		return -1;
	}

	/* AT+SQNSD=<connId>,0,<port>,"<ip>",0,0,1
	 * connId  = socket id (1-6)
	 * 0       = TCP
	 * port    = remote port
	 * ip      = remote IP
	 * 0       = closureType
	 * 0       = local port (auto)
	 * 1       = command mode
	 */
	snprintk(buf, sizeof(buf), "AT+SQNSD=%d,0,%u,\"%s\",0,0,1", sock->id,
		 dst_port, ip_str);

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, buf,
			     &mdata.sem_response, MDM_CMD_CONN_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", buf, ret);
		socket_close(sock);
		errno = -ret;
		return -1;
	}

	/* Connected successfully. */
	sock->is_connected = true;
	errno = 0;
	return 0;
}

static int offload_close(void *obj)
{
	struct modem_socket *sock = (struct modem_socket *)obj;

	/* Make sure socket is allocated */
	if (modem_socket_is_allocated(&mdata.socket_config, sock) == false) {
		return 0;
	}

	/* Close the socket only if it is connected. */
	if (sock->is_connected) {
		socket_close(sock);
	}

	return 0;
}

static ssize_t offload_sendmsg(void *obj, const struct msghdr *msg, int flags)
{
	ssize_t sent = 0;
	int rc;

	LOG_DBG("msg_iovlen:%zd flags:%d", msg->msg_iovlen, flags);

	for (int i = 0; i < msg->msg_iovlen; i++) {
		const char *buf = msg->msg_iov[i].iov_base;
		size_t len = msg->msg_iov[i].iov_len;

		while (len > 0) {
			rc = offload_sendto(obj, buf, len, flags,
					    msg->msg_name, msg->msg_namelen);
			if (rc < 0) {
				if (rc == -EAGAIN) {
					k_sleep(MDM_SENDMSG_SLEEP);
				} else {
					sent = rc;
					break;
				}
			} else {
				sent += rc;
				buf += rc;
				len -= rc;
			}
		}
	}

	return (ssize_t)sent;
}

/* ========================================================================
 * RX thread
 * ======================================================================== */

static void modem_rx(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		modem_iface_uart_rx_wait(&mctx.iface, K_FOREVER);
		modem_cmd_handler_process(&mctx.cmd_handler, &mctx.iface);
	}
}

/* ========================================================================
 * RSSI query work
 * ======================================================================== */

static void modem_rssi_query_work(struct k_work *work)
{
	struct modem_cmd cmd =
		MODEM_CMD("+CSQ: ", on_cmd_atcmdinfo_rssi_csq, 2U, ",");
	static char *send_cmd = "AT+CSQ";
	int ret;

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, &cmd, 1U,
			     send_cmd, &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("AT+CSQ ret:%d", ret);
	}

	/* Re-start RSSI query work */
	if (work) {
		k_work_reschedule_for_queue(&modem_workq,
					    &mdata.rssi_query_work,
					    K_SECONDS(RSSI_TIMEOUT_SECS));
	}
}

/* ========================================================================
 * VTable and network interface
 * ======================================================================== */

static const struct socket_op_vtable offload_socket_fd_op_vtable = {
	.fd_vtable =
		{
			.read = offload_read,
			.write = offload_write,
			.close = offload_close,
			.ioctl = offload_ioctl,
		},
	.bind = NULL,
	.connect = offload_connect,
	.sendto = offload_sendto,
	.recvfrom = offload_recvfrom,
	.listen = NULL,
	.accept = NULL,
	.sendmsg = offload_sendmsg,
	.getsockopt = NULL,
	.setsockopt = NULL,
};

static int offload_socket(int family, int type, int proto);

static void modem_net_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct modem_data *data = dev->data;

	net_if_set_link_addr(iface, modem_get_mac(dev), sizeof(data->mac_addr),
			     NET_LINK_ETHERNET);
	data->net_iface = iface;

	net_if_socket_offload_set(iface, offload_socket);
}

static struct offloaded_if_api api_funcs = {
	.iface_api.init = modem_net_iface_init,
};

static bool offload_is_supported(int family, int type, int proto)
{
	if (family != AF_INET && family != AF_INET6) {
		return false;
	}

	if (type != SOCK_STREAM) {
		return false;
	}

	if (proto != IPPROTO_TCP) {
		return false;
	}

	return true;
}

static int offload_socket(int family, int type, int proto)
{
	int ret;

	ret = modem_socket_get(&mdata.socket_config, family, type, proto);
	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	errno = 0;
	return ret;
}

/* ========================================================================
 * Pin init (modem reset)
 * ======================================================================== */

static void pin_init(void)
{
	LOG_INF("Setting Modem Pins");

	/* Assert reset */
	gpio_pin_set_dt(&reset_gpio, 1);
	k_sleep(K_MSEC(100));

	/* Deassert reset */
	gpio_pin_set_dt(&reset_gpio, 0);
	k_sleep(K_SECONDS(2));

	LOG_INF("... Done!");
}

/* ========================================================================
 * Command tables
 * ======================================================================== */

static const struct modem_cmd response_cmds[] = {
	MODEM_CMD("OK", on_cmd_ok, 0U, ""),
	MODEM_CMD("ERROR", on_cmd_error, 0U, ""),
	MODEM_CMD("+CME ERROR: ", on_cmd_exterror, 1U, ""),
};

static const struct modem_cmd unsol_cmds[] = {
	MODEM_CMD("+SQNSRING: ", on_cmd_unsol_ring, 1U, ""),
	MODEM_CMD("+SQNSH: ", on_cmd_unsol_close, 1U, ""),
	MODEM_CMD(MDM_UNSOL_RDY, on_cmd_unsol_rdy, 0U, ""),
};

/* Setup commands sent at boot time */
static const struct setup_cmd setup_cmds[] = {
	/* Disable echo */
	SETUP_CMD_NOHANDLE("ATE0"),
	/* Enable extended error reporting */
	SETUP_CMD_NOHANDLE("AT+CMEE=1"),
	/* Enable network registration URC */
	SETUP_CMD_NOHANDLE("AT+CEREG=1"),

	/* Read modem info */
	SETUP_CMD("AT+CGMI", "", on_cmd_atcmdinfo_manufacturer, 0U, ""),
	SETUP_CMD("AT+CGMM", "", on_cmd_atcmdinfo_model, 0U, ""),
	SETUP_CMD("AT+CGMR", "", on_cmd_atcmdinfo_revision, 0U, ""),
	SETUP_CMD("AT+CGSN", "", on_cmd_atcmdinfo_imei, 0U, ""),

	/* Socket config: use SQNSRING normal mode (just connId) and
	 * raw binary for send/recv (sendDataMode=0, recvDataMode=0).
	 * AT+SQNSCFGEXT=<connId>,<srMode>,<sendDataMode>,<keepalive>,
	 *               <listenAutoRsp>,<recvDataMode>
	 */
	SETUP_CMD_NOHANDLE("AT+SQNSCFGEXT=1,0,0,0,0,0"),
	SETUP_CMD_NOHANDLE("AT+SQNSCFGEXT=2,0,0,0,0,0"),
	SETUP_CMD_NOHANDLE("AT+SQNSCFGEXT=3,0,0,0,0,0"),
	SETUP_CMD_NOHANDLE("AT+SQNSCFGEXT=4,0,0,0,0,0"),
	SETUP_CMD_NOHANDLE("AT+SQNSCFGEXT=5,0,0,0,0,0"),
	SETUP_CMD_NOHANDLE("AT+SQNSCFGEXT=6,0,0,0,0,0"),

	/* Configure PDP context for all sockets:
	 * AT+SQNSCFG=<connId>,<cid>,<pktSz>,<maxTo>,<connTo>,<txTo>
	 * cid=1, pktSz=0(auto), maxTo=90, connTo=600, txTo=50
	 */
	SETUP_CMD_NOHANDLE("AT+SQNSCFG=1,1,0,90,600,50"),
	SETUP_CMD_NOHANDLE("AT+SQNSCFG=2,1,0,90,600,50"),
	SETUP_CMD_NOHANDLE("AT+SQNSCFG=3,1,0,90,600,50"),
	SETUP_CMD_NOHANDLE("AT+SQNSCFG=4,1,0,90,600,50"),
	SETUP_CMD_NOHANDLE("AT+SQNSCFG=5,1,0,90,600,50"),
	SETUP_CMD_NOHANDLE("AT+SQNSCFG=6,1,0,90,600,50"),

	/* Define PDP context */
	SETUP_CMD_NOHANDLE("AT+CGDCONT=1,\"IP\",\"" MDM_APN "\""),
};

/* ========================================================================
 * PDP context activation
 * ======================================================================== */

static int modem_pdp_context_activate(void)
{
	int ret;
	int retry_count = 0;

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U,
			     "AT+CGACT=1,1", &mdata.sem_response,
			     MDM_CMD_TIMEOUT);

	while (ret == -EIO && retry_count < MDM_PDP_ACT_RETRY_COUNT) {
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U,
				     "AT+CGACT=0,1", &mdata.sem_response,
				     MDM_CMD_TIMEOUT);

		if (ret != 0) {
			return ret;
		}

		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U,
				     "AT+CGACT=1,1", &mdata.sem_response,
				     MDM_CMD_TIMEOUT);

		retry_count++;
	}

	if (ret == -EIO && retry_count >= MDM_PDP_ACT_RETRY_COUNT) {
		LOG_ERR("Retried activating/deactivating too many times.");
	}

	return ret;
}

/* ========================================================================
 * Modem setup
 * ======================================================================== */

static int modem_setup(void)
{
	int ret = 0, counter;
	int rssi_retry_count = 0, init_retry_count = 0;

	pin_init();

restart:

	counter = 0;

	/* stop RSSI delay work */
	k_work_cancel_delayable(&mdata.rssi_query_work);

	/* Let the modem respond with +SYSSTART or equivalent. */
	LOG_INF("Waiting for modem to respond");
	ret = k_sem_take(&mdata.sem_response, MDM_MAX_BOOT_TIME);
	if (ret < 0) {
		LOG_ERR("Timeout waiting for RDY");
		goto error;
	}

	/* Run setup commands on the modem. */
	ret = modem_cmd_handler_setup_cmds(&mctx.iface, &mctx.cmd_handler,
					   setup_cmds, ARRAY_SIZE(setup_cmds),
					   &mdata.sem_response,
					   MDM_REGISTRATION_TIMEOUT);
	if (ret < 0) {
		goto error;
	}

restart_rssi:

	/* query modem RSSI */
	modem_rssi_query_work(NULL);
	k_sleep(MDM_WAIT_FOR_RSSI_DELAY);

	/* Keep trying to read RSSI until we get a valid value */
	while (counter++ < MDM_WAIT_FOR_RSSI_COUNT &&
	       (mdata.mdm_rssi >= 0 || mdata.mdm_rssi <= -1000)) {
		modem_rssi_query_work(NULL);
		k_sleep(MDM_WAIT_FOR_RSSI_DELAY);
	}

	if (mdata.mdm_rssi >= 0 || mdata.mdm_rssi <= -1000) {
		rssi_retry_count++;

		if (rssi_retry_count >= MDM_NETWORK_RETRY_COUNT) {
			LOG_ERR("Failed network init. Too many attempts!");
			ret = -ENETUNREACH;
			goto error;
		}

		LOG_ERR("Failed network init. Restarting process.");
		counter = 0;
		goto restart_rssi;
	}

	/* Network is ready */
	LOG_INF("Network is ready.");
	k_work_reschedule_for_queue(&modem_workq, &mdata.rssi_query_work,
				    K_SECONDS(RSSI_TIMEOUT_SECS));

	/* Activate PDP context */
	ret = modem_pdp_context_activate();
	if (ret < 0 && init_retry_count++ < MDM_INIT_RETRY_COUNT) {
		LOG_ERR("Error activating modem with pdp context");
		goto restart;
	}

error:
	return ret;
}

/* ========================================================================
 * Init
 * ======================================================================== */

static int sqn_gm02s_init(const struct device *dev)
{
	int ret;

	ARG_UNUSED(dev);

	k_sem_init(&mdata.sem_response, 0, 1);
	k_sem_init(&mdata.sem_tx_ready, 0, 1);
	k_sem_init(&mdata.sem_sock_conn, 0, 1);
	k_work_queue_start(&modem_workq, modem_workq_stack,
			   K_KERNEL_STACK_SIZEOF(modem_workq_stack),
			   K_PRIO_COOP(7), NULL);

	/* socket config */
	ret = modem_socket_init(&mdata.socket_config, &mdata.sockets[0],
				ARRAY_SIZE(mdata.sockets),
				MDM_BASE_SOCKET_NUM, true,
				&offload_socket_fd_op_vtable);
	if (ret < 0) {
		goto error;
	}

	/* cmd handler setup */
	const struct modem_cmd_handler_config cmd_handler_config = {
		.match_buf = &mdata.cmd_match_buf[0],
		.match_buf_len = sizeof(mdata.cmd_match_buf),
		.buf_pool = &mdm_recv_pool,
		.alloc_timeout = BUF_ALLOC_TIMEOUT,
		.eol = "\r\n",
		.user_data = NULL,
		.response_cmds = response_cmds,
		.response_cmds_len = ARRAY_SIZE(response_cmds),
		.unsol_cmds = unsol_cmds,
		.unsol_cmds_len = ARRAY_SIZE(unsol_cmds),
	};

	ret = modem_cmd_handler_init(&mctx.cmd_handler,
				     &mdata.cmd_handler_data,
				     &cmd_handler_config);
	if (ret < 0) {
		goto error;
	}

	/* modem interface */
	const struct modem_iface_uart_config uart_config = {
		.rx_rb_buf = &mdata.iface_rb_buf[0],
		.rx_rb_buf_len = sizeof(mdata.iface_rb_buf),
		.dev = MDM_UART_DEV,
		.hw_flow_control = DT_PROP(MDM_UART_NODE, hw_flow_control),
	};

	ret = modem_iface_uart_init(&mctx.iface, &mdata.iface_data,
				    &uart_config);
	if (ret < 0) {
		goto error;
	}

	/* modem data storage */
	mctx.data_manufacturer = mdata.mdm_manufacturer;
	mctx.data_model = mdata.mdm_model;
	mctx.data_revision = mdata.mdm_revision;
	mctx.data_imei = mdata.mdm_imei;
	mctx.data_rssi = &mdata.mdm_rssi;

	/* pin setup */
	ret = gpio_pin_configure_dt(&reset_gpio, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "reset");
		goto error;
	}

#if DT_INST_NODE_HAS_PROP(0, mdm_power_gpios)
	ret = gpio_pin_configure_dt(&power_gpio, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "power");
		goto error;
	}
#endif

	/* modem context setup */
	mctx.driver_data = &mdata;

	ret = modem_context_register(&mctx);
	if (ret < 0) {
		LOG_ERR("Error registering modem context: %d", ret);
		goto error;
	}

	/* start RX thread */
	k_thread_create(&modem_rx_thread, modem_rx_stack,
			K_KERNEL_STACK_SIZEOF(modem_rx_stack), modem_rx, NULL,
			NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

	/* Init RSSI query */
	k_work_init_delayable(&mdata.rssi_query_work, modem_rssi_query_work);
	return modem_setup();

error:
	return ret;
}

/* Register the device with the Networking stack. */
NET_DEVICE_DT_INST_OFFLOAD_DEFINE(0, sqn_gm02s_init, NULL, &mdata, NULL,
				  CONFIG_MODEM_SQN_GM02S_INIT_PRIORITY,
				  &api_funcs, MDM_MAX_DATA_LENGTH);

/* Register NET sockets. */
NET_SOCKET_OFFLOAD_REGISTER(sqn_gm02s, CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY,
			    AF_UNSPEC, offload_is_supported, offload_socket);
