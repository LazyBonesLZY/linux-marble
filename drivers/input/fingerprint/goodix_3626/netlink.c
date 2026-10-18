// SPDX-License-Identifier: GPL-2.0
/*
 * netlink interface for the Goodix fingerprint platform driver.
 *
 * The vendor HAL opens a NETLINK_TEST (25) socket and waits for one-byte
 * events (GF_NET_EVENT_IRQ / FB_BLACK / FB_UNBLACK) sent by this driver.
 *
 * Copyright (c) 2017 Goodix
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <net/sock.h>
#include <net/netlink.h>

#define NETLINK_TEST		25
#define MAX_MSGSIZE		32

static int pid = -1;
static struct sock *nl_sk;

int netlink_init(void);
void netlink_exit(void);
void sendnlmsg(char *message);

void sendnlmsg(char *message)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	int slen;
	int len;

	if (!message || !nl_sk || pid <= 0)
		return;

	slen = strnlen(message, MAX_MSGSIZE - 1);
	len = NLMSG_SPACE(slen + 1);

	skb = alloc_skb(len, GFP_KERNEL);
	if (!skb) {
		pr_err("alloc_skb error\n");
		return;
	}

	nlh = nlmsg_put(skb, 0, 0, 0, slen + 1, 0);
	if (!nlh) {
		kfree_skb(skb);
		return;
	}

	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;
	memcpy(NLMSG_DATA(nlh), message, slen + 1);

	if (netlink_unicast(nl_sk, skb, pid, MSG_DONTWAIT) < 0)
		pr_err("send msg from kernel to userspace failed\n");
}

static void nl_data_ready(struct sk_buff *__skb)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	char str[100];
	size_t copy;

	skb = skb_get(__skb);
	if (!skb)
		return;

	if (skb->len < NLMSG_SPACE(0))
		goto out;

	nlh = nlmsg_hdr(skb);
	copy = min_t(size_t, skb->len - NLMSG_HDRLEN, sizeof(str) - 1);
	memcpy(str, NLMSG_DATA(nlh), copy);
	str[copy] = '\0';
	pid = nlh->nlmsg_pid;
	pr_debug("%s: userspace pid %d\n", __func__, pid);

out:
	kfree_skb(skb);
}

int netlink_init(void)
{
	struct netlink_kernel_cfg netlink_cfg = {
		.groups = 0,
		.flags = 0,
		.input = nl_data_ready,
	};

	nl_sk = netlink_kernel_create(&init_net, NETLINK_TEST, &netlink_cfg);
	if (!nl_sk) {
		pr_err("create netlink socket error\n");
		return -ENOMEM;
	}

	return 0;
}

void netlink_exit(void)
{
	if (nl_sk) {
		netlink_kernel_release(nl_sk);
		nl_sk = NULL;
	}
	pid = -1;
	pr_info("self module exited\n");
}
