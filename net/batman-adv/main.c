/*
 * Copyright (C) 2007-2011 B.A.T.M.A.N. contributors:
 *
 * Marek Lindner, Simon Wunderlich
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of version 2 of the GNU General Public
 * License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA
 *
 */

#include "main.h"
#include "bat_sysfs.h"
#include "bat_debugfs.h"
#include "routing.h"
#include "send.h"
#include "originator.h"
#include "soft-interface.h"
#include "icmp_socket.h"
#include "translation-table.h"
#include "hard-interface.h"
#include "gateway_client.h"
#include "vis.h"
#include "hash.h"


/* List manipulations on hardif_list have to be rtnl_lock()'ed,
 * list traversals just rcu-locked */
struct list_head hardif_list;

unsigned char broadcast_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

struct workqueue_struct *bat_event_workqueue;

static int __init batman_init(void)
{
	INIT_LIST_HEAD(&hardif_list);

	/* the name should not be longer than 10 chars - see
	 * http://lwn.net/Articles/23634/ */
	bat_event_workqueue = create_singlethread_workqueue("bat_events");

	if (!bat_event_workqueue)
		return -ENOMEM;

	bat_socket_init();
	debugfs_init();

	register_netdevice_notifier(&hard_if_notifier);

	pr_info("B.A.T.M.A.N. advanced %s%s (compatibility version %i) "
		"loaded\n", SOURCE_VERSION, REVISION_VERSION_STR,
		COMPAT_VERSION);

	return 0;
}

static void __exit batman_exit(void)
{
	debugfs_destroy();
	unregister_netdevice_notifier(&hard_if_notifier);
	hardif_remove_interfaces();

	flush_workqueue(bat_event_workqueue);
	destroy_workqueue(bat_event_workqueue);
	bat_event_workqueue = NULL;

	rcu_barrier();
}

int mesh_init(struct net_device *soft_iface)
{
	struct bat_priv *bat_priv = netdev_priv(soft_iface);

	spin_lock_init(&bat_priv->forw_bat_list_lock);
	spin_lock_init(&bat_priv->forw_bcast_list_lock);
	spin_lock_init(&bat_priv->tt_lhash_lock);
	spin_lock_init(&bat_priv->tt_ghash_lock);
	spin_lock_init(&bat_priv->gw_list_lock);
	spin_lock_init(&bat_priv->vis_hash_lock);
	spin_lock_init(&bat_priv->vis_list_lock);
	spin_lock_init(&bat_priv->softif_neigh_lock);
	spin_lock_init(&bat_priv->softif_neigh_vid_lock);

	INIT_HLIST_HEAD(&bat_priv->forw_bat_list);
	INIT_HLIST_HEAD(&bat_priv->forw_bcast_list);
	INIT_HLIST_HEAD(&bat_priv->gw_list);
	INIT_HLIST_HEAD(&bat_priv->softif_neigh_vids);

	if (originator_init(bat_priv) < 1)
		goto err;

	if (tt_local_init(bat_priv) < 1)
		goto err;

	if (tt_global_init(bat_priv) < 1)
		goto err;

	tt_local_add(soft_iface, soft_iface->dev_addr);

	if (vis_init(bat_priv) < 1)
		goto err;

	atomic_set(&bat_priv->mesh_state, MESH_ACTIVE);
	goto end;

err:
	pr_err("Unable to allocate memory for mesh information structures: "
	       "out of mem ?\n");
	mesh_free(soft_iface);
	return -1;

end:
	return 0;
}

void mesh_free(struct net_device *soft_iface)
{
	struct bat_priv *bat_priv = netdev_priv(soft_iface);

	atomic_set(&bat_priv->mesh_state, MESH_DEACTIVATING);

	purge_outstanding_packets(bat_priv, NULL);

	vis_quit(bat_priv);

	gw_node_purge(bat_priv);
	originator_free(bat_priv);

	tt_local_free(bat_priv);
	tt_global_free(bat_priv);

	softif_neigh_purge(bat_priv);

	atomic_set(&bat_priv->mesh_state, MESH_INACTIVE);
}

void inc_module_count(void)
{
	try_module_get(THIS_MODULE);
}

void dec_module_count(void)
{
	module_put(THIS_MODULE);
}

int is_my_mac(uint8_t *addr)
{
<<<<<<< HEAD
	struct hard_iface *hard_iface;

	rcu_read_lock();
	list_for_each_entry_rcu(hard_iface, &hardif_list, list) {
		if (hard_iface->if_status != IF_ACTIVE)
			continue;

		if (compare_eth(hard_iface->net_dev->dev_addr, addr)) {
			rcu_read_unlock();
			return 1;
		}
=======
	struct batadv_algo_ops *bat_algo_ops = NULL, *bat_algo_ops_tmp;

	hlist_for_each_entry(bat_algo_ops_tmp, &batadv_algo_list, list) {
		if (strcmp(bat_algo_ops_tmp->name, name) != 0)
			continue;

		bat_algo_ops = bat_algo_ops_tmp;
		break;
	}

	return bat_algo_ops;
}

int batadv_algo_register(struct batadv_algo_ops *bat_algo_ops)
{
	struct batadv_algo_ops *bat_algo_ops_tmp;
	int ret;

	bat_algo_ops_tmp = batadv_algo_get(bat_algo_ops->name);
	if (bat_algo_ops_tmp) {
		pr_info("Trying to register already registered routing algorithm: %s\n",
			bat_algo_ops->name);
		ret = -EEXIST;
		goto out;
	}

	/* all algorithms must implement all ops (for now) */
	if (!bat_algo_ops->bat_iface_enable ||
	    !bat_algo_ops->bat_iface_disable ||
	    !bat_algo_ops->bat_iface_update_mac ||
	    !bat_algo_ops->bat_primary_iface_set ||
	    !bat_algo_ops->bat_ogm_schedule ||
	    !bat_algo_ops->bat_ogm_emit) {
		pr_info("Routing algo '%s' does not implement required ops\n",
			bat_algo_ops->name);
		ret = -EINVAL;
		goto out;
	}

	INIT_HLIST_NODE(&bat_algo_ops->list);
	hlist_add_head(&bat_algo_ops->list, &batadv_algo_list);
	ret = 0;

out:
	return ret;
}

int batadv_algo_select(struct batadv_priv *bat_priv, char *name)
{
	struct batadv_algo_ops *bat_algo_ops;
	int ret = -EINVAL;

	bat_algo_ops = batadv_algo_get(name);
	if (!bat_algo_ops)
		goto out;

	bat_priv->bat_algo_ops = bat_algo_ops;
	ret = 0;

out:
	return ret;
}

int batadv_algo_seq_print_text(struct seq_file *seq, void *offset)
{
	struct batadv_algo_ops *bat_algo_ops;

	seq_printf(seq, "Available routing algorithms:\n");

	hlist_for_each_entry(bat_algo_ops, &batadv_algo_list, list) {
		seq_printf(seq, "%s\n", bat_algo_ops->name);
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators
	}
	rcu_read_unlock();
	return 0;

}

module_init(batman_init);
module_exit(batman_exit);

MODULE_LICENSE("GPL");

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_SUPPORTED_DEVICE(DRIVER_DEVICE);
#ifdef REVISION_VERSION
MODULE_VERSION(SOURCE_VERSION "-" REVISION_VERSION);
#else
MODULE_VERSION(SOURCE_VERSION);
#endif
