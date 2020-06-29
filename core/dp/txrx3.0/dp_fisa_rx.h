/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <dp_types.h>
#include <qdf_status.h>

//#define FISA_DEBUG_ENABLE

#ifdef FISA_DEBUG_ENABLE
#define dp_fisa_debug dp_info
#else
#define dp_fisa_debug dp_debug
#endif

#if defined(WLAN_SUPPORT_RX_FISA)
/**
 * dp_rx_dump_fisa_stats() - Dump fisa stats
 * @soc: core txrx main context
 *
 * Return: QDF_STATUS
 */
QDF_STATUS dp_rx_dump_fisa_stats(struct dp_soc *soc);

/**
 * dp_fisa_rx() - FISA Rx packet delivery entry function
 * @soc: core txrx main context
 * @vdev: core txrx vdev
 * @nbuf_list: Delivery list of nbufs
 *
 * Return: QDF_STATUS
 */
QDF_STATUS dp_fisa_rx(struct dp_soc *dp_fisa_rx_hdl, struct dp_vdev *vdev,
		      qdf_nbuf_t nbuf_list);

/**
 * dp_rx_fisa_flush_by_ctx_id() - FISA Rx flush function to flush
 *				  aggregation at end of NAPI
 * @soc: core txrx main context
 * @napi_id: Flows which are rxed on the NAPI ID to be flushed
 *
 * Return: QDF_STATUS
 */
QDF_STATUS dp_rx_fisa_flush_by_ctx_id(struct dp_soc *soc, int napi_id);

/**
 * dp_rx_fisa_flush_by_vdev_id() - Flush fisa aggregates per vdev id
 * @soc: core txrx main context
 * @vdev_id: vdev ID
 *
 * Return: Success on flushing the flows for the vdev
 */
QDF_STATUS dp_rx_fisa_flush_by_vdev_id(struct dp_soc *soc, uint8_t vdev_id);
#else
static QDF_STATUS dp_rx_dump_fisa_stats(struct dp_soc *soc)
{
	return QDF_STATUS_SUCCESS;
}

void dp_rx_dump_fisa_table(struct dp_soc *soc)
{
}
#endif
