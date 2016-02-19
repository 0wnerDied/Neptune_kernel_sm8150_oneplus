/*
 * libfdt - Flat Device Tree manipulation
 * Copyright (C) 2006 David Gibson, IBM Corporation.
 *
 * libfdt is dual licensed: you can use it either under the terms of
 * the GPL, or the BSD license, at your option.
 *
 *  a) This library is free software; you can redistribute it and/or
 *     modify it under the terms of the GNU General Public License as
 *     published by the Free Software Foundation; either version 2 of the
 *     License, or (at your option) any later version.
 *
 *     This library is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public
 *     License along with this library; if not, write to the Free
 *     Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
 *     MA 02110-1301 USA
 *
 * Alternatively,
 *
 *  b) Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *     1. Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *     2. Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 *     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *     CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *     INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *     MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 *     CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *     SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *     NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *     HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *     CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *     OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *     EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "libfdt_env.h"

#include <fdt.h>
#include <libfdt.h>

#include "libfdt_internal.h"

static int rw_check_header(void *fdt)
{
	int err;

	if ((err = _fdt_check_header(fdt)))
		return err;
	if (fdt_version(fdt) < 0x11)
		return -FDT_ERR_BADVERSION;
	if (fdt_off_mem_rsvmap(fdt) < ALIGN(sizeof(struct fdt_header), 8))
		return -FDT_ERR_BADLAYOUT;
	if (fdt_off_dt_struct(fdt) <
	    (fdt_off_mem_rsvmap(fdt) + sizeof(struct fdt_reserve_entry)))
		return -FDT_ERR_BADLAYOUT;
	if (fdt_off_dt_strings(fdt) <
	    (fdt_off_dt_struct(fdt) + fdt_size_dt_struct(fdt)))
		return -FDT_ERR_BADLAYOUT;
	if (fdt_totalsize(fdt) <
	    (fdt_off_dt_strings(fdt) + fdt_size_dt_strings(fdt)))
		return -FDT_ERR_BADLAYOUT;
	return 0;
}

#define RW_CHECK_HEADER(fdt) \
	{ \
		int err; \
		if ((err = rw_check_header(fdt)) != 0) \
			return err; \
	}

static inline int _blob_data_size(void *fdt)
{
	return fdt_off_dt_strings(fdt) + fdt_size_dt_strings(fdt);
}

static int _blob_splice(void *fdt, void *p, int oldlen, int newlen)
{
	void *end = fdt + _blob_data_size(fdt);

	if (((p + oldlen) < p) || ((p + oldlen) > end))
		return -FDT_ERR_BADOFFSET;
	if ((end - oldlen + newlen) > (fdt + fdt_totalsize(fdt)))
		return -FDT_ERR_NOSPACE;
	memmove(p + newlen, p + oldlen, end - p - oldlen);
	return 0;
}

static int _blob_splice_struct(void *fdt, void *p,
			       int oldlen, int newlen)
{
	int delta = newlen - oldlen;
	int err;

	if ((err = _blob_splice(fdt, p, oldlen, newlen)))
		return err;

	fdt_set_header(fdt, size_dt_struct, fdt_size_dt_struct(fdt) + delta);
	fdt_set_header(fdt, off_dt_strings, fdt_off_dt_strings(fdt) + delta);
	return 0;
}

static int _blob_splice_string(void *fdt, int newlen)
{
	void *p = fdt + fdt_off_dt_strings(fdt) + fdt_size_dt_strings(fdt);
	int err;

	if ((err = _blob_splice(fdt, p, 0, newlen)))
		return err;

	fdt_set_header(fdt, size_dt_strings, fdt_size_dt_strings(fdt) + newlen);
	return 0;
}

static int _find_add_string(void *fdt, const char *s)
{
	char *strtab = (char *)fdt + fdt_off_dt_strings(fdt);
	const char *p;
	char *new;
	int len = strlen(s) + 1;
	int err;

	p = _fdt_find_string(strtab, fdt_size_dt_strings(fdt), s);
	if (p)
		/* found it */
		return (p - strtab);

	new = strtab + fdt_size_dt_strings(fdt);
	err = _blob_splice_string(fdt, len);
	if (err)
		return err;

	memcpy(new, s, len);
	return (new - strtab);
}

static int _resize_property(void *fdt, int nodeoffset, const char *name, int len,
			    struct fdt_property **prop)
{
	int oldlen;
	int err;

	*prop = fdt_get_property_w(fdt, nodeoffset, name, &oldlen);
	if (! (*prop))
		return oldlen;

	if ((err = _blob_splice_struct(fdt, (*prop)->data,
				       ALIGN(oldlen, FDT_TAGSIZE),
				       ALIGN(len, FDT_TAGSIZE))))
		return err;

	(*prop)->len = cpu_to_fdt32(len);
	return 0;
}

static int _add_property(void *fdt, int nodeoffset, const char *name, int len,
			 struct fdt_property **prop)
{
	uint32_t tag;
	int proplen;
	int nextoffset;
	int namestroff;
	int err;

	tag = _fdt_next_tag(fdt, nodeoffset, &nextoffset);
	if (tag != FDT_BEGIN_NODE)
		return -FDT_ERR_BADOFFSET;

	namestroff = _find_add_string(fdt, name);
	if (namestroff < 0)
		return namestroff;

	*prop = _fdt_offset_ptr_w(fdt, nextoffset);
	proplen = sizeof(**prop) + ALIGN(len, FDT_TAGSIZE);

	err = _blob_splice_struct(fdt, *prop, 0, proplen);
	if (err)
		return err;

	(*prop)->tag = cpu_to_fdt32(FDT_PROP);
	(*prop)->nameoff = cpu_to_fdt32(namestroff);
	(*prop)->len = cpu_to_fdt32(len);
	return 0;
}

int fdt_setprop(void *fdt, int nodeoffset, const char *name,
		const void *val, int len)
{
	struct fdt_property *prop;
	int err;

	if ((err = rw_check_header(fdt)))
		return err;

	err = _resize_property(fdt, nodeoffset, name, len, &prop);
	if (err == -FDT_ERR_NOTFOUND)
		err = _add_property(fdt, nodeoffset, name, len, &prop);
	if (err)
		return err;

	memcpy(prop->data, val, len);
	return 0;
}

int fdt_delprop(void *fdt, int nodeoffset, const char *name)
{
	struct fdt_property *prop;
	int len, proplen;

	RW_CHECK_HEADER(fdt);

	prop = fdt_get_property_w(fdt, nodeoffset, name, &len);
	if (! prop)
		return len;

	proplen = sizeof(*prop) + ALIGN(len, FDT_TAGSIZE);
	return _blob_splice_struct(fdt, prop, proplen, 0);
}

int fdt_add_subnode_namelen(void *fdt, int parentoffset,
			    const char *name, int namelen)
{
	struct fdt_node_header *nh;
	int offset, nextoffset;
	int nodelen;
	int err;
	uint32_t tag;
	uint32_t *endtag;

	RW_CHECK_HEADER(fdt);

	offset = fdt_subnode_offset_namelen(fdt, parentoffset, name, namelen);
	if (offset >= 0)
		return -FDT_ERR_EXISTS;
	else if (offset != -FDT_ERR_NOTFOUND)
		return offset;

	/* Try to place the new node after the parent's properties */
	_fdt_next_tag(fdt, parentoffset, &nextoffset); /* skip the BEGIN_NODE */
	do {
		offset = nextoffset;
		tag = _fdt_next_tag(fdt, offset, &nextoffset);
	} while (tag == FDT_PROP);

	nh = _fdt_offset_ptr_w(fdt, offset);
	nodelen = sizeof(*nh) + ALIGN(namelen+1, FDT_TAGSIZE) + FDT_TAGSIZE;

	err = _blob_splice_struct(fdt, nh, 0, nodelen);
	if (err)
		return err;

	nh->tag = cpu_to_fdt32(FDT_BEGIN_NODE);
	memset(nh->name, 0, ALIGN(namelen+1, FDT_TAGSIZE));
	memcpy(nh->name, name, namelen);
	endtag = (uint32_t *)((void *)nh + nodelen - FDT_TAGSIZE);
	*endtag = cpu_to_fdt32(FDT_END_NODE);

	return offset;
}

int fdt_add_subnode(void *fdt, int parentoffset, const char *name)
{
	return fdt_add_subnode_namelen(fdt, parentoffset, name, strlen(name));
}

int fdt_del_node(void *fdt, int nodeoffset)
{
	int endoffset;

	endoffset = _fdt_node_end_offset(fdt, nodeoffset);
	if (endoffset < 0)
		return endoffset;

	return _blob_splice_struct(fdt, _fdt_offset_ptr_w(fdt, nodeoffset),
				   endoffset - nodeoffset, 0);
}

int fdt_open_into(void *fdt, void *buf, int bufsize)
{
	int err;

	err = fdt_move(fdt, buf, bufsize);
	if (err)
		return err;

	fdt = buf;

	fdt_set_header(fdt, totalsize, bufsize);

	/* FIXME: re-order if necessary */

	err = rw_check_header(fdt);
	if (err)
		return err;

	return 0;
}

int fdt_pack(void *fdt)
{
	int err;

	err = rw_check_header(fdt);
	if (err)
		return err;

	/* FIXME: pack components */
	fdt_set_header(fdt, totalsize, _blob_data_size(fdt));
	return 0;
}
