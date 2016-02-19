/*
 * libfdt - Flat Device Tree manipulation
 *	Testcase for fdt_subnode_offset()
 * Copyright (C) 2006 David Gibson, IBM Corporation.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <fdt.h>
#include <libfdt.h>

#include "tests.h"
#include "testdata.h"

int check_subnode(struct fdt_header *fdt, int parent, const char *name)
{
	int offset;
	struct fdt_node_header *nh;
	uint32_t tag;

	verbose_printf("Checking subnode \"%s\" of %d...", name, parent);
	offset = fdt_subnode_offset(fdt, parent, name);
	verbose_printf("offset %d...", offset);
	if (offset < 0)
		FAIL("fdt_subnode_offset(\"%s\"): %s", name, fdt_strerror(offset));
	nh = fdt_offset_ptr_typed(fdt, offset, nh);
	verbose_printf("pointer %p\n", nh);
	if (! nh)
		FAIL("NULL retrieving subnode \"%s\"", name);

	tag = fdt32_to_cpu(nh->tag);

	if (tag != FDT_BEGIN_NODE)
		FAIL("Incorrect tag 0x%08x on property \"%s\"", tag, name);
	if (!streq(nh->name, name))
		FAIL("Subnode name mismatch \"%s\" instead of \"%s\"",
		     nh->name, name);

	return offset;
}

int main(int argc, char *argv[])
{
	void *fdt;
	int subnode1_offset, subnode2_offset;
	int subsubnode1_offset, subsubnode2_offset;

	test_init(argc, argv);
	fdt = load_blob_arg(argc, argv);

	subnode1_offset = check_subnode(fdt, 0, "subnode1");
	subnode2_offset = check_subnode(fdt, 0, "subnode2");

	if (subnode1_offset == subnode2_offset)
		FAIL("Different subnodes have same offset");

	check_property_typed(fdt, subnode1_offset, "prop-int", TEST_VALUE_1);
	check_property_typed(fdt, subnode2_offset, "prop-int", TEST_VALUE_2);

	subsubnode1_offset = check_subnode(fdt, subnode1_offset, "subsubnode");
	subsubnode2_offset = check_subnode(fdt, subnode2_offset, "subsubnode");

	check_property_typed(fdt, subsubnode1_offset, "prop-int", TEST_VALUE_1);
	check_property_typed(fdt, subsubnode2_offset, "prop-int", TEST_VALUE_2);

	PASS();
}
