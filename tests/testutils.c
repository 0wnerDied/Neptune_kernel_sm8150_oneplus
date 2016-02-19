/*
 * libfdt - Flat Device Tree manipulation
 *	Testcase common utility functions
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

#define _GNU_SOURCE /* for strsignal() */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>

#include <libfdt.h>

#include "tests.h"

int verbose_test = 1;
char *test_name;

void  __attribute__((weak)) cleanup(void)
{
}

static void sigint_handler(int signum, siginfo_t *si, void *uc)
{
	cleanup();
	fprintf(stderr, "%s: %s (pid=%d)\n", test_name,
		strsignal(signum), getpid());
	exit(RC_BUG);
}

void test_init(int argc, char *argv[])
{
	int err;
	struct sigaction sa_int = {
		.sa_sigaction = sigint_handler,
	};

	test_name = argv[0];

	err = sigaction(SIGINT, &sa_int, NULL);
	if (err)
		FAIL("Can't install SIGINT handler");

	if (getenv("QUIET_TEST"))
		verbose_test = 0;

	verbose_printf("Starting testcase \"%s\", pid %d\n",
		       test_name, getpid());
}

void check_property(void *fdt, int nodeoffset, const char *name,
		    int len, const void *val)
{
	const struct fdt_property *prop;
	int retlen;
	uint32_t tag, nameoff, proplen;
	const char *propname;

	verbose_printf("Checking property \"%s\"...", name);
	prop = fdt_get_property(fdt, nodeoffset, name, &retlen);
	verbose_printf("pointer %p\n", prop);
	if (! prop)
		FAIL("Error retreiving \"%s\" pointer: %s", name,
		     fdt_strerror(retlen));

	tag = fdt32_to_cpu(prop->tag);
	nameoff = fdt32_to_cpu(prop->nameoff);
	proplen = fdt32_to_cpu(prop->len);

	if (tag != FDT_PROP)
		FAIL("Incorrect tag 0x%08x on property \"%s\"", tag, name);

	propname = fdt_string(fdt, nameoff);
	if (!propname || !streq(propname, name))
		FAIL("Property name mismatch \"%s\" instead of \"%s\"",
		     propname, name);
	if (proplen != retlen)
		FAIL("Length retrieved for \"%s\" by fdt_get_property()"
		     " differs from stored length (%d != %d)",
		     name, retlen, proplen);
	if (proplen != len)
		FAIL("Size mismatch on property \"%s\": %d insead of %d",
		     name, proplen, len);
	if (memcmp(val, prop->data, len) != 0)
		FAIL("Data mismatch on property \"%s\"", name);
	
}

const void *check_getprop(void *fdt, int nodeoffset, const char *name,
			  int len, const void *val)
{
	const void *propval;
	int proplen;

	propval = fdt_getprop(fdt, nodeoffset, name, &proplen);
	if (! propval)
		FAIL("fdt_getprop(\"%s\"): %s", name, fdt_strerror(proplen));

	if (proplen != len)
		FAIL("Size mismatch on property \"%s\": %d insead of %d",
		     name, proplen, len);
	if (memcmp(val, propval, len) != 0)
		FAIL("Data mismatch on property \"%s\"", name);

	return propval;
}

#define CHUNKSIZE	128

void *load_blob(const char *filename)
{
	int fd;
	int offset = 0;
	int bufsize = 1024;
	char *p = NULL;
	int ret;

	fd = open(filename, O_RDONLY);
	if (fd < 0)
		CONFIG("Couldn't open blob from \"%s\": %s", filename,
		       strerror(errno));

	p = xmalloc(bufsize);
	do {
		if (offset == bufsize) {
			bufsize *= 2;
			p = xrealloc(p, bufsize);
		}

		ret = read(fd, &p[offset], bufsize - offset);
		if (ret < 0)
			CONFIG("Couldn't read from \"%s\": %s", filename,
			       strerror(errno));

		offset += ret;
	} while (ret != 0);

	return p;
}

void *load_blob_arg(int argc, char *argv[])
{
	if (argc != 2)
		CONFIG("Usage: %s <dtb file>", argv[0]);
	return load_blob(argv[1]);
}

void save_blob(const char *filename, void *fdt)
{
	int fd;
	int totalsize;
	int offset;
	void *p;
	int ret;

	fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0666);
	if (fd < 0)
		CONFIG("Couldn't open \"%s\" to write blob: %s", filename,
		       strerror(errno));

	totalsize = fdt_totalsize(fdt);
	offset = 0;
	p = fdt;

	while (offset < totalsize) {
		ret = write(fd, p + offset, totalsize - offset);
		if (ret < 0)
			CONFIG("Couldn't write to \"%s\": %s", filename,
			       strerror(errno));
		offset += ret;
	}
}
