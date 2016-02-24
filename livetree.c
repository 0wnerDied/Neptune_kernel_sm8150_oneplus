/*
 * (C) Copyright David Gibson <dwg@au1.ibm.com>, IBM Corporation.  2005.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307
 *                                                                   USA
 */

#include "dtc.h"

/*
 * Tree building functions
 */

void add_label(struct label **labels, char *label)
{
	struct label *new;

	/* Make sure the label isn't already there */
	for_each_label_withdel(*labels, new)
		if (streq(new->label, label)) {
			new->deleted = 0;
			return;
		}

	new = xmalloc(sizeof(*new));
	memset(new, 0, sizeof(*new));
	new->label = label;
	new->next = *labels;
	*labels = new;
}

void delete_labels(struct label **labels)
{
	struct label *label;

	for_each_label(*labels, label)
		label->deleted = 1;
}

struct property *build_property(char *name, struct data val)
{
	struct property *new = xmalloc(sizeof(*new));

	memset(new, 0, sizeof(*new));

	new->name = name;
	new->val = val;

	return new;
}

struct property *build_property_delete(char *name)
{
	struct property *new = xmalloc(sizeof(*new));

	memset(new, 0, sizeof(*new));

	new->name = name;
	new->deleted = 1;

	return new;
}

struct property *chain_property(struct property *first, struct property *list)
{
	assert(first->next == NULL);

	first->next = list;
	return first;
}

struct property *reverse_properties(struct property *first)
{
	struct property *p = first;
	struct property *head = NULL;
	struct property *next;

	while (p) {
		next = p->next;
		p->next = head;
		head = p;
		p = next;
	}
	return head;
}

struct node *build_node(struct property *proplist, struct node *children)
{
	struct node *new = xmalloc(sizeof(*new));
	struct node *child;

	memset(new, 0, sizeof(*new));

	new->proplist = reverse_properties(proplist);
	new->children = children;

	for_each_child(new, child) {
		child->parent = new;
	}

	return new;
}

struct node *build_node_delete(void)
{
	struct node *new = xmalloc(sizeof(*new));

	memset(new, 0, sizeof(*new));

	new->deleted = 1;

	return new;
}

struct node *name_node(struct node *node, char *name)
{
	assert(node->name == NULL);

	node->name = name;

	return node;
}

struct node *merge_nodes(struct node *old_node, struct node *new_node)
{
	struct property *new_prop, *old_prop;
	struct node *new_child, *old_child;
	struct label *l;

	old_node->deleted = 0;

	/* Add new node labels to old node */
	for_each_label_withdel(new_node->labels, l)
		add_label(&old_node->labels, l->label);

	/* Move properties from the new node to the old node.  If there
	 * is a collision, replace the old value with the new */
	while (new_node->proplist) {
		/* Pop the property off the list */
		new_prop = new_node->proplist;
		new_node->proplist = new_prop->next;
		new_prop->next = NULL;

		if (new_prop->deleted) {
			delete_property_by_name(old_node, new_prop->name);
			free(new_prop);
			continue;
		}

		/* Look for a collision, set new value if there is */
		for_each_property_withdel(old_node, old_prop) {
			if (streq(old_prop->name, new_prop->name)) {
				/* Add new labels to old property */
				for_each_label_withdel(new_prop->labels, l)
					add_label(&old_prop->labels, l->label);

				old_prop->val = new_prop->val;
				old_prop->deleted = 0;
				free(new_prop);
				new_prop = NULL;
				break;
			}
		}

		/* if no collision occurred, add property to the old node. */
		if (new_prop)
			add_property(old_node, new_prop);
	}

	/* Move the override child nodes into the primary node.  If
	 * there is a collision, then merge the nodes. */
	while (new_node->children) {
		/* Pop the child node off the list */
		new_child = new_node->children;
		new_node->children = new_child->next_sibling;
		new_child->parent = NULL;
		new_child->next_sibling = NULL;

		if (new_child->deleted) {
			delete_node_by_name(old_node, new_child->name);
			free(new_child);
			continue;
		}

		/* Search for a collision.  Merge if there is */
		for_each_child_withdel(old_node, old_child) {
			if (streq(old_child->name, new_child->name)) {
				merge_nodes(old_child, new_child);
				new_child = NULL;
				break;
			}
		}

		/* if no collision occured, add child to the old node. */
		if (new_child)
			add_child(old_node, new_child);
	}

	/* The new node contents are now merged into the old node.  Free
	 * the new node. */
	free(new_node);

	return old_node;
}

void add_orphan_node(struct node *dt, struct node *new_node, char *ref)
{
	static unsigned int next_orphan_fragment = 0;
	struct node *ovl = xmalloc(sizeof(*ovl));
	struct property *p;
	struct data d = empty_data;
	char *name;
	int ret;

	memset(ovl, 0, sizeof(*ovl));

	d = data_add_marker(d, REF_PHANDLE, ref);
	d = data_append_integer(d, 0xffffffff, 32);

	p = build_property("target", d);
	add_property(ovl, p);

	ret = asprintf(&name, "fragment@%u",
			next_orphan_fragment++);
	if (ret == -1)
		die("asprintf() failed\n");
	name_node(ovl, name);
	name_node(new_node, "__overlay__");

	add_child(dt, ovl);
	add_child(ovl, new_node);
}

struct node *chain_node(struct node *first, struct node *list)
{
	assert(first->next_sibling == NULL);

	first->next_sibling = list;
	return first;
}

void add_property(struct node *node, struct property *prop)
{
	struct property **p;

	prop->next = NULL;

	p = &node->proplist;
	while (*p)
		p = &((*p)->next);

	*p = prop;
}

void delete_property_by_name(struct node *node, char *name)
{
	struct property *prop = node->proplist;

	while (prop) {
		if (!strcmp(prop->name, name)) {
			delete_property(prop);
			return;
		}
		prop = prop->next;
	}
}

void delete_property(struct property *prop)
{
	prop->deleted = 1;
	delete_labels(&prop->labels);
}

void add_child(struct node *parent, struct node *child)
{
	struct node **p;

	child->next_sibling = NULL;
	child->parent = parent;

	p = &parent->children;
	while (*p)
		p = &((*p)->next_sibling);

	*p = child;
}

void delete_node_by_name(struct node *parent, char *name)
{
	struct node *node = parent->children;

	while (node) {
		if (!strcmp(node->name, name)) {
			delete_node(node);
			return;
		}
		node = node->next_sibling;
	}
}

void delete_node(struct node *node)
{
	struct property *prop;
	struct node *child;

	node->deleted = 1;
	for_each_child(node, child)
		delete_node(child);
	for_each_property(node, prop)
		delete_property(prop);
	delete_labels(&node->labels);
}

struct reserve_info *build_reserve_entry(uint64_t address, uint64_t size)
{
	struct reserve_info *new = xmalloc(sizeof(*new));

	memset(new, 0, sizeof(*new));

	new->re.address = address;
	new->re.size = size;

	return new;
}

struct reserve_info *chain_reserve_entry(struct reserve_info *first,
					struct reserve_info *list)
{
	assert(first->next == NULL);

	first->next = list;
	return first;
}

struct reserve_info *add_reserve_entry(struct reserve_info *list,
				      struct reserve_info *new)
{
	struct reserve_info *last;

	new->next = NULL;

	if (! list)
		return new;

	for (last = list; last->next; last = last->next)
		;

	last->next = new;

	return list;
}

struct boot_info *build_boot_info(struct reserve_info *reservelist,
				  struct node *tree, uint32_t boot_cpuid_phys)
{
	struct boot_info *bi;

	bi = xmalloc(sizeof(*bi));
	bi->reservelist = reservelist;
	bi->dt = tree;
	bi->boot_cpuid_phys = boot_cpuid_phys;

	return bi;
}

/*
 * Tree accessor functions
 */

const char *get_unitname(struct node *node)
{
	if (node->name[node->basenamelen] == '\0')
		return "";
	else
		return node->name + node->basenamelen + 1;
}

struct property *get_property(struct node *node, const char *propname)
{
	struct property *prop;

	for_each_property(node, prop)
		if (streq(prop->name, propname))
			return prop;

	return NULL;
}

cell_t propval_cell(struct property *prop)
{
	assert(prop->val.len == sizeof(cell_t));
	return fdt32_to_cpu(*((cell_t *)prop->val.val));
}

struct property *get_property_by_label(struct node *tree, const char *label,
				       struct node **node)
{
	struct property *prop;
	struct node *c;

	*node = tree;

	for_each_property(tree, prop) {
		struct label *l;

		for_each_label(prop->labels, l)
			if (streq(l->label, label))
				return prop;
	}

	for_each_child(tree, c) {
		prop = get_property_by_label(c, label, node);
		if (prop)
			return prop;
	}

	*node = NULL;
	return NULL;
}

struct marker *get_marker_label(struct node *tree, const char *label,
				struct node **node, struct property **prop)
{
	struct marker *m;
	struct property *p;
	struct node *c;

	*node = tree;

	for_each_property(tree, p) {
		*prop = p;
		m = p->val.markers;
		for_each_marker_of_type(m, LABEL)
			if (streq(m->ref, label))
				return m;
	}

	for_each_child(tree, c) {
		m = get_marker_label(c, label, node, prop);
		if (m)
			return m;
	}

	*prop = NULL;
	*node = NULL;
	return NULL;
}

struct node *get_subnode(struct node *node, const char *nodename)
{
	struct node *child;

	for_each_child(node, child)
		if (streq(child->name, nodename))
			return child;

	return NULL;
}

struct node *get_node_by_path(struct node *tree, const char *path)
{
	const char *p;
	struct node *child;

	if (!path || ! (*path)) {
		if (tree->deleted)
			return NULL;
		return tree;
	}

	while (path[0] == '/')
		path++;

	p = strchr(path, '/');

	for_each_child(tree, child) {
		if (p && strneq(path, child->name, p-path))
			return get_node_by_path(child, p+1);
		else if (!p && streq(path, child->name))
			return child;
	}

	return NULL;
}

struct node *get_node_by_label(struct node *tree, const char *label)
{
	struct node *child, *node;
	struct label *l;

	assert(label && (strlen(label) > 0));

	for_each_label(tree->labels, l)
		if (streq(l->label, label))
			return tree;

	for_each_child(tree, child) {
		node = get_node_by_label(child, label);
		if (node)
			return node;
	}

	return NULL;
}

struct node *get_node_by_phandle(struct node *tree, cell_t phandle)
{
	struct node *child, *node;

	assert((phandle != 0) && (phandle != -1));

	if (tree->phandle == phandle) {
		if (tree->deleted)
			return NULL;
		return tree;
	}

	for_each_child(tree, child) {
		node = get_node_by_phandle(child, phandle);
		if (node)
			return node;
	}

	return NULL;
}

struct node *get_node_by_ref(struct node *tree, const char *ref)
{
	if (streq(ref, "/"))
		return tree;
	else if (ref[0] == '/')
		return get_node_by_path(tree, ref);
	else
		return get_node_by_label(tree, ref);
}

cell_t get_node_phandle(struct node *root, struct node *node)
{
	static cell_t phandle = 1; /* FIXME: ick, static local */

	if ((node->phandle != 0) && (node->phandle != -1))
		return node->phandle;

	while (get_node_by_phandle(root, phandle))
		phandle++;

	node->phandle = phandle;

	if (!get_property(node, "linux,phandle")
	    && (phandle_format & PHANDLE_LEGACY))
		add_property(node,
			     build_property("linux,phandle",
					    data_append_cell(empty_data, phandle)));

	if (!get_property(node, "phandle")
	    && (phandle_format & PHANDLE_EPAPR))
		add_property(node,
			     build_property("phandle",
					    data_append_cell(empty_data, phandle)));

	/* If the node *does* have a phandle property, we must
	 * be dealing with a self-referencing phandle, which will be
	 * fixed up momentarily in the caller */

	return node->phandle;
}

uint32_t guess_boot_cpuid(struct node *tree)
{
	struct node *cpus, *bootcpu;
	struct property *reg;

	cpus = get_node_by_path(tree, "/cpus");
	if (!cpus)
		return 0;


	bootcpu = cpus->children;
	if (!bootcpu)
		return 0;

	reg = get_property(bootcpu, "reg");
	if (!reg || (reg->val.len != sizeof(uint32_t)))
		return 0;

	/* FIXME: Sanity check node? */

	return propval_cell(reg);
}

static int cmp_reserve_info(const void *ax, const void *bx)
{
	const struct reserve_info *a, *b;

	a = *((const struct reserve_info * const *)ax);
	b = *((const struct reserve_info * const *)bx);

	if (a->re.address < b->re.address)
		return -1;
	else if (a->re.address > b->re.address)
		return 1;
	else if (a->re.size < b->re.size)
		return -1;
	else if (a->re.size > b->re.size)
		return 1;
	else
		return 0;
}

static void sort_reserve_entries(struct boot_info *bi)
{
	struct reserve_info *ri, **tbl;
	int n = 0, i = 0;

	for (ri = bi->reservelist;
	     ri;
	     ri = ri->next)
		n++;

	if (n == 0)
		return;

	tbl = xmalloc(n * sizeof(*tbl));

	for (ri = bi->reservelist;
	     ri;
	     ri = ri->next)
		tbl[i++] = ri;

	qsort(tbl, n, sizeof(*tbl), cmp_reserve_info);

	bi->reservelist = tbl[0];
	for (i = 0; i < (n-1); i++)
		tbl[i]->next = tbl[i+1];
	tbl[n-1]->next = NULL;

	free(tbl);
}

static int cmp_prop(const void *ax, const void *bx)
{
	const struct property *a, *b;

	a = *((const struct property * const *)ax);
	b = *((const struct property * const *)bx);

	return strcmp(a->name, b->name);
}

static void sort_properties(struct node *node)
{
	int n = 0, i = 0;
	struct property *prop, **tbl;

	for_each_property_withdel(node, prop)
		n++;

	if (n == 0)
		return;

	tbl = xmalloc(n * sizeof(*tbl));

	for_each_property_withdel(node, prop)
		tbl[i++] = prop;

	qsort(tbl, n, sizeof(*tbl), cmp_prop);

	node->proplist = tbl[0];
	for (i = 0; i < (n-1); i++)
		tbl[i]->next = tbl[i+1];
	tbl[n-1]->next = NULL;

	free(tbl);
}

static int cmp_subnode(const void *ax, const void *bx)
{
	const struct node *a, *b;

	a = *((const struct node * const *)ax);
	b = *((const struct node * const *)bx);

	return strcmp(a->name, b->name);
}

static void sort_subnodes(struct node *node)
{
	int n = 0, i = 0;
	struct node *subnode, **tbl;

	for_each_child_withdel(node, subnode)
		n++;

	if (n == 0)
		return;

	tbl = xmalloc(n * sizeof(*tbl));

	for_each_child_withdel(node, subnode)
		tbl[i++] = subnode;

	qsort(tbl, n, sizeof(*tbl), cmp_subnode);

	node->children = tbl[0];
	for (i = 0; i < (n-1); i++)
		tbl[i]->next_sibling = tbl[i+1];
	tbl[n-1]->next_sibling = NULL;

	free(tbl);
}

static void sort_node(struct node *node)
{
	struct node *c;

	sort_properties(node);
	sort_subnodes(node);
	for_each_child_withdel(node, c)
		sort_node(c);
}

void sort_tree(struct boot_info *bi)
{
	sort_reserve_entries(bi);
	sort_node(bi->dt);
}

void generate_label_node(struct node *node, struct node *dt)
{
	struct node *c, *an;
	struct property *p;
	struct label *l;
	int has_label;
	char *gen_node_name;

	if (auto_label_aliases)
		gen_node_name = "aliases";
	else
		gen_node_name = "__symbols__";

	/* Make sure the label isn't already there */
	has_label = 0;
	for_each_label(node->labels, l) {
		has_label = 1;
		break;
	}

	if (has_label) {

		/* an is the aliases/__symbols__ node */
		an = get_subnode(dt, gen_node_name);
		/* if no node exists, create it */
		if (!an) {
			an = build_node(NULL, NULL);
			name_node(an, gen_node_name);
			add_child(dt, an);
		}

		/* now add the label in the node */
		for_each_label(node->labels, l) {
			/* check whether the label already exists */
			p = get_property(an, l->label);
			if (p) {
				fprintf(stderr, "WARNING: label %s already"
					" exists in /%s", l->label,
					gen_node_name);
				continue;
			}

			/* insert it */
			p = build_property(l->label,
				data_copy_escape_string(node->fullpath,
						strlen(node->fullpath)));
			add_property(an, p);
		}

		/* force allocation of a phandle for this node */
		if (symbol_fixup_support)
			(void)get_node_phandle(dt, node);
	}

	for_each_child(node, c)
		generate_label_node(c, dt);
}

static void add_fixup_entry(struct node *dt, struct node *node,
		struct property *prop, struct marker *m)
{
	struct node *fn;	/* local fixup node */
	struct property *p;
	char *fixups_name = "__fixups__";
	struct data d;
	char *entry;
	int ret;

	/* fn is the node we're putting entries in */
	fn = get_subnode(dt, fixups_name);
	/* if no node exists, create it */
	if (!fn) {
		fn = build_node(NULL, NULL);
		name_node(fn, fixups_name);
		add_child(dt, fn);
	}

	ret = asprintf(&entry, "%s:%s:%u",
			node->fullpath, prop->name, m->offset);
	if (ret == -1)
		die("asprintf() failed\n");

	p = get_property(fn, m->ref);
	d = data_append_data(p ? p->val : empty_data, entry, strlen(entry) + 1);
	if (!p)
		add_property(fn, build_property(m->ref, d));
	else
		p->val = d;
}

static void add_local_fixup_entry(struct node *dt, struct node *node,
		struct property *prop, struct marker *m,
		struct node *refnode)
{
	struct node *lfn, *wn, *nwn;	/* local fixup node, walk node, new */
	struct property *p;
	struct data d;
	char *local_fixups_name = "__local_fixups__";
	char *s, *e, *comp;
	int len;

	/* fn is the node we're putting entries in */
	lfn = get_subnode(dt, local_fixups_name);
	/* if no node exists, create it */
	if (!lfn) {
		lfn = build_node(NULL, NULL);
		name_node(lfn, local_fixups_name);
		add_child(dt, lfn);
	}

	/* walk the path components creating nodes if they don't exist */
	comp = NULL;
	/* start skipping the first / */
	s = node->fullpath + 1;
	wn = lfn;
	while (*s) {
		/* retrieve path component */
		e = strchr(s, '/');
		if (e == NULL)
			e = s + strlen(s);
		len = e - s;
		comp = xrealloc(comp, len + 1);
		memcpy(comp, s, len);
		comp[len] = '\0';

		/* if no node exists, create it */
		nwn = get_subnode(wn, comp);
		if (!nwn) {
			nwn = build_node(NULL, NULL);
			name_node(nwn, strdup(comp));
			add_child(wn, nwn);
		}
		wn = nwn;

		/* last path component */
		if (!*e)
			break;

		/* next path component */
		s = e + 1;
	}
	free(comp);

	p = get_property(wn, prop->name);
	d = data_append_cell(p ? p->val : empty_data, (cell_t)m->offset);
	if (!p)
		add_property(wn, build_property(prop->name, d));
	else
		p->val = d;
}

void generate_fixups_node(struct node *node, struct node *dt)
{
	struct node *c;
	struct property *prop;
	struct marker *m;
	struct node *refnode;

	for_each_property(node, prop) {
		m = prop->val.markers;
		for_each_marker_of_type(m, REF_PHANDLE) {
			refnode = get_node_by_ref(dt, m->ref);
			if (!refnode)
				add_fixup_entry(dt, node, prop, m);
			else
				add_local_fixup_entry(dt, node, prop, m,
						refnode);
		}
	}

	for_each_child(node, c)
		generate_fixups_node(c, dt);
}
