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

%locations

%{
#include "dtc.h"
#include "srcpos.h"

int yylex(void);
unsigned long long eval_literal(const char *s, int base, int bits);

extern struct boot_info *the_boot_info;
extern int treesource_error;

%}

%union {
	char *propnodename;
	char *literal;
	char *labelref;
	unsigned int cbase;
	u8 byte;
	struct data data;

	u64 addr;
	cell_t cell;
	struct property *prop;
	struct property *proplist;
	struct node *node;
	struct node *nodelist;
	struct reserve_info *re;
}

%token DT_V1
%token DT_MEMRESERVE
%token <propnodename> DT_PROPNODENAME
%token <literal> DT_LITERAL
%token <literal> DT_LEGACYLITERAL
%token <cbase> DT_BASE
%token <byte> DT_BYTE
%token <data> DT_STRING
%token <labelref> DT_LABEL
%token <labelref> DT_REF

%type <data> propdata
%type <data> propdataprefix
%type <re> memreserve
%type <re> memreserves
%type <re> v0_memreserve
%type <re> v0_memreserves
%type <addr> addr
%type <data> celllist
%type <cbase> cellbase
%type <cell> cellval
%type <data> bytestring
%type <prop> propdef
%type <proplist> proplist

%type <node> devicetree
%type <node> nodedef
%type <node> subnode
%type <nodelist> subnodes
%type <labelref> label

%%

sourcefile:
	  DT_V1 ';' memreserves devicetree
		{
			the_boot_info = build_boot_info($3, $4);
		}
	| v0_memreserves devicetree
		{
			the_boot_info = build_boot_info($1, $2);
		}
	;

memreserves:
	  /* empty */
		{
			$$ = NULL;
		}
	| memreserve memreserves
		{
			$$ = chain_reserve_entry($1, $2);
		}
	;

memreserve:
	  label DT_MEMRESERVE addr addr ';'
		{
			$$ = build_reserve_entry($3, $4, $1);
		}
	;

v0_memreserves:
	  /* empty */
		{
			$$ = NULL;
		}
	| v0_memreserve v0_memreserves
		{
			$$ = chain_reserve_entry($1, $2);
		};
	;

v0_memreserve:
	  memreserve
		{
			$$ = $1;
		}
	| label DT_MEMRESERVE addr '-' addr ';'
		{
			$$ = build_reserve_entry($3, $5 - $3 + 1, $1);
		}
	;

addr:
	  DT_LITERAL
		{
			$$ = eval_literal($1, 0, 64);
		}
	| DT_LEGACYLITERAL
		{
			$$ = eval_literal($1, 16, 64);
		}
	  ;

devicetree:
	  '/' nodedef
		{
			$$ = name_node($2, "", NULL);
		}
	;

nodedef:
	  '{' proplist subnodes '}' ';'
		{
			$$ = build_node($2, $3);
		}
	;

proplist:
	  /* empty */
		{
			$$ = NULL;
		}
	| proplist propdef
		{
			$$ = chain_property($2, $1);
		}
	;

propdef:
	  label DT_PROPNODENAME '=' propdata ';'
		{
			$$ = build_property($2, $4, $1);
		}
	| label DT_PROPNODENAME ';'
		{
			$$ = build_property($2, empty_data, $1);
		}
	;

propdata:
	  propdataprefix DT_STRING
		{
			$$ = data_merge($1, $2);
		}
	| propdataprefix '<' celllist '>'
		{
			$$ = data_merge($1, $3);
		}
	| propdataprefix '[' bytestring ']'
		{
			$$ = data_merge($1, $3);
		}
	| propdataprefix DT_REF
		{
			$$ = data_add_marker($1, REF_PATH, $2);
		}
	| propdata DT_LABEL
		{
			$$ = data_add_marker($1, LABEL, $2);
		}
	;

propdataprefix:
	  /* empty */
		{
			$$ = empty_data;
		}
	| propdata ','
		{
			$$ = $1;
		}
	| propdataprefix DT_LABEL
		{
			$$ = data_add_marker($1, LABEL, $2);
		}
	;

celllist:
	  /* empty */
		{
			$$ = empty_data;
		}
	| celllist cellval
		{
			$$ = data_append_cell($1, $2);
		}
	| celllist DT_REF
		{
			$$ = data_append_cell(data_add_marker($1, REF_PHANDLE,
							      $2), -1);
		}
	| celllist DT_LABEL
		{
			$$ = data_add_marker($1, LABEL, $2);
		}
	;

cellbase:
	  /* empty */
		{
			$$ = 16;
		}
	| DT_BASE
	;

cellval:
	  DT_LITERAL
		{
			$$ = eval_literal($1, 0, 32);
		}
	| cellbase DT_LEGACYLITERAL
		{
			$$ = eval_literal($2, $1, 32);
		}
	;

bytestring:
	  /* empty */
		{
			$$ = empty_data;
		}
	| bytestring DT_BYTE
		{
			$$ = data_append_byte($1, $2);
		}
	| bytestring DT_LABEL
		{
			$$ = data_add_marker($1, LABEL, $2);
		}
	;

subnodes:
	  /* empty */
		{
			$$ = NULL;
		}
	|  subnode subnodes
		{
			$$ = chain_node($1, $2);
		}
	| subnode propdef
		{
			yyerror("syntax error: properties must precede subnodes");
			YYERROR;
		}
	;

subnode:
	  label DT_PROPNODENAME nodedef
		{
			$$ = name_node($3, $2, $1);
		}
	;

label:
	  /* empty */
		{
			$$ = NULL;
		}
	| DT_LABEL
		{
			$$ = $1;
		}
	;

%%

void yyerrorf(char const *s, ...)
{
	const char *fname = srcpos_file ? srcpos_file->name : "<no-file>";
	va_list va;
	va_start(va, s);

	if (strcmp(fname, "-") == 0)
		fname = "stdin";

	fprintf(stderr, "%s:%d ", fname, yylloc.first_line);
	vfprintf(stderr, s, va);
	fprintf(stderr, "\n");

	treesource_error = 1;
	va_end(va);
}

void yyerror (char const *s)
{
	yyerrorf("%s", s);
}

unsigned long long eval_literal(const char *s, int base, int bits)
{
	unsigned long long val;
	char *e;

	errno = 0;
	val = strtoull(s, &e, base);
	if (*e)
		yyerror("bad characters in literal");
	else if ((errno == ERANGE)
		 || ((bits < 64) && (val >= (1ULL << bits))))
		yyerror("literal out of range");
	else if (errno != 0)
		yyerror("bad literal");
	return val;
}
