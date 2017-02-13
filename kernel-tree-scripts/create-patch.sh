#!/bin/sh

WG="$(readlink -f "$(dirname "$(readlink -f "$0")")/../../src/")"

for i in "$WG"/*.c "$WG"/*.h "$WG"/selftest/*.h "$WG"/crypto/*.c "$WG"/crypto/*.h "$WG"/crypto/*.S "$WG"/Kbuild "$WG"/Kconfig $(find "$WG"/compat -name '*.c' -o -name '*.h' -o -name '*.include'); do
	diff -u /dev/null "$i" | sed "s:${WG}:b/net/wireguard:;s:Kbuild:Makefile:"
done

cat <<_EOF
--- a/net/Kconfig
+++ b/net/Kconfig
@@ -85,1 +85,2 @@ config INET
 if INET
+source "net/wireguard/Kconfig"
--- a/net/Makefile
+++ b/net/Makefile
@@ -16,1 +16,2 @@
 obj-\$(CONFIG_NETFILTER)		+= netfilter/
+obj-\$(CONFIG_WIREGUARD)		+= wireguard/
_EOF
