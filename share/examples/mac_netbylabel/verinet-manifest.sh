#!/bin/sh
#
# verinet-manifest.sh - generate a veriexec(8) manifest that labels a
# fixed allow-list of binaries "net/allow" for mac_netbylabel(4).
#
# Each line emitted is:   <path-relative-to-base> sha256=<hex> label=<label>
# Paths are relative to BASE (default /), which is how veriexec resolves
# them; a single base of / covers files on any mount.
#
# Usage:  verinet-manifest.sh [-b base] [-L label] [file ...]
#   -b base    root the relative paths are taken against (default /)
#   -L label   label to apply (default net/allow)
#   file ...   binaries to allow; if none given, the built-in pkgbase
#              standard-install allow-list is used.
#
# Redirect stdout to /etc/veriexec/manifest.  Missing files are reported
# on stderr and skipped (role-dependent entries may not be installed).

set -u

BASE=/
LABEL=net/allow

while getopts "b:L:h" o; do
	case "$o" in
	b) BASE=$OPTARG ;;
	L) LABEL=$OPTARG ;;
	h) echo "usage: $0 [-b base] [-L label] [file ...]"; exit 0 ;;
	*) echo "usage: $0 [-b base] [-L label] [file ...]" >&2; exit 2 ;;
	esac
done
shift $((OPTIND - 1))

# Built-in allow-list: pkgbase standard install (outbound clients +
# network listeners; freebsd-update/fetch deliberately omitted).
if [ $# -eq 0 ]; then
	set -- \
	    /sbin/dhclient \
	    /usr/sbin/local-unbound \
	    /usr/sbin/ntpd \
	    /usr/sbin/rtsold \
	    /usr/local/sbin/pkg \
	    /usr/local/sbin/pkg-static \
	    /usr/sbin/pkg \
	    /usr/sbin/sshd
fi

# normalize BASE to a prefix ending in a single '/'
case "$BASE" in
/)	bpfx=/ ;;
*/)	bpfx=$BASE ;;
*)	bpfx=$BASE/ ;;
esac

printf '# veriexec manifest for mac_netbylabel - generated %s\n' "$(date)"
printf '# base=%s label=%s\n' "$BASE" "$LABEL"

missing=0
for f in "$@"; do
	if [ ! -f "$f" ]; then
		printf 'SKIP (not present): %s\n' "$f" >&2
		missing=$((missing + 1))
		continue
	fi
	case "$f" in
	"$bpfx"*)	rel=${f#"$bpfx"} ;;
	/*)		rel=${f#/} ;;
	*)		rel=$f ;;
	esac
	if ! hash=$(sha256 -q "$f" 2>/dev/null); then
		printf 'ERROR hashing: %s\n' "$f" >&2
		continue
	fi
	printf '%s sha256=%s label=%s\n' "$rel" "$hash" "$LABEL"
done

if [ "$missing" -gt 0 ]; then
	printf '# note: %d allow-list entries were not present (see stderr)\n' \
	    "$missing"
fi
exit 0
