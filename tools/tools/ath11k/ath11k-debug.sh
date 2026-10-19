#!/bin/sh
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (c) 2026 Quentin Thebault
#
# Load ath11k(4) with every debug knob that matters for bring-up and
# collect what is needed to debug it into one tarball.
#
# Usage: ath11k-debug.sh [-a] [-n] [-t seconds]
#   -a          also associate with wpa_supplicant(8) using
#               /etc/wpa_supplicant.conf and run dhclient(8)
#   -n          do not load the driver (it is already loaded, or loaded at
#               boot); only collect
#   -t seconds  how long to wait for the interface (default 90)
#
# Run as root from a console or over a wired link: the driver may hang or
# panic the machine.  Set dumpdev="AUTO" in rc.conf(5) beforehand so a
# panic leaves a crash dump, and kern.msgbufsize="4194304" in
# loader.conf(5) so the boot messages are not overwritten.

set -u

ASSOC=0
LOAD=1
WAIT=90
while getopts "ant:" opt; do
	case "$opt" in
	a) ASSOC=1 ;;
	n) LOAD=0 ;;
	t) WAIT=$OPTARG ;;
	*) echo "usage: $0 [-a] [-n] [-t seconds]" >&2; exit 2 ;;
	esac
done

if [ "$(id -u)" -ne 0 ]; then
	echo "$0: must run as root" >&2
	exit 1
fi

STAMP=$(date +%Y%m%d-%H%M%S)
OUT=/var/tmp/ath11k-debug-$STAMP
mkdir -p "$OUT" || exit 1
LOG=$OUT/script.log

say() {
	echo "$*" | tee -a "$LOG"
}

# Run a command, saving its output as $OUT/<name>.
grab() {
	name=$1; shift
	{ echo "# $*"; "$@" 2>&1; } > "$OUT/$name" 2>&1
}

# ath11k debug mask: WMI HTC MAC BOOT QMI REG PCI, no per-packet data path.
ATH11K_MASK=0x1276
# LinuxKPI 802.11: TODO IMPROVE CHANDEF TRACE SCAN STA HW_CRYPTO MO.
LKPI80211_MASK=0x13005b

say "ath11k debug run $STAMP, output in $OUT"

#
# Before: system, device, firmware.
#
grab uname uname -a
grab freebsd-version freebsd-version -kru
grab kenv-smbios sh -c 'kenv | grep -E "^smbios\.(system|bios|planar)"'
grab kldstat-before kldstat -v
grab loader.conf sh -c 'cat /boot/loader.conf /boot/loader.conf.local 2>/dev/null'
grab rc.conf-wlan sh -c 'grep -E "wlan|ath11k|ifconfig|dumpdev|kld_list" /etc/rc.conf /etc/rc.conf.local 2>/dev/null'
grab sysctl-msgbuf sysctl kern.msgbufsize kern.shutdown.dumpdevname
grab sysctl-pci sysctl hw.pci
grab firmware sh -c 'ls -l /boot/firmware/ath11k* /boot/firmware/ath11k/WCN6855/*/* /boot/modules/*ath11k* 2>/dev/null; pkg info -x wifi-firmware 2>/dev/null'
grab vmstat-i-before vmstat -i

# The card: Qualcomm 17cb:1103 (WCN6855), or any 17cb device.
SEL=$(pciconf -l | awk '/vendor=0x17cb/ { sub(/^[^@]*@/, "", $1); sub(/:$/, "", $1); print $1; exit }')
if [ -z "$SEL" ]; then
	say "no Qualcomm PCI device found (pciconf -l | grep 0x17cb)"
else
	say "device at $SEL"
	OWNER=$(pciconf -l "$SEL" | sed -n 's/@.*//p')
	case "$OWNER" in
	ppt*)
		say "FAIL: $OWNER (bhyve passthrough) owns the card, ath11k cannot attach."
		say "      Remove it from pptdevs in /boot/loader.conf and reboot."
		exit 1
		;;
	none*|ath11k*) ;;
	*)	say "warning: $OWNER already owns the card" ;;
	esac
	grab pciconf-before pciconf -lvbc "$SEL"
	grab pcicfg-before pciconf -r -b "$SEL" 0:0xff
fi

MSGBUF=$(sysctl -n kern.msgbufsize 2>/dev/null || echo 0)
[ "$MSGBUF" -ge 1048576 ] ||
	say "warning: kern.msgbufsize is $MSGBUF, set kern.msgbufsize=\"4194304\" in loader.conf"
[ -n "$(sysctl -n kern.shutdown.dumpdevname 2>/dev/null)" ] ||
	say "warning: no dump device, a panic will leave no crash dump (dumpdev=\"AUTO\" in rc.conf)"

#
# Debug knobs.  Module ones are read at load, so set them as tunables.
#
kenv compat.linuxkpi.ath11k_core_debug_mask=$ATH11K_MASK >/dev/null
kenv compat.linuxkpi.mhi_debug=1 >/dev/null
kenv compat.linuxkpi.qrtr_debug=1 >/dev/null
sysctl compat.linuxkpi.ath11k_core_debug_mask=$ATH11K_MASK >/dev/null 2>&1
sysctl compat.linuxkpi.mhi_debug=1 >/dev/null 2>&1
sysctl compat.linuxkpi.qrtr_debug=1 >/dev/null 2>&1
sysctl compat.linuxkpi.80211.debug=$LKPI80211_MASK >/dev/null 2>&1 ||
	say "note: compat.linuxkpi.80211.debug not available"

# Remember where the logs were.
MSGSTART=$(wc -l < /var/log/messages 2>/dev/null || echo 0)
dmesg -a > "$OUT/dmesg-before.txt" 2>&1

#
# Load and wait for the wlan device.
#
wlandev() {
	sysctl -n net.wlan.devices 2>/dev/null | tr ' ' '\n' | grep -E '^ath11k|^wlan' | head -1
}

# ath11k encrypts data in firmware with keys from set_key; LinuxKPI only
# calls set_key with hardware crypto on.  Both are boot-time tunables.
for t in hw_crypto tkip; do
	[ "$(sysctl -n compat.linuxkpi.80211.$t 2>/dev/null)" = 1 ] ||
	    say "WARN: compat.linuxkpi.80211.$t is not 1; set it in" \
		"/boot/loader.conf or encrypted data will not pass"
done

if [ $LOAD -eq 1 ]; then
	if kldstat -q -n if_ath11k; then
		say "if_ath11k already loaded"
	else
		say "loading if_ath11k"
		kldload if_ath11k 2>&1 | tee -a "$LOG"
	fi
fi

say "waiting up to ${WAIT}s for the wlan device"
DEV=""
i=0
while [ $i -lt "$WAIT" ]; do
	DEV=$(wlandev)
	[ -n "$DEV" ] && break
	sleep 2
	i=$((i + 2))
	[ $((i % 10)) -eq 0 ] && say "  ${i}s: $(pciconf -l "$SEL" 2>/dev/null | sed -n 's/@.*//p'), $(dmesg | grep -cE 'ath11k|mhi|qrtr') driver messages so far"
done

if [ -z "$DEV" ]; then
	say "FAIL: no wlan device after ${WAIT}s; dumping MHI state"
	sysctl compat.linuxkpi.mhi_dump=1 >/dev/null 2>&1
else
	say "wlan device $DEV after ${i}s"
	WLAN=wlan99
	ifconfig $WLAN destroy >/dev/null 2>&1
	if ifconfig $WLAN create wlandev "$DEV" >> "$LOG" 2>&1; then
		wlandebug -i $WLAN +state+scan+assoc+auth+crypto+11n+output+input >> "$LOG" 2>&1
		ifconfig $WLAN up >> "$LOG" 2>&1
		# "up" starts a scan; "ifconfig scan" during it gets EINPROGRESS
		# and lists the still-empty cache, so poll the cache instead.
		say "scanning (up to 30s)"
		j=0
		while [ $j -lt 30 ]; do
			[ -n "$(ifconfig $WLAN list scan 2>/dev/null)" ] && break
			sleep 2
			j=$((j + 2))
		done
		sleep 5
		grab scan ifconfig $WLAN list scan
		say "$(grep -v '^#' "$OUT/scan" | sed 1d | grep -c .) APs in the scan cache after $((j + 5))s"
		if [ $ASSOC -eq 1 ]; then
			say "associating with /etc/wpa_supplicant.conf"
			# Base wpa_supplicant has no -f; keep it in the
			# foreground of a background job to capture -dd.
			wpa_supplicant -i $WLAN -c /etc/wpa_supplicant.conf \
			    -dd -t > "$OUT/wpa_supplicant.log" 2>&1 &
			WPAPID=$!
			i=0
			while [ $i -lt 60 ]; do
				ifconfig $WLAN | grep -q "status: associated" && break
				kill -0 $WPAPID 2>/dev/null || break
				sleep 1
				i=$((i + 1))
			done
			kill -0 $WPAPID 2>/dev/null ||
			    say "FAIL: wpa_supplicant exited; see wpa_supplicant.log"
			grab ifconfig-assoc ifconfig -v $WLAN
			if ifconfig $WLAN | grep -q "status: associated"; then
				say "associated after ${i}s; dhclient"
				grab dhclient timeout 30 dhclient $WLAN
				grab ping sh -c "ping -c 5 -t 10 \$(netstat -rn -f inet | awk '/^default/ {print \$2; exit}')"
			else
				say "FAIL: not associated after ${i}s"
			fi
			kill $WPAPID 2>/dev/null
			wait $WPAPID 2>/dev/null
		fi
		grab wlanstats-$WLAN sh -c "wlanstats -i $WLAN 2>/dev/null || true"
	else
		say "FAIL: could not create $WLAN on $DEV"
	fi
fi

#
# After.
#
sleep 2
sysctl compat.linuxkpi.mhi_dump=1 >/dev/null 2>&1
sleep 1
dmesg -a > "$OUT/dmesg.txt" 2>&1
tail -n +"$((MSGSTART + 1))" /var/log/messages > "$OUT/messages.txt" 2>/dev/null
grab kldstat-after kldstat -v
grab vmstat-i-after vmstat -i
grab ifconfig-after ifconfig -a
grab sysctl-wlan sysctl net.wlan
grab sysctl-compat sysctl compat.linuxkpi
if [ -n "$SEL" ]; then
	grab pciconf-after pciconf -lvbc "$SEL"
	grab pcicfg-after pciconf -r -b "$SEL" 0:0xff
	grab devinfo sh -c "devinfo -rv | grep -B2 -A8 -i ath11k"
fi

# Summary of the lines that usually tell what went wrong.
grep -hE "ath11k|mhi|qrtr|qmi|MSI|firmware|lkpi|LinuxKPI|panic|Fatal|timeout|failed" \
    "$OUT/messages.txt" "$OUT/dmesg.txt" 2>/dev/null | sort -u | head -400 > "$OUT/summary.txt"

TAR=/var/tmp/ath11k-debug-$STAMP.tar.gz
tar -czf "$TAR" -C /var/tmp "ath11k-debug-$STAMP"
say "done: $TAR"
