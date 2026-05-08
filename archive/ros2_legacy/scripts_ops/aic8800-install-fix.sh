#!/bin/bash
# aic8800-install-fix.sh — atomic install of the AIC8800DC patch stack.
# Rebuilds DKMS module, verifies the no-latch patch made it into the .ko,
# reloads the driver, installs the upgraded watchdog and shutdown-clean unit.
# Single sudo call. Aborts on any verification failure (no silent half-success).
#
# Run as: sudo bash scripts/ops/aic8800-install-fix.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "ERROR: must be run as root (sudo bash $0)" >&2
    exit 1
fi

DKMS_NAME="aic8800dc/6.4.3.0-patched.1"
DKMS_SRC="/usr/src/aic8800dc-6.4.3.0-patched.1"
KVER="$(uname -r)"
KO_PATH="/lib/modules/${KVER}/updates/dkms/aic8800_fdrv.ko"
REPO_OPS="$(cd "$(dirname "$0")" && pwd)"

step() { printf "\n\033[1;36m==> %s\033[0m\n" "$*"; }
ok()   { printf "\033[1;32m   OK: %s\033[0m\n" "$*"; }
die()  { printf "\033[1;31m   FAIL: %s\033[0m\n" "$*" >&2; exit 1; }

# ---------------------------------------------------------------- 0. preflight
step "0. Preflight: verify patches present in source"
grep -q 'not latching STATE_CRASHED' "$DKMS_SRC/drivers/aic8800/aic8800_fdrv/rwnx_cmds.c" \
    || die "no-latch patch missing from rwnx_cmds.c — bailing"
! grep -q 'cmd_mgr->state = RWNX_CMD_MGR_STATE_CRASHED' "$DKMS_SRC/drivers/aic8800/aic8800_fdrv/rwnx_cmds.c" \
    || die "stale STATE_CRASHED write still in rwnx_cmds.c — bailing"
grep -q 'eth_hw_addr_set(ndev, mac_addr)' "$DKMS_SRC/drivers/aic8800/aic8800_fdrv/rwnx_main.c" \
    || die "dev_addr patch missing from rwnx_main.c — bailing"
ok "source patches present"

step "0b. Preflight: ops files staged in repo"
for f in aic8800-watchdog aic8800-clean-shutdown aic8800-clean-shutdown.service; do
    [[ -f "$REPO_OPS/$f" ]] || die "missing $REPO_OPS/$f"
done
bash -n "$REPO_OPS/aic8800-watchdog"        || die "syntax error in aic8800-watchdog"
bash -n "$REPO_OPS/aic8800-clean-shutdown"  || die "syntax error in aic8800-clean-shutdown"
ok "all ops files present and syntax-clean"

# ---------------------------------------------------------------- 1. stop services
step "1. Stop watchdog (so it doesn't fight us during reload)"
systemctl stop aic8800-watchdog.service 2>/dev/null || true
ok "watchdog stopped"

# ---------------------------------------------------------------- 2. dkms rebuild
step "2. DKMS remove + reinstall (forces fresh build)"
dkms status "$DKMS_NAME" 2>/dev/null | grep -q "$DKMS_NAME" \
    && dkms remove "$DKMS_NAME" --all || true
dkms install "$DKMS_NAME" -k "$KVER" || die "dkms install failed"
ok "dkms install completed"

# ---------------------------------------------------------------- 3. verify .ko
step "3. Verify the freshly-built .ko contains the no-latch patch"
[[ -f "$KO_PATH" ]] || die ".ko not found at $KO_PATH"
HITS=$(strings "$KO_PATH" | grep -c "not latching" || true)
[[ "$HITS" -ge 2 ]] || die "no-latch strings not in .ko (found $HITS, expected >=2). Build did NOT pick up the patch."
ok ".ko at $KO_PATH contains $HITS no-latch markers"

# ---------------------------------------------------------------- 4. reload module
step "4. Reload aic8800_fdrv with the new .ko"
# Best-effort cleanup so modprobe -r doesn't deadlock on netdev refs
IFACE=$(nmcli -t -f DEVICE,TYPE device 2>/dev/null \
        | awk -F: '$2=="wifi" && $1 ~ /^wlx/ {print $1; exit}')
[[ -n "$IFACE" ]] && nmcli device disconnect "$IFACE" 2>/dev/null || true
pkill -STOP wpa_supplicant 2>/dev/null || true
sleep 1
modprobe -r aic8800_fdrv aic_load_fw 2>/dev/null \
    || { rmmod -f aic8800_fdrv 2>/dev/null || true; rmmod -f aic_load_fw 2>/dev/null || true; }
sleep 2
modprobe aic8800_fdrv || die "modprobe aic8800_fdrv failed"
pkill -CONT wpa_supplicant 2>/dev/null || true
sleep 3
ok "aic8800_fdrv reloaded"

# ---------------------------------------------------------------- 5. install watchdog/shutdown
step "5. Install upgraded watchdog + shutdown-clean unit"
install -m 0755 -o root -g root "$REPO_OPS/aic8800-watchdog"        /usr/local/sbin/aic8800-watchdog
install -m 0755 -o root -g root "$REPO_OPS/aic8800-clean-shutdown"  /usr/local/sbin/aic8800-clean-shutdown
install -m 0644 -o root -g root "$REPO_OPS/aic8800-clean-shutdown.service" /etc/systemd/system/aic8800-clean-shutdown.service
ok "binaries + unit file installed"

step "5b. Verify new watchdog has hard_reload code"
grep -q 'check_cmd_crashed' /usr/local/sbin/aic8800-watchdog \
    || die "/usr/local/sbin/aic8800-watchdog is the OLD version (no check_cmd_crashed)"
ok "/usr/local/sbin/aic8800-watchdog is the new version"

# ---------------------------------------------------------------- 6. systemd
step "6. systemctl daemon-reload + enable services"
systemctl daemon-reload
systemctl enable --now aic8800-clean-shutdown.service
systemctl restart aic8800-watchdog.service
sleep 1
systemctl is-active aic8800-watchdog.service        >/dev/null || die "watchdog service didn't start"
systemctl is-enabled aic8800-clean-shutdown.service >/dev/null || die "shutdown unit not enabled"
ok "watchdog active, shutdown unit enabled"

# ---------------------------------------------------------------- 7. final status
step "7. Final status"
printf "\nLoaded module:\n"
modinfo aic8800_fdrv | grep -E "^(filename|srcversion|vermagic)" || true
printf "\nWifi interface:\n"
ip -br link show 2>/dev/null | grep -E '^wl' || echo "(no wifi iface yet — will appear after firmware reload completes)"
printf "\nWatchdog status (last 10 lines):\n"
journalctl -u aic8800-watchdog.service --since "1 min ago" --no-pager 2>/dev/null | tail -10 || true
printf "\nRecent aic kernel events:\n"
journalctl -k --since "1 min ago" --no-pager 2>/dev/null \
    | grep -iE 'aic|cmd queue|cmd timeout|not latching|dev_addr_check' | tail -10 || true

printf "\n\033[1;32mAll steps completed.\033[0m\n"
echo
echo "Next steps:"
echo "  1. Connect to ASK4:   nmcli dev wifi connect 'ASK4 Wireless' --ask"
echo "  2. Open captive portal: xdg-open http://neverssl.com"
echo "  3. After login, browse normally. cmd queue crashed lines should NOT recur."
echo "  4. Monitor watchdog:  journalctl -u aic8800-watchdog -f"
