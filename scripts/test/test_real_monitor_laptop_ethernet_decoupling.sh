#!/usr/bin/env bash
# Regression guard for keeping real_run_monitor.sh focused on orchestration.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MONITOR="${ROOT_DIR}/scripts/real/real_run_monitor.sh"
HELPER="${ROOT_DIR}/scripts/real/laptop_ethernet_setup.sh"

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

[[ -f "$HELPER" ]] || fail "missing laptop Ethernet helper: $HELPER"

grep -q 'LAPTOP_ETHERNET_SETUP_SCRIPT=' "$MONITOR" \
  || fail "monitor does not declare LAPTOP_ETHERNET_SETUP_SCRIPT"
grep -q 'source "$LAPTOP_ETHERNET_SETUP_SCRIPT"' "$MONITOR" \
  || fail "monitor does not source laptop Ethernet helper"
grep -q 'setup_real_monitor_laptop_ethernet' "$MONITOR" \
  || fail "monitor does not call setup_real_monitor_laptop_ethernet"

if grep -q 'setup_cyclonedds_ethernet' "$MONITOR"; then
  fail "monitor still calls setup_cyclonedds_ethernet directly"
fi
if grep -q '_go2_net_iface' "$MONITOR" || grep -q '_dds_iface_from_uri' "$MONITOR"; then
  fail "monitor still owns laptop Ethernet helper functions"
fi

# The helper should be sourceable without performing network or ROS setup.
# shellcheck source=/dev/null
source "$HELPER"
declare -F setup_real_monitor_laptop_ethernet >/dev/null \
  || fail "helper does not define setup_real_monitor_laptop_ethernet"
declare -F real_monitor_dds_iface_from_uri >/dev/null \
  || fail "helper does not define real_monitor_dds_iface_from_uri"

tmp_xml="$(mktemp)"
trap 'rm -f "$tmp_xml"' EXIT
cat >"$tmp_xml" <<'EOF_XML'
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="enxrobot0" priority="default" multicast="true" />
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
EOF_XML

iface="$(real_monitor_dds_iface_from_uri "file://${tmp_xml}")"
[[ "$iface" == "enxrobot0" ]] || fail "unexpected DDS iface parse result: ${iface}"

echo "real monitor laptop Ethernet decoupling test passed"
