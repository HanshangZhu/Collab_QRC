#!/usr/bin/env bash
# laptop_ethernet_setup.sh - laptop-side Ethernet/CycloneDDS setup for
# real_run_monitor.sh.
#
# Source this helper from laptop launchers that need Unitree Sport API DDS
# discovery on the Go2 192.168.123.x subnet. It keeps device-specific NIC
# detection and stale CycloneDDS cleanup out of orchestration scripts.

real_monitor_go2_net_iface() {
  ip -o -4 addr show | awk '$4 ~ /^192\.168\.123\./ {print $2; exit}'
}

real_monitor_dds_iface_from_uri() {
  local uri="${1:-}"
  [[ "$uri" == file://* ]] || return 0
  local dds_file="${uri#file://}"
  [[ -f "$dds_file" ]] || return 0
  sed -nE 's/.*<NetworkInterface name="([^"]+)".*/\1/p' "$dds_file" | head -1
}

real_monitor_resolve_go2_iface() {
  if [[ -n "${GO2W_ETH_IFACE:-}" ]] && ip link show "$GO2W_ETH_IFACE" &>/dev/null; then
    echo "$GO2W_ETH_IFACE"
    return 0
  fi
  real_monitor_go2_net_iface
}

setup_real_monitor_laptop_ethernet() {
  local helper_dir
  helper_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  local connect_script="${REAL_MONITOR_CONNECT_ETHERNET_SCRIPT:-${helper_dir}/connect_ethernet.sh}"

  if [[ ! -f "$connect_script" ]]; then
    echo "WARN: ${connect_script} not found; Unitree DDS may not discover Sport API." >&2
    return 0
  fi

  local go2w_iface_auto
  go2w_iface_auto="$(real_monitor_resolve_go2_iface)"
  if [[ -n "$go2w_iface_auto" ]]; then
    export GO2W_ETH_IFACE="$go2w_iface_auto"

    local stale_dds_iface
    stale_dds_iface="$(real_monitor_dds_iface_from_uri "${CYCLONEDDS_URI:-}")"
    if [[ -n "$stale_dds_iface" && "$stale_dds_iface" != "$GO2W_ETH_IFACE" ]]; then
      echo "WARN: replacing stale CYCLONEDDS_URI (${stale_dds_iface}) with ${GO2W_ETH_IFACE}."
      unset CYCLONEDDS_URI RMW_IMPLEMENTATION
    fi
  fi

  local had_errexit=0 had_nounset=0
  [[ $- == *e* ]] && had_errexit=1
  [[ $- == *u* ]] && had_nounset=1

  # The monitor also talks to the NX at .18 through UDP/SSH; Sport API itself
  # is on the Go2 controller at .161. connect_ethernet.sh keeps .161 as peer.
  # shellcheck source=/dev/null
  source "$connect_script"

  if (( ! had_errexit )); then
    set +e
  fi

  if [[ -n "${go2w_iface_auto:-}" ]]; then
    # connect_ethernet.sh snapshots ETH_IFACE when sourced. Force both names so
    # sourced shells with stale defaults cannot regenerate DDS XML for an old NIC.
    export GO2W_ETH_IFACE="$go2w_iface_auto"
    ETH_IFACE="$go2w_iface_auto"
  fi

  echo "  CycloneDDS interface: ${ETH_IFACE:-unknown}"
  set +u
  setup_cyclonedds_ethernet
  local setup_rc=$?
  if (( had_nounset )); then
    set -u
  else
    set +u
  fi
  (( setup_rc == 0 )) || return "$setup_rc"

  local dds_iface
  dds_iface="$(real_monitor_dds_iface_from_uri "${CYCLONEDDS_URI:-}")"
  if [[ -n "$dds_iface" ]] && ! ip link show "$dds_iface" &>/dev/null; then
    echo "ERROR: CycloneDDS is bound to missing interface '${dds_iface}'." >&2
    echo "       Current 192.168.123.x interface: $(real_monitor_go2_net_iface)" >&2
    return 1
  fi
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  setup_real_monitor_laptop_ethernet "$@"
fi
