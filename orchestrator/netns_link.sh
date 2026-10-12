#!/usr/bin/env bash
# Two Linux network namespaces joined only through an ns-3 802.11
# ad hoc link (tap-wifi-virtual-machine example). Needs root.
#   sudo orchestrator/netns_link.sh up|down|test
# Idempotent: `up` first runs `down`.
set -u
SUBNET_PREFIX="${SUBNET_PREFIX:-10.10.0}"
AGENTS=(left right)
declare -A IPSUF=([left]=1 [right]=2)

down() {
  for a in "${AGENTS[@]}"; do
    ip netns del "ns-$a" 2>/dev/null
    ip link del "br-$a" 2>/dev/null
    ip link del "veth-$a-br" 2>/dev/null
    ip link del "tap-$a" 2>/dev/null
  done
  echo "netns_link: torn down"
}

up() {
  down >/dev/null
  for a in "${AGENTS[@]}"; do
    ip netns add "ns-$a"
    ip tuntap add dev "tap-$a" mode tap
    ip link add "br-$a" type bridge
    ip link add "veth-$a-br" type veth peer name "veth-$a-in"
    ip link set "veth-$a-in" netns "ns-$a"
    # No IPv6 on the emulated chain: its autoconfig chatter (RS/MLD) would hit
    # ns-3 the instant it opens the TAP and adds noise to the channel.
    for d in "tap-$a" "br-$a" "veth-$a-br"; do
      sysctl -qw "net.ipv6.conf.$d.disable_ipv6=1" 2>/dev/null
    done
    ip netns exec "ns-$a" sysctl -qw net.ipv6.conf.all.disable_ipv6=1 net.ipv6.conf.default.disable_ipv6=1
    ip link set "tap-$a" master "br-$a"
    ip link set "veth-$a-br" master "br-$a"
    ip link set "tap-$a" promisc on up
    ip link set "veth-$a-br" up
    ip link set "br-$a" up
    ip netns exec "ns-$a" ip link set "veth-$a-in" name radio0
    ip netns exec "ns-$a" ip addr add "$SUBNET_PREFIX.${IPSUF[$a]}/24" dev radio0
    ip netns exec "ns-$a" ip link set radio0 up
    ip netns exec "ns-$a" ip link set lo up
  done
  echo "netns_link: up (left=$SUBNET_PREFIX.1 right=$SUBNET_PREFIX.2); now start ns-3"
}

case "${1:-}" in
  up) up ;;
  down) down ;;
  test) ip netns exec ns-left ping -c 5 -W 2 "$SUBNET_PREFIX.2" ;;
  *) echo "usage: $0 up|down|test"; exit 2 ;;
esac
