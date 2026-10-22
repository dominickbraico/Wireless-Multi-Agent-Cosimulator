#!/usr/bin/env python3
"""Registry of network technologies the ns-3 container can emulate.

`network.technology` in the experiment YAML picks an entry (default `wifi`). The entry names the ns-3 scenario program
that the ns3 container runs, and the YAML section that holds that technology's parameters (`network.<config_section>`).

Adding a technology = write an ns-3 scenario that honours the contract in docs/CONTAINERS.md ("Changing the networking
technology"), add it under ns3/scenarios/, and add one entry below. Nothing else in the orchestrator changes.
"""

TECHNOLOGIES = {
    "wifi": {
        "scenario": "wifi_adhoc_tap",
        "config_section": "wifi",
        "summary": "IEEE 802.11 ad hoc on a shared Yans channel (standard, rates and receiver model set in network.wifi)",
    },
}


def resolve(cfg):
    """-> (name, registry entry) for the experiment, or raises SystemExit with the list of valid choices."""
    net = cfg["network"]
    name = net.get("technology", "wifi")
    if name not in TECHNOLOGIES:
        raise SystemExit(f"network.technology '{name}' is not registered. Available: {', '.join(sorted(TECHNOLOGIES))}. "
                         "See docs/CONTAINERS.md, 'Changing the networking technology'.")
    entry = TECHNOLOGIES[name]
    if entry["config_section"] not in net:
        raise SystemExit(f"network.technology is '{name}' but the YAML has no network.{entry['config_section']} section")
    return name, entry
