# ns-3 patches

Patches applied on top of the pinned ns-3 release (3.48) by `ns3/build_ns3.sh`.
Keep this list short; each entry must say how to remove the patch.

## 0001-wifi-mac-sync-link-address-on-SetAddress.patch

| | |
|---|---|
| Applies to | ns-3.48, `src/wifi/model/wifi-mac.cc` |
| Symptom | Any packet entering through a TapBridge in `UseLocal` mode onto a Wi-Fi device segfaults ns-3 (release), or fails `NS_ASSERT(linkId.has_value())` in `wifi-mac-queue-scheduler-impl.h` (debug). The stock `tap-wifi-virtual-machine` example crashes the same way. |
| Cause | TapBridge (`UseLocal`) calls `WifiNetDevice::SetAddress()` to give the device the Linux TAP's MAC. `WifiMac::SetAddress()` only updated `m_address`; the link's `FrameExchangeManager` kept its old address, so `GetLinkIdByAddress(Addr2)` found no link. |
| Change | `WifiMac::SetAddress()` also calls `feManager->SetAddress()` when the device has exactly one link. |
| Limits | Single-link devices only (802.11a/g/n/ac/ax, one PHY). Multi-link (802.11be MLO) devices are not handled; revisit if ever used with TapBridge. |
| Verified | `orchestrator/netns_link.sh` check: ping through ns-3 works; with ns-3 stopped, ping fails. Debug build: 0 asserts. |
| Remove when | An ns-3 release fixes this upstream. Delete the patch, rebuild, and re-run the netns_link check. Not yet reported upstream. |

## Pitfall when patching an existing build tree
After editing ns-3 sources, check that `build/lib/libns3.48-wifi-optimized.so`
has a new timestamp. A stale `--out=` in the CMake cache once sent rebuilt
libraries to a different directory, leaving an unpatched library in use.
