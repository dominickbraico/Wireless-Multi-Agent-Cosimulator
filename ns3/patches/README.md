# ns-3 patches

Patches that `ns3/build_ns3.sh` applies on top of the pinned ns-3 release (3.48). Keep the list short. Each entry
states how to remove the patch.

## 0001-wifi-mac-sync-link-address-on-SetAddress.patch

| Item | Description |
|---|---|
| Applies to | ns-3 3.48, `src/wifi/model/wifi-mac.cc` |
| Symptom | A packet that enters a Wi-Fi device through a TapBridge in `UseLocal` mode crashes ns-3. |
| Cause | `WifiMac::SetAddress()` updates `m_address` but not the address held by the link's `FrameExchangeManager`. |
| Change | `WifiMac::SetAddress()` also calls `feManager->SetAddress()` when the device has exactly one link. |
| Verified by | `orchestrator/netns_link.sh`: ping works through ns-3 and fails with ns-3 stopped; a debug build raises no assert. |
| Remove when | An ns-3 release fixes this upstream. Delete the patch, rebuild and re-run `orchestrator/netns_link.sh`. |

Details of the symptom: a release build segfaults; a debug build fails `NS_ASSERT(linkId.has_value())` in
`wifi-mac-queue-scheduler-impl.h`. The stock `tap-wifi-virtual-machine` example crashes the same way.

Details of the cause: TapBridge in `UseLocal` mode calls `WifiNetDevice::SetAddress()` to give the device the MAC
address of the Linux TAP. The frame exchange manager keeps the old address, so `GetLinkIdByAddress(Addr2)` finds no
link.

## Rebuilding after a source change

After editing ns-3 sources, check that `build/lib/libns3.48-wifi-optimized.so` has a new timestamp. A stale `--out=`
value in the CMake cache can send rebuilt libraries to another directory and leave the unpatched library in use.

## Limitations

- The patch handles single-link devices only (802.11a/g/n/ac/ax, one PHY). Multi-link (802.11be) devices are not
  handled.
- The defect is not reported upstream.
