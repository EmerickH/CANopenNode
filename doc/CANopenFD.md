CANopen FD
==========

CANopenNode can be built for CANopen FD (CiA 1301) instead of classic CANopen (CiA 301). Both can not share the same
bus, so the choice is made at compile time with `CO_CONFIG_FD`, typically in `CO_driver_target.h`:

```c
#define CO_CONFIG_FD (CO_CONFIG_FD_ENABLE)
```

Status
------

| Area | Status |
| --- | --- |
| CAN FD frames up to 64 bytes in the driver interface | Done: `CO_CAN_MAX_DLEN`, `CO_CANfd_roundLength()`, `CO_CANfd_lengthToDlc()`, `CO_CANfd_dlcToLength()`, `CO_CANsetDataBitRate()` |
| PDO up to 64 bytes | Done: `CO_PDO_MAX_SIZE` and `CO_PDO_MAX_MAPPED_ENTRIES` default to 64, TPDO padded with zeros to the next valid CAN FD length, RPDO accepts the padded frame. Bitwise mapping is limited to 8 bytes. |
| USDO server | Local unicast, multicast and broadcast; expedited and segmented download and upload; abort. Bulk transfer, remote access and concurrent sessions are not implemented yet. |
| SDO server and client | Not used in CANopen FD builds. SDO client and gateway (CiA 309) can not be enabled yet. |
| Node guarding | Not available (no remote frames). |
| USDO client, EMCY FD (20-byte), active error history 1031h, LSS FD (CiA 1305), network-ID | Not implemented yet. |
| `CO_MULTIPLE_OD` | Not supported with CANopen FD yet. |

Driver requirements
-------------------

- `CO_CAN_MAX_DLEN` must be 64 and `CO_CANtx_t.data` must hold 64 bytes.
- Frame length inside the stack is always in bytes. `CO_CANsend()` must use `CO_CANtx_t.DLC` as the frame length,
  because the USDO server sets it before each transmission. The driver converts it to the CAN FD DLC code.
- `CO_CANrxMsg_readDLC()` returns the length in bytes (0 to 64).
- `CO_CANsetDataBitRate()` sets the data phase bit rate; nominal bit rate is set by `CO_CANmodule_init()`.
- Remote frames are not used, `CO_CANrxBufferInit()` and `CO_CANtxBufferInit()` may reject `rtr == true`.

USDO frame layout
-----------------

The USDO byte layout is taken from public descriptions of CiA 1301 (command specifiers, field order and frame lengths)
and must be verified against the CiA 1301 document before claiming conformance. All positions are defined by the
`CO_USDO_POS_xx` macros in `301/CO_USDOserver.h`, so they can be corrected in one place.

Tests
-----

`test/` contains host unit tests with an in-memory CAN driver. Run `make -C test` to build and run them for the
CANopen FD and the classic CAN configuration.
