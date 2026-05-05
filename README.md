# NRF2401 Template

Simple nRF24 starter repo with working transmitter/receiver templates.

## Folders

- `examples/NRF_TRANSMITTER_CODE/`
- `examples/NRF_RECEIVER_CODE/`
- `examples/DUAL_NRF_SAME_BUS_TEST/`

## Why This Repo Exists

When I start a new radio link, I use these examples to quickly verify:

- wiring and pin mapping
- channel/address matching
- payload compatibility
- basic delivery health (ACK behavior)

## Bring-Up Basics

1. Stable 3.3V power + local decoupling near the module.
2. Match channel/rate/address/CRC/payload settings on both nodes.
3. Confirm `radio.isChipConnected()` before debugging protocol logic.
4. Add receiver-side failsafe before connecting this to motors/actuators.
