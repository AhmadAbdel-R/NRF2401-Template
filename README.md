# NRF2401 Template

Starter templates for nRF24L01+ links (transmitter, receiver, and dual-radio test).

## Project Layout

- `examples/NRF_TRANSMITTER_CODE/` -> TX example
- `examples/NRF_RECEIVER_CODE/` -> RX example
- `examples/DUAL_NRF_SAME_BUS_TEST/` -> dual-radio bus test

## What This Repo Solves

- Clean baseline for bring-up of nRF24 modules
- Known-good RF24 configuration matching between TX and RX
- Simple packet transport pattern for control links

## Quick Bring-Up Checklist

1. Use stable 3.3V power with local decoupling at the radio.
2. Match TX/RX settings exactly:
   - channel
   - data rate
   - address width
   - CRC length
   - payload size
3. Verify wiring for `CE`, `CSN`, `MOSI`, `MISO`, `SCK`.
4. Confirm `radio.isChipConnected()` returns true.

## Packet Example

The default examples use a compact control packet with:

- throttle, yaw, pitch, roll
- flags byte
- sequence counter

## Safety Note

Always implement failsafe behavior on receiver side (disarm/zero throttle on link timeout).
