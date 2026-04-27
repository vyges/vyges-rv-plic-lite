# vyges-rv-plic-lite

Lightweight RISC-V Platform-Level Interrupt Controller with a native TL-UL slave interface.
Vyges-original IP, no OpenTitan RACL or lifecycle dependencies.

## Features

- Up to 32 external interrupt sources (configurable via `NUM_SOURCES`)
- 3-bit priority per source (0 = disabled, 7 = highest)
- Priority threshold register
- Interrupt pending (RO) and enable (RW) bitmaps
- Claim/complete mechanism per the RISC-V PLIC specification
- Single `irq_o` output for one hart target
- Single-cycle TL-UL response (always ready)

## Register Map

| Offset        | Name             | Access | Description                                       |
|---------------|------------------|--------|---------------------------------------------------|
| 0x000 - 0x07C | PRIO[0..31]     | RW     | 3-bit priority for each source (one word each)    |
| 0x080         | PENDING          | RO     | 32-bit pending interrupt bitmap                   |
| 0x100         | ENABLE           | RW     | 32-bit interrupt enable bitmap                    |
| 0x200         | THRESHOLD        | RW     | 3-bit priority threshold                          |
| 0x204         | CLAIM_COMPLETE   | RW     | Read = claim (returns source ID), Write = complete |

## Parameters

| Parameter    | Default | Range | Description                      |
|--------------|---------|-------|----------------------------------|
| NUM_SOURCES  | 32      | 1-32  | Number of external interrupt sources |

## Claim/Complete Protocol

1. CPU reads `CLAIM_COMPLETE` (0x204) -- returns the ID of the highest-priority pending and enabled source above the threshold. The source is marked as "claimed" and will not re-trigger `irq_o` until completed.
2. CPU services the interrupt.
3. CPU writes the source ID back to `CLAIM_COMPLETE` (0x204) to signal completion. The claimed bit is cleared, allowing the source to pend again if still asserted.

Reading `CLAIM_COMPLETE` when no interrupt is pending returns 0.

## Dependencies

- **opentitan-tlul** -- provides `tlul_pkg` (TL-UL type definitions and opcodes).

## License

Apache-2.0. Copyright 2026 Vyges Inc.
