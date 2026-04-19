# Copyright 2026 Vyges.
# SPDX-License-Identifier: Apache-2.0
#
# cocotb testbench for rv_plic_lite
#
# Exercises the TL-UL slave response path of rv_plic_lite. The key
# regression is BACK_TO_BACK WRITES: the original combinational
# (0-cycle) d_valid version dropped the second write on a tight
# write stream to this slave. The registered (1-cycle) d_valid
# version handles it cleanly.
#
# Run with:
#   cd tb/cocotb && make
#
# Requires: cocotb >= 1.8, verilator >= 5.0 (--timing support)

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ClockCycles, Timer

# ─── TL-UL opcodes ──────────────────────────────────────────────────────────
TL_OP_PUT_FULL = 0x0
TL_OP_GET      = 0x4
TL_D_ACCESSACK     = 0x0
TL_D_ACCESSACKDATA = 0x1

# ─── PLIC register map ──────────────────────────────────────────────────────
ADDR_PRIO_BASE      = 0x000  # PRIO[i] = +i*4
ADDR_PRIO_END       = 0x07C  # inclusive
ADDR_PENDING        = 0x080
ADDR_ENABLE         = 0x100
ADDR_THRESHOLD      = 0x200
ADDR_CLAIM_COMPLETE = 0x204

CLK_PERIOD_NS = 10  # 100 MHz


# ─── Helpers ────────────────────────────────────────────────────────────────
async def reset_dut(dut, cycles=4):
    """Hold reset for N cycles with all inputs idle."""
    dut.rst_ni.value       = 0
    dut.a_valid_i.value    = 0
    dut.a_opcode_i.value   = 0
    dut.a_size_i.value     = 2
    dut.a_source_i.value   = 0
    dut.a_address_i.value  = 0
    dut.a_mask_i.value     = 0xF
    dut.a_data_i.value     = 0
    dut.intr_src_i.value   = 0
    await ClockCycles(dut.clk_i, cycles)
    dut.rst_ni.value = 1
    await ClockCycles(dut.clk_i, 2)


async def drive_a_channel(dut, *, op, addr, data=0, source=1):
    """Hold a_valid for one cycle with the given payload."""
    dut.a_valid_i.value   = 1
    dut.a_opcode_i.value  = op
    dut.a_size_i.value    = 2   # 4 bytes
    dut.a_source_i.value  = source
    dut.a_address_i.value = addr
    dut.a_mask_i.value    = 0xF
    dut.a_data_i.value    = data
    await RisingEdge(dut.clk_i)
    dut.a_valid_i.value = 0


async def wait_d_valid(dut, timeout_cycles=20):
    """Wait until d_valid rises. Returns the cycle delay after a_valid."""
    for i in range(timeout_cycles):
        await ReadOnly()
        if int(dut.d_valid_o.value) == 1:
            return i
        await RisingEdge(dut.clk_i)
    raise TimeoutError(
        f"d_valid did not assert within {timeout_cycles} cycles"
    )


async def tl_write(dut, addr, data, source=1):
    """Issue one TL-UL PutFullData and capture the response."""
    await drive_a_channel(dut, op=TL_OP_PUT_FULL,
                          addr=addr, data=data, source=source)
    delay = await wait_d_valid(dut)
    await ReadOnly()
    opcode = int(dut.d_opcode_o.value)
    rsp_source = int(dut.d_source_o.value)
    err = int(dut.d_error_o.value)
    await RisingEdge(dut.clk_i)
    return {
        "delay": delay, "opcode": opcode,
        "source": rsp_source, "error": err,
    }


async def tl_read(dut, addr, source=1):
    """Issue one TL-UL Get and capture the response."""
    await drive_a_channel(dut, op=TL_OP_GET,
                          addr=addr, data=0, source=source)
    delay = await wait_d_valid(dut)
    await ReadOnly()
    data = int(dut.d_data_o.value)
    opcode = int(dut.d_opcode_o.value)
    rsp_source = int(dut.d_source_o.value)
    err = int(dut.d_error_o.value)
    await RisingEdge(dut.clk_i)
    return {
        "delay": delay, "data": data, "opcode": opcode,
        "source": rsp_source, "error": err,
    }


# ─── Tests ──────────────────────────────────────────────────────────────────
@cocotb.test()
async def test_reset_defaults(dut):
    """After reset, PRIO, ENABLE, THRESHOLD should all read 0."""
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    await reset_dut(dut)

    for i in range(32):
        r = await tl_read(dut, ADDR_PRIO_BASE + i * 4)
        assert r["data"] == 0, (
            f"PRIO[{i}] reset default = 0x{r['data']:X}, expected 0x0"
        )
    r = await tl_read(dut, ADDR_ENABLE)
    assert r["data"] == 0, f"ENABLE reset = 0x{r['data']:X}, expected 0x0"
    r = await tl_read(dut, ADDR_THRESHOLD)
    assert r["data"] == 0, f"THRESHOLD reset = 0x{r['data']:X}, expected 0x0"


@cocotb.test()
async def test_prio_write_readback(dut):
    """Each PRIO[i] must accept a 3-bit write and read back the same value."""
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    await reset_dut(dut)

    for i in range(32):
        await tl_write(dut, ADDR_PRIO_BASE + i * 4, (i & 0x7))
    for i in range(32):
        r = await tl_read(dut, ADDR_PRIO_BASE + i * 4)
        assert r["data"] == (i & 0x7), (
            f"PRIO[{i}] read 0x{r['data']:X}, expected 0x{i & 0x7:X}"
        )


@cocotb.test()
async def test_response_is_registered_not_combinational(dut):
    """d_valid must not follow a_valid combinationally. Under the old
    combinational-d_valid RTL d_valid rose the same delta a_valid rose
    (before any clock edge) -- which caused the back-to-back write hang
    on the FPGA (Item 14). This test drives a_valid high WITHOUT a clock
    edge and asserts d_valid stays 0 until the next edge samples it."""
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    await reset_dut(dut)

    # Settle on a known edge, then drive inputs between edges
    await RisingEdge(dut.clk_i)
    dut.a_valid_i.value   = 1
    dut.a_opcode_i.value  = TL_OP_PUT_FULL
    dut.a_size_i.value    = 2
    dut.a_source_i.value  = 1
    dut.a_address_i.value = ADDR_PRIO_BASE + 4
    dut.a_mask_i.value    = 0xF
    dut.a_data_i.value    = 0x5
    # Wait a small delta for any combinational propagation, but NOT a clock edge
    await Timer(1, units="ns")
    await ReadOnly()
    d_valid_before_edge = int(dut.d_valid_o.value)

    # Now cross the clock edge that samples a_valid
    await RisingEdge(dut.clk_i)
    await ReadOnly()
    d_valid_after_edge = int(dut.d_valid_o.value)
    await RisingEdge(dut.clk_i)
    dut.a_valid_i.value = 0

    assert d_valid_before_edge == 0, (
        f"d_valid = {d_valid_before_edge} BEFORE the sampling clock edge -- "
        "response path is combinational (would regress Fix on Item 14)"
    )
    assert d_valid_after_edge == 1, (
        f"d_valid = {d_valid_after_edge} AFTER the sampling clock edge -- "
        "registered response did not propagate"
    )


@cocotb.test()
async def test_back_to_back_writes(dut):
    """Back-to-back writes to the PLIC must ALL land -- this is the
    exact failure mode that hung CPU firmware's plic_init() for-loop
    on the Arty A7 bring-up (Item 14 in todo.md). Pre-fix, the second
    write dropped."""
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    await reset_dut(dut)

    # Drive PRIO[1]..PRIO[14] = 1 back-to-back, mirroring the firmware's
    # for (i = 1; i <= 14; i++) loop exactly.
    for i in range(1, 15):
        await tl_write(dut, ADDR_PRIO_BASE + i * 4, 0x1)
    # ENABLE mask matching plic_init()
    await tl_write(dut, ADDR_ENABLE, 0x3E)
    await tl_write(dut, ADDR_THRESHOLD, 0x0)

    # Verify every write landed
    for i in range(1, 15):
        r = await tl_read(dut, ADDR_PRIO_BASE + i * 4)
        assert r["data"] == 0x1, (
            f"PRIO[{i}] = 0x{r['data']:X}, expected 0x1 "
            "(back-to-back write lost)"
        )
    r = await tl_read(dut, ADDR_ENABLE)
    assert r["data"] == 0x3E, (
        f"ENABLE = 0x{r['data']:X}, expected 0x3E"
    )
    r = await tl_read(dut, ADDR_THRESHOLD)
    assert r["data"] == 0x0, (
        f"THRESHOLD = 0x{r['data']:X}, expected 0x0"
    )
    # PRIO[0] and PRIO[15..31] should still be 0
    r = await tl_read(dut, ADDR_PRIO_BASE)
    assert r["data"] == 0, f"PRIO[0] = 0x{r['data']:X}, expected 0"
    for i in range(15, 32):
        r = await tl_read(dut, ADDR_PRIO_BASE + i * 4)
        assert r["data"] == 0, (
            f"PRIO[{i}] = 0x{r['data']:X}, expected 0 (never written)"
        )


@cocotb.test()
async def test_source_routing(dut):
    """d_source must mirror a_source of the request -- catches the same
    class of bug as Fix I in spi-host-lite (unused rsp_source_q)."""
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    await reset_dut(dut)

    for src in (0x01, 0x05, 0x7F, 0xFF):
        r = await tl_write(dut, ADDR_PRIO_BASE + 4, 0x2, source=src)
        assert r["source"] == src, (
            f"write: d_source = 0x{r['source']:X}, "
            f"expected a_source 0x{src:X}"
        )
        r = await tl_read(dut, ADDR_PRIO_BASE + 4, source=src)
        assert r["source"] == src, (
            f"read: d_source = 0x{r['source']:X}, "
            f"expected a_source 0x{src:X}"
        )


@cocotb.test()
async def test_irq_aggregation(dut):
    """With PRIO[i]>0, ENABLE bit set, and intr_src[i]=1, irq_o must rise."""
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    await reset_dut(dut)

    await tl_write(dut, ADDR_PRIO_BASE + 4, 0x3)  # PRIO[1] = 3
    await tl_write(dut, ADDR_ENABLE, 0x2)         # enable source 1
    await tl_write(dut, ADDR_THRESHOLD, 0x0)

    dut.intr_src_i.value = 0x2  # source 1 fires
    await ClockCycles(dut.clk_i, 3)
    assert int(dut.irq_o.value) == 1, (
        "irq_o = 0 -- expected rise when PRIO+ENABLE+src all set"
    )

    dut.intr_src_i.value = 0x0
    await ClockCycles(dut.clk_i, 3)
    assert int(dut.irq_o.value) == 0, (
        "irq_o stuck high after deasserting intr_src"
    )
