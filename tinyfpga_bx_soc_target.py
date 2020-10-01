#!/usr/bin/env python3

# SoC target for TinyFPGA_BX modelled closely on the Fomu
# derived from LiteX-Boards litex_boards/targets/fomu.py.
#
# Copyright (c) 2019 Sean Cross <sean@xobs.io>
# Copyright (c) 2018 David Shah <dave@ds0.me>
# Copyright (c) 2020 Brett Downing <brettrd@brettrd.com>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder, builder_argdict, builder_args
from litex.soc.integration.soc_core import soc_core_argdict, soc_core_args
from litex.soc.integration.doc import AutoDoc

from valentyusb.usbcore import io as usbio
from valentyusb.usbcore.cpu import dummyusb, epfifo, eptri

import os, shutil, subprocess

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module, AutoDoc):
    """TinyFPGA_BX Clock Resource Generator

    TinyFPGA_BX is a USB device, which means it must have a 12 MHz clock.  Valentyusb
    oversamples the clock by 4x, which drives the requirement for a 48 MHz clock.
    The ICE40LP8k is a relatively low speed grade of FPGA that is incapable of
    running the entire design at 48 MHz, so the majority of the logic is placed
    in the 16 MHz domain while only critical USB logic is placed in the fast
    48 MHz domain.

    TinyFPGA_BX has a 16 MHz crystal on it, which provides the raw clock input.
    This signal is fed through the ICE40 PLL in order to multiply up to a 48 MHz
    signal, and divides 48MHz down to 12MHz for the USB core using a simple counter.
    This clock architecture follows the TinyFPGA_BX bootloader.
    The Fomu uses the PLL to divide the 48MHz oscillator to 12MHz system speed.
    Unfortunately, the iCE40LP8K-CM81 on the TinyFPGA_BX only has one PLL.
    
    The following clock domains are available on this design:

    +---------+------------+---------------------------------+
    | Name    | Frequency  | Description                     |
    +=========+============+=================================+
    | usb_48  | 48 MHz     | Raw USB signals and pulse logic |
    +---------+------------+---------------------------------+
    | usb_12  | 12 MHz     | USB control logic               |
    +---------+------------+---------------------------------+
    | sys     | 16 MHz     | System CPU and wishbone bus     |
    +---------+------------+---------------------------------+
    """
    def __init__(self, platform):
        clk16_raw = platform.request("clk16")
        clk48 = Signal()
        clk12 = Signal()

        reset_delay = Signal(12, reset=4095)
        self.clock_domains.cd_por = ClockDomain()
        self.reset = Signal()

        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_usb_12 = ClockDomain()
        self.clock_domains.cd_usb_48 = ClockDomain()

        platform.add_period_constraint(self.cd_usb_48.clk, 1e9/48e6)
        platform.add_period_constraint(self.cd_usb_12.clk, 1e9/12e6)
        platform.add_period_constraint(self.cd_sys.clk, 1e9/16e6)
        platform.add_period_constraint(clk16_raw, 1e9/16e6)

        # POR reset logic- POR generated from sys clk, POR logic feeds sys clk
        # reset.
        self.comb += [
            self.cd_por.clk.eq(self.cd_sys.clk),
            self.cd_sys.rst.eq(reset_delay != 0),
            self.cd_usb_48.rst.eq(reset_delay != 0),
            self.cd_usb_12.rst.eq(reset_delay != 0),
        ]

        # assign source signals to the clock domains
        self.comb += [
            self.cd_sys.clk.eq(clk16_raw),
            self.cd_usb_48.clk.eq(clk48),
            self.cd_usb_12.clk.eq(clk12),
        ]

        # divide down a USB 12MHz signal inside the USB 48MHz domain
        clk12_counter = Signal(2)
        self.sync.usb_48 += clk12_counter.eq(clk12_counter + 1)
        self.comb += clk12.eq(clk12_counter[1])

        # use the PLL to spin up a 48MHz clock from the 16MHz crystal
        self.specials += Instance(
            "SB_PLL40_CORE",
            # Parameters
            p_DIVR = 0,
            p_DIVF = 47,
            p_DIVQ = 4,
            p_FILTER_RANGE = 1,
            p_FEEDBACK_PATH = "SIMPLE",
            p_DELAY_ADJUSTMENT_MODE_FEEDBACK = "FIXED",
            p_FDA_FEEDBACK = 0,
            p_DELAY_ADJUSTMENT_MODE_RELATIVE = "FIXED",
            p_FDA_RELATIVE = 0,
            p_SHIFTREG_DIV_MODE = 0,
            p_PLLOUT_SELECT = "GENCLK",
            p_ENABLE_ICEGATE = 0,
            # IO
            i_REFERENCECLK = clk16_raw,
            o_PLLOUTCORE = clk48,
            # o_PLLOUTGLOBAL = clk48,
            #i_EXTFEEDBACK,
            #i_DYNAMICDELAY,
            #o_LOCK,
            i_BYPASS = 0,
            i_RESETB = 1,
            #i_LATCHINPUTVALUE,
            #o_SDO,
            #i_SDI,
        )


        self.sync.por += \
            If(reset_delay != 0,
                reset_delay.eq(reset_delay - 1)
            )
        self.specials += AsyncResetSynchronizer(self.cd_por, self.reset)


# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    """A SoC on Fomu, optionally with a softcore CPU"""

    # Create a default CSR map to prevent values from getting reassigned.
    # This increases consistency across litex versions.
    SoCCore.csr_map = {
        "ctrl":           0,  # provided by default (optional)
        "crg":            1,  # user
        "uart_phy":       2,  # provided by default (optional)
        "uart":           3,  # provided by default (optional)
        "identifier_mem": 4,  # provided by default (optional)
        "timer0":         5,  # provided by default (optional)
        "cpu_or_bridge":  8,
        "usb":            9,
        "picorvspi":      10,
        "touch":          11,
        "reboot":         12,
        "rgb":            13,
        "version":        14,
    }

    # Statically-define the memory map, to prevent it from shifting across
    # various litex versions.
    SoCCore.mem_map = {
        "rom":      0x00000000,  # (default shadow @0x80000000)
        "sram":     0x10000000,  # (default shadow @0xa0000000)
        "spiflash": 0x20000000,  # (default shadow @0xa0000000)
        "main_ram": 0x40000000,  # (default shadow @0xc0000000)
        "csr":      0xe0000000,  # (default shadow @0x60000000)
    }

    def __init__(self,
        pnr_placer="heap", pnr_seed=0, usb_core="dummyusb", usb_bridge=False,
        use_dsp=True, **kwargs):
        """Create a basic SoC for the TinyFPGA_BX.

        Create a basic SoC for TinyFPGA_BX, including a 48 MHz and 12 MHz clock
        domain called `usb_48` and `usb_12`.  The `sys` frequency will
        run at 16 MHz.

        The USB core will optionally have a bridge to the Wishbone bus.

        Args:
            pnr_placer (str): Which placer to use in nextpnr
            pnr_seed (int): Which seed to use in nextpnr
            usb_core (str): The name of the USB core to use, if any: dummyusb, epfifo, eptri
            usb_bridge (bool): Whether to include a USB-to-Wishbone bridge
        Raises:
            ValueError: If the `usb_core` is unrecognized
        Returns:
            Newly-constructed SoC
        """
        from litex_boards.platforms.tinyfpga_bx import Platform
        platform = Platform()

        if "cpu_type" not in kwargs:
            kwargs["cpu_type"] = None
            kwargs["cpu_variant"] = None

        clk_freq = int(12e6)

        if "with_uart" not in kwargs:
            kwargs["with_uart"] = False

        if "with_ctrl" not in kwargs:
            kwargs["with_ctrl"] = False

        kwargs["integrated_sram_size"] = 0
        SoCCore.__init__(self, platform, clk_freq, **kwargs)

        self.submodules.crg = _CRG(platform)

        if usb_core is not None:
            # Add USB pads.  We use DummyUsb, which simply enumerates as a USB
            # device.  Then all interaction is done via the wishbone bridge.
            usb_pads = platform.request("usb")
            usb_iobuf = usbio.IoBuf(usb_pads.d_p, usb_pads.d_n, usb_pads.pullup)
            if usb_core == "dummyusb":
                self.submodules.usb = dummyusb.DummyUsb(usb_iobuf, debug=usb_bridge)
            elif usb_core == "epfifo":
                self.submodules.usb = epfifo.PerEndpointFifo(usb_iobuf, debug=usb_bridge)
            elif usb_core == "eptri":
                self.submodules.usb = eptri.TriEndpointInterface(usb_iobuf, debug=usb_bridge)
            else:
                raise ValueError("unrecognized usb_core: {}".format(usb_core))
            if usb_bridge:
                self.add_wb_master(self.usb.debug_bridge.wishbone)

        # Override default LiteX's yosys/build templates
        assert hasattr(platform.toolchain, "yosys_template")
        assert hasattr(platform.toolchain, "build_template")
        platform.toolchain.yosys_template = [
            "{read_files}",
            "attrmap -tocase keep -imap keep=\"true\" keep=1 -imap keep=\"false\" keep=0 -remove keep=0",
            "synth_ice40 -json {build_name}.json -top {build_name}",
        ]
        platform.toolchain.build_template = [
            "yosys -q -l {build_name}.rpt {build_name}.ys",
            "nextpnr-ice40 --json {build_name}.json --pcf {build_name}.pcf --asc {build_name}.txt \
            --pre-pack {build_name}_pre_pack.py --{architecture} --package {package}",
            "icepack {build_name}.txt {build_name}.bin"
        ]

        # Add "-relut -dffe_min_ce_use 4" to the synth_ice40 command.
        # The "-reult" adds an additional LUT pass to pack more stuff in,
        # and the "-dffe_min_ce_use 4" flag prevents Yosys from generating a
        # Clock Enable signal for a LUT that has fewer than 4 flip-flops.
        # This increases density, and lets us use the FPGA more efficiently.
        platform.toolchain.yosys_template[2] += " -relut -abc2 -dffe_min_ce_use 4 -relut"
        if use_dsp:
            platform.toolchain.yosys_template[2] += " -dsp"

        # Disable final deep-sleep power down so firmware words are loaded
        # onto softcore's address bus.
        platform.toolchain.build_template[2] = "icepack -s {build_name}.txt {build_name}.bin"

        # Allow us to set the nextpnr seed
        platform.toolchain.build_template[1] += " --seed " + str(pnr_seed)

        if pnr_placer is not None:
            platform.toolchain.build_template[1] += " --placer {}".format(pnr_placer)


class USBSoC(BaseSoC):
    """A SoC for TinyFPGA_BX with interrupts for a softcore CPU"""

    interrupt_map = {
        "usb": 3,
    }
    interrupt_map.update(SoCCore.interrupt_map)


# Build --------------------------------------------------------------------------------------------

def add_dfu_suffix(fn):
    fn_base, _ext = os.path.splitext(fn)
    fn_dfu = fn_base + '.dfu'
    shutil.copyfile(fn, fn_dfu)
    subprocess.check_call(['dfu-suffix', '--pid', '6130', '--vid', '1d50', '--add', fn_dfu])

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on TinyFPGA_BX")
    parser.add_argument("--build",  action="store_true", help="Build bitstream")
    parser.add_argument("--seed",   default=0, help="Seed to use in Nextpnr")
    parser.add_argument("--placer", default="heap", choices=["sa", "heap"], help="Which placer to use in Nextpnr")
    builder_args(parser)
    soc_core_args(parser)
    args = parser.parse_args()

    soc = BaseSoC(pnr_placer=args.placer, pnr_seed=args.seed, debug=True, **soc_core_argdict(args))
    builder = Builder(soc, **builder_argdict(args))
    builder.build(run=args.build)

if __name__ == "__main__":
    main()
