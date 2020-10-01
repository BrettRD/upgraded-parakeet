#!/usr/bin/env python3

# Disable pylint's E1101, which breaks completely on migen
#pylint:disable=E1101

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.generic_platform import Pins, IOStandard
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.interconnect.csr import AutoCSR, CSRStatus, CSRStorage

from tinyfpga_bx_soc_target import BaseSoC, add_dfu_suffix

from valentyusb.usbcore import io as usbio
from valentyusb.usbcore.cpu import dummyusb

import argparse

import ppm

class Blink(Module):
    def __init__(self, pads):
        count_max = int(16e6)
        user_led = pads
        blink = Signal()
        counter = Signal(max=count_max)
        self.sync += If(counter >= count_max,
            counter.eq(0),
            blink.eq(blink + 1)
        ).Else(
            counter.eq(counter + 1)
        )
        self.comb += user_led.eq(blink)


class TrivialRegister(Module, AutoCSR):
    def __init__(self):
        self.reg = CSRStorage(8, name="out_triv")


# wrap the ppm input device in CSR decoration and pad assignments
class PPMinputRegister(Module, AutoCSR):
    def __init__(self, ppm_pad, servo_pads=None, channels=8, timeout=5e6, default_clk_period=1e9/16e6):
        self.input = Signal()

        self.output = []
        for chan in range(channels):
            tmp = CSRStatus(8, name='channel_'+str(chan))
            self.output.append(tmp)
            setattr(self, f"tmp{chan}", tmp)

        ppm_dev = ppm.PPMinput(channels=channels, timeout=timeout, default_clk_period=default_clk_period)
        # assign an input pin to the PPM device
        self.comb += ppm_dev.ppm.eq(ppm_pad)
        for chan in range(channels):
            self.comb += self.output[chan].status.eq(ppm_dev.widths[chan])
            if servo_pads != None:
                self.comb += servo_pads[chan].eq(ppm_dev.pwm[chan])






def main():
    parser = argparse.ArgumentParser(
        description="Build TinyFPGA_BX Main Gateware")
    parser.add_argument(
        "--seed", default=0, help="seed to use in nextpnr"
    )
    parser.add_argument(
        "--placer", default="heap", choices=["sa", "heap"], help="which placer to use in nextpnr"
    )
    args = parser.parse_args()

    soc = BaseSoC(pnr_seed=args.seed, pnr_placer=args.placer, usb_bridge=True)


    user_led = soc.platform.request("user_led", 0)

    ppm_input_extension = [("ppm_input", 0, Pins("GPIO:2"), IOStandard("LVCMOS33"))]
    soc.platform.add_extension(ppm_input_extension)
    ppm_input_pin = soc.platform.request("ppm_input", 0)



    soc.submodules.blink = Blink(user_led)
    soc.submodules.triv_reg = TrivialRegister()
    soc.submodules.ppm_doodad = PPMinputRegister(ppm_input_pin, channels=8)

    soc.add_csr("triv_reg")
    soc.add_csr("ppm_doodad")

    builder = Builder(soc,
                    output_dir="build", csr_csv="build/csr.csv",
                    compile_software=False)
    vns = builder.build()
    soc.do_exit(vns)
    # tinyfpga_bx uses tinyprog for now, this should move over to dfu
    #add_dfu_suffix(os.path.join('build', 'gateware', 'tinyfpga_bx.bin'))


if __name__ == "__main__":
    main()
