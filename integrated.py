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

import ppm_litex_wrapper as liteppm
import esc

class Blink(Module):
    def __init__(self, pads, sig):
        user_led = pads
        self.comb += user_led.eq(sig)


class TrivialRegister(Module, AutoCSR):
    def __init__(self):
        self.reg = CSRStatus(8, name="out_triv", reset=37)
        self.count = Signal(8)
        self.sync += self.count.eq(self.count+1)
        self.comb += self.reg.status.eq(self.count)




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



    channels=2
    ppm_loop_pin = Signal()
    pwm_pins = [Signal() for _ in range(channels)]
    #ppm_input_extension = [("ppm_input_pin", 0, Pins("GPIO:2"), IOStandard("LVCMOS33"))]
    #soc.platform.add_extension(ppm_input_extension)
    #ppm_input_pin = soc.platform.request("ppm_input_pin", 0)
    soc.submodules.ppm_input = liteppm.PPMinputRegister(ppm_loop_pin, servo_pads=pwm_pins, channels=channels)
    soc.add_csr("ppm_input")


    #ppm_output_extension = [("ppm_output_pin", 0, Pins("GPIO:3"), IOStandard("LVCMOS33"))]
    #soc.platform.add_extension(ppm_output_extension)
    #ppm_output_pin = soc.platform.request("ppm_output_pin", 0)
    soc.submodules.ppm_output = liteppm.PPMoutputRegister(ppm_loop_pin, channels=channels)
    soc.add_csr("ppm_output")


    #pwm_input_extension = [("pwm_input_pin", 0, Pins("GPIO:4"), IOStandard("LVCMOS33"))]
    #soc.platform.add_extension(pwm_input_extension)
    #pwm_input_pin = soc.platform.request("pwm_input_pin", 0)
    soc.submodules.pwm_input = liteppm.PWMinputRegister(ppm_loop_pin)
    soc.add_csr("pwm_input")


    user_led = soc.platform.request("user_led", 0)
    soc.submodules.blink = Blink(user_led, ppm_loop_pin)


    soc.submodules.triv_reg = TrivialRegister()
    soc.add_csr("triv_reg")



    builder = Builder(soc,
                    output_dir="build", csr_csv="build/csr.csv",
                    compile_software=False)
    vns = builder.build()
    soc.do_exit(vns)
    # tinyfpga_bx uses tinyprog for now, this should move over to dfu
    #add_dfu_suffix(os.path.join('build', 'gateware', 'tinyfpga_bx.bin'))


if __name__ == "__main__":
    main()
