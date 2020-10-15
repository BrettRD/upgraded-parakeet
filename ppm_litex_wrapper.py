from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
from litex.soc.interconnect.csr import AutoCSR, CSRStatus, CSRStorage
import ppm

# wrap the ppm input device in CSR decoration and pad assignments
class PPMinputRegister(Module, AutoCSR):
    def __init__(
            self,
            ppm_pad,
            servo_pads=None,
            ppm_dev=None,
            channels=8,
            timeout=4000e3,
            clk_period=1e9/16e6,
            resolution=1e3,
            max_width=2000e3,
            csr_width=8,
            alignment=2,
        ):
        # instantiate the device if it isn't provided
        if ppm_dev == None:
            self.submodules.ppm_dev = ppm.PPMinput(
                channels=channels,
                timeout=timeout,
                clk_period=clk_period,
                resolution=resolution,
                max_width=max_width,
            )
        else:
            self.ppm_dev = ppm_dev
            channels = self.ppm_dev.channels

        # assign an input pin to the PPM device
        self.comb += self.ppm_dev.ppm.eq(ppm_pad)

        self.widths = []
        for chan in range(channels):
            tmp = CSRStatus(csr_width, name='channel_'+str(chan))
            self.widths.append(tmp)
            setattr(self, f"tmp{chan}", tmp)

        for chan in range(channels):
            ppm_size = self.ppm_dev.widths[chan].nbits
            if csr_width >= ppm_size:
                self.comb += self.widths[chan].status[:ppm_size].eq(self.ppm_dev.widths[chan])
            else:
                self.comb += self.widths[chan].status.eq(self.ppm_dev.widths[chan][alignment:csr_width+alignment])

            if servo_pads != None:
                assert(len(servo_pads) == channels)
                self.comb += servo_pads[chan].eq(self.ppm_dev.pwm[chan])




# wrap the ppm output device in CSR decoration and pad assignments
class PPMoutputRegister(Module, AutoCSR):
    def __init__(
            self,
            ppm_pad,
            servo_pads=None,
            ppm_dev=None,
            channels=8,
            frequency=50,
            clk_period=1e9/16e6,
            resolution=1e3,
            pulse_width=300e3,
            max_width=2000e3,
            min_width=1000e3,
            csr_width=8,
            alignment=2,
        ):
        if ppm_dev == None:
            self.submodules.ppm_dev = ppm.PPMoutput(
                channels=channels,
                frequency=frequency,
                clk_period=clk_period,
                resolution=resolution,
                pulse_width=pulse_width,
                max_width=max_width,
                min_width=min_width
            )
        else:
            self.ppm_dev = ppm_dev
            channels = self.ppm_dev.channels

        self.widths = []
        for chan in range(channels):
            tmp = CSRStorage(csr_width, name='channel_'+str(chan))
            self.widths.append(tmp)
            setattr(self, f"tmp{chan}", tmp)

        # assign an input pin to the PPM device
        self.comb += ppm_pad.eq(self.ppm_dev.ppm)
        for chan in range(channels):
            ppm_size = self.ppm_dev.widths[chan].nbits
            if csr_width >= ppm_size:
                self.comb += self.ppm_dev.widths[chan].eq(self.widths[chan].storage[:ppm_size])
            else:
                self.comb += self.ppm_dev.widths[chan][alignment:csr_width+alignment].eq(self.widths[chan].storage)
                self.comb += self.ppm_dev.widths[chan][:alignment].eq(0)

            if servo_pads != None:
                self.comb += servo_pads[chan].eq(self.ppm_dev.pwm[chan])



class PWMinputRegister(Module, AutoCSR):
    def __init__(
            self,
            pwm_pad,
            pwm_dev=None,
            clk_period=1e9/16e6,
            resolution=1e3,
            max_width=2000e3,
            csr_width=8,
            alignment=2,
        ):
        if pwm_dev == None:
            self.submodules.pwm_dev = ppm.PWMinput(
                clk_period=clk_period,
                resolution=resolution,
                max_width=max_width,
            )
        else:
            self.pwm_dev = pwm_dev

        # assign an input pin to the PPM device
        self.comb += self.pwm_dev.pwm.eq(pwm_pad)

        self.width = CSRStatus(csr_width, name='width')

        pwm_size = self.pwm_dev.width.nbits
        if csr_width >= pwm_size:
            self.comb += self.width.status[:pwm_size].eq(self.pwm_dev.width)
        else:
            self.comb += self.width.status.eq(self.pwm_dev.width[alignment:csr_width+alignment])
