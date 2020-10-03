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
            default_clk_period=1e9/16e6,
            resolution=1e3,
            max_width=2000e3,
            csr_width=8,
            alignment=0

        ):
        # instantiate the device if it isn't provided
        if ppm_dev == None:
            ppm_dev = ppm.PPMinput(
                channels=channels,
                timeout=timeout,
                default_clk_period=default_clk_period,
                resolution=resolution
            )
        else:
            channels = ppm_dev.channels

        # assign an input pin to the PPM device
        self.comb += ppm_dev.ppm.eq(ppm_pad)

        self.output = []
        for chan in range(channels):
            tmp = CSRStatus(csr_width, name='channel_'+str(chan))
            self.output.append(tmp)
            setattr(self, f"tmp{chan}", tmp)

        for chan in range(channels):
            ppm_size = ppm_dev.widths[chan].nbits
            if csr_width >= ppm_size:
                self.comb += self.output[chan].status[:ppm_size].eq(ppm_dev.widths[chan])
            else:
                self.comb += self.output[chan].status.eq(ppm_dev.widths[chan][alignment:csr_width+alignment])

            if servo_pads != None:
                assert(len(servo_pads) == channels)
                self.comb += servo_pads[chan].eq(ppm_dev.pwm[chan])

# wrap the ppm output device in CSR decoration and pad assignments
class PPMoutputRegister(Module, AutoCSR):
    def __init__(
            self,
            ppm_pad,
            servo_pads=None,
            ppm_dev=None,
            channels=8,
            frequency=50,
            default_clk_period=1e9/16e6,
            resolution=1e3,
            pulse_width=300e3,
            max_width=2000e3,
            min_width=1000e3,
            csr_width=8,
            alignment=2,
        ):
        if ppm_dev == None:
            ppm_dev = ppm.PPMoutput(
                channels=channels,
                frequency=frequency,
                default_clk_period=default_clk_period,
                resolution=resolution,
                pulse_width=pulse_width,
                max_width=max_width,
                min_width=min_width
            )
        else:
            channels = ppm_dev.channels

        self.output = []
        for chan in range(channels):
            tmp = CSRStorage(csr_width, name='channel_'+str(chan))
            self.output.append(tmp)
            setattr(self, f"tmp{chan}", tmp)

        # assign an input pin to the PPM device
        self.comb += ppm_pad.eq(ppm_dev.ppm)
        for chan in range(channels):
            ppm_size = ppm_dev.widths[chan].nbits
            if csr_width >= ppm_size:
                self.comb += ppm_dev.widths[chan].eq(self.output[chan].storage[:ppm_size])
            else:
                self.comb += ppm_dev.widths[chan][alignment:csr_width+alignment].eq(self.output[chan].storage)
                self.comb += ppm_dev.widths[chan][:alignment].eq(0)

            if servo_pads != None:
                self.comb += servo_pads[chan].eq(ppm_dev.pwm[chan])




class PWMinputRegister(Module, AutoCSR):
    def __init__(
            self,
            pwm_pad,
            pwm_dev=None,
            default_clk_period=1e9/16e6,
            resolution=1e3,
            max_width=2000e3,
            csr_width=8,
            alignment=2,
        ):
        if pwm_dev == None:
            pwm_dev = ppm.PWMinput(
                default_clk_period=default_clk_period,
                resolution=resolution,
                max_width=max_width,
            )
        else:

        # assign an input pin to the PPM device
        self.comb += pwm_pad.eq(pwm_dev.pwm)

        self.output = CSRStorage(csr_width, name='width')

        pwm_size = pwm_dev.width.nbits
        if csr_width >= pwm_size:
            self.comb += self.output.status[:pwm_size].eq(pwm_dev.width)
        else:
            self.comb += self.output.status.eq(pwm_dev.width[alignment:csr_width+alignment])
