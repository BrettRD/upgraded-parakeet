from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
from litex.soc.interconnect.csr import AutoCSR, CSRStatus, CSRStorage
import ppm

# wrap the ppm input device in CSR decoration and pad assignments
class PPMinputRegister(Module, AutoCSR):
    def __init__(self, ppm_pad, servo_pads=None, channels=8, timeout=5e6, default_clk_period=1e9/16e6):

        ppm_dev = ppm.PPMinput(channels=channels, timeout=timeout, default_clk_period=default_clk_period)

        self.input = Signal()
        self.output = []
        for chan in range(channels):
            tmp = CSRStatus(8, name='channel_'+str(chan))   # XXX need to set bit width properly
            self.output.append(tmp)
            setattr(self, f"tmp{chan}", tmp)

        # assign an input pin to the PPM device
        self.comb += ppm_dev.ppm.eq(ppm_pad)
        for chan in range(channels):
            self.comb += self.output[chan].status.eq(ppm_dev.widths[chan])
            if servo_pads != None:
                self.comb += servo_pads[chan].eq(ppm_dev.pwm[chan])

# wrap the ppm output device in CSR decoration and pad assignments
class PPMoutputRegister(Module, AutoCSR):
    def __init__(self, ppm_pad, servo_pads=None, channels=8, frequency=50, default_clk_period=1e9/16e6):
        self.input = Signal()

        self.output = []
        for chan in range(channels):
            tmp = CSRStorage(8, name='channel_'+str(chan))
            self.output.append(tmp)
            setattr(self, f"tmp{chan}", tmp)

        ppm_dev = ppm.PPMoutput(channels=channels, timeout=timeout, default_clk_period=default_clk_period)
        # assign an input pin to the PPM device
        self.comb += ppm_pad.eq(ppm_dev.ppm)
        for chan in range(channels):
            self.comb += ppm_dev.widths[chan].eq(self.output[chan].storage)
            if servo_pads != None:
                self.comb += servo_pads[chan].eq(ppm_dev.pwm[chan])

