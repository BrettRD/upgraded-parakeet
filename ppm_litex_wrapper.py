from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
from litex.soc.interconnect.csr import AutoCSR, CSRStatus, CSRStorage
import ppm

# wrap the ppm input device in CSR decoration and pad assignments
class PPMinputRegister(Module, AutoCSR):
    def __init__(self, ppm_pad, servo_pads=None, channels=8, timeout=5e6, default_clk_period=1e9/16e6, resolution=1e3, csr_width=8):
        ppm_dev = ppm.PPMinput(channels=channels, timeout=timeout, default_clk_period=default_clk_period, resolution=resolution)
        self.input = Signal()
        self.output = []
        for chan in range(channels):
            tmp = CSRStatus(csr_width, name='channel_'+str(chan))
            self.output.append(tmp)
            setattr(self, f"tmp{chan}", tmp)

        # assign an input pin to the PPM device
        self.comb += ppm_dev.ppm.eq(ppm_pad)
        for chan in range(channels):
            self.comb += self.output[chan].status[:csr_width].eq(ppm_dev.widths[chan][-csr_width:])
            # XXX deal with the case where the csr_width is bigger than ppm_dev.widths[n]
            if servo_pads != None:
                self.comb += servo_pads[chan].eq(ppm_dev.pwm[chan])

# wrap the ppm output device in CSR decoration and pad assignments
class PPMoutputRegister(Module, AutoCSR):
    def __init__(self, ppm_pad, servo_pads=None, channels=8, frequency=50, default_clk_period=1e9/16e6, resolution=1e3, pulse_width=300e3, max_width=2000e3, min_width=500e3, csr_width=8):
        ppm_dev = ppm.PPMoutput(channels=channels, frequency=frequency, default_clk_period=default_clk_period, resolution=resolution, pulse_width=pulse_width, max_width=max_width, min_width=min_width)
        self.input = Signal()
        self.output = []
        for chan in range(channels):
            tmp = CSRStorage(csr_width, name='channel_'+str(chan))
            self.output.append(tmp)
            setattr(self, f"tmp{chan}", tmp)

        # assign an input pin to the PPM device
        self.comb += ppm_pad.eq(ppm_dev.ppm)
        for chan in range(channels):
            self.comb += ppm_dev.widths[chan][-csr_width:].eq(self.output[chan].storage[:csr_width])
            # XXX deal with the case where the csr_width is bigger than ppm_dev.widths[n]
            if servo_pads != None:
                self.comb += servo_pads[chan].eq(ppm_dev.pwm[chan])

