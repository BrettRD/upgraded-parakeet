from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

# hobby servo PPM
# counts the widths of a pulse chain, storing the clock counts in widths


class PPMinput(Module):
    def __init__(self, channels=8, timeout=(5*1e6)/default_clk_period):
        self.channels = channels
        self.timeout = timeout
        self.ppm = Signal(1)
        self.ppm_prev = Signal(1)
        self.ppm_rising = Signal(1)
        self.active = Signal(1)
        self.widths = [Signal(32) for _ in range(self.channels)] # measured for output
        self.channel_counter = Signal(4)                    
        self.timer = Signal(32)
        self.pwm = [Signal(1) for _ in range(self.channels)]
        
        self.sync += self.ppm_prev.eq(self.ppm)
        self.comb += self.ppm_rising.eq((self.ppm == 1) and (self.ppm_prev == 0))

        for chan in range(self.channels):
            If(self.active == True,
                self.comb += self.pwm[chan].eq(self.channel_counter == chan)
            ).Else(
                self.comb += self.pwm[chan].eq(0)
            )

        If(self.ppm_rising,
            self.sync += self.timer.eq(0)
            If(self.active == False,
                self.sync += self.active.eq(True)
                self.sync += self.channel_counter.eq(0)
            ).Else(
                for chan in range(self.channels):
                    If(self.channel_counter == chan, 
                        self.sync += self.widths[chan].eq(self.timer)
                    )

                self.sync += self.timer.eq(0)
                self.sync += self.channel_counter.eq(self.channel_counter + 1)
            )
        ).Else(
            If(self.active == True,
                If(self.timer >= self.timeout,
                    self.sync += self.active.eq(False)
                ).Else(
                    self.sync += self.timer.eq(self.timer + 1)
                )
            )
        )


class PPMoutput(Module):
    def __init__(self, channels, frequency):
        cycle_period = 1e9/frequency
        max_count = cycle_period / default_clk_period
        self.pulse_period = (0.3*1e6)/default_clk_period
        self.channels = channels
        self.widths = [Signal(32, reset = 2*1e6) for _ in range(self.channels)]
        self.timer = Signal(32)
        self.max_timer = Signal(32)
        self.channel_counter = Signal(4)
        self.pwm = [Signal(1) for _ in range(self.channels)]
        self.ppm = Signal(1)
        self.enable = Signal(1)


        for chan in range(self.channels):
            If(self.channel_counter == chan, 
                self.comb += max_timer.eq(self.widths[chan])
            )
            If(self.enable == True,
                self.comb += self.pwm[chan].eq(self.channel_counter == chan)
            ).Else(
                self.comb += self.pwm[chan].eq(0)
            )

        self.comb += self.ppm.eq(self.timer < self.pulse_period)

        If(self.counter >= max_count,
            self.sync += self.counter.eq(0)
            self.sync += self.timer.eq(0)
            self.sync += self.channel_counter.eq(0)
        ).Else(
            self.sync += self.counter.eq(self.counter+1)
            If(self.channel_counter < self.channels,
                If(self.timer < self.max_timer,
                    self.sync += self.channel_counter.eq(self.channel_counter + 1)
                    self.sync += self.timer.eq(0)
                ).Else(
                    self.sync += self.timer.eq(self.timer + 1)
                )
            )
        )
