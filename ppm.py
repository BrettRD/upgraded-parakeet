from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

# hobby servo PPM
# counts the widths of a pulse chain, storing the clock counts in widths
# channels: the maximum number of channels to be decoded and reported
# timeout: nanoseconds after the last pulse to reset the channel counter, also the maximum width of a servo pulse
# default_clk_period: system clock in nanoseconds
# resolution: nanoseconds per pulse interval increment

class PPMinput(Module):
    def __init__(self, channels=8, timeout=5e6, default_clk_period=1e9/16e6, resolution=1e3):
        self.channels = channels
        self.prescale = int(resolution/default_clk_period)
        self.blanking_timeout = int((timeout/default_clk_period)/self.prescale)

        self.prescaler = Signal(max=self.prescale)

        self.timer = Signal(max=self.blanking_timeout)

        self.ppm = Signal()
        self.ppm_prev = Signal()
        self.active = Signal()
        self.widths = [Signal(max=self.blanking_timeout) for _ in range(self.channels)] # measured for output
        self.channel_counter = Signal(max=self.channels+1)
        self.pwm = [Signal() for _ in range(self.channels)]

        self.sync +=    If(self.prescaler+1 < self.prescale,
                            self.prescaler.eq(self.prescaler+1)
                        ).Else(
                            self.prescaler.eq(0),
                            self.ppm_prev.eq(self.ppm),
                            If((self.ppm == 1) & (self.ppm_prev == 0),  # on a ppm rising edge
                                self.timer.eq(0),
                                If(self.active == False,
                                    # first ppm pulse after blanking
                                    self.active.eq(True),
                                    self.channel_counter.eq(0)
                                ).Else(
                                    # all subsequent ppm pulses
                                    [If(self.channel_counter == chan, 
                                        self.widths[chan].eq(self.timer),                   # capture the interval for the valid channel
                                        self.channel_counter.eq(self.channel_counter + 1)   # increment the channel_counter
                                    ) for chan in range(self.channels)]
                                    # channel_counter will park above the last channel for any trailling pulses
                                )
                            ).Else( # no ppm edge
                                If(self.active == True,
                                    # wait for the blanking interval
                                    If(self.timer + 1 >= self.blanking_timeout,
                                        self.active.eq(False)
                                    ).Else(
                                        self.timer.eq(self.timer + 1)
                                    )
                                )
                            )
                        )

        # decode the ppm input into servo channels
        for chan in range(self.channels):
            self.comb +=    If(self.active == True,
                                self.pwm[chan].eq(self.channel_counter == chan)
                            ).Else(
                                self.pwm[chan].eq(0)
                            )


# hobby servo PPM output
# generates a sequence of pulses separated by specified times
# channels is the maximum number of channels to be encoded
# frequency is the whole sequence repeat rate (50Hz for conventional hobby servos)


class PPMoutput(Module):
    def __init__(self, channels=8, frequency=50, default_clk_period=1e9/16e6):
        max_count = int((1e9 / frequency) / default_clk_period)
        self.pulse_period = int((0.3*1e6) / default_clk_period)
        self.channels = channels
        self.widths = [Signal(32, reset = 2*1e6) for _ in range(self.channels)]
        self.timer = Signal(32)
        self.max_timer = Signal(32)
        self.channel_counter = Signal(max=self.channels+1)
        self.pwm = [Signal(1) for _ in range(self.channels)]
        self.ppm = Signal(1)
        self.enable = Signal(1)


        for chan in range(self.channels):
            self.comb +=    If(self.channel_counter == chan, 
                                max_timer.eq(self.widths[chan])
                            )
            self.comb +=    If(self.enable == True,
                                self.pwm[chan].eq(self.channel_counter == chan)
                            ).Else(
                                self.pwm[chan].eq(0)
                            )

        self.comb += self.ppm.eq(self.timer < self.pulse_period)

        self.sync +=    If(self.counter >= max_count,
                            self.counter.eq(0),
                            self.timer.eq(0),
                            self.channel_counter.eq(0)
                        ).Else(
                            self.counter.eq(self.counter+1),
                            If(self.channel_counter < self.channels,
                                If(self.timer + 1 < self.max_timer,
                                    self.timer.eq(self.timer + 1)
                                ).Else(
                                    self.channel_counter.eq(self.channel_counter + 1),
                                    self.timer.eq(0)
                                )
                            )
                        )
