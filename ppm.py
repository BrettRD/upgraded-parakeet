from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

# hobby servo PPM
# counts the widths of a pulse chain, storing the clock counts in widths
# channels: the maximum number of channels to be decoded and reported
# timeout: nanoseconds after the last pulse to reset the channel counter, also the maximum width of a servo pulse
# default_clk_period: system clock in nanoseconds
# resolution: nanoseconds per increment

class PPMinput(Module):
    def __init__(self, channels=8, timeout=5e6, default_clk_period=1e9/16e6, resolution=1e3):
        self.channels = channels
        self.prescale = int(resolution/default_clk_period)
        self.blanking_timeout = int((timeout/default_clk_period)/self.prescale)

        assert(self.channels > 0)
        assert(self.prescale > 0)

        self.prescaler = Signal(max=self.prescale)
        self.timer = Signal(max=self.blanking_timeout)

        self.ppm = Signal()
        self.ppm_prev = Signal()
        self.active = Signal()
        self.widths = [Signal(max=self.blanking_timeout) for _ in range(self.channels)] # measured for output
        self.channel_counter = Signal(max=self.channels+1)
        self.pwm = [Signal() for _ in range(self.channels)]

        self.sync +=    If(self.prescaler + 1 < self.prescale,
                            self.prescaler.eq(self.prescaler + 1)
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
# channels: the maximum number of channels to be encoded
# frequency: the whole sequence repeat rate (50Hz for conventional hobby servos)
# resolution: nanoseconds per increment
# pulse_width: width of each pulse in ns (300uS for conventional hobby radios)
# max_width:  maximum interval between rising edges of one sequence in nanoseconds (2500uS for conventional hobby servos)
# min_width:  minimum interval between rising edges of one sequence in nanoseconds (500uS for conventional hobby servos)


class PPMoutput(Module):
    def __init__(self, channels=8, frequency=50, default_clk_period=1e9/16e6, resolution=1e3, pulse_width=300e3, max_width=2000e3, min_width=500e3):
        self.prescale = int(resolution/default_clk_period)

        self.sequence_end = int(((1e9 / frequency) / default_clk_period)/self.prescale)
        self.pulse_width = int((pulse_width/default_clk_period)/self.prescale)
        self.max_width = int((max_width/default_clk_period)/self.prescale)
        self.min_width = int((min_width/default_clk_period)/self.prescale)
        self.channels = channels

        assert(self.channels > 0)
        assert(self.prescale > 0)
        assert(self.min_width > self.pulse_width) # Make sure there's a falling edge between channels
        assert(self.max_width > self.min_width)
        assert(self.sequence_end > (self.channels+1)*self.max_width) # Make sure there's a blanking interval between sequences

        self.prescaler = Signal(max=self.prescale)

        self.channel_timer = Signal(max=self.max_width) # times each channel
        self.max_timer = Signal(max=self.max_width, reset=int((self.max_width+self.min_width)/2))  # select channel
        self.widths = [Signal(max=self.max_width, reset=int((self.max_width+self.min_width)/2)) for _ in range(self.channels)]

        self.sequence_timer = Signal(max=self.sequence_end)
        self.channel_counter = Signal(max=self.channels + 1)
        self.pwm = [Signal() for _ in range(self.channels)]
        self.ppm = Signal()
        self.enable = Signal()

        for chan in range(self.channels):
            self.comb +=    If(self.channel_counter == chan, 
                                If(self.widths[chan] > self.min_width,  # clamp the minimum range
                                    self.max_timer.eq(self.widths[chan])
                                ).Else(
                                    self.max_timer.eq(self.min_width)
                                )
                            )
            self.comb += self.pwm[chan].eq(self.channel_counter == chan)

        self.comb += self.ppm.eq(self.channel_timer < self.pulse_width)

        self.sync +=    If(self.prescaler + 1 < self.prescale,
                            self.prescaler.eq(self.prescaler + 1)
                        ).Else(
                            self.prescaler.eq(0),
                            If(self.sequence_timer + 1 < self.sequence_end,
                                self.sequence_timer.eq(self.sequence_timer + 1),
                                If(self.channel_counter < self.channels,    # increment past the final channel for the PWM lines
                                    If(self.channel_timer + 1 < self.max_timer,
                                        self.channel_timer.eq(self.channel_timer + 1)
                                    ).Else(
                                        self.channel_counter.eq(self.channel_counter + 1),
                                        self.channel_timer.eq(0)
                                    )
                                )
                            ).Else(
                                self.sequence_timer.eq(0),
                                self.channel_timer.eq(0),
                                self.channel_counter.eq(0)
                            )
                        )
