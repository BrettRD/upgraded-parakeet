from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
import ppm


import ppm_input_testbench
import ppm_output_testbench


class PPMRoundTrip(Module):
  def __init__(self,
      channels=8,
      timeout=4000e3,
      frequency=50,
      default_clk_period=1e9/16e6,
      resolution=1e3,
      pulse_width=300e3,
      max_width=2000e3,
      min_width=500e3,
    ):

    self.submodules.ppm_tx = ppm.PPMoutput(
                            channels=channels,
                            frequency=frequency,
                            default_clk_period=default_clk_period,
                            resolution=resolution,
                            pulse_width=pulse_width,
                            max_width=max_width,
                            min_width=min_width
                        )
    self.submodules.ppm_rx = ppm.PPMinput(
                            channels=channels,
                            timeout=timeout,
                            default_clk_period=default_clk_period,
                            resolution=resolution
                        )

    self.comb += self.ppm_rx.ppm.eq(self.ppm_tx.ppm)



def print_state(dut):
  yield from ppm_input_testbench.print_state(dut.ppm_rx)
  yield from ppm_output_testbench.print_state(dut.ppm_tx)


def test_0(dut, channel_vals):
  for chan in range(len(channel_vals)):
    yield dut.ppm_tx.widths[chan].eq(channel_vals[chan])
  yield

  yield from print_state(dut)
  yield from ppm_input_testbench.skip_us(dut.ppm_rx, 20000)  
  yield from print_state(dut)
  for chan in range(len(channel_vals)):
    assert (yield dut.ppm_tx.widths[chan]) == (yield dut.ppm_rx.widths[chan])






def testbench(dut):
  yield from test_0(dut, [
                            1500,
                            500,
                            1500,
                            2000,
                            700,
                            1600,
                            1500,
                            1400,
                          ])


if __name__ == "__main__":

    channels=8
    timeout=5e6
    frequency=50
    default_clk_period=1e9/16e6
    resolution=1e3
    pulse_width=300e3
    max_width=2000e3
    min_width=500e3

    ppm_round_trip = PPMRoundTrip(
                                        channels=channels,
                                        timeout=timeout,
                                        frequency=frequency,
                                        default_clk_period=default_clk_period,
                                        resolution=resolution,
                                        pulse_width=pulse_width,
                                        max_width=max_width,
                                        min_width=min_width
                                    )


    run_simulation(ppm_round_trip, testbench(ppm_round_trip))