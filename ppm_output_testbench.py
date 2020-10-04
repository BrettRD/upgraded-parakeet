from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
import ppm



def print_state(dut):
  print(f"prescaler = {(yield dut.prescaler)}")
  print(f"ppm = {(yield dut.ppm)}")
  print(f"channel_counter = {(yield dut.channel_counter)}")
  print(f"channel_timer = {(yield dut.channel_timer)}")
  print(f"sequence_timer = {(yield dut.sequence_timer)}")
  for chan in range(dut.channels):
    print(f"width_{chan} = {(yield dut.widths[chan])}")


def skip_us(dut, us):
  print(f"{us} us")
  for _ in range(us*dut.prescale):
    yield

def test_0(dut):
  for chan in range(dut.channels):
    yield dut.widths[chan].eq(1500)
  yield dut.widths[3].eq(300) # should truncate to 500 

  yield
  yield
  
  yield from skip_us(dut, 1)
  yield from print_state(dut)
  yield from skip_us(dut, 10)
  yield from print_state(dut)
  yield from skip_us(dut, 288)
  yield from print_state(dut)
  yield from skip_us(dut, 1)
  yield from print_state(dut)
  yield from skip_us(dut, 1200)
  yield from print_state(dut)
  yield from skip_us(dut, 1500)
  yield from print_state(dut)
  yield from skip_us(dut, 1500)
  yield from print_state(dut)
  yield from skip_us(dut, 500)
  yield from print_state(dut)
  yield from skip_us(dut, 1500)
  yield from print_state(dut)



def testbench(dut):
  print("testing ppm_output device")
  yield from test_0(dut)


if __name__ == "__main__":

    channels=8
    timeout=4000e3
    frequency=50
    default_clk_period=1e9/16e6
    resolution=1e3
    pulse_width=300e3
    max_width=2000e3
    min_width=500e3

    ppm_output = ppm.PPMoutput(
                                channels=channels,
                                frequency=frequency,
                                default_clk_period=default_clk_period,
                                resolution=resolution,
                                pulse_width=pulse_width,
                                max_width=max_width,
                                min_width=min_width
                            )

    run_simulation(ppm_output, testbench(ppm_output))