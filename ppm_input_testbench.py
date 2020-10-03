from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
import ppm


def print_state(dut):
  print(f"prescaler = {(yield dut.prescaler)}")
  print(f"active = {(yield dut.active)}")
  print(f"ppm = {(yield dut.ppm)}")
  print(f"ppm_prev = {(yield dut.ppm_prev)}")
  print(f"channel_counter = {(yield dut.channel_counter)}")
  print(f"timer = {(yield dut.timer)}")

def skip_us(dut, us):
  print(f"{us} us")
  for _ in range(us*dut.prescale):
    yield


def test_0(dut):
  print(f"blanking time = {(dut.blanking_timeout)}")

  print('testing prescaler')
  yield 
  yield
  yield
  assert (yield dut.prescaler) > 0
  yield
  yield from print_state(dut)
  print("ppm on")
  yield dut.ppm.eq(1)
  yield from skip_us(dut, 1)
  yield from print_state(dut)
  yield from skip_us(dut, 299)
  print("ppm off")
  yield dut.ppm.eq(0)
  yield from print_state(dut)
  yield from skip_us(dut, 1)
  yield from print_state(dut)
  yield from skip_us(dut, 700)
  print("ppm on")
  yield dut.ppm.eq(1)
  yield from skip_us(dut, 1)
  yield from print_state(dut)
  yield from skip_us(dut, 4001)
  yield from print_state(dut)
  print("ppm off")
  yield dut.ppm.eq(0)
  yield from skip_us(dut, 1)
  yield from print_state(dut)
  print("ppm on")
  yield dut.ppm.eq(1)
  yield from skip_us(dut, 1)
  yield from print_state(dut)





def testbench(dut):
  print("testing ppm_input device")
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

    ppm_input = ppm.PPMinput(
                                channels=channels,
                                timeout=timeout,
                                default_clk_period=default_clk_period,
                                resolution=resolution
                            )


    run_simulation(ppm_input, testbench(ppm_input))