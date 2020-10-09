from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
import esc


def print_state(dut):
  print(f"hall = {(yield dut.hall)}")
  print(f"dir = {(yield dut.dir)}")
  print(f"strobe = {(yield dut.strobe)}")
  print(f"hall_glitch = {(yield dut.hall_glitch)}")
  print(f"phase_observation = {(yield dut.phase_observation)}")

  #print(f"hall_prev = {(yield dut.hall_prev)}")
  #print(f"hall_error = {(yield dut.hall_error)}")
  #print(f"hall_error_prev = {(yield dut.hall_error_prev)}")
  #print(f"hall_idx = {(yield dut.hall_idx)}")
  #print(f"hall_idx_prev = {(yield dut.hall_idx_prev)}")
  #print(f"dir_prev = {(yield dut.dir_prev)}")
  


def test_0(dut):
  pt=esc.phase_table(phase_max=12)
  hall_seq = [1,3,2,6,4,5]
  hall_seq = hall_seq + hall_seq
  yield dut.hall.eq(hall_seq[-1])
  yield 
  assert (yield dut.hall_glitch) == 1 # transition from zero has no defined position
  assert (yield dut.strobe) == 0  # exiting error state, don't strobe
  yield 
  assert (yield dut.hall_glitch) == 0 # settled inputs with valid state
  assert (yield dut.strobe) == 0 # strobe is off for settled input

  for h in hall_seq:
    print("forward")
    yield dut.hall.eq(h)
    yield 
    yield from print_state(dut)
    assert (yield dut.strobe) == 1 # strobe is on after valid transition
    yield
    yield from print_state(dut)
    assert (yield dut.strobe) == 0 # strobe is off for settled input
    yield
    yield from print_state(dut)

  yield dut.hall.eq(hall_seq[0])
  yield 
  yield 

  hall_seq.reverse()
  for h in hall_seq:
    print("reverse")
    yield dut.hall.eq(h)
    yield 
    yield from print_state(dut)
    assert (yield dut.strobe) == 1 # strobe is on after valid transition
    yield
    yield from print_state(dut)
    assert (yield dut.strobe) == 0 # strobe is off for settled input
    yield
    yield from print_state(dut)


def testbench(dut):
  print("testing hall observer device")
  yield from test_0(dut)



if __name__ == "__main__":
    pt=esc.phase_table(phase_max=12)
    hall_observer = esc.ObserverHall(pt)
    run_simulation(hall_observer, testbench(hall_observer))