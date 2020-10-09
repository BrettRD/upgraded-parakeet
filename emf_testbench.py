from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
import esc
import brushless_servo as bldc
from math import *

def print_state(dut, a, v, pt=esc.phase_table(phase_max=12)):
  print(f"comp = {(yield dut.comp)}")
  print(f"strobe = {(yield dut.strobe)}")
  print(f"phase_observation = {(yield dut.phase_observation)}")
  print(f"glitch = {(yield dut.glitch)}")
  print(f"phase = {(yield dut.phase)}")
  print(f"rotor angle = {a}")
  print(f"rotor phase = {rotor_phase(a, pt)}")
  print(f"back emf = {bldc.emf(a,v)}")
  print(f"comparators = {bldc.comparator_bits(bldc.emf(a,v))}")



def rotor_phase(angle, pt=esc.phase_table(phase_max=12)):
  return int(6 * angle / tau) % 6



def test_0(dut):
  pt=esc.phase_table(phase_max=12)
  v = -1 # rotor angular velocity
  a = 0*tau/24 # rotor angle
  comp = bldc.comparator_bits(bldc.emf(a,v))
  phase = rotor_phase(a, pt)
  yield dut.comp.eq(comp)
  yield dut.phase.eq(phase)
  yield 
  assert (yield dut.glitch) == 0 # settled inputs with valid state
  yield from print_state(dut, a, v, pt=pt)
  yield 
  yield from print_state(dut, a, v, pt=pt)
  yield 

  assert (yield dut.glitch) == 0 # settled inputs with valid state
  a=2*tau
  for i in range(48):
    print(f"\nstep")

    v = -1 # rotor angular velocity
    a += v *tau/24 # rotor angle
    comp = bldc.comparator_bits(bldc.emf(a,v))
    phase = rotor_phase(a, pt)
    yield dut.comp.eq(comp)
    yield dut.phase.eq(phase)
    yield 
    yield from print_state(dut, a, v, pt=pt)
    assert (yield dut.glitch) == 0 # settled inputs with valid state
    yield 
    yield from print_state(dut, a, v, pt=pt)
    assert (yield dut.glitch) == 0 # settled inputs with valid state



def testbench(dut):
  print("testing emf observer device")
  yield from test_0(dut)



if __name__ == "__main__":
    pt = esc.phase_table(phase_max=12)
    emf_observer = esc.ObserverEMF(pt)
    run_simulation(emf_observer, testbench(emf_observer))