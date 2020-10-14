from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
import esc
import brushless_servo as bldc
from math import *

def print_state(dut, a, v, pt=esc.phase_table(phase_max=12)):
  print(f"comp_pins = {(yield dut.comp_pins)}")
  print(f"hall_pins = {(yield dut.hall_pins)}")



class ESCInspector(Module):
  def __init__(self, pt, clk_period):
    self.pt = pt
    self.clk_period = clk_period
    self.inv_pins = [
                      [Signal(name="inv_pin_a_l"), Signal(name="inv_pin_a_h")],
                      [Signal(name="inv_pin_b_l"), Signal(name="inv_pin_b_h")],
                      [Signal(name="inv_pin_c_l"), Signal(name="inv_pin_c_h")]
                    ]
    self.hall_pins = Cat(Signal(name="hall_a"), Signal(name="hall_b"), Signal(name="hall_c"))
    self.comp_pins = Cat(Signal(name="comp_a"), Signal(name="comp_b"), Signal(name="comp_c"))
    self.servo_pin = Signal()
    self.submodules.esc = esc.ESC(self.pt, clk_period, self.inv_pins, self.comp_pins, self.hall_pins, self.servo_pin)
    self.voltage_max = 2**10
    self.phase_voltages = [Signal(max=self.voltage_max, name=f"phase_{phase}_voltage") for phase in range(3)]

  def update(self,a,v):
    phase = bldc.rotor_phase(a, self.pt)
    for chan in range(3):
      voltage =  int(bldc.emf(a,v)[chan] + (self.voltage_max/2))
      if voltage >= self.voltage_max:
        voltage = self.voltage_max-1
      if voltage < 0:
        voltage = 0
      yield self.phase_voltages[chan].eq(voltage)
    yield self.comp_pins.eq(bldc.comparator_bits(bldc.emf(a,v)))
    yield self.hall_pins.eq(bldc.hall_bits(a,self.pt))







def test_0(dut):
  v = -1000 # rotor angular velocity
  a = 0*tau/24 # rotor angle
  

  for ms in range(30):
    print(f"ms={ms}")
    for us in range(1000):
      for clk in range(16):
        yield from dut.update(a,v)
        yield 
        a += v*(dut.clk_period/1e9)

def testbench(dut):
  print("testing emf observer device")
  yield from test_0(dut)



if __name__ == "__main__":
    clk_period = int(1e9/16e6)
    pt = esc.phase_table(phase_max=12*(2**23))
    emf_observer = ESCInspector(pt, clk_period)
    run_simulation(emf_observer, testbench(emf_observer),clocks={"sys": 63}, vcd_name="esc_full.vcd")