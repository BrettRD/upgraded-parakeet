from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
from random import randint

class slow_division(Module):
  def __init__(self, num_width=32, den_width=32):
    self.num_width = num_width
    self.den_width = den_width

    self.remainder = Signal(num_width)
    self.quotient = Signal(num_width, reset=1)
    self.divisor = Signal(den_width)
    self.fractional = Signal(num_width-1)  # optional fractional component of divisor    
    self.stop = Signal(1, reset=1)

    div = Cat(self.fractional, self.divisor)
    rem = self.remainder
    quo = Cat(self.quotient, self.stop)
    self.sync += [
      If(self.stop == 0,
        div.eq(div >> 1),           # bit shift divisor down
        If(rem >= div,
          rem.eq(rem-div),
          quo.eq( Cat(1, quo[0:-1]) ),
        ).Else(
          quo.eq( Cat(0, quo[0:-1]) ),
        ),
      )
    ]

  def start(self, num, den):
    return [
      If(self.stop == 1,
        self.remainder.eq(num),
        self.divisor.eq(den),
        self.fractional.eq(0),
        self.quotient.eq(1),
        self.stop.eq(0),
      )
    ]



def test_0(dut, num, den):
  yield dut.remainder.eq(num)
  yield dut.divisor.eq(den)
  yield dut.fractional.eq(0)
  yield dut.quotient.eq(1)  # becomes stop bit
  yield dut.stop.eq(0)      # goes high when done
  yield
  for _ in range(dut.num_width-1):
    yield 
  assert((yield dut.stop)==0) #should be running after num_width-1 cycles 
  yield 
  assert((yield dut.stop)==1) #should finish after num_width cycles 
  print(f"numerator = {num}") 
  print(f"denominator = {den}") 
  print(f"quotient = {yield dut.quotient}")
  print(f"remainder = {yield dut.remainder}")
  assert((yield dut.remainder) < den) # check it did anything
  assert((yield dut.quotient) * den + (yield dut.remainder) == num) # test correctness
  print(f"pass") 

def testbench(dut):
  print("testing emf observer device")
  for _ in range(100):
    num_idx = randint(1,dut.num_width)
    num = randint(1,(2**num_idx)-1)
    den_idx = randint(1,dut.den_width)
    den = randint(1,(2**den_idx)-1)
    yield from test_0(dut, num, den)



if __name__ == "__main__":
    dut = slow_division(num_width=37, den_width=21)
    run_simulation(dut, testbench(dut))