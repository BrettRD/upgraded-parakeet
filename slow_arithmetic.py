from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
from random import randint

class slow_division(Module):
  def __init__(self, width=32):
    self.width = width
    self.remainder = Signal(width)
    self.quotient = Signal(width, reset=1)
    self.divisor = Signal(width)
    self.fractional = Signal(width-1)  # optional fractional component of divisor    
    self.stop = Signal(1, reset=1)

    div = Cat(self.fractional, self.divisor)
    rem = self.remainder
    quo = Cat(self.quotient, self.stop)
    self.sync += [
      If(self.stop == 0,
        quo[1:].eq(quo[0:-1]),      # bit shift quotient up
        div[0:-1].eq(div[1:]),      # bit shift divisor down
        If(rem >= div,
          rem.eq(rem-div),
          quo[0].eq(1)
        ).Else(
          quo[0].eq(0)
        ),
      )
    ]


def test_0(dut, num, den):
  yield dut.remainder.eq(num)
  yield dut.divisor.eq(den)
  yield dut.fractional.eq(0)
  yield dut.quotient.eq(1)  # becomes stop bit
  yield dut.stop.eq(0)      # goes high when done
  yield
  for _ in range(dut.width-1):
    yield 
  print(f"stop = {yield dut.stop}")      
  yield 
  print(f"stop = {yield dut.stop}")      
  yield 
  yield 
  yield 

  print(f"numerator = {num}") 
  print(f"denominator = {den}") 
  print(f"remainder = {yield dut.remainder}") 
  print(f"quotient = {yield dut.quotient}") 
  assert((yield dut.quotient) * den + (yield dut.remainder) == num)
  print(f"pass") 

def testbench(dut):
  print("testing emf observer device")
  for _ in range(100):
    num = randint(1,2**32-1)
    idx = randint(1,32)
    den = randint(1,(2**idx)-1)
    yield from test_0(dut, num, den)



if __name__ == "__main__":
    dut = slow_division(width=32)
    run_simulation(dut, testbench(dut))