from math import *


# a simple model of a brushless servo motor back-emf
# a: the angle of the rotor at this instant (radians)
# v: the velocity of the rotor (radians per second)
# kv: the constant of voltage, (RPM/Volt)
def emf(a, v, kv=60):
  # immediately convert to engineering units
  kv =  60 / (tau * kv)  # volts(radian/sec)
  assert(kv > 0)
  phase_voltages = [kv * v * cos(a - offset) for offset in [0, tau/3, 2*tau/3]]
  return phase_voltages

def comparator_bits(phase_voltages):
  assert(len(phase_voltages) == 3)
  val = 0
  for i in range(3):
    val |= (phase_voltages[i] > sum(phase_voltages)/3) << i
  return val

# position 0-6
def rotor_phase(angle, pt):
  return int((6 * angle / tau) % 6)

# position 0-pt['a_360']
def rotor_pos(angle, pt):
  return int(pt['a_360'] * angle / tau) % pt['a_360']

# hall table lookup of phase index
def hall_bits(angle,pt):
  phase = rotor_phase(angle, pt)
  assert(phase>=0)
  assert(phase<6)
  # XXX do a reverse lookup on pt[hall]
  return pt['hall'].index(8|phase)

