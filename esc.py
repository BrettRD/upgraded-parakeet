from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

# BLDC ESC (BLDC mode, nothing fancy here)


def phase_table(phase_max=12):

    a_60 = (phase_max/6)    # 60 degrees of rotation
    a_30 = (phase_max/12)   # 30 degrees of rotation

    hall={    
                    0b001 : 0x0 ,
                    0b011 : 0x1 ,
                    0b010 : 0x2 ,
                    0b110 : 0x3 ,
                    0b100 : 0x4 ,
                    0b101 : 0x5 ,
                    0b000 : 0x0 | 0b1000,  # error state
                    0b111 : 0x0 | 0b1000,  # error state
        }

    # phase wire assignments for each phase
    emf_fwd=[   1,     0,     1,     0,     1,     0,       ]   # edge direction for sense coil
    sense=[     0b100, 0b001, 0b010, 0b100, 0b001, 0b010,   ]   # windings used for sense by phase index
    drive_f=[   0b001, 0b100, 0b100, 0b010, 0b010, 0b001,   ]   # windings used for forward-high drive by phase index
    drive_r=[   0b010, 0b010, 0b001, 0b001, 0b100, 0b100,   ]   # windings used for forward-low drive by phase index

    # generate three phase tables with offsets
    events = []
    for n in range(3):
        phase = {
            'tdc'        : int(a_30 * ((4*n+ 0)%12)),
            'pos_end'    : int(a_30 * ((4*n+ 2)%12)),
            'fall_start' : int(a_30 * ((4*n+ 2)%12)),
            'fall_edge'  : int(a_30 * ((4*n+ 3)%12)),
            'fall_end'   : int(a_30 * ((4*n+ 4)%12)),
            'neg_start'  : int(a_30 * ((4*n+ 4)%12)),
            'bdc'        : int(a_30 * ((4*n+ 6)%12)),
            'neg_end'    : int(a_30 * ((4*n+ 8)%12)),
            'rise_start' : int(a_30 * ((4*n+ 8)%12)),
            'rise_edge'  : int(a_30 * ((4*n+ 9)%12)),
            'rise_end'   : int(a_30 * ((4*n+10)%12)),
            'pos_start'  : int(a_30 * ((4*n+10)%12)),
        }
        events.append(phase)

    # angles by phase angle (0-6)
    # peaks refer to "top dead centre" and "bottom dead centre" of a coil's cosine wave, which corresponds with a Fet switching moment
    # zeroes refer to the EMF zero crossings which happen in the middle of a phase
    peaks  = [ int(a_60 * i)              for i in range(6)]
    zeroes = [ int(a_30 * ((2*i +1)%12))  for i in range(6)]

    return {
                'a_30':a_30,
                'a_60':a_60,
                'a_360':6*a_60,
                'hall':hall,
                'events':events,
                'peaks':peaks,
                'zeroes':zeroes,
                'emf_fwd':emf_fwd,
                'sense':sense,
                'drive_f':drive_f,
                'drive_r':drive_r,
            }

# Monostable
# self.out:  pin being monitored
# self.hot:  flag indicating self.out was recently active 
# t_off:  default hold time following self.out falling edge
# t_max:  maximum configurable hold time
class Monostable(Module):
    def __init__(self, t_off=16, t_max=16):
        assert(t_max>=t_off)
        self.out = Signal()
        self.hot = Signal()
        self.timer = Signal(max=t_max)
        self.t_off = Signal(max=t_max, reset=t_off)
        self.strobe = Signal(reset=1) #allow clock prescalers

        self.comb += self.hot.eq((self.out) or (self.timer > 0))
        self.sync +=    If(self.out,
                            self.timer.eq(self.t_off)
                        ).Elif(self.hot,
                            If(self.strobe,
                                self.timer.eq(self.timer - 1)
                            )
                        )


# half-bridge shoot-through protection
# this uses the Monostable module to prevent shoot-through
# l_toff and h_toff are used to enforce a switching dead-time for
# gate drivers with low switching speeds
# l_toff: time taken (in sysclock cycles) for the low side driver to turn off
# h_toff: time taken (in sysclock cycles) for the high side driver to turn off
# self.en:  turn on one of the transistors
# self.dir: transistor select (1=high, 0=low)
# self.l_out: low side transistor gate drive (active high)
# self.h_out: high side transistor gate drive (active high)
class HalfBridge(Module):
    def __init__(self, l_toff=16, h_toff=16):
        self.en = Signal(1)
        self.dir = Signal(1)

        self.submodule.l_gate = Monostable(h_toff, t_max=h_toff)
        self.submodule.h_gate = Monostable(l_toff, t_max=l_toff)
        self.l_out = self.l_gate.out
        self.h_out = self.h_gate.out
        
        # would have a combinatorial race condition without dir
        self.comb += self.l_out.eq((self.en == True) and (self.dir==0) and (self.h_gate.hot==False))
        self.comb += self.h_out.eq((self.en == True) and (self.dir==1) and (self.l_gate.hot==False))



# window comparator module
class Sequencer(Module):
    def __init__(self, count_rise=0, count_fall=0, count_max=2**16, count_min=0):
        self.counter = Signal(max=count_max, min=count_min)
        self.flag = Signal()
        self.count_fall = Signal(max=count_max, min=count_min, reset=count_fall)
        self.count_rise = Signal(max=count_max, min=count_min, reset=count_rise)
        self.comb += If(self.count_rise <= self.count_fall,
                        self.flag.eq((self.counter >= self.count_rise) & (self.counter < self.count_fall))
                    ).Else(
                        self.flag.eq((self.counter >= self.count_rise) | (self.counter < self.count_fall))
                    )


class SimpleCounter(Module):
    def __init__(self, count_max=2**16, count_min=0):
        self.strobe = Signal(reset=1)
        self.max_count = Signal(max=count_max, min=count_min, reset=count_max)
        self.counter = Signal(max=count_max, min=count_min, reset=count_min)

        self.sync += If(self.strobe,
                        If(self.counter >= self.max_count,
                            self.counter.eq(count_min)
                        ).Else(
                            self.counter.eq(self.counter + 1)
                        )
                    )


class PWM(Module):
    def __init__(self, count_max=2**16, compare=0, phase_correct=False):
        self.tim = SimpleCounter(count_max=count_max)
        self.seq = Sequencer(count_max=count_max, count_fall=compare, count_rise=0)
        self.counter = self.tim.counter
        self.pwm = self.seq.flag
        self.compare = self.seq.count_fall
        self.phase_correct = Signal(1, reset=phase_correct)
        self.comb += If(self.phase_correct,
                        self.seq.count_rise.eq(self.tim.max_count - self.compare)
                    ).Else(
                        self.seq.count_rise.eq(0)  # already implicit in reset value
                    )


# updown counter used for rotor position tracking
class UpDownCounter(Module):
    def __init__(self, count_max=2**16, count_min=0):
        self.strobe = Signal(reset=1)
        self.counter = Signal(max=count_max, min=count_min)
        self.max_count = Signal(max=count_max, min=count_min, reset=count_max)
        self.min_count = Signal(max=count_max, min=count_min, reset=count_min)
        self.increment = Signal(max=count_max-count_min, signed=True)
        self.dir = Signal()

        self.sync +=    If(self.strobe,
                            If(self.dir == 0,
                                If((self.counter + self.increment) >= self.max_count,
                                    self.counter.eq((self.counter + self.increment) - self.max_count + self.min_count)
                                ).Else(
                                    self.counter.eq(self.counter + self.increment)
                                )
                            ).Else(
                                If((self.counter - self.increment) < self.min_count,
                                    self.counter.eq((self.counter - self.increment) + self.max_count - self.min_count)
                                ).Else(
                                    self.counter.eq(self.counter - self.increment)
                                )
                            )
                        )


# A three-phase switching circuit
# PWM is applied to High side mosfets,
# Braking is performed by running low-drive PWM on the phase wire that is most positive.
#  This allows flyback to be absorbed by the high-side body-diode onto the positive supply rail.
# If enable is high and PWM is low, the most negative phase line will be tied to the negative rail at all times.
# Toggling brake with enable and PWM fixed high will result in alternating high and low transistors,
#  this may be useful for efficient generator modes but is not recommended without a phase current control loop
# enable:  enable line for all transistors
# direction:  direction to apply torque
# phase: rotor position from 0-5 (inclusive)
# pwm:  enable line for high-side transistors
class Inverter(Module):
    def __init__(self, enable, direction, phase, brake, pwm):
        pt = phase_table()
        f_table = Array(pt['drive_f'])  # transistors driven high forward, low reverse
        r_table = Array(pt['drive_r'])  # transistors driven low forward, high reverse

        self.inv_dir = Replicate(direction, 3)    # direction the inverter should spin
        self.inv_en = Replicate(enable, 3)     # global enable
        self.inv_pwm = Replicate(pwm, 3)    # global PWM line
        self.br_dir = Signal(3)     
        self.br_en = Signal(3)

        # instantiate gate drivers and shoot-through protection
        self.submodule.br_a = HalfBridge(l_toff=16, h_toff=16)
        self.submodule.br_b = HalfBridge(l_toff=16, h_toff=16)
        self.submodule.br_c = HalfBridge(l_toff=16, h_toff=16)

        # select drive transistors based on the phase
        self.comb +=    If(direction == 0,
                            self.br_en.eq( self.inv_en & ((f_table[phase] & self.inv_pwm) | r_table[phase])),
                            If(brake == False,
                                self.br_dir.eq(f_table[phase])
                            ).Else(
                                self.br_dir.eq(0)
                            )
                        ).Else(
                            self.br_en.eq( self.inv_en & ((r_table[phase] & self.inv_pwm) | f_table[phase])),
                            If(brake == False,
                                self.br_dir.eq(r_table[phase])
                            ).Else(
                                self.br_dir.eq(0)
                            )
                        )

        # apply per-phase dir and en to half-bridges
        self.comb += [
            Cat(self.br_a.en,  self.br_b.en,  self.br_c.en ).eq(self.br_en),
            Cat(self.br_a.dir, self.br_b.dir, self.br_c.dir).eq(self.br_dir)
        ]


# observer to look for rising and falling edges on the undriven phase
#
# phase:  the approximate rotor position (0-6)
# pt:  phase table with observables and masks embedded
# self.comp: comparator inputs
# self.strobe: rises as the relevant phase makes a zero-crossing
# self.dir: observed direction of rotation
# self.phase_observation: the angle of the zero-crossing marked by the strobe
# self.glitch:  marks observations as invalid if the direction changed


class ObserverEMF(Module):
    def __init__(self, phase, pt):

        # fetch parts of the phase table
        s_t = Array(pt['sense'])    # sense mask
        d_t = Array(pt['emf_fwd'])  # expected edge direction for fwd rotation
        e_t = Array(pt['zeroes'])    # angles for the given edges

        # three comparators, assigned to pins at top level
        self.comp = Signal(3)
        self.strobe = Signal()
        self.dir = Signal()
        self.phase_observation = Signal(max=pt['a_360'])
        self.glitch = Signal()

        comp_edge = Signal(3)
        comp_prev = Signal(3)
        edge_dir = Signal()
        dir_prev = Signal()    # stored state for next edge
        glitch_prev = Signal() # store the validity of the last measurement

        # capture the comparator inputs for edge detection
        self.sync += comp_prev.eq(self.comp)

        # turn the three comparators into a strobe and direction
        self.comb += [
                self.phase_observation.eq(e_t[phase]),              # present the angle we expect to find the upcoming edge at
                comp_edge.eq(s_t[phase] & (self.comp ^ comp_prev)), # mask out irrelevant edges according to the phase table (delete PWM noise)
                self.strobe.eq(comp_edge != 0),                     # set a strobe on every relevant comparator edge
                edge_dir.eq((comp_edge & self.comp) != 0),          # determine if the edge was rising or falling
        ]

        # on the comparator's edge, we know the direction of rotation
        self.comb += If(self.strobe,
                        self.dir.eq(d_t[phase] ^ edge_dir),
                        self.glitch.eq(self.dir != dir_prev)   # changes of direction are not usable
                    ).Else(
                        self.dir.eq(dir_prev),                 # outside of the strobe present the stored state
                        self.glitch.eq(glitch_prev)
                    )

        # store state values for after the strobe
        self.sync += If(self.strobe,
                        dir_prev.eq(self.dir),
                        glitch_prev.eq(self.glitch)
                    )



# observer to estimate phase by hall effect inputs
# pt: phase table
# self.phase_observation: rotor position observed - 
#   normally a simple decode of the hall inputs to phase angle.
#   on strobe, this holds an angle denoting the transition points
# self.strobe:  pulses high for one clock cycle marking that an observation corresponds to a transition
# self.dir:  direction of rotation, updates on valid hall edges
# self.hall:  hall sensor input (3b)
# self.hall_glitch: marks observations as invalid due to improper decode or transition


class ObserverHall(Module):
    def __init__(self, pt):
        hall_t = Array(pt['hall'])
        trans_t = Array(pt['peaks'])                # angles for the transitions of the hall signals
        dwell_t = Array(pt['zeroes'])               # angles for settled hall signals

        self.phase_observation = Signal(max=pt['a_360'])  # observation output
        self.strobe = Signal()                      # the observation captures a transition this clock cycle
        self.dir = Signal()                         # direction of last transition
        self.hall = Signal(3)                       # hall inputs
        self.hall_glitch = Signal()                 # observation is invalid

        hall_prev = Signal(3)           # last captured hall input
        hall_error = Signal()           # hall signals are currently invalid
        hall_error_prev = Signal()      # hall signals were invalid at last clock
        hall_idx = Signal(3)            # decoded hall position (0..5)
        hall_idx_prev = Signal(3)       # decoded hall position (0..5) at last clock
        dir_prev = Signal()             # storage reg for direction

        # capture the hall input and decode it.
        self.sync += hall_prev.eq(self.hall)
        self.comb += [  Cat(hall_idx,      hall_error      ).eq(   hall_t[self.hall]  ),
                        Cat(hall_idx_prev, hall_error_prev ).eq(   hall_t[hall_prev]  ),
                    ]

        self.comb +=    If((self.hall == hall_prev),
                            self.phase_observation.eq(dwell_t[hall_idx]),
                            self.dir.eq(dir_prev),
                            self.hall_glitch.eq(hall_error)
                        ).Elif(((hall_idx == (hall_idx_prev + 1)) or ((hall_idx==0) and (hall_idx_prev==5))),
                            self.phase_observation.eq(trans_t[hall_idx]),
                            self.dir.eq(0),
                            self.hall_glitch.eq(hall_error or hall_error_prev),
                        ).Elif(((hall_idx == (hall_idx_prev - 1)) or ((hall_idx==5) and (hall_idx_prev==0))),
                            self.phase_observation.eq(trans_t[hall_idx_prev]),
                            self.dir.eq(1),
                            self.hall_glitch.eq(hall_error or hall_error_prev),
                        ).Else(
                            self.phase_observation.eq(dwell_t[hall_idx]),
                            self.dir.eq(dir_prev),
                            self.hall_glitch.eq(True)
                        )

        self.comb += self.strobe.eq((self.hall != hall_prev) and (self.hall_glitch == False))
        self.sync +=    If(self.strobe == True,
                            If((hall_idx == (hall_idx_prev+1)) or ((hall_idx==0) and (hall_idx_prev==5)),
                                dir_prev.eq(self.dir)
                            ).Elif((hall_idx_prev == (hall_idx+1)) or ((hall_idx==5) and (hall_idx_prev==0)),
                                dir_prev.eq(self.dir)
                            )
                        )





# XXX do we update single steps on the phase angle according to a variable prescaler?
#     or do we update multiple steps against a much larger prescaler?
#     conventional systems run a timer between two crossings and take a ratio of that for commutation timers
#     30000rpm, 4pole (100Hz) motor
#     16MHz sysclk, 3*2**14 phase_max, updating by one every clock cycle makes 325Hz 
# drop phase resolution to 3*2**10, run variable prescaler single increment
class PhaseEstimator(Module):
    def __init__(self, phase_max=3*(2**(16-2)), prescale_count_max=2**16):
        
        phase_obs_dist = (phase_max) / 6 # should be a power of two
        # XXX assert that phase_timer_divider is a power of two

        self.phase_counter = UpDownCounter(count_max=phase_max)

        self.phase_capture = Signal(max=phase_max)# records the phase at last edge observation
        self.phase_observation = Signal(max=phase_max)    # comb set as output of observer

        self.time_capture = Signal(max=prescale_count_max) # records the interval between edge observations
        self.timer = Signal(max=prescale_count_max)        # times the duration between observations, does not overflow
        self.phase_timer = Signal(max=prescale_count_max)  # counts up to a fraction of time_capture, and increments the phase estimate
        self.phase_timer_prescale = Signal(max=prescale_count_max)

        self.strobe = Signal() # comb set as output of observer
        self.glitch = Signal() # comb set as output of observer
        self.dir = Signal()    # comb set as output of observer

        self.phase_counter.strobe = self.phase_timer_strobe
        self.phase_estimate = self.phase_counter.counter
        self.comb += self.phase_counter.increment.eq(1)
        self.comb += self.phase_counter.dir.eq(self.dir)

        self.comb += self.phase_timer_prescale.eq(self.time_capture / phase_obs_dist) # division by power of two becomes bit shift

        self.comb +=    If(self.strobe,
                            self.phase_timer_strobe.eq(0)
                        ).Else(
                            If(self.phase_timer >= self.phase_timer_prescale,
                                self.phase_timer_strobe.eq(1)
                            ).Else(
                                self.phase_timer_strobe.eq(0)
                            )
                        )

        self.sync +=    If(self.strobe,
                            self.timer.eq(0),
                            If(self.glitch == False,
                                self.phase_capture.eq(self.phase_observation),
                                self.time_capture.eq(self.timer),
                                self.phase_timer.eq(0),
                                self.phase_estimate.eq(self.phase_observation)
                            )
                        ).Else(
                            If(timer < prescale_count_max,
                                self.timer.eq(self.timer + 1)
                            ),
                            If(self.phase_timer >= self.phase_timer_prescale,
                                self.phase_timer.eq(0)
                            ).Else(
                                self.phase_timer.eq(self.phase_timer + 1)
                            )
                        )







