from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

# BLDC ESC (BLDC mode, nothing fancy here)


def phase_table(phase_max):
    table=[]
    for n in range(3):
        phase = {
            'pos_start'  : int(((4*n+ 0)%12)*(phase_max/12)),
            'pos_end'    : int(((4*n+ 4)%12)*(phase_max/12)),
            'fall_start' : int(((4*n+ 4)%12)*(phase_max/12)),
            'fall_edge'  : int(((4*n+ 5)%12)*(phase_max/12)),
            'fall_end'   : int(((4*n+ 6)%12)*(phase_max/12)),
            'neg_start'  : int(((4*n+ 6)%12)*(phase_max/12)),
            'neg_end'    : int(((4*n+10)%12)*(phase_max/12)),
            'rise_start' : int(((4*n+10)%12)*(phase_max/12)),
            'rise_edge'  : int(((4*n+11)%12)*(phase_max/12)),
            'rise_end'   : int(((4*n+12)%12)*(phase_max/12))
        }
        table.append(phase)
    return table

class Monostable(Module):
    def __init__(self, t_off=16, t_max=16):
        assert(t_max>=t_off)
        self.out = Signal()
        self.hot = Signal()
        self.timer = Signal(max=t_max)
        self.t_off = Signal(max=t_max, reset=t_off)
        self.strobe = Signal(reset=1) #allow clock prescalers

        self.comb += hot.eq((self.out) or (self.timer > 0))
        self.sync +=    If(self.out,
                            self.timer.eq(self.t_off)
                        ).Elif(self.hot,
                            If(self.strobe,
                                self.timer.eq(self.timer - 1)
                            )
                        )


# half-bridge shoot-through protection
class HalfBridge(Module):
    def __init__(self, l_toff=16, h_toff=16):
        self.en = Signal(1)
        self.dir = Signal(1)

        self.l_gate = Monostable(h_toff, t_max=h_toff)
        self.h_gate = Monostable(l_toff, t_max=l_toff)
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
        self.max_count = Signal(max=count_max, min=count_min, reset=count_max)
        self.min_count = Signal(max=count_max, min=count_min, reset=count_min)
        self.increment = Signal(max=count_max-count_min, signed=True)
        self.dir = Signal()

        self.counter = Signal(count_width)

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


class PhaseSelector(Module):
    def __init__(self, phase_max=3*(2**(16-2))):
        pt = phase_table(phase_max) # XXX find a convention for max vs max+1
        self.phase = Signal(max=phase_max)
        # period where phase can be driven to turn clockwise
        self.uvw_pos  = [Sequencer(count_rise=pt[n]['pos_start'], count_fall=pt[n]['pos_end'], count_max=phase_max) for n in range(3)]
        # period where the phase back-emf should show a rising zero crossing
        self.uvw_fall = [Sequencer(count_rise=pt[n]['fall_start'], count_fall=pt[n]['fall_end'], count_max=phase_max) for n in range(3)]
        # period where phase can be driven to turn counter-clockwise
        self.uvw_neg  = [Sequencer(count_rise=pt[n]['neg_start'], count_fall=pt[n]['neg_end'], count_max=phase_max) for n in range(3)]
        # period where the phase back-emf should show a falling zero crossing
        self.uvw_rise = [Sequencer(count_rise=pt[n]['rise_start'], count_fall=pt[n]['rise_end'], count_max=phase_max) for n in range(3)]
        # common clock
        for n in range(3):
            self.comb += self.uvw_pos[n].counter.eq(self.phase)
            self.comb += self.uvw_fall[n].counter.eq(self.phase)
            self.comb += self.uvw_neg[n].counter.eq(self.phase)
            self.comb += self.uvw_rise[n].counter.eq(self.phase)


class Inverter(Module):
    def __init__(self, selector=PhaseSelector(phase_max=3*(2**(16-2))), phase_max=3*(2**(16-2)), pwm_count_max=2**16):

        self.dir = Signal()
        self.pwm = PWM(count_max=pwm_count_max)
        self.uvw_bridge = [HalfBridge(l_toff=16, h_toff=16) for n in range(3)]
        self.selector = selector
        self.phase = self.selector.phase
        self.duty = self.pwm.compare

        for n in range(3):
            self.comb +=    If(self.dir,
                                self.uvw_bridge[n].en.eq((self.selector.uvw_pos[n].flag and self.pwm.pwm) or self.selector.uvw_neg[n].flag),
                                self.uvw_bridge[n].dir.eq(self.selector.uvw_pos[n].flag)
                            ).Else(
                                self.uvw_bridge[n].en.eq((self.selector.uvw_neg[n].flag and self.pwm.pwm) or self.selector.uvw_pos[n].flag),
                                self.uvw_bridge[n].dir.eq(self.selector.uvw_neg[n].flag)
                            )

class ObserverEMF(Module):
    def __init__(self, selector=PhaseSelector(phase_max=3*(2**(16-2))), phase_max=3*(2**(16-2))):
        pt = phase_table(phase_max)
        self.selector = selector
        # three comparators, assigned to pins at top level
        self.comp = [Signal() for _ in range(3)]
        self.comp_prev = [Signal() for _ in range(3)]
        self.comp_rise = [Signal() for _ in range(3)]
        self.comp_fall = [Signal() for _ in range(3)]
        self.comp_edge = [Signal() for _ in range(3)]
        self.comp_mask = [Signal() for _ in range(3)]
        self.strobe = Signal()
        self.dir = Signal()
        self.phase_observation = Signal(max=phase_max)

        self.comb += self.strobe.eq(self.comp_edge[0] or self.comp_edge[1] or self.comp_edge[2])

        for n in range(3):
            self.sync += self.comp_prev[n].eq(self.comp[n])
            self.comb += self.comp_mask[n].eq(self.selector.uvw_rise[n].flag or self.selector.uvw_fall[n].flag)
            self.comb += self.comp_edge[n].eq( (self.comp[n] != self.comp_prev[n]) and self.comp_mask[n] )
            self.comb += self.comp_rise[n].eq(self.comp_edge[n] and self.comp[n])
            self.comb += self.comp_fall[n].eq(self.comp_edge[n] and self.comp[n])

            self.comb +=    If(self.selector.uvw_rise[n].flag,
                                self.phase_observation.eq(pt[n]['rise_edge'])
                            ).Elif(self.selector.uvw_fall[n].flag,
                                self.phase_observation.eq(pt[n]['fall_edge'])
                            )
            self.sync +=    If(self.comp_edge[n],
                                self.dir.eq(self.comp_rise[n] ^ self.selector.uvw_rise[n].flag)
                            )
            # XXX assert that rise and fall can't be driven together
            # XXX assert that self.selector.uvw_rise[n].flag and self.selector.uvw_fall are never on at the same time
            # XXX assert that phase_observation is never undriven or doubly driven,
            #       either would indicate a timing error

class ObserverHall(Module):
    def __init__(self, phase_max=3*(2**(16-2))):
        pt = phase_table(phase_max)
        #hall_table={0:8, 1:0,3:1,2:2,6:3,4:4,5:5, 7:8}
        hall_table = Array([0,0,2,1,4,5,3,0])
        hall_angle_rough = Array([((2*n+ 1)%12)*((phase_max)/12) for n in range(6)])
        hall_angle_edge  = Array([((2*n+ 0)%12)*((phase_max)/12) for n in range(6)])

        self.phase_angle = Signal(max=phase_max) # current observation
        self.strobe = Signal()        # the observation captures an edge this clock cycle
        self.hall = Signal(3)          # hall inputs
        self.dir = Signal()       # direction of last transition
        self.hall_glitch = Signal()   # observation is invalid

        hall_prev = Signal(3)          # last captured hall input
        hall_error = Signal()         # hall signals are currently invalid
        hall_error_prev = Signal()    # hall signals were invalid at last clock
        hall_idx = Signal(3)           # decoded hall position (0..5)
        hall_idx_prev = Signal(3)      # decoded hall position (0..5) at last clock
        dir_prev = Signal()       # storage reg for direction

        self.sync += hall_prev.eq(self.hall)
        self.comb += self.strobe.eq(self.hall != hall_prev)
        self.comb += hall_error.eq(self.hall == 0 or self.hall==7)
        self.comb += hall_error_prev.eq(hall_prev == 0 or hall_prev==7)
        self.comb += hall_idx.eq(hall_table[self.hall])
        self.comb += hall_idx_prev.eq(hall_table[hall_prev])

        self.comb +=    If(self.strobe == False,
                            self.phase_angle.eq(hall_angle_rough[hall_idx]),
                            self.dir.eq(dir_prev),
                            self.hall_glitch.eq(hall_error)
                        ).Elif((hall_idx == (hall_idx_prev+1)) or ((hall_idx==0) and (hall_idx_prev==5)),
                            self.phase_angle.eq(hall_angle_edge[hall_idx]),
                            self.dir.eq(0),
                            self.hall_glitch.eq(hall_error or hall_error_prev),
                        ).Elif((hall_idx_prev == (hall_idx+1)) or ((hall_idx==5) and (hall_idx_prev==0)),
                            self.phase_angle.eq(hall_angle_edge[hall_idx_prev]),
                            self.dir.eq(1),
                            self.hall_glitch.eq(hall_error or hall_error_prev),
                        ).Else(
                            self.phase_angle.eq(hall_angle_rough[hall_idx]),
                            self.dir.eq(dir_prev),
                            self.hall_glitch.eq(True)
                        )

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







