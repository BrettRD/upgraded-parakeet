from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

# BLDC ESC (BLDC mode, nothing fancy here)


def phase_table(phase_max):
    table=[]
    for n in range(3):
        phase = {
            'pos_start'  : ((4*n+ 0)%12)*(phase_max/12),
            'pos_end'    : ((4*n+ 4)%12)*(phase_max/12),
            'fall_start' : ((4*n+ 4)%12)*(phase_max/12),
            'fall_edge'  : ((4*n+ 5)%12)*(phase_max/12),
            'fall_end'   : ((4*n+ 6)%12)*(phase_max/12),
            'neg_start'  : ((4*n+ 6)%12)*(phase_max/12),
            'neg_end'    : ((4*n+10)%12)*(phase_max/12),
            'rise_start' : ((4*n+10)%12)*(phase_max/12),
            'rise_edge'  : ((4*n+11)%12)*(phase_max/12),
            'rise_end'   : ((4*n+12)%12)*(phase_max/12)
        }
        table.append(phase)
    return table

class Monostable(Module):
    def __init__(self, count_width=16, toff=16):

        self.out = Signal(1)
        self.hot = Signal(1)
        self.timer = Signal(count_width)
        self.toff = Signal(count_width, rst=toff)

        self.comb += hot.eq((self.out==1) or (self.timer > 0))
        If(self.out==True,
            self.sync += self.timer.eq(self.toff)
        ).Elif((self.hot == True) and (self.out==False),
            self.sync += self.timer.eq(self.timer - 1)
        ).Else(
            self.sync += self.timer.eq(0)
        )


# half-bridge shoot-through protection
class HalfBridge(Module):
    def __init__(self, l_toff=16, h_toff=16):
        self.en = Signal(1)
        self.dir = Signal(1)

        self.l_gate = Monostable(h_toff)
        self.h_gate = Monostable(h_toff)
        self.l_out = self.l_gate.out
        self.h_out = self.h_gate.out
        
        # would have a combinatorial race condition without dir
        self.comb += self.l_out.eq((self.en == True) and (self.dir==0) and (self.h_gate.hot==False))
        self.comb += self.h_out.eq((self.en == True) and (self.dir==1) and (self.l_gate.hot==False))



# window comparator module
class Sequencer(Module):
    def __init__(self, count_width=16, count_rise=0, count_fall=0):
        self.counter = Signal(count_width)
        self.flag = Signal(1)
        self.count_fall = Signal(count_width, rst=count_fall)
        self.count_rise = Signal(count_width, rst=count_rise)
        If(self.count_rise <= self.count_fall,
            self.comb += self.flag.eq((self.counter >= self.count_rise) and (self.counter < self.count_fall))
        ).Else(
            self.comb += self.flag.eq((self.counter >= self.count_rise) or (self.counter < self.count_fall))
        )


class SimpleCounter(Module):
    def __init__(self, count_width=16, max_count=(2**16)-1):
        self.strobe = Signal(1)
        self.max_count = Signal(count_width+1, rst=max_count)
        self.increment = Signal(count_width, signed=True)
        self.counter = Signal(count_width)

        If(self.strobe,
            If(self.counter >= self.max_count,
                self.sync += self.counter.eq(0)
            ).Else(
                self.sync += self.counter.eq(self.counter + 1)
            )
        )


class PWM(Module):
    def __init__(self, count_width=16, max_count=(2**16)-1, compare=0, phase_correct=False):
        self.tim = SimpleCounter(count_width=count_width, max_count=max_count)
        self.seq = Sequencer(count_width=count_width, count_fall=compare, count_rise=0)
        self.counter = self.tim.counter
        self.pwm = self.seq.flag
        self.compare = self.seq.count_fall
        self.phase_correct = Signal(1, rst=phase_correct)
        If(self.phase_correct,
            self.comb += self.seq.count_rise.eq(self.tim.max_count - self.compare)
        ).Else(
            self.comb += self.seq.count_rise.eq(0)  # already implicit in reset value
        )


# updown counter used for rotor position tracking
class UpDownCounter(Module):
    def __init__(self, count_width=16, max_count=(2**16)-1):
        self.strobe = Signal(1)
        self.max_count = Signal(count_width+1, rst=max_count)
        self.increment = Signal(count_width)
        self.dir = Signal(1)

        self.counter = Signal(count_width)

        If(self.strobe,
            If(self.dir == 0,
                If((self.counter + self.increment) > self.max_count,
                    self.sync += self.counter.eq((self.counter + self.increment) - self.max_count)
                ).Else(
                    self.sync += self.counter.eq(self.counter + self.increment)
                )
            ).Else(
                If((self.counter - self.increment) < 0,
                    self.sync += self.counter.eq((self.counter - self.increment) + self.max_count)
                ).Else(
                    self.sync += self.counter.eq(self.counter - self.increment)
                )
        )


class PhaseSelector(Module):
    def __init__(self, phase_width=16, phase_max=3*(2**(count_width-2))-1):
        pt = phase_table(phase_max)
        self.phase = Signal(count_width)
        # period where phase can be driven to turn clockwise
        self.uvw_pos  = [Sequencer(count_width=phase_width, count_rise=pt[n]['pos_start'], count_fall=pt[n]['pos_end'] for n in range(3)]
        # period where the phase back-emf should show a rising zero crossing
        self.uvw_fall = [Sequencer(count_width=phase_width, count_rise=pt[n]['fall_start'], count_fall=pt[n]['fall_end'] for n in range(3)]
        # period where phase can be driven to turn counter-clockwise
        self.uvw_neg  = [Sequencer(count_width=phase_width, count_rise=pt[n]['neg_start'], count_fall=pt[n]['neg_end'] for n in range(3)]
        # period where the phase back-emf should show a falling zero crossing
        self.uvw_rise = [Sequencer(count_width=phase_width, count_rise=pt[n]['rise_start'], count_fall=pt[n]['rise_end'] for n in range(3)]
        # common clock
        for n in range(3):
            self.comb += uvw_pos[n].counter.eq(self.phase)
            self.comb += uvw_fall[n].counter.eq(self.phase)
            self.comb += uvw_neg[n].counter.eq(self.phase)
            self.comb += uvw_rise[n].counter.eq(self.phase)


class Inverter(Module):
    def __init__(self, selector=PhaseSelector(count_width=phase_width, max_count=phase_max), phase_width=16, phase_max=3*(2**(count_width-2))-1, pwm_width=16):
        self.dir = Signal(1)
        self.pwm = PWM(count_width=pwm_width)
        self.uvw_bridge = [HalfBridge(l_toff=16, h_toff==16) for n in range(3)]
        self.selector = selector
        self.phase = self.selector.phase
        self.duty = self.pwm.compare

        for n in range(3):
            If(self.dir,
                self.comb += self.uvw_bridge[n].en.eq((self.selector.uvw_pos[n].flag and self.pwm.pwm) or self.selector.uvw_neg[n].flag)
                self.comb += self.uvw_bridge[n].dir.eq(self.selector.uvw_pos[n].flag)
            ).Else
                self.comb += self.uvw_bridge[n].en.eq((self.selector.uvw_neg[n].flag and self.pwm.pwm) or self.selector.uvw_pos[n].flag)
                self.comb += self.uvw_bridge[n].dir.eq(self.selector.uvw_neg[n].flag)
            )

class ObserverEMF(Module):
    def __init__(self, selector=PhaseSelector(count_width=phase_width, max_count=phase_max), phase_width = 16, phase_max=3*(2**(phase_width-2))-1)
        pt = phase_table(phase_max)
        self.selector = selector
        # three comparators, assigned to pins at top level
        self.comp = [Signal(1) for _ in range(3)]
        self.comp_prev = [Signal(1) for _ in range(3)]
        self.comp_rise = [Signal(1) for _ in range(3)]
        self.comp_fall = [Signal(1) for _ in range(3)]
        self.comp_edge = [Signal(1) for _ in range(3)]
        self.comp_mask = [Signal(1) for _ in range(3)]
        self.strobe = Signal(1)
        self.dir = Signal(1)
        self.phase_observation = Signal(phase_width)

        self.comb += self.strobe.eq(self.comp_edge[0] or self.comp_edge[1] or self.comp_edge[2])

        for n in range(3):
            self.sync += self.comp_prev[n].eq(self.comp[n])
            self.comb += self.comp_mask[n].eq(self.selector.uvw_rise[n].flag or self.selector.uvw_fall[n].flag)
            self.comb += self.comp_edge[n].eq( (self.comp[n] != self.comp_prev[n]) and (self.comp_mask[n]==1) )
            self.comb += self.comp_rise[n].eq( (self.comp_edge[n]==1) and (self.comp[n]==1) )
            self.comb += self.comp_fall[n].eq( (self.comp_edge[n]==1) and (self.comp[n]==0) )

            If(self.selector.uvw_rise[n].flag,
                self.comb += self.phase_observation.eq(pt[n]['rise_edge'])
                If(self.comp_rise==1,
                    self.sync += self.dir.eq(0)
                ).Elif(self.comp_fall==1,
                    self.sync += self.dir.eq(1)
                )
            ).Elif(self.selector.uvw_fall[n].flag,
                self.comb += self.phase_observation.eq(pt[n]['fall_edge'])
                If(self.comp_fall==1,
                    self.sync += self.dir.eq(0)
                ).Elif(self.comp_rise==1,
                    self.sync += self.dir.eq(1)
                )
            )
            # XXX assert that phase_observation is never undriven or doubly driven,
            #       either would indicate a timing error

class ObserverHall(Module):
    def __init__(self, phase_width = 16, phase_max=3*(2**(phase_width-2))-1)
        pt = phase_table(phase_max)
        #hall_table={0:8, 1:0,3:1,2:2,6:3,4:4,5:5, 7:8}
        hall_table = Array([0,0,2,1,4,5,3,0])   
        hall_angle_rough = Array([((2*n+ 1)%12)*(phase_max/12) for n in range(6)])
        hall_angle_edge  = Array([((2*n+ 0)%12)*(phase_max/12) for n in range(6)])

        self.phase_angle = Signal(phase_width) # current observation
        self.strobe = Signal(1)        # the observation captures an edge this clock cycle
        self.hall = Signal(3)          # hall inputs
        self.dir = Signal(1)       # direction of last transition
        self.hall_glitch = Signal(1)   # observation is invalid

        hall_prev = Signal(3)          # last captured hall input
        hall_error = Signal(1)         # hall signals are currently invalid
        hall_error_prev = Signal(1)    # hall signals were invalid at last clock
        hall_idx = Signal(3)           # decoded hall position (0..5)
        hall_idx_prev = Signal(3)      # decoded hall position (0..5) at last clock
        dir_prev = Signal(1)       # storage reg for direction

        self.sync += hall_prev.eq(self.hall)
        self.comb += self.strobe.eq(self.hall != hall_prev)
        self.comb += hall_error.eq(self.hall == 0 or self.hall==7)
        self.comb += hall_error_prev.eq(hall_prev == 0 or hall_prev==7)
        self.comb += hall_idx.eq(hall_table[self.hall])
        self.comb += hall_idx_prev.eq(hall_table[hall_prev])

        ).If(self.strobe == False,
            self.comb += self.phase_angle.eq(hall_angle_rough[hall_idx]),
            self.comb += self.dir.eq(dir_prev),
            self.comb += self.hall_glitch.eq(hall_error)

        ).Elif((hall_idx == (hall_idx_prev+1)) or ((hall_idx==0) and (hall_idx_prev==5)),
            self.comb += self.phase_angle.eq(hall_angle_edge[hall_idx]),
            self.comb += self.dir.eq(0),
            self.comb += self.hall_glitch.eq(hall_error or hall_error_prev),
            self.sync += dir_prev.eq(self.dir)

        ).Elif((hall_idx_prev == (hall_idx+1)) or ((hall_idx==5) and (hall_idx_prev==0)),
            self.comb += self.phase_angle.eq(hall_angle_edge[hall_idx_prev]),
            self.comb += self.dir.eq(1),
            self.comb += self.hall_glitch.eq(hall_error or hall_error_prev),
            self.sync += dir_prev.eq(self.dir)

        ).Else(
            self.comb += self.phase_angle.eq(hall_angle_rough[hall_idx]),
            self.comb += self.dir.eq(dir_prev),
            self.comb += self.hall_glitch.eq(True)
        )




# XXX do we update single steps on the phase angle according to a variable prescaler?
#     or do we update multiple steps against a much larger prescaler?
#     conventional systems run a timer between two crossings and take a ratio of that for commutation timers
#     30000rpm, 4pole (100Hz) motor
#     16MHz sysclk, 3*2**14 phase_max, updating by one every clock cycle makes 325Hz 
# drop phase resolution to 3*2**10, run variable prescaler single increment
class PhaseEstimator(Module):
    def __init__(self, phase_width=16, phase_max=3*(2**(phase_width-2))-1, timer_width=16, phase_observation, strobe):
        
        phase_obs_dist = (phase_max+1) / 6 # should be a power of two
        # XXX assert that phase_timer_divider is a power of two

        self.phase_counter = UpDownCounter(count_width=phase_width, max_count=phase_max)

        timer_max = 2**timer_width-1
        self.phase_observation = Signal(phase_width)    # comb set as output of observer
        self.strobe = Signal(1) # comb set as output of observer
        self.glitch = Signal(1) # comb set as output of observer
        self.dir = Signal(1)    # comb set as output of observer
        
        self.timer = Signal(timer_width)        # times the duration between observations, does not overflow
        self.time_capture = Signal(timer_width) # records the interval between edge observations
        
        self.phase_capture = Signal(phase_width)# records the phase at last edge observation

        self.phase_timer = Signal(timer_width)  # counts up to a fraction of time_capture, and increments the phase estimate
        self.phase_timer_prescale = Signal(timer_width)
        
        self.phase_counter.strobe = self.phase_timer_strobe
        self.phase_estimate = self.phase_counter.counter
        self.comb += self.phase_counter.increment.eq(1)
        self.comb += self.phase_counter.dir.eq(self.dir)


        self.comb += self.phase_timer_prescale.eq(self.time_capture / phase_obs_dist) # division by power of two becomes bit shift


        If(self.strobe,
            self.sync += self.timer.eq(0)
            self.comb += self.phase_timer_strobe.eq(0)
            If(self.glitch == False,
                self.sync += self.phase_capture.eq(self.phase_observation)
                self.sync += self.time_capture.eq(self.timer)
                self.sync += self.phase_timer.eq(0)
                self.sync += self.phase_estimate.eq(self.phase_observation)
            )
        ).Else(
            If(timer < timer_max,
                self.sync += self.timer.eq(self.timer + 1)
            )
            If(self.phase_timer >= self.phase_timer_prescale,
                self.comb += self.phase_timer_strobe.eq(1)
                self.sync += self.phase_timer.eq(0)
            ).Else(
                self.comb += self.phase_timer_strobe.eq(0)
                self.sync += self.phase_timer.eq(self.phase_timer + 1)
            )
        )









