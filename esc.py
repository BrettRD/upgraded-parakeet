from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
from slow_arithmetic import slow_division
from ppm import PWMinput
# BLDC ESC (BLDC mode, nothing fancy here)


def phase_table(phase_max=12):
    a_360 = int(phase_max)
    a_60 = int(a_360/6)    # 60 degrees of rotation
    a_30 = int(a_360/12)   # 30 degrees of rotation

    hall_dict={
                0b001 : 0x0 ,
                0b011 : 0x1 ,
                0b010 : 0x2 ,
                0b110 : 0x3 ,
                0b100 : 0x4 ,
                0b101 : 0x5 ,
                0b000 : 0x0 | 0b1000,  # error state
                0b111 : 0x0 | 0b1000,  # error state
        }

    # migen doesn't like generating lookup tables from dicts
    hall = [
                8, # 0b000 error
                0, # 0b001
                2, # 0b010
                1, # 0b011
                4, # 0b100
                5, # 0b101
                3, # 0b110
                8, # 0b111 error
    ]

    # phase wire assignments for each phase
    emf_fwd=[   1,     0,     1,     0,     1,     0,       ]   # edge direction for sense coil
    sense=[     0b010, 0b001, 0b100, 0b010, 0b001, 0b100,   ]   # windings used for sense by phase index
    drive_f=[   0b001, 0b010, 0b010, 0b100, 0b100, 0b001,   ]   # windings used for forward-high drive by phase index
    drive_r=[   0b100, 0b100, 0b001, 0b001, 0b010, 0b010,   ]   # windings used for forward-low drive by phase index

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

        self.comb += self.hot.eq((self.out) | (self.timer > 0))
        self.sync +=    If(self.out,
                            self.timer.eq(self.t_off)
                        ).Elif(self.hot,
                            If(self.strobe,
                                self.timer.eq(self.timer - 1)
                            )
                        )


# half-bridge shoot-through protection
# this uses Monostable module to prevent shoot-through
# l_toff and h_toff are used to enforce a switching dead-time for
# gate drivers with low switching speeds
# l_toff: time taken (in sysclock cycles) for the low side driver to turn off
# h_toff: time taken (in sysclock cycles) for the high side driver to turn off
# en:  Signal() turn on one of the transistors
# dir: transistor select (1=high, 0=low)
# l_pin: low side transistor gate drive (active high)
# h_pin: high side transistor gate drive (active high)
class HalfBridge(Module):
    def __init__(self, en, dir, l_pin, h_pin, l_toff=16, h_toff=16):
        self.en = Signal(1)
        self.dir = Signal(1)
        self.submodules.l_gate = Monostable(h_toff, t_max=h_toff)
        self.submodules.h_gate = Monostable(l_toff, t_max=l_toff)
        self.l_out = self.l_gate.out
        self.h_out = self.h_gate.out

        self.comb += [
            self.en.eq(en),
            self.dir.eq(dir),
            self.l_out.eq((self.en == True) and (self.dir==0) and (self.h_gate.hot==False)),
            self.h_out.eq((self.en == True) and (self.dir==1) and (self.l_gate.hot==False)),
            l_pin.eq(self.l_out),
            h_pin.eq(self.h_out),
        ]

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
        self.counter = Signal(max=count_max, min=count_min)
        self.max_count = count_max
        self.min_count = count_min
        self.increment = Signal(max=count_max-count_min)
        self.dir = Signal() # count up is zero

    def sync_step(self):
        return [
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
        ]


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
# pins:  nested list containing gate drive pins for transistors [[a_l,a_h],[b_l,b_h],[c_l,c_h]]
class Inverter(Module):
    def __init__(self, enable, direction, phase, brake, pwm, pins):
        pt = phase_table()
        f_table = Array(pt['drive_f'])  # transistors driven high forward, low reverse
        r_table = Array(pt['drive_r'])  # transistors driven low forward, high reverse

        self.inv_dir = Replicate(direction, 3)    # direction the inverter should spin
        self.inv_en = Replicate(enable, 3)     # global enable
        self.inv_pwm = Replicate(pwm, 3)    # global PWM line
        self.br_dir = Signal(3)     
        self.br_en = Signal(3)

        # instantiate gate drivers and shoot-through protection
        self.submodules.br_a = HalfBridge(self.br_en[0], self.br_dir[0], pins[0][0], pins[0][1], l_toff=16, h_toff=16)
        self.submodules.br_b = HalfBridge(self.br_en[1], self.br_dir[1], pins[1][0], pins[1][1], l_toff=16, h_toff=16)
        self.submodules.br_c = HalfBridge(self.br_en[2], self.br_dir[2], pins[2][0], pins[2][1], l_toff=16, h_toff=16)

        # select drive transistors based on the phase
        self.comb += [
            If(direction == 0,
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
        ]


# observer to look for rising and falling edges on the undriven phase, outputs a strobe
#
# pt:  phase table with observables and masks embedded
# self.comp: comparator inputs
# self.strobe: rises as the relevant phase makes a zero-crossing
# self.phase_observation: the angle of the zero-crossing marked by the strobe
# self.glitch:  marks observations as invalid if the direction changed
# self.phase:  the approximate rotor position (0-6)
# XXX this is probably going to suffer from commutation noise, 
#     may need to mask the comparators deeper into the cycle
# XXX find a way to test for muliple zero crossings per phase
class ObserverEMF(Module):
    def __init__(self, pt, comp_pins):

        # fetch parts of the phase table
        s_t = Array(pt['sense'])    # sense mask
        d_t = Array(pt['emf_fwd'])  # expected edge direction for fwd rotation
        e_t = Array(pt['zeroes'])    # angles for the given edges

        # three comparators, assigned to pins at top level
        self.comp = Signal(3)
        self.strobe = Signal()
        self.phase_observation = Signal(max=pt['a_360'])
        self.glitch = Signal()
        self.phase = Signal(3)

        comp_edge = Signal(3)
        comp_prev = Signal(3)
        edge_dir = Signal()
        glitch_prev = Signal() # store the validity of the last measurement

        # capture the comparator inputs for edge detection
        self.sync += comp_prev.eq(self.comp)

        # turn the three comparators and masks into a strobe for the zero crossing
        self.comb += [
                self.comp.eq(comp_pins),                    # capture the input pins
                self.phase_observation.eq(e_t[self.phase]),              # output the angle we expect to find the upcoming edge at
                comp_edge.eq(s_t[self.phase] & (self.comp ^ comp_prev)), # mask out irrelevant edges according to the phase table (delete PWM noise)
                self.strobe.eq(comp_edge != 0),                     # set a strobe on every relevant comparator edge
                edge_dir.eq((comp_edge & self.comp) != 0),          # determine if the edge was rising or falling

                If(self.strobe,
                    self.glitch.eq(d_t[self.phase] ^ edge_dir)      # a glitch is an unexpected zero crossing
                ).Else(
                    self.glitch.eq(glitch_prev)
                ),
            ]

        # store state values for after the strobe
        self.sync += If(self.strobe,
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
    def __init__(self, pt, hall_pins):
        hall_t = Array(pt['hall'])
        trans_t = Array(pt['peaks'])                # angles for the transitions of the hall signals
        dwell_t = Array(pt['zeroes'])               # angles for settled hall signals

        self.hall = Signal(3)                       # hall inputs
        self.dir = Signal()                         # direction of last transition
        self.strobe = Signal()                      # the observation captures a transition this clock cycle
        self.phase_observation = Signal(max=pt['a_360'])  # observation output
        self.glitch = Signal()                 # observation is invalid

        hall_prev = Signal(3)           # last captured hall input
        hall_error = Signal()           # hall signals are currently invalid
        hall_error_prev = Signal()      # hall signals were invalid at last clock
        hall_idx = Signal(3)            # decoded hall position (0..5)
        hall_idx_prev = Signal(3)       # decoded hall position (0..5) at last clock
        dir_prev = Signal()             # storage reg for direction

        # capture the hall input and decode it.
        self.sync += hall_prev.eq(self.hall)
        self.comb += [
                        self.hall.eq(hall_pins),
                        Cat(hall_idx,      hall_error      ).eq(   hall_t[self.hall]  ),
                        Cat(hall_idx_prev, hall_error_prev ).eq(   hall_t[hall_prev]  ),
                    ]

        self.comb +=    If((self.hall == hall_prev),
                            self.phase_observation.eq(dwell_t[hall_idx]),
                            self.dir.eq(dir_prev),
                            self.glitch.eq(hall_error)
                        ).Elif(((hall_idx == (hall_idx_prev + 1)) | ((hall_idx==0) & (hall_idx_prev==5))),
                            self.phase_observation.eq(trans_t[hall_idx]),
                            self.dir.eq(0),
                            self.glitch.eq(hall_error | hall_error_prev),
                        ).Elif(((hall_idx == (hall_idx_prev - 1)) | ((hall_idx==5) & (hall_idx_prev==0))),
                            self.phase_observation.eq(trans_t[hall_idx_prev]-1),
                            self.dir.eq(1),
                            self.glitch.eq(hall_error | hall_error_prev),
                        ).Else(
                            self.phase_observation.eq(dwell_t[hall_idx]),
                            self.dir.eq(dir_prev),
                            self.glitch.eq(True)
                        )
        self.comb +=    self.strobe.eq(self.hall != hall_prev)
        self.sync +=    If((self.strobe == True)  & (self.glitch == False),
                            If((hall_idx == (hall_idx_prev+1)) | ((hall_idx==0) & (hall_idx_prev==5)),
                                dir_prev.eq(self.dir)
                            ).Elif((hall_idx_prev == (hall_idx+1)) | ((hall_idx==5) & (hall_idx_prev==0)),
                                dir_prev.eq(self.dir)
                            )
                        )




# VelocityEstimator
# estimate the phase of the rotor based on various information sources
# XXX can the direction be pulled from the comparator parity with the
#     phase sector? or is PWM noise too much?
#
class VelocityEstimator(Module):
    def __init__(self, pt, observer, v_min=1):
        self.strobe = observer.strobe # output of observer
        self.glitch = observer.glitch # output of observer
        self.phase_next_obs = observer.phase_observation # output of observer

        self.distance = Signal(max=pt['a_360'], reset=pt['a_60'])

        self.submodules.div = slow_division(num_width=self.distance.nbits, den_width=self.distance.nbits)  # 32b width 

        t_max = int((pt['a_360']/6) / v_min)

        self.obs_timer = Signal(max=pt['a_360'])        

        self.phase_last_obs = Signal(max=pt['a_360'])     # records the observer output at last valid edge
        self.last_obs_ok = Signal()

        self.velocity_strobe = Signal()
        self.velocity = Signal(max=pt['a_360'], reset=0)  # velocity estimate
        self.velocity_d = Signal(max=pt['a_360'], reset=0)  # velocity estimate
        self.ready = Signal(reset=1)                    # track the state of divider
        self.velocity_good = Signal()                       # the currently held velocity is still valid

        
        
            # XXX deal with rollover correctly
        #    If(phase_next_obs >= phase_last_obs,
        #        self.distance.eq(phase_next_obs - phase_last_obs),
        #    ).Else(
        #        self.distance.eq(phase_last_obs - phase_next_obs),
        #    ),
        self.comb += [
            self.velocity_strobe.eq((self.div.stop == 1) & (self.ready == 0)),
            If(self.velocity_strobe,
                self.velocity.eq(self.div.quotient),
            ).Else(
                self.velocity.eq(self.velocity_d),
            )
        ]

        # implement dx/dt = x/t
        self.sync += [
            If(self.strobe == True,
                If(self.glitch == False,
                    # capture the observation
                    self.phase_last_obs.eq(self.phase_next_obs),
                    self.last_obs_ok.eq(True),
                    self.obs_timer.eq(0),

                    # if the last observation was also good, load the numbers into the divider
                    # XXX check the distance is non-zero
                    If(self.last_obs_ok == True,
                        If((self.div.stop == 1) & (self.ready==1),   # check the divider is ready
                            If((self.distance>0) & (self.obs_timer>0),
                                self.div.start(self.distance, self.obs_timer),
                                self.ready.eq(0)
                            )
                        )
                    )
                ).Else(
                    # strobe and glitch, this position observation is broken
                    self.last_obs_ok.eq(False),
                    self.velocity_good.eq(False),
                ),
            ).Else(
                If(t_max > self.obs_timer + 1,
                    self.obs_timer.eq(self.obs_timer+1),
                ).Else(
                    self.last_obs_ok.eq(False)  # last overflow was too long ago to be useful
                ),
            ),

            # write the divider result to the estimate
            If(self.velocity_strobe,
                self.ready.eq(1),
                self.velocity_d.eq(self.div.quotient),
                self.velocity_good.eq(True),
            ),
        ]







class PositionEstimator(Module):
    def __init__(self, pt, sensor_fusion):

        # signals from the selected observer
        self.phase_observation = sensor_fusion.selected_position
        self.phase_observation_strobe = sensor_fusion.p_strobe
        self.velocity = sensor_fusion.selected_velocity
        self.velocity_observation_strobe = sensor_fusion.v_strobe
        self.dir = sensor_fusion.selected_dir
        self.dir_strobe = sensor_fusion.d_strobe

        self.submodules.phase_counter = UpDownCounter(count_max=pt['a_360'])
        #self.phase_counter.counter
        #self.phase_counter.increment
        #self.phase_counter.dir

        self.phase_estimate = self.phase_counter.counter
        self.comb += self.phase_counter.dir.eq(self.dir)

        # if an observation becomes valid, sync to it.
        self.sync += [
            If(self.phase_observation_strobe,
                self.phase_counter.counter.eq(self.phase_observation)
            ).Else(
                # XXX consider a prescaler
                self.phase_counter.sync_step()
            ),
            If(self.velocity_observation_strobe,
                self.phase_counter.increment.eq(self.velocity),
            ),
            If(self.dir_strobe,
                self.phase_counter.dir.eq(self.dir),
            ),
        ]

        # feed the estimate back into the emf observer via sensor_fusion
        self.comb += [
            sensor_fusion.pos_est.eq(self.phase_estimate)
        ]



# pick the data source, nothing fancy here
class ESCSensorFusion(Module):
    def __init__(self, pt, emf_v_min, hall_p, hall_v, emf_p, emf_v, open_v):

        # one velocity estimator per sensor source
        # one position estimator in total

        self.emf_p_obs = emf_p.phase_observation
        self.emf_p_strobe = emf_p.strobe
        self.emf_p_ok = Signal()
        self.comb += self.emf_p_ok.eq(emf_p.glitch == 0)
        self.emf_v_obs = emf_v.velocity
        self.emf_v_strobe = emf_v.velocity_strobe
        self.emf_v_ok = emf_v.velocity_good
        self.emf_d = Signal()   # not used,  XXX velocity should be signed

        self.hall_p_obs = hall_p.phase_observation
        self.hall_p_strobe = hall_p.strobe
        self.hall_p_ok = Signal()
        self.comb += self.hall_p_ok.eq(hall_p.glitch == 0)
        self.hall_v_obs = hall_v.velocity
        self.hall_v_strobe = hall_v.velocity_strobe
        self.hall_v_ok = hall_v.velocity_good
        self.hall_d = hall_p.dir  # valid on p_strobe

        self.open_v_obs = open_v.velocity   # fallback velocity input for ramp generator
        self.open_d = open_v.dir          
        self.open_v_strobe = open_v.velocity_strobe


        self.selected_position = Signal(max=pt['a_360'])    #input from position estimator observation
        self.p_strobe = Signal()                        #input from position estimator observation strobe
        
        self.selected_velocity = Signal(max=pt['a_360'])    #input from position estimator velocity
        self.v_strobe = Signal()                        #input from position estimator velocity strobe

        self.selected_dir = Signal()                    #input from position estimator direction
        self.d_strobe = Signal()                    #input from position estimator direction

        self.emf_v_min = Signal(max=pt['a_360'], reset=emf_v_min)
        self.hall_v_min = Signal(max=pt['a_360'], reset=1)

        self.emf_mode = Signal()    # phase comes from velocity estimator
        self.hall_mode = Signal()   # phase comes from hall sensor
        self.open_mode  = Signal()  # phase comes from free-running position estimate with velocity ramp

        self.pos_est = Signal(max=pt['a_360'])  # from phase estimator


        self.inverter_phase = Signal(max=6)

        self.comb += [
            # if EMF is good, completely ignore the hall effect
            If((self.emf_v_obs > self.emf_v_min) & self.emf_v_ok,
                If(self.emf_v_strobe,
                    self.selected_velocity.eq(self.emf_v_obs),
                    self.v_strobe.eq(1),
                ),
                If(self.emf_p_strobe,
                    self.selected_position.eq(self.emf_p_obs),
                    self.p_strobe.eq(1),
                ),
                self.inverter_phase.eq(self.pos_est[-3:]),
                self.emf_mode.eq(1),
            ).Else(
                # if hall velocity is good, capture the value
                If((self.hall_v_obs > self.hall_v_min) & self.hall_v_ok,
                    If(self.hall_v_strobe,
                        self.selected_velocity.eq(self.hall_v_obs),
                        self.v_strobe.eq(1),
                    )
                ),
                # direction is only valid on a hall edge
                If(self.hall_p_strobe,
                    self.selected_position.eq(self.hall_p_obs),
                    self.p_strobe.eq(1),
                    self.selected_dir.eq(self.hall_d),
                    self.d_strobe.eq(1),
                ),
                # if the hall effect decode is valid, use it directly, otherwise use the last estimate
                If(self.hall_p_ok,
                    self.inverter_phase.eq(self.hall_p_obs[-3:]),
                    self.hall_mode.eq(1),
                ).Else(
                    self.inverter_phase.eq(self.pos_est[-3:]),
                    self.open_mode.eq(1),
                    If(self.open_v_strobe,          # in open loop mode, allow the throttle to send a velocity ramp
                        self.selected_velocity.eq(self.open_v_obs),
                        self.selected_dir.eq(self.open_d),
                        self.v_strobe.eq(1),
                    )
                )
            )
        ]



class Throttle(Module):
    def __init__(self, max=2**12, min=0, duty_max=2**12):
        assert(max > min)
        assert(min >= 0)

        self.input_strobe = Signal()
        self.input    = Signal(max=max, min=min, reset=int((max+min)/2))
        self.centre    = Signal(max=max, min=min, reset=int((max+min)/2))
        self.dead_band = Signal(max=max, min=min, reset=int((max-min)/20))
        self.input_timeout = Signal()

        self.arm = Signal()
        self.arm_d = Signal()

        self.dir = Signal()
        self.output = Signal(max=duty_max)
        self.output_enable = Signal()
        self.output_strobe = Signal()

        self.comb += [
            If(self.arm,
                If(self.input > self.centre,
                    self.dir.eq(0),   # 0 is forwards
                    If((self.input - self.centre) - self.dead_band > 0,
                        self.output.eq((self.input - self.centre) - self.dead_band),
                        self.output_enable.eq(1),
                    )
                ).Else(
                    self.dir.eq(1),
                    If((self.centre - self.input) - self.dead_band > 0,
                        self.output.eq((self.centre - self.input) - self.dead_band),
                        self.output_enable.eq(1),
                    )
                )
            ),
            If(self.input_timeout == 1,
                self.arm.eq(False),
            ).Else(
                self.arm.eq(self.arm_d),
            ),
            If(self.arm,
                self.output_strobe.eq(self.input_strobe),
            ).Else(
                self.output_strobe.eq( (self.arm == 0) & (self.arm_d == 1) ),
            )
        ]

        self.sync += [
            If(self.input_timeout == 1,
                self.arm_d.eq(0)

            ).Else(
                If(self.input_strobe,
                    If(self.input > self.centre,
                        If(self.input < (self.centre + self.dead_band),
                            self.arm_d.eq(True),
                        )
                    ).Else(
                        If(self.input > (self.centre - self.dead_band),
                            self.arm_d.eq(True),
                        )
                    )
                )
            )
        ]



# hacky wiring class to allow a sensorless motor to start
# not actually a ramp, XXX develop mutlitple control loops to map command input
class VelocityRamp(Module):
    def __init__(self, pt, v_default, command_src):
        self.command_input = command_src.output
        self.command_dir = command_src.dir
        self.command_stobe = Signal()
        self.velocity = Signal(max=pt['a_360'])
        self.velocity_strobe = Signal()
        self.dir = Signal()
        self.comb += [
            self.velocity.eq(v_default),
            self.velocity_strobe.eq(self.command_stobe),
            self.dir.eq(self.command_dir),
        ]



# pt=phase_table(phase_max=12*(2**24))
# inverter pins = [[inv_pin_a_l, inv_pin_a_h], [inv_pin_b_l, inv_pin_b_h], [inv_pin_c_l, inv_pin_c_h]]
# servo_pin = Signal()
class ESC(Module):
    def __init__(self, pt, clk_period, inverter_pins, comp_pins, hall_pins, servo_pin):

        self.pwm_max = (2**16)
        self.emf_v_min = int(pt['a_360'] / (1e9/clk_period))  # 1 rotation per second

        self.inv_enable = Signal()
        self.inv_dir = Signal()
        self.inv_phase = Signal(max=6)
        self.inv_brake = Signal()
        self.inv_pwm = Signal()
        self.inv_pins = inverter_pins

        self.servo_input = servo_pin

        self.submodules.throttle = Throttle()
        self.submodules.pos_emf  = ObserverEMF(pt, comp_pins)
        self.submodules.pos_hall = ObserverHall(pt, hall_pins)
        self.submodules.vel_emf  = VelocityEstimator(pt, self.pos_emf, v_min=self.emf_v_min)
        self.submodules.vel_hall = VelocityEstimator(pt, self.pos_hall)
        self.submodules.vel_open = VelocityRamp(pt=pt, v_default=2*self.emf_v_min, command_src=self.throttle)
        self.submodules.sensor_fusion = ESCSensorFusion(pt, self.emf_v_min,
                                                            self.pos_hall, self.vel_hall,
                                                            self.pos_emf , self.vel_emf,
                                                            self.vel_open
                                                        )
        self.submodules.pos_est  = PositionEstimator(pt, self.sensor_fusion)
        self.submodules.inverter = Inverter(
                                                self.inv_enable,
                                                self.inv_dir,
                                                self.inv_phase,
                                                self.inv_brake,
                                                self.inv_pwm,
                                                self.inv_pins
                                            )
        self.submodules.drive_pwm = PWM(count_max=self.pwm_max)

        self.submodules.servo_capture = PWMinput(clk_period=clk_period, resolution=1e3, max_width=2000e3)

        # XXX this should become a throttle control loop
        # control duty, velocity, and position

        self.comb += [
            # control input is via servo pwm
            self.servo_capture.pwm.eq(self.servo_input),
            # throttle handles arming and fwd/rev
            self.throttle.input.eq(self.servo_capture.width),
            self.throttle.input_strobe.eq(self.servo_capture.strobe),
            self.throttle.input_timeout.eq(self.servo_capture.overflow),
            # duty control mode with braking
            self.drive_pwm.compare.eq(self.throttle.output),
            self.inv_enable.eq(self.throttle.output_enable),
            self.inv_phase.eq(self.sensor_fusion.inverter_phase),
            self.pos_emf.phase.eq(self.inv_phase),
            self.inv_pwm.eq(self.drive_pwm.pwm),
            If(self.pos_est.dir != self.throttle.dir,
                If(self.sensor_fusion.emf_mode == True,
                    self.inv_dir.eq(self.pos_est.dir),
                    self.inv_brake.eq(1),   # apply braking PWM to the positive phase
                ).Else(
                    self.inv_dir.eq(self.throttle.dir),
                    self.inv_brake.eq(0),   # apply counter torque
                ),
            ).Else(
                    self.inv_dir.eq(self.pos_est.dir),
                    self.inv_brake.eq(1),   # apply torque with rotation
            ),
        ]
        # self.sensor_fusion.inverter_phase # used in preference to phase estimate
        # self.pos_est.phase_estimate # not used directly

