from visualization import *
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--log", action='store_true', help='a logging flag')
args = parser.parse_args()

''' Tests '''
basic_valid = [0, 1, 2]

basic_pass = [[0, 1, 2, 3], [3, 4, 0, 1, 2], [1, 2, 0, 1, 0, 1, 2, 1]]
basic_fail = [[1, 2, 0, 0], [2, 1, 0, 0, 1], [3, 4, 5, 6]]

for p in basic_pass:
    assert Network.contains(basic_valid, p)
for f in basic_fail:
    assert not Network.contains(basic_valid, f)

print('Basic Contain Tests Passed.')

sequences_pass = [[Pulse.Nil, Pulse.Sync, Pulse.Vert, Pulse.Sync, Pulse.Horiz],
                [Pulse.Sync, Pulse.Vert, Pulse.Sync, Pulse.Horiz, Pulse.Sync],
                [Pulse.Sync, Pulse.Horiz, Pulse.Sync, Pulse.Vert, Pulse.Sync],
                [Pulse.Sync, Pulse.Horiz, Pulse.Sync, Pulse.Vert, Pulse.Nil]]
sequences_fail = [[Pulse.Nil, Pulse.Nil, Pulse.Nil, Pulse.Sync, Pulse.Horiz],
                [Pulse.Sync, Pulse.Vert, Pulse.Sync, Pulse.Vert, Pulse.Sync],
                [Pulse.Sync, Pulse.Horiz, Pulse.Sync, Pulse.Nil, Pulse.Sync],
                [Pulse.Nil, Pulse.Sync, Pulse.Horiz, Pulse.Sync, Pulse.Horiz]]

for p in sequences_pass:
    assert Network.validSequence(p)
for f in sequences_fail:
    assert not Network.validSequence(f)

print('Valid Sequence Tests Passed.')

test_input = [[(0, 58), (6562, 6575), (17000, 17069), (20569, 20588), (34000, 34055)]]
corresponding = [[Pulse.Sync, Pulse.Horiz, Pulse.Sync, Pulse.Vert, Pulse.Sync]]
test_input = [[(s*Pulse.CLOCK_SPEED_MHZ, e*Pulse.CLOCK_SPEED_MHZ) for (s, e) in pulses] for pulses in test_input]

json = "start = 28209434, end = 28211509, start = 28342127, end = 28343340, start = 28476125, end = 28479205, start = 28599659, end = 28603329, start = 28605252, end = 28605834"
parsed =  [int(s) for s in json.replace(",", "").replace("{", "").replace("}", "").split() if s.isdigit()]
real = [(s, e) for s, e in zip(*[iter(parsed)]*2)]

print(real)
test_input.append(real)

print([((s -  real[0][0]) / Pulse.CLOCK_SPEED_MHZ, (e - real[0][0]) / Pulse.CLOCK_SPEED_MHZ) for (s, e) in real])

'''
for i, pulses in enumerate(test_input):
    tracking = None
    for j, pulse in enumerate(pulses):
        tracking = Pulse(pulse[0], pulse[1], parent=tracking)
        assert tracking.type == corresponding[i][j]

print('Pulse Type Tests Passed.')
'''

network = Network.initialize(logging=args.log)

for inp in test_input:
    network.update(raw_pulses=inp)

network.read()

print('Network Updated.')

''' C Equivalent Testing '''

class pulse_t:
    def __init__(self, start, end, sync_sweep=-1):
        self.start = start
        self.end = end
        self.sync_sweep = sync_sweep

    def __repr__(self):
        return "pulse_t<s: " + str(self.start) + ", e: " + str(self.end) + ", t: " + str(self.sync_sweep) + ">"

def check(pulses_local, sweep_velocity = Pulse.SWEEP_VELOCITY):
    PULSE_TRACK_COUNT = 5
    init_sync_index = PULSE_TRACK_COUNT

    valid_seq_a = [ Pulse.Sync, Pulse.Horiz, Pulse.Sync, Pulse.Vert ]
    valid_seq_b = [ Pulse.Sync, Pulse.Vert, Pulse.Sync, Pulse.Horiz ]
    sweep_axes_check = 0
    for i in range(PULSE_TRACK_COUNT):
        period = Pulse.get_period_us(pulses_local[i].start, pulses_local[i].end)
        if (period < Pulse.MIN_SYNC_PERIOD_US): # sweep pulse
            if (init_sync_index != PULSE_TRACK_COUNT):
                parent_period = Pulse.get_period_us(pulses_local[i-1].start, pulses_local[i-1].end)
                axis = (int((48*parent_period - 2501) / 500.0) & 0b001) + 1
                pulses_local[i].sync_sweep = axis # 1 if horizontal, 2 if vertical

                ind = i - init_sync_index
                if (axis == valid_seq_a[ind] or axis == valid_seq_b[ind]):
                    sweep_axes_check += axis
                else:
                    return False, -1, -1, -1, []
        elif (period < Pulse.MAX_SYNC_PERIOD_US): # sync pulse
            if ((int((48*period - 2501) / 500.0) & 0b100) >> 2 == 1): # skip pulse
                if (i > 0 and i < PULSE_TRACK_COUNT-1):
                    return False, -1, -1, -1, []
            elif (init_sync_index == PULSE_TRACK_COUNT):
                init_sync_index = i # set initial valid sync pulse index
            pulses_local[i].sync_sweep = Pulse.Sync
        else: # neither
            pulses_local[i].sync_sweep = -1;
            return False, -1, -1, -1, []

    if (init_sync_index == PULSE_TRACK_COUNT or sweep_axes_check != 3): return False, -1, -1, -1, []

    r_init, phi_init, theta_init = False, False, False

    for i in range(init_sync_index, PULSE_TRACK_COUNT-1):
        curr_pulse = pulses_local[i]
        next_pulse = pulses_local[i+1]

        if next_pulse.sync_sweep == Pulse.Sync:
            r = Pulse.get_period_us(next_pulse.start, next_pulse.end)
            r_init = True
        elif next_pulse.sync_sweep == Pulse.Horiz:
            phi = Pulse.get_period_us(curr_pulse.end, next_pulse.start) * sweep_velocity
            phi_init = True
        elif next_pulse.sync_sweep == Pulse.Vert:
            theta = Pulse.get_period_us(curr_pulse.end, next_pulse.start) * sweep_velocity
            theta_init = True
        else:
            return False, -1, -1, -1, []

    return r_init and phi_init and theta_init, r, phi, theta, pulses_local

pulses = [pulse_t(s, e) for (s, e) in test_input[0]]
b, r, p, t, pulses = check(pulses)
assert b
print(degrees(p), 70.2431999986*2)
print(degrees(t), 37.7999999992*2)
for i, p in enumerate(pulses):
    assert p.sync_sweep == corresponding[0][i]

print('Passed C Equivalent Testing.')
