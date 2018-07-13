from visualization import *

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

print('Valid Sequence Tests Passed')

test_input = [[(0, 58), (6562, 6575), (17000, 17069), (20569, 20588), (34000, 34055)]]
corresponding = [[Pulse.Sync, Pulse.Horiz, Pulse.Sync, Pulse.Vert, Pulse.Sync]]
test_input = [[(s*Pulse.CLOCK_SPEED_MHZ, e*Pulse.CLOCK_SPEED_MHZ) for (s, e) in pulses] for pulses in test_input]

for i, pulses in enumerate(test_input):
    tracking = None
    for j, pulse in enumerate(pulses):
        tracking = Pulse(pulse[0], pulse[1], parent=tracking)
        assert tracking.type == corresponding[i][j]

print('Pulse Type Tests Passed')

network = Network.initialize()

for inp in test_input:
    network.update(raw_pulses=inp)

print('Network Updated')
