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

_raw = [(6562, 6575), (17000, 17069), (20569, 20588), (34000, 34055), (0, 58)]
corresponding = [[Pulse.Sync, Pulse.Horiz, Pulse.Sync, Pulse.Vert, Pulse.Sync]]
_real = [(s*Pulse.CLOCK_SPEED_MHZ, e*Pulse.CLOCK_SPEED_MHZ) for (s, e) in _raw]

test_input = [(_real, 4)]

def convert_to_pulses(json):
    parsed =  [int(s) for s in json.replace(",", "").replace("{", "").replace("}", "").split() if s.isdigit()]
    real = [(s, e) for s, e in zip(*[iter(parsed)]*2)]
    printable = [((s -  real[0][0]) / Pulse.CLOCK_SPEED_MHZ, (e - real[0][0]) / Pulse.CLOCK_SPEED_MHZ) for (s, e) in real]
    return real, str(printable)

jsons = []
'''jsons.append(("{rise = 3068581, fall = 3068875, type = -1}, {rise = 3221448, fall = 3224514, type = -1}, {rise = 3331492, fall = 3331778, type = -1}, {rise = 3488094, fall = 3490834, type = -1}, {rise = 2954749, fall = 2957479, type = -1}", 4))
jsons.append(("{rise = 787753, fall = 789825, type = -1}, {rise = 254404, fall = 256476, type = -1}, {rise = 386774, fall = 387077, type = -1}, {rise = 521092, fall = 524168, type = -1}, {rise = 681141, fall = 681470, type = -1}", 1))
jsons.append(("{rise = 10160584, fall = 10163325, type = -1}, {rise = 10344020, fall = 10344362, type = -1}, {rise = 10427242, fall = 10430315, type = -1}, {rise = 10559854, fall = 10560346, type = -1}, {rise = 10026506, fall = 10026995, type = -1}", 4))
jsons.append(("{rise = 12052884, fall = 12055277, type = -1}, {rise = 12157665, fall = 12158021, type = -1}, {rise = 11624314, fall = 11624673, type = -1}, {rise = 11786206, fall = 11788932, type = -1}, {rise = 11954574, fall = 11954828, type = -1}", 2))
jsons.append(("{rise = 16383488, fall = 16383737, type = -1}, {rise = 16481595, fall = 16483998, type = -1}, {rise = 15948237, fall = 15950632, type = -1}, {rise = 16054524, fall = 16054893, type = -1}, {rise = 16214941, fall = 16217010, type = -1}", 2))
jsons.append(("{rise = 10227231, fall = 10227605, type = -1}, {rise = 10387757, fall = 10389827, type = -1}, {rise = 9854414, fall = 9856485, type = -1}, {rise = 10022795, fall = 10023048, type = -1}, {rise = 10121091, fall = 10124158, type = -1}", 2))
'''
#jsons.append(("{rise = 4653401, fall = 4655794, type = -1}, {rise = 4758619, fall = 4758969, type = -1}, {rise = 4920060, fall = 4922123, type = -1}, {rise = 4386686, fall = 4388737, type = -1}, {rise = 4555723, fall = 4555972, type = -1}", 3))
#jsons.append(("{rise = 5941165, fall = 5941426, type = -1}, {rise = 6039885, fall = 6042954, type = -1}, {rise = 6145084, fall = 6145446, type = -1}, {rise = 6306606, fall = 6309343, type = -1}, {rise = 5773253, fall = 5775323, type = -1}", 4))
#jsons.append(("{rise = 11317363, fall = 11317617, type = -1}, {rise = 11414439, fall = 11416843, type = -1}, {rise = 11521304, fall = 11521675, type = -1}, {rise = 11521304, fall = 11521675, type = -1}, {rise = 11681124, fall = 11683180, type = -1}", 0))
#jsons.append(("{rise = 1317213, fall = 1317468, type = -1}, {rise = 1414494, fall = 1416897, type = -1}, {rise = 1520283, fall = 1520395, type = -1}, {rise = 1521125, fall = 1521495, type = -1}, {rise = 1681148, fall = 1683882, type = -1}", 0))
#jsons.append(("{rise = 3148219, fall = 3148260, type = -1}, {rise = 3161622, fall = 3161888, type = -1}, {rise = 3258289, fall = 3260691, type = -1}, {rise = 3363349, fall = 3363471, type = -1}, {rise = 3364136, fall = 3364500, type = -1}", 0))
#jsons.append(("{rise = 16368729, fall = 16370800, type = -1}, {rise = 16004489, fall = 16004746, type = -1}, {rise = 16102018, fall = 16105088, type = -1}, {rise = 16207507, fall = 16207624, type = -1}, {rise = 16208347, fall = 16208716, type = -1}", 1))

'''jsons.append(("{rise = 15293818, fall = 15296550, type = -1}, {rise = 15416893, fall = 15417169, type = -1}, {rise = 15560525, fall = 15563595, type = -1}, {rise = 15669227, fall = 15669527, type = -1}, {rise = 15860525, fall = 15863595, type = -1}", 0))
jsons.append(("{rise = 7209749, fall = 7212144, type = -1}, {rise = 7316080, fall = 7316264, type = -1}, {rise = 6782719, fall = 6782904, type = -1}, {rise = 6943188, fall = 6945252, type = -1}, {rise = 7067905, fall = 7068083, type = -1}", 2))
jsons.append(("{rise = 12276191, fall = 12278595, type = -1}, {rise = 12384626, fall = 12384926, type = -1}, {rise = 12542622, fall = 12544679, type = -1}, {rise = 12653014, fall = 12653110, type = -1}, {rise = 12665874, fall = 12666149, type = -1}", 0))

jsons.append(("{rise = 13904793, fall = 13905101, type = -1}, {rise = 14061647, fall = 14064379, type = -1}, {rise = 14185373, fall = 14185654, type = -1}, {rise = 13652030, fall = 13652314, type = -1}, {rise = 13794997, fall = 13798062, type = -1}", 3))
jsons.append(("{rise = 1302039, fall = 1304090, type = -1}, {rise = 1385929, fall = 1386169, type = -1}, {rise = 852574, fall = 852818, type = -1}, {rise = 1035381, fall = 1037780, type = -1}, {rise = 1145947, fall = 1146217, type = -1}", 2))
jsons.append(("{rise = 8083553, fall = 8083848, type = -1}, {rise = 8238842, fall = 8241576, type = -1}, {rise = 8322617, fall = 8322872, type = -1}, {rise = 7789265, fall = 7789522, type = -1}, {rise = 7972167, fall = 7975230, type = -1}", 3))
jsons.append(("{rise = 15339778, fall = 15340051, type = -1}, {rise = 15524268, fall = 15527338, type = -1}, {rise = 15634218, fall = 15634526, type = -1}, {rise = 15790983, fall = 15793717, type = -1}, {rise = 15257625, fall = 15259697, type = -1}", 4))
'''
#jsons.append(("{rise = 13670111, fall = 13672842, type = -1}, {rise = 13751962, fall = 13752229, type = -1}, {rise = 13936776, fall = 13939172, type = -1}, {rise = 13403429, fall = 13405829, type = -1}, {rise = 13513247, fall = 13513551, type = -1}", 3))
#jsons.append(("{rise = 5782443, fall = 5782708, type = -1}, {rise = 5939834, fall = 5941906, type = -1}, {rise = 5406484, fall = 5408553, type = -1}, {rise = 5528056, fall = 5528335, type = -1}, {rise = 5673144, fall = 5675548, type = -1}", 2))
#jsons.append(("{rise = 12707383, fall = 12707676, type = -1}, {rise = 12863326, fall = 12865386, type = -1}, {rise = 12985575, fall = 12985849, type = -1}, {rise = 12452221, fall = 12452491, type = -1}, {rise = 12596630, fall = 12599037, type = -1}", 3))
jsons.append(("{rise = 11713044, fall = 11713304, type = -1}, {rise = 11857712, fall = 11860118, type = -1}, {rise = 11967728, fall = 11968002, type = -1}, {rise = 11434379, fall = 11434652, type = -1}, {rise = 11591030, fall = 11593105, type = -1}", 3))
for json, mod in jsons:
    real, printable = convert_to_pulses(json)
    test_input.append((real, mod))
    print("CONVERTED JSON: " + printable)

wsn = [
    [(10022764, 10023035), (9489407, 9489683), (9646077, 9648817), (9768132, 9768388), (9912765, 9915172)],
    [(6562813, 6565220), (6672804, 6673073), (6829472, 6831540), (6296122, 6298862), (6418201, 6418456)],
    [(3212861, 3215932), (3322848, 3323115), (3479534, 3481608), (2946184, 2948921), (3068265, 3068523)],
    [(129558, 131630), (16373421, 16376153), (16495505, 16495761), (16640068, 16642473), (16750054, 16750326)],
    [(13678907, 13679165), (13145561, 13145817), (13290137, 13292541), (13400127, 13400399), (13556831, 13559571)],
    [(9795577, 9795831), (9939964, 9942371), (10049962, 10050228), (10206858, 10209600), (10328925, 10329182)],
    [(6590232, 6592640), (6700234, 6700504), (6856916, 6859654), (6978977, 6979236), (6445620, 6445877)],
    [(3350264, 3350536), (3506920, 3509659), (3628980, 3629240), (3095620, 3095880), (3240255, 3243325)],
    [(16522877, 16523133), (16667514, 16669919), (311, 583), (156973, 159040), (279016, 279277)],
    [(13706269, 13706529), (13850920, 13853990), (13317567, 13319973), (13427582, 13427853), (13584233, 13586308)],
    [(10500973, 10503381), (9967618, 9970023), (10077641, 10077910), (10234300, 10237042), (10356321, 10356580)]
    ]

for w in wsn:
    test_input.append((w, 0))

'''
for i, pulses in enumerate(test_input):
    tracking = None
    for j, pulse in enumerate(pulses):
        tracking = Pulse(pulse[0], pulse[1], parent=tracking)
        assert tracking.type == corresponding[i][j]

print('Pulse Type Tests Passed.')
'''

network = Network.initialize(logging=args.log)

count = 0
for inp in test_input:
    print(inp)
    if network.update(raw_pulses=inp[0], modular_ptr=inp[1]):
        count += 1

network.read()

print
print('DONE! Network Updated. (' + str(count) + '/' + str(len(test_input)) + ' = ' + str(100 * count / float(len(test_input))) + '% transmissions were valid.)')
print

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

pulses = [pulse_t(s, e) for (s, e) in test_input[0][0]]
b, r, p, t, pulses = check(pulses)
assert b
print(degrees(p), 70.2431999986*2)
print(degrees(t), 37.7999999992*2)
for i, p in enumerate(pulses):
    assert p.sync_sweep == corresponding[0][i]

print('Passed C Equivalent Testing.')
