from visual import *
from math import sin, cos, tan, asin, acos, atan, sqrt, pi

class Pulse:
    Nil, Sync, Horiz, Vert = -1, 0, 1, 2

    LOCALIZATION_PERIOD_MS = 300
    PULSE_TRACK_COUNT = 5

    MIN_SYNC_PERIOD_US = 51
    MAX_SYNC_PERIOD_US = 138

    PI = 3.14159265
    SWEEP_PERIOD_US = 16666.666667
    SWEEP_VELOCITY = pi / SWEEP_PERIOD_US

    BEAM_WIDTH_MM = 1.0 # FIXME: Back-Calculate

    CLOCK_SPEED_MHZ = 32.0

    def __init__(self, start, end, prev_pulse=None):
        self.start = start
        self.end = end
        self.prev_pulse = prev_pulse

        self.type = self.classify()

    def classify(self):
        '''
        Classifies the pulse as Nil, Sync, Horiz, or Vert.
        '''
        if (self.prev_pulse and self.prev_pulse.isSync() and period < MIN_SYNC_PERIOD_US): # sweep pulse
            prediction = self.prev_pulse.predictedSweep()
            if prediction == 1 or prediction == 2:
                return prediction

        if (period >= MIN_SYNC_PERIOD_US and period < MAX_SYNC_PERIOD_US): # sync pulse
            if (self.predictedSweep() != 0): # valid pulse (not a skip)
                return Pulse.Sync

        return Pulse.Nil

    ''' Universal Attributes '''

    def period(self):
        '''
        Returns the period of the pulse in microseconds.
        '''
        return Pulse.get_period_us(self.start, self.end)

    ''' Sweep Pulse Methods '''

    def getAngle(self):
        '''
        Returns the calculated angle given by the sweep pulse
        and corresponding sync pulse timing data.

        Raises a TypeError if self is not a Sweep pulse.
        '''
        if not self.isSweep():
            raise TypeError("Pulse must be a Sweep Pulse.")
        return Pulse.get_period_us(self.prev_pulse.end, self.start) * Pulse.SWEEP_VELOCITY

    def getRadial_mm(self):
        '''
        Returns the radial distance in millimeters as defined by the
        timing data given by the sweep pulse.
        '''
        if not self.isSweep():
            raise TypeError("Pulse must be a Sweep Pulse.")
        alpha = self.period() * Pulse.SWEEP_VELOCITY
        return BEAM_WIDTH_MM / (2.0 * sin(alpha / 2.0))

    def getRadial(self):
        '''
        Returns the radial distance in centimeters as defined by the
        timing data given by the sweep pulse.
        '''
        return self.getRadial_mm() * 10

    def getBeamWidth(self, distance_cm):
        '''
        TEST FUNCTION. Back-calculates sweep beam width.
        '''
        if not self.isSweep():
            raise TypeError("Pulse must be a Sweep Pulse.")
        alpha = self.period() * Pulse.SWEEP_VELOCITY
        return 2.0 * distance_cm * sin(alpha / 2.0)

    ''' Sync Pulse Methods '''

    def predictedSweep(self):
        '''
        Returns the predicted sweep axis of the following pulse.
        0 if Skip, 1 if Horizontal, 2 if Vertical.

        Raises a TypeError if self is not a Sync pulse.
        '''
        if not self.isSync():
            raise TypeError("Pulse must be a Sync Pulse.")
        if (self.period() & 0b100) == 1:
            return 0

        return (self.period() & 0b001) + 1

    ''' Type Checking '''

    def isSync(self):
        '''
        Returns whether the pulse is a sync pulse.
        '''
        return self.type == Pulse.Sync

    def isHoriz(self):
        '''
        Returns whether the pulse is a horizontal sweep pulse.
        '''
        return self.type == Pulse.Horiz

    def isVert(self):
        '''
        Returns whether the pulse is a vertical sweep pulse.
        '''
        return self.type == Pulse.Vert

    def isSweep(self):
        return self.isHoriz() or self.isVert()

    def isValid(self):
        '''
        Returns whether the pulse is a valid pulse.

        i.e. not a skip pulse or a sweep pulse without a corresponding sync.
        '''
        return self.type != Pulse.Nil

    ''' Static Methods '''

    def get_period_us(start, end):
        return (end - start) / Pulse.CLOCK_SPEED_MHZ

class Mimsy:

    spherical, cartesian = 0, 1

    def __init__(self, system, data=(0, 0, 0), id=0):
        self.system = system

        if self.systemSpherical():
            self._radial, self._phi, self.theta = [float(elem) for elem in data]
        elif self.systemCartesian():
            self._x, self._y, self._z = [float(elem) for elem in data]
        else:
            raise ValueError('Invalid system input.')

        self.id = id

    def update(self, data):
        if self.systemCartesian():
            self._x, self._y, self._z = [float(elem) for elem in data]
        elif self.systemSpherical():
            self._radial, self._phi, self.theta = [float(elem) for elem in data]

    def x(self):
        if self.systemSpherical():
            return self.radial() * sin(self.theta()) * cos(self.phi())
        return self._x

    def y(self):
        if self.systemSpherical():
            return self.radial() * sin(self.theta()) * sin(self.phi())
        return self._y

    def z(self):
        if self.systemSpherical():
            return self.radial() * cos(self.theta())
        return self._z

    def radial(self):
        if self.systemCartesian():
            return sqrt(self.x()**2 + self.y()**2 + self.z()**2)
        return self._radial

    def theta(self):
        if self.systemCartesian():
            return acos(self.z() / self.radial())
        return self._theta

    def phi(self):
        if self.systemCartesian():
            return atan(self.y() / self.x())
        return self._phi

    def systemSpherical(self):
        return self.system == Mimsy.spherical

    def systemCartesian(self):
        return self.system == Mimsy.cartesian

    def switchSystem(self):
        if self.systemCartesian():
            self._radial = self.radial()
            self._phi = self.phi()
            self._theta = self.theta()
        elif self.systemSpherical():
            self._x = self.x()
            self._y = self.y()
            self._z = self.z()
        self.system = 1 - self.system # flip 0 to 1 and vice versa

class Network:

    VALID_PULSES = [ [Pulse.Sync, Pulse.Horiz, Pulse.Sync, Pulse.Vert],
                        [Pulse.Sync, Pulse.Vert, Pulse.Sync, Pulse.Horiz]]

    TEST_DIST_CM = 100 # 1 meter default

    def __init__(self, system=Mimsy.spherical, reference=(0, 0, 0), station_dims=(1, 1, 1)):
        # Initialize base station
        self.reference = reference
        self.base_station = box(pos=self.reference, length=station_dims[0],
                                width=station_dims[1], height=station_dims[2])

        self.mimsy = Mimsy(system)

    def updateMimsy(self, data=[], raw_pulses=[]):
        if raw_pulses:
            valid, data = Network.parsePulseData(raw_pulses)

        if valid:
            self.mimsy.update(data)
            self.vector = vector(mimsy.x() - self.reference[0], mimsy.y() - self.reference[1], mimsy.z() - self.reference[2])
        else:
            print('Invalid pulse timing data, aborted mimsy position update.')

    ''' Static Methods '''

    @staticmethod
    def parsePulseData(raw_pulses):
        pulses = []
        tracking = None
        for elem in raw_pulses:
            tracking = Pulse(elem[0], elem[1], tracking)
            pulses.append(tracking)

        if not validSequence(pulses):
            return False, None

        r_horiz, r_vert = 0, 0
        phi, theta = 0, 0
        for i, pulse in enumerate(pulses):
            if pulse.isHoriz():
                r_horiz = pulse.getRadial()
                phi = pulse.getAngle()
            elif pulse.isVert():
                r_vert = pulse.getRadial()
                theta = pulse.getAngle()

        radial = (r_horiz + r_vert) / 2.0
        return True, (radial, phi, theta)

    @staticmethod
    def validSequence(pulses):
        received = [p.type for p in pulses]
        return contains(VALID_PULSES[0], received) or contains(VALID_PULSES[1], received)

    @staticmethod
    def contains(sub, array):
        sub_size = len(sub)
        for i in range(0, len(array)):
            if array[i:i+sub_size] == sub:
                return True

        return False
