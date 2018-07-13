from visual import *
from math import sin, cos, tan, asin, acos, atan, sqrt, pi, radians
import datetime

class Pulse:
    Unclassified, Nil, Sync, Horiz, Vert = -69, -1, 0, 1, 2

    LOCALIZATION_PERIOD_MS = 300
    PULSE_TRACK_COUNT = 5

    MIN_SYNC_PERIOD_US = 51
    MAX_SYNC_PERIOD_US = 138

    PI = 3.14159265
    SWEEP_PERIOD_US = 8333.3333
    SWEEP_VELOCITY = pi / SWEEP_PERIOD_US

    SENSOR_WIDTH_MM = 1.0 # FIXME: Look at data-sheet or measure
    SENSOR_WIDTH_CM = SENSOR_WIDTH_MM / 10.0

    CLOCK_SPEED_MHZ = 32.0

    def __init__(self, start, end, parent=None):
        self.start = start
        self.end = end
        self.parent = parent

        self.type = Pulse.Unclassified
        self.type = self.classify()

    def classify(self):
        '''
        Classifies the pulse as Nil, Sync, Horiz, or Vert.
        '''
        if (self.parent and self.parent.isSync()):
            if (self.period() < Pulse.MIN_SYNC_PERIOD_US): # sweep pulse
                prediction = self.parent.predictedSweep()
                if prediction == Pulse.Horiz or prediction == Pulse.Vert:
                    return prediction
        elif (self.period() >= Pulse.MIN_SYNC_PERIOD_US and self.period() < Pulse.MAX_SYNC_PERIOD_US
            and self.predictedSweep() != 0): # sync pulse (not skip)
                return Pulse.Sync

        return Pulse.Nil

    def __repr__(self):
        if self.isSync():
            end = ", predicted=" + str(self.predictedSweep()) + ">"
        elif self.isSweep():
            end = ", hit=" + str(self.sweepHit()) + ", parent=" + self.parent.typeString() + ">"
        else:
            end = ">"
        return "Pulse Object:<period=" + str(self.period()) + ", type=" + self.typeString() + end

    ''' Universal Attributes '''

    def period(self):
        '''
        Returns the period of the pulse in microseconds.
        '''
        return Pulse.get_period_us(self.start, self.end)

    ''' Sweep Pulse Methods '''

    def sweepHit(self):
        '''
        Returns the time in milliseconds that the sweep beam
        took to hit the photodiode after the sync.
        '''
        if self.isClassified() and not self.isSweep():
            raise TypeError("Pulse must be a Sweep Pulse.")
        return Pulse.get_period_us(self.parent.end, self.start) / 1000.0

    def getAngle(self):
        '''
        Returns the calculated angle given by the sweep pulse
        and corresponding sync pulse timing data.

        Raises a TypeError if self is not a Sweep pulse.
        '''
        if self.isClassified() and not self.isSweep():
            raise TypeError("Pulse must be a Sweep Pulse.")
        return self.sweepHit() * Pulse.SWEEP_VELOCITY

    def getRadial_mm(self, phi=90):
        '''
        Returns the radial distance in millimeters as defined by the
        timing data given by the sweep pulse.
        '''
        if self.isClassified() and not self.isSweep():
            raise TypeError("Pulse must be a Sweep Pulse.")
        alpha = self.period() * Pulse.SWEEP_VELOCITY
        return Pulse.SENSOR_WIDTH_MM * sin(radians(phi)) / float(alpha)

    def getRadial(self, phi=90):
        '''
        Returns the radial distance in centimeters as defined by the
        timing data given by the sweep pulse.
        '''
        return self.getRadial_mm(phi) / 10.0

    def getEffectiveSensorWidth_mm(self, distance_mm, phi=90):
        '''
        TEST FUNCTION. Back-calculates effective sensor width in millimeters.
        '''
        if self.isClassified() and not self.isSweep():
            raise TypeError("Pulse must be a Sweep Pulse.")
        alpha = self.period() * Pulse.SWEEP_VELOCITY
        return distance_mm * alpha / sin(radians(phi))

    def getEffectiveSensorWidth(self, distance_cm, phi=90):
        '''
        TEST FUNCTION. Back-calculates effective sensor width in centimeters.
        '''
        return self.getEffectiveSensorWidth_mm(distance_cm * 10.0) / 10.0

    ''' Sync Pulse Methods '''

    def predictedSweep(self):
        '''
        Returns the predicted sweep axis of the following pulse.
        0 if Skip, 1 if Horizontal, 2 if Vertical.

        Raises a TypeError if self is not a Sync pulse.
        '''
        if self.isClassified() and not self.isSync():
            raise TypeError("Pulse must be a Sync Pulse.")
        bits = self.syncBits()
        if (bits & 0b100) == 1:
            return 0
        return (bits & 0b001) + 1

    def syncBits(self):
        '''
        Returns a number defining our 3 information bits: skip, data, axis.
        Given by sync pulse period in microseconds (us).
        '''
        if self.isClassified() and not self.isSync():
            raise TypeError("Pulse must be a Sync Pulse.")
        return int((48*self.period() - 2501) / 500.0)

    ''' Type Checking '''

    def typeString(self):
        '''
        Returns the corresponding type string for self.type.
        '''
        if self.isValid():
            if self.isSync():
                return "Sync"
            elif self.isHoriz():
                return "Horiz"
            elif self.isVert():
                return "Vert"
            else:
                return "Unclassified"
        else:
            return "Nil"

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

    def isClassified(self):
        '''
        Returns whether the pulse is unclassified.
        '''
        return self.type != Pulse.Unclassified

    ''' Static Methods '''

    @staticmethod
    def get_period_us(start, end):
        if end > start:
            return (end - start) / Pulse.CLOCK_SPEED_MHZ
        else:
            return ((0xFFFFFFFF - start) + end) / Pulse.CLOCK_SPEED_MHZ

class Mimsy:

    spherical, cartesian = 0, 1

    def __init__(self, system, data=(0, 0, 0), id=0):
        self.system = system

        if self.systemSpherical():
            self._radial, self._phi, self._theta = [float(elem) for elem in data]
        elif self.systemCartesian():
            self._x, self._y, self._z = [float(elem) for elem in data]
        else:
            raise ValueError('Invalid system input.')

        self.id = id

    def update(self, data):
        if self.systemCartesian():
            self._x, self._y, self._z = [float(elem) for elem in data]
        elif self.systemSpherical():
            self._radial, self._phi, self._theta = [float(elem) for elem in data]

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
                        [Pulse.Sync, Pulse.Vert, Pulse.Sync, Pulse.Horiz] ]

    MAX_RANGE_CM = 1.5 * 100
    TEST_DIST_CM = 100 # 1 meter default

    LOGS = './logs/'

    def __init__(self, system=Mimsy.spherical, reference=(0, 0, 0), station_dims=(10, 6.35, 10), logging=True):
        self.logging = logging

        # Initialize base station
        self.reference = reference

        self.scene = display(title='Network Visualization', x=0, y=0)
        self.scene.background = (0.5,0.5,0.5)

        self.scene.lights = [vector(1,0,0), vector(0, 1, 0), vector(0, 0, 1), \
            vector(-1,0,0), vector(0, -1, 0), vector(0, 0, -1)]
        self.scene.ambient = 0

        self.scene.forward = (1,-1,-1)
        self.scene.range = Network.MAX_RANGE_CM
        self.base_station = box(pos=self.reference, length=station_dims[0],
                                width=station_dims[1], height=station_dims[2],
                                color=(.1,.1,.1))
        date = datetime.datetime.now().strftime("%H-%M-%S-%m-%d-%Y")
        self.filename = Network.LOGS + "network-data-" + str(date) + ".txt"

        self.mimsy = Mimsy(system)

    ''' Class Methods '''

    @classmethod
    def initialize(cls, spherical=True, logging=True):
        if spherical:
            return cls(logging=logging)
        return cls(system=Mimsy.cartesian, logging=logging)

    ''' Instance Methods '''

    def write(self, data=''):
        if self.logging:
            with open(self.filename, 'a+') as file:
                file.write(str(data) + '\n')

    def update(self, data=[], raw_pulses=[]):
        if raw_pulses:
            valid, data = self.parsePulseData(raw_pulses)

        if valid:
            self.mimsy.update(data)
            self.vector = arrow(axis=(self.mimsy.x() - self.reference[0],
                                    self.mimsy.y() - self.reference[1],
                                    self.mimsy.z() - self.reference[2]),
                                color=(0,0,1), shaftwidth=1)
        else:
            print('Invalid pulse timing data, aborted mimsy position update.')

    def parsePulseData(self, raw_pulses):
        pulses = []
        tracking = None
        for elem in raw_pulses:
            tracking = Pulse(elem[0], elem[1], parent=tracking)
            pulses.append(tracking)
            self.write(str(tracking))
        self.write()

        if not Network.validSequence([p.type for p in pulses]):
            return False, None

        pulses = [pulse for pulse in pulses if pulse.type != Pulse.Nil]

        r_horiz, r_vert = 0, 0
        phi, theta = 0, 0
        for i, pulse in enumerate(pulses):
            if pulse.isHoriz():
                r_horiz = pulse.getEffectiveSensorWidth(Network.TEST_DIST_CM)
                phi = pulse.getAngle()
                self.write(str(pulse.start) + "-Phi: " + str(degrees(phi))) # degrees
                self.write(str(pulse.end) + "-H_SW: " + str(r_horiz)) # cm
            elif pulse.isVert():
                r_vert = pulse.getEffectiveSensorWidth(Network.TEST_DIST_CM)
                theta = pulse.getAngle()
                self.write(str(pulse.start) + "-Theta: " + str(degrees(theta))) # degrees
                self.write(str(pulse.end) + "-V_SW: " + str(r_vert)) # cm

        radial = Network.TEST_DIST_CM # (r_horiz + r_vert) / 2.0
        return True, (radial, phi, theta)

    ''' Static Methods '''

    @staticmethod
    def validSequence(received_types):
        return Network.contains(Network.VALID_PULSES[0], received_types) or \
                Network.contains(Network.VALID_PULSES[1], received_types)

    @staticmethod
    def contains(sub, array):
        sub_size = len(sub)
        for i in range(0, len(array)):
            if array[i:i+sub_size] == sub:
                return True
        return False
