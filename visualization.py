from visual import *
from math import sin, cos, tan, asin, acos, atan, sqrt, pi, radians, degrees
import datetime

class Pulse:
    Unclassified, Nil, Sync, Horiz, Vert, Sweep = -69, -1, 0, 1, 2, 3

    LOCALIZATION_PERIOD_MS = 300
    PULSE_TRACK_COUNT = 5

    MIN_SYNC_PERIOD_US = 2501 / 48.0
    MAX_SYNC_PERIOD_US = 138

    SWEEP_PERIOD_US = 8333.3333
    SWEEP_VELOCITY = pi / SWEEP_PERIOD_US

    SENSOR_WIDTH_MM = 1.0 # FIXME: Look at data-sheet or measure
    SENSOR_WIDTH_CM = SENSOR_WIDTH_MM / 10.0

    CLOCK_SPEED_MHZ = 32.0

    TIMER_32, TIMER_24, TIMER_16 = 0xFFFFFFFF, 0xFFFFFF, 0xFFFF

    def __init__(self, start, end, parent=None, overflow=TIMER_24):
        self.start = start
        self.end = end
        self.parent = parent
        self.overflow = overflow

        self.type = Pulse.Unclassified
        self.type = self.classify()

    def naiveClassify(self):
        '''
        Naively classifies the pulse as Sync or Sweep.
        '''
        if (self.period() < Pulse.MIN_SYNC_PERIOD_US):
            if self.parent:
                return Pulse.Sweep
            else:
                return Pulse.Nil
        else:
            return Pulse.Sync

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
            end = ", hit=" + str(self.sweepHit_ms()) + ", parent=" + self.parent.typeString() + ">"
        else:
            end = ">"
        return "Pulse Object:<period=" + str(self.period()) + ", type=" + self.typeString() + end

    ''' Universal Attributes '''

    def period(self):
        '''
        Returns the period of the pulse in microseconds.
        '''
        return Pulse.get_period_us(self.start, self.end, self.overflow)

    def getStart(self):
        '''
        Returns the start time of the pulse in microseconds.
        '''
        return self.start / Pulse.CLOCK_SPEED_MHZ

    def getEnd(self):
        '''
        Returns the end time of the pulse in microseconds.
        '''
        return self.end / Pulse.CLOCK_SPEED_MHZ

    ''' Sweep Pulse Methods '''

    def sweepHit(self):
        '''
        Returns the time in microseconds that the sweep beam
        took to hit the photodiode after the sync.
        '''
        if self.isClassified() and not self.isSweep():
            raise TypeError("Pulse must be a Sweep Pulse.")
        return Pulse.get_period_us(self.parent.end, self.start, self.overflow)

    def sweepHit_ms(self):
        '''
        Returns the time in milliseconds that the sweep beam
        took to hit the photodiode after the sync.
        '''
        return self.sweepHit() / 1000.0

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
            elif self.isSweep():
                return "Sweep"
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
        '''
        Returns whether the pulse is a sweep pulse.
        '''
        return self.type == Pulse.Sweep or self.isHoriz() or self.isVert()

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
    def get_period_us(start, end, overflow=TIMER_24):
        if end > start:
            return (end - start) / Pulse.CLOCK_SPEED_MHZ
        else:
            return ((overflow - start) + end) / Pulse.CLOCK_SPEED_MHZ

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

    VALID_PULSES = [ [Pulse.Sync, Pulse.Sweep, Pulse.Sync, Pulse.Sweep],
                    [Pulse.Sync, Pulse.Horiz, Pulse.Sync, Pulse.Vert],
                    [Pulse.Sync, Pulse.Vert, Pulse.Sync, Pulse.Horiz]]

    MAX_RANGE_CM = 1.5 * 100
    TEST_DIST_CM = 100 # 1 meter default

    LOGS = './logs/'

    def __init__(self, system=Mimsy.spherical, reference=(0, 0, 0), station_dims=(10, 6.35, 10), logging=True):
        self.logging = logging

        # Initialize base station
        self.reference = reference

        self.scene = display(title='Network Visualization', x=0, y=0)
        self.scene.background = (0.5,0.5,0.5)
        self.center = self.scene.center
        self.scene.forward = (0.414414, -0.367281, -0.832686)

        self.scene.lights = [vector(1,0,0), vector(0, 1, 0), vector(0, 0, 1), \
            vector(-1,0,0), vector(0, -1, 0), vector(0, 0, -1)]
        self.scene.ambient = 0

        # self.scene.forward = (-1,-1,-1)
        self.scene.range = Network.MAX_RANGE_CM
        self.base_station = box(pos=self.reference, length=station_dims[0],
                                width=station_dims[1], height=station_dims[2],
                                color=(.1,.1,.1))
        date = datetime.datetime.now().strftime("%H-%M-%S-%m-%d-%Y")
        self.filename = Network.LOGS + "network-data-" + str(date) + ".txt"
        self.vector = arrow(axis=(30,0,0), color=(0,0,1), shaftwidth=1)

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
        else:
            print(data)

    def read(self):
        if self.logging:
            with open(self.filename, 'r') as file:
                print(file.read())

    def update(self, data=[], raw_pulses=[], modular_ptr=0):
        raw_pulses = [raw_pulses[(modular_ptr+i) % Pulse.PULSE_TRACK_COUNT] for i in range(0, Pulse.PULSE_TRACK_COUNT)]

        if raw_pulses:
            valid, data = self.parsePulseData(raw_pulses)

        if not raw_pulses or valid:
            self.mimsy.update(data)
            print((self.mimsy.x() - self.reference[0],
                                    self.mimsy.y() - self.reference[1],
                                    self.mimsy.z() - self.reference[2]))
            print(degrees(self.mimsy.theta()), degrees(self.mimsy.phi()), self.mimsy.radial())
            self.vector.axis = (self.mimsy.z() - self.reference[2],
                                    self.mimsy.x() - self.reference[0],
                                    self.mimsy.y() - self.reference[1])
            # Turns off the default user spin and zoom and handles these functions itself.
            # This gives more control to the program and addresses the problem that at the time of writing,
            # Visual has a hidden user scaling variable that makes it impossible to force the camera position
            # by setting range, if the user has already zoomed using the mouse.

            self.scene.userzoom = False
            self.scene.userspin = False
            rangemin = 1
            rangemax = 100

            i_x = 0
            i_y = 0

            updating = False

            brk = False
            _rate = 100

            while not brk:
                rate(_rate)
                if self.scene.kb.keys:
                    k = self.scene.kb.getkey()
                    _rate = 100

                    change = 15

                    if k == 'i':
                        cx, cy, cz = self.center
                        self.scene.center = (cx,cy,cz)
                        self.scene.forward = (0,0,-1)
                    elif k == '1':
                        self.scene.forward = (1,0,-.25)
                    elif k == '2':
                        self.scene.forward = (0,1,-1)
                    elif k == '3':
                        self.scene.forward = (1,0,0)
                    elif k == '4':
                        self.scene.forward = (0,-1,0)
                    elif k == 'shift+down' and self.scene.range.x < rangemax:
                        self.scene.range = self.scene.range.x + .5
                    elif k == 'shift+up' and self.scene.range.x > rangemin:
                        self.scene.range = self.scene.range.x - .5
                    elif k == 'up':
                        self.scene.center = (self.scene.center.x, \
                            self.scene.center.y + .1, self.scene.center.z)
                    elif k == 'down':
                        self.scene.center = (self.scene.center.x, \
                            self.scene.center.y - .1, self.scene.center.z)
                    elif k == 'right':
                        self.scene.center = (self.scene.center.x + .1, \
                             self.scene.center.y, self.scene.center.z)
                    elif k == 'left':
                        self.scene.center = (self.scene.center.x - .1, \
                            self.scene.center.y, self.scene.center.z)
                    elif k == 'shift+left':
                        self.scene.center = (self.scene.center.x, \
                            self.scene.center.y, self.scene.center.z + .1)
                    elif k == 'shift+right':
                        self.scene.center = (self.scene.center.x, \
                            self.scene.center.y, self.scene.center.z - .1)
                    elif k == 'w':
                        self.scene.forward = (self.scene.forward.x, \
                            self.scene.forward.y - .1, self.scene.forward.z)
                    elif k == 's':
                        self.scene.forward = (self.scene.forward.x, \
                            self.scene.forward.y + .1, self.scene.forward.z)
                    elif k == 'a':
                        self.scene.forward = (self.scene.forward.x - .1, \
                            self.scene.forward.y, self.scene.forward.z)
                    elif k == 'd':
                        self.scene.forward = (self.scene.forward.x + .1, \
                            self.scene.forward.y, self.scene.forward.z)
                    elif k == 'A':
                        self.scene.forward = (self.scene.forward.x, \
                            self.scene.forward.y, self.scene.forward.z - .1)
                    elif k == 'D':
                        self.scene.forward = (self.scene.forward.x, \
                            self.scene.forward.y, self.scene.forward.z + .1)
                    elif k == '.' or k == 'q':
                        brk = True
                    print(self.scene.forward)
        else:
            print('Invalid pulse timing data, aborted mimsy position update.')

        return valid

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
                self.write(str(pulse.end) + "-H_ESW: " + str(r_horiz)) # cm
            elif pulse.isVert():
                r_vert = pulse.getEffectiveSensorWidth(Network.TEST_DIST_CM)
                theta = pulse.getAngle()
                self.write(str(pulse.start) + "-Theta: " + str(degrees(theta))) # degrees
                self.write(str(pulse.end) + "-V_ESW: " + str(r_vert)) # cm
        self.write()

        radial = Network.TEST_DIST_CM # (r_horiz + r_vert) / 2.0
        return True, (radial, pi-phi, pi-theta) # pi-theta --> correction

    ''' Static Methods '''

    @staticmethod
    def validSequence(received_types):
        return Network.contains(Network.VALID_PULSES[0], received_types) or \
                Network.contains(Network.VALID_PULSES[1], received_types) or \
                Network.contains(Network.VALID_PULSES[2], received_types)

    @staticmethod
    def contains(sub, array):
        sub_size = len(sub)
        for i in range(0, len(array)):
            if array[i:i+sub_size] == sub:
                return True
        return False
