import os
from copy import deepcopy
from multiprocessing import Pool
from functools import partial
import tempfile
from pybullet_utils import bullet_client
import pybullet_data
import pybullet
import math
import numpy as np
import datetime
from sfers.robot_simulation.create_actuator import RobotBase


class RobotSimulator(RobotBase):
    def __init__(self, parameter='parameter 2'):
        RobotBase.__init__(self, parameter=parameter)
        self.step = 0
        self.simTime = 0
        self.thickness = 0.1
        self.isGravity = True
        self.dateTime = datetime.datetime.today().strftime('%m_%d_%Y_%H_%M')
        self.N = None
        self.m = None
        self.drivenFrequency = None
        self.dampingEta = None
        self.recordStepInterval = None
        self.simCycles = None
        self.timeStep = None
        self.dataTime = None
        self.dataTheta = None
        self.dataAngularVelocities = None
        self.dataTor = None
        self.dataPositions = None
        self.dataMotorVoltages = None
        self.dataPositionVelocities = None
        self.linkLength = None
        self.jointLength = None
        self.jointIndex = None
        self.linkIndex = None
        self.boxId = None
        self.jointNumber = None
        self.theta = None
        self.angularVelocities = None
        self.positions = None
        self.motorVoltages = None
        self.Tor = None
        self.positionVelocities = None
        self.voltage = None
        self.filename = None

    def reset(self):
        RobotBase.reset(self)
        self.step = 0
        self.dataTime = []
        self.dataTheta = []
        self.dataAngularVelocities = []
        self.dataTor = []
        self.dataPositions = []
        self.dataMotorVoltages = []
        self.dataPositionVelocities = []
        self.linkLength = self.actuatorLength / self.m
        self.jointLength = 0.5 * self.linkLength
        self._p = None
        self._p = bullet_client.BulletClient(connection_mode=pybullet.DIRECT)
        self._p.resetSimulation()
        self._p.resetDebugVisualizerCamera(self.N * 15, -360, -16, [0, 0, 1])
        (self.boxId, self.jointNumber) = self.create1DMultiActuators(self.N, self.m, self.actuatorMass,
                                                                     self.actuatorLength,
                                                                     self.width, self.thickness)
        self.jointIndex = range(self.jointNumber)
        self.linkIndex = range(self.jointNumber + 1)

        self._p.setTimeStep(self.timeStep)
        if self.isGravity:
            self._p.setGravity(0, 0, -self.gravity)
        else:
            self._p.setGravity(0, 0, -1)

    def close(self):
        self._p.disconnect()

    def TorVolThe(self, Theta, angularVelocity, Voltage):
        thetaTarget = -self.beta * Voltage
        K = self.halfEI / self.jointLength
        omega = 2 * math.pi * 1.0
        Tor = -self.width * K * ((Theta - thetaTarget) + (self.dampingEta / omega) * angularVelocity)
        return Tor

    def printProgress(self):
        print('N=', self.N, ', m=', self.m, "Frequency=", round(self.drivenFrequency, 2), "Hz, Actuator length",
              self.actuatorLength,
              "cm, step=", self.step + 1)

    def simStep(self, actuatorVoltages):
        if (self.step + 1) % 100000 == 0:
            self.printProgress()

        self.simTime = self.step * self.timeStep
        [self.theta, self.angularVelocities, self.positions, self.motorVoltages, self.Tor,
         self.positionVelocities] = self.voltageTorqueControlStep(
            self.boxId,
            actuatorVoltages,
            self.TorVolThe,
            self.N,
            self.m,
            self.jointNumber,
            self.jointIndex,
            self.linkIndex,
            self.jointLength)
        if (self.step + 1) % self.recordStepInterval == 0 or self.step == 0 or self.step == self.simCycles - 1:
            self.dataTime.append(self.simTime)
            self.dataTheta.append(self.theta)
            self.dataAngularVelocities.append(self.angularVelocities)
            self.dataPositions.append(self.positions)
            self.dataMotorVoltages.append(self.motorVoltages)
            self.dataTor.append(self.Tor)
            self.dataPositionVelocities.append(self.positionVelocities)
        self._p.stepSimulation()
        self.step = self.step + 1

    def get_positions(self, positions):
        positions = self.position_offset(positions)
        return positions

    def get_shape(self, positions):
        positions = self.get_positions(positions)
        xAxis = np.array([positions[i][0] for i in range(len(positions))])
        zAxis = np.array([positions[i][2] for i in range(len(positions))])
        return xAxis, zAxis

    @staticmethod
    def get_shape_velocity(positionVelocities):
        xAxisVelocity = np.array([positionVelocities[i][0] for i in range(len(positionVelocities))])
        zAxisVelocity = np.array([positionVelocities[i][2] for i in range(len(positionVelocities))])
        return xAxisVelocity, zAxisVelocity

    def shape(self):
        positions = self.positions
        xAxis, zAxis = self.get_shape(positions)
        return xAxis, zAxis

    def shape_velocity(self):
        positionVelocities = self.positionVelocities
        xAxisVelocity, zAxisVelocity = self.get_shape_velocity(positionVelocities)
        return xAxisVelocity, zAxisVelocity

    def position_offset(self, positions):
        jointLength = self.jointLength
        offset = jointLength / 2.0
        for i in range(len(positions)):
            positions[i] = self.position_transform(positions[i], offset, angle=0.0)
        return positions

    def save_simulator_state(self):
        state_file = tempfile.NamedTemporaryFile(delete=False)
        self._p.saveBullet(bulletFileName=state_file.name)
        state_file.close()
        return state_file

    def load_simulator_state(self, state_file_name):
        self._p.restoreState(fileName=state_file_name)

    def copy(self, state_file_name):
        simulator = deepcopy(self)
        simulator.reset()
        simulator.load_simulator_state(state_file_name)
        return simulator

    def save(self):
        raise NotImplementedError

    def drive_cooldown(self, saving=True, closing=True):
        if closing:
            self.close()
        if not saving:
            return None
        self.save()
        return self.filename

    def get_phase_velocity(self, frequency):
        angularFrequency = 2 * np.pi * frequency
        massLoad = self.load / self.gravity
        return np.power(self.EI * (angularFrequency ** 2) / massLoad, 0.25)


class SingleActuatorCantilever(RobotSimulator):
    def __init__(self, parameter='parameter 2', outfilename=None):
        RobotSimulator.__init__(self, parameter=parameter)
        self.isGravity = True
        self.N = 1
        self.m = 5
        self.drivenFrequency = 1.0
        self.dampingEta = 0.008
        self.timeStep = 2e-5 / 240
        self.simCycles = int(2.4e7)
        self.recordStepInterval = 50
        self.outfilename = outfilename

    def reset(self):
        RobotSimulator.reset(self)
        self._p.changeDynamics(self.boxId, -1, spinningFriction=0.001, rollingFriction=0.001,
                               linearDamping=0.0, angularDamping=0.0, jointDamping=0.0)
        cid = self._p.createConstraint(self.boxId, -1, -1, -1, self._p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])

    def save(self):
        self.dataTime = np.array(self.dataTime)
        self.dataTheta = np.array(self.dataTheta)
        self.dataPositions = np.array(self.dataPositions)
        self.dataTor = np.array(self.dataTor)
        dateTime = datetime.datetime.today().strftime('%m_%d_%Y_%H_%M')
        self.filename = 'AC_Transient_ZZ9_ActuatorLength' + str(
            self.actuatorLength) + 'cm_m_' + str(self.m) + '_Frequency_' + str(
            round(self.drivenFrequency, 2)) + '_Hz_' + dateTime + '.npz'
        if self.outfilename is None:
            outfile = 'data/' + self.filename
        else:
            outfile = self.outfilename
        np.savez_compressed(outfile, dataTime=self.dataTime, dataTheta=self.dataTheta,
                            dataAngularVelocities=self.dataAngularVelocities,
                            dataPositions=self.dataPositions, dataMotorVoltages=self.dataMotorVoltages,
                            dataTor=self.dataTor,
                            dataPositionVelocities=self.dataPositionVelocities,
                            timeStep=self.timeStep,
                            simCycles=self.simCycles, recordStepInterval=self.recordStepInterval, N=self.N, m=self.m,
                            actuatorLength=self.actuatorLength,
                            drivenFrequency=self.drivenFrequency)

    def actuatorDCDrive(self, arg, saving=True, closing=True):
        actuatorVoltages = arg[0]
        self.voltage = actuatorVoltages[0]
        self.reset()
        for step in range(self.simCycles):
            self.simStep(actuatorVoltages)
        filename = self.drive_cooldown(saving=saving, closing=closing)
        return filename

    def actuatorACDrive(self, arg, saving=True, closing=True):
        m = arg[0]
        drivenFrequency = arg[1]
        actuatorLength = arg[2]
        self.m = m
        self.drivenFrequency = drivenFrequency
        self.actuatorLength = actuatorLength
        self.reset()
        for step in range(self.simCycles):
            actuatorVoltages = [-750 * (1 - math.cos(2 * math.pi * drivenFrequency * self.simTime))]
            self.simStep(actuatorVoltages)
        filename = self.drive_cooldown(saving=saving, closing=closing)
        return filename

    def drive(self, arg, saving=True, closing=True):
        return self.actuatorACDrive(arg, saving=saving, closing=closing)

    def stepDrive(self, saving=True, closing=True):
        self.reset()
        for step in range(self.simCycles):
            if self.simTime < 2 * self.timeStep:
                actuatorVoltages = [0.0]
            else:
                actuatorVoltages = [-1500.0]
            self.simStep(actuatorVoltages)
        filename = self.drive_cooldown(saving=saving, closing=closing)
        return filename

    def modelResonantFrequencies(self):
        """
        Resonant frequency of a cantilever from a simple Euler-Bernoulli model
        """
        basicFrequency = 3.5160 * np.sqrt(self.EI / (self.load / self.gravity)) / (self.actuatorLength ** 2) / (
                2 * np.pi)
        frequencies = [basicFrequency]
        return frequencies


class SingleActuatorCantileverDCResponseVoltageScan(SingleActuatorCantilever):
    def __init__(self, parameter='parameter 2'):
        SingleActuatorCantilever.__init__(self, parameter=parameter, outfilename=None)
        self.m = 3
        self.isGravity = True
        self.dampingEta = 0.1
        self.voltages = [0.0, -600.0, -800.0, -1000.0]
        self.poolNumber = 40
        self.foldername = None

    def reset(self):
        SingleActuatorCantilever.reset(self)
        self.foldername = 'Actuator_DC_Voltage_Scan_ZZ9_DampingEta_' + str(
            round(self.dampingEta, 4)) + '_' + self.dateTime

    def drive(self, arg, saving=True, closing=True):
        actuatorVoltages, = arg
        simulator = deepcopy(self)
        return simulator.actuatorDCDrive([actuatorVoltages, ], saving=saving, closing=closing)

    def scan(self, saving=True, closing=True):
        voltages = self.voltages
        args = [([voltage],) for voltage in voltages]
        poolNumber = np.minimum(len(args), self.poolNumber)
        pooler = Pool(poolNumber)
        filenames = pooler.map(partial(self.drive, saving=saving, closing=closing), args)

        dateTime = datetime.datetime.today().strftime('%m_%d_%Y_%H_%M')
        filename = self.foldername + '/' + 'Actuator_DC_Voltage_Scan_ZZ9_' + str(
            round(self.dampingEta, 2)) + '_' + dateTime + '.npz'
        outfile = 'data/' + filename
        if saving:
            np.savez(outfile, filenames=filenames, foldername=self.foldername, N=self.N, voltages=voltages)
        return filenames

    def save(self):
        if not os.path.exists('data/' + self.foldername):
            os.makedirs('data/' + self.foldername)
        self.dataTime = np.array(self.dataTime)
        self.dataTheta = np.array(self.dataTheta)
        self.dataPositions = np.array(self.dataPositions)
        self.dataTor = np.array(self.dataTor)
        dateTime = datetime.datetime.today().strftime('%m_%d_%Y_%H_%M')
        self.filename = self.foldername + '/' + 'DC_ZZ9_ActuatorLength' + str(
            self.actuatorLength) + 'cm_m_' + str(self.m) + '_Voltage_' + str(
            round(self.voltage, 2)) + '_V_' + '.npz'
        outfile = 'data/' + self.filename
        np.savez_compressed(outfile, dataTime=self.dataTime, dataTheta=self.dataTheta,
                            dataAngularVelocities=self.dataAngularVelocities,
                            dataPositions=self.dataPositions, dataMotorVoltages=self.dataMotorVoltages,
                            dataTor=self.dataTor,
                            dataPositionVelocities=self.dataPositionVelocities,
                            timeStep=self.timeStep,
                            simCycles=self.simCycles, recordStepInterval=self.recordStepInterval, N=self.N, m=self.m,
                            actuatorLength=self.actuatorLength,
                            drivenFrequency=self.drivenFrequency,
                            voltage=self.voltage)


class SingleActuatorCantileverACResponse(SingleActuatorCantilever):
    def __init__(self, parameter='parameter 2'):
        SingleActuatorCantilever.__init__(self, parameter=parameter, outfilename=None)
        self.ms = [5]
        self.drivenFrequencies = np.linspace(1, 40, num=40)
        self.actuatorLengthes = [10]
        self.poolNumber = 40

    def reset(self):
        SingleActuatorCantilever.reset(self)
        self.foldername = 'Actuator_AC_damp_Scan_ZZ9_DampingEta_' + str(round(self.dampingEta, 4)) + '_' + self.dateTime

    def drive(self, arg, saving=True, closing=True):
        m, drivenFrequency, actuatorLength = arg
        simulator = deepcopy(self)
        return simulator.actuatorACDrive([m, drivenFrequency, actuatorLength], saving=saving, closing=closing)

    def actuatorACScan(self, saving=True, closing=True):
        ms = self.ms
        drivenFrequencies = self.drivenFrequencies
        actuatorLengthes = self.actuatorLengthes
        poolNumber = self.poolNumber
        args = [(ms[i], drivenFrequencies[j], actuatorLengthes[k]) for i in range(len(ms)) for j in
                range(len(drivenFrequencies)) for k in range(len(actuatorLengthes))]

        pooler = Pool(poolNumber)
        filenames = pooler.map(partial(self.drive, saving=saving, closing=closing), args)

        dateTime = datetime.datetime.today().strftime('%m_%d_%Y_%H_%M')
        filename = self.foldername + '/' + 'Actuator_AC_damp_Scan_ZZ9_' + str(
            round(self.dampingEta, 2)) + '_' + dateTime + '.npz'
        outfile = 'data/' + filename
        if saving:
            np.savez(outfile, filenames=filenames, foldername=self.foldername, N=self.N, ms=ms,
                     drivenFrequencies=drivenFrequencies)
        return filenames

    def save(self):
        if not os.path.exists('data/' + self.foldername):
            os.makedirs('data/' + self.foldername)
        self.dataTime = np.array(self.dataTime)
        self.dataTheta = np.array(self.dataTheta)
        self.dataPositions = np.array(self.dataPositions)
        self.dataTor = np.array(self.dataTor)
        dateTime = datetime.datetime.today().strftime('%m_%d_%Y_%H_%M')
        self.filename = self.foldername + '/' + 'AC_Transient_ZZ9_ActuatorLength' + str(
            self.actuatorLength) + 'cm_m_' + str(self.m) + '_Frequency_' + str(
            round(self.drivenFrequency, 2)) + '_Hz_' + '.npz'
        outfile = 'data/' + self.filename
        np.savez_compressed(outfile, dataTime=self.dataTime, dataTheta=self.dataTheta,
                            dataAngularVelocities=self.dataAngularVelocities,
                            dataPositions=self.dataPositions, dataMotorVoltages=self.dataMotorVoltages,
                            dataTor=self.dataTor,
                            dataPositionVelocities=self.dataPositionVelocities,
                            timeStep=self.timeStep,
                            simCycles=self.simCycles, recordStepInterval=self.recordStepInterval, N=self.N, m=self.m,
                            actuatorLength=self.actuatorLength,
                            drivenFrequency=self.drivenFrequency)

    def scan(self, saving=True, closing=True):
        return self.actuatorACScan(saving=saving, closing=closing)


class inchworm(RobotSimulator):
    def __init__(self, parameter='parameter 2', outfilename=None):
        RobotSimulator.__init__(self, parameter=parameter)
        self.frictionLow = 2.5
        self.frictionHigh = 2.5
        self.groundFriction = 1.0
        self.highFrictionalActuatorFraction = 0.5
        self.isGravity = True
        self.drivenFrequency = 1  # in Hz
        self.N = 5
        self.m = 1
        self.thickness = 0.1
        self.period = 0.0
        self.timeStep = 2e-3 / 240
        self.dampingEta = 0.008
        self.recordStepInterval = 125
        self.simCycles = int(1.5e7)
        self.onVoltages = (520.0, 520.0, -1500.0, 520.0, 520.0)
        self.folderLabel = 'SimpleWalker_ZZ7_'
        self.filename = None
        self.planeId = None
        self.outfilename = outfilename

    @staticmethod
    def setFrictionalCoefficients(frictionLow, frictionHigh, N, m, fraction=0.5):
        high_friction_segments = int(math.ceil(m * fraction + 0.5))
        low_friction_segments = int(N * m + 1 - high_friction_segments)
        lateralFrictionCoefficients = [frictionLow for _ in
                                       range(low_friction_segments)]

        for _ in range(high_friction_segments):
            lateralFrictionCoefficients.append(frictionHigh)

        return lateralFrictionCoefficients

    def reset(self):
        RobotSimulator.reset(self)
        self.lateralFrictionCoefficients = self.setFrictionalCoefficients(self.frictionLow,
                                                                          self.frictionHigh,
                                                                          self.N,
                                                                          self.m,
                                                                          fraction=self.highFrictionalActuatorFraction)
        self._p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.planeId = self._p.loadURDF('plane.urdf', basePosition=[0, 0, -0.5 * self.thickness], globalScaling=10.0)

        self._p.changeDynamics(self.planeId, -1, spinningFriction=0.0, rollingFriction=0.0,
                               lateralFriction=self.groundFriction,
                               linearDamping=0.0, angularDamping=0.0, jointDamping=0.0)
        self._p.changeDynamics(self.planeId, 0, spinningFriction=0.0, rollingFriction=0.0,
                               lateralFriction=self.groundFriction,
                               linearDamping=0.0, angularDamping=0.0, jointDamping=0.0)
        self._p.changeDynamics(self.boxId, -1, spinningFriction=0.0, rollingFriction=0.0,
                               lateralFriction=self.lateralFrictionCoefficients[0],
                               linearDamping=0.0, angularDamping=0.0, jointDamping=0.0)
        for joint in self.jointIndex:
            self._p.changeDynamics(self.boxId, joint, spinningFriction=0.0, rollingFriction=0.0,
                                   lateralFriction=self.lateralFrictionCoefficients[joint + 1],
                                   linearDamping=0.0, angularDamping=0.0, jointDamping=0.0)

    def controlStep(self, actuatorVoltages):
        for simStep in range(self.simCycles):
            self.simStep(actuatorVoltages)

        return [list(-1 * np.array(self.theta)), list(-1 * np.array(self.angularVelocities)), self.positions,
                self.motorVoltages, self.Tor]

    def applyVoltages(self, time, period):
        """
        Default voltages for frictionless inchworm only driving act #5 forward moving
        """
        if time % period < (1.0 / 4.0) * period:
            voltages = (0.0, 0.0, 0.0, 0.0, 0.0)
        elif time % period < (1.0 / 2.0) * period:
            voltages = (0.0, self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
        elif time % period < (3.0 / 4.0) * period:
            voltages = (self.onVoltages[0], self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
        else:
            voltages = (self.onVoltages[0], 0.0, 0.0, 0.0, 0.0)
        return voltages

    def applyInchwormVoltages(self, time, period):
        return self.applyVoltages(time, period)

    def robotDrive(self, arg, saving=True, closing=True):
        period = arg[0]
        self.period = period
        self.reset()
        for step in range(self.simCycles):
            actuatorVoltages = self.applyVoltages(self.simTime, period)
            self.simStep(actuatorVoltages)
        filename = self.drive_cooldown(saving=saving, closing=closing)
        return filename

    def drive(self, arg, saving=True, closing=True):
        return self.robotDrive(arg, saving=saving, closing=closing)

    def dataToArray(self):
        self.dataTime = np.array(self.dataTime)
        self.dataTheta = np.array(self.dataTheta)
        self.dataAngularVelocities = np.array(self.dataAngularVelocities)
        self.dataPositions = np.array(self.dataPositions)
        self.dataTor = np.array(self.dataTor)
        self.dataMotorVoltages = np.array(self.dataMotorVoltages)
        self.dataPositionVelocities = np.array(self.dataPositionVelocities)

    def saveData(self, outfile, dateTime):
        np.savez_compressed(outfile, dataTime=self.dataTime, dataTheta=self.dataTheta,
                            dataAngularVelocities=self.dataAngularVelocities,
                            dataPositions=self.dataPositions, dataMotorVoltages=self.dataMotorVoltages,
                            dataTor=self.dataTor,
                            dataPositionVelocities=self.dataPositionVelocities,
                            timeStep=self.timeStep,
                            simCycles=self.simCycles, recordStepInterval=self.recordStepInterval, N=self.N,
                            m=self.m, actuatorLength=self.actuatorLength,
                            drivenFrequency=self.drivenFrequency, dampingEta=self.dampingEta,
                            dateTime=dateTime, frictionLow=self.frictionLow,
                            frictionHigh=self.frictionHigh, period=self.period)

    def save(self):
        self.dataToArray()
        dateTime, self.filename = self.filenameGenerate()
        if self.outfilename is None:
            outfile = 'data/' + self.filename
        else:
            outfile = self.outfilename
        self.saveData(outfile, dateTime)

    def filenameGenerate(self):
        dateTime = datetime.datetime.today().strftime('%m_%d_%Y_%H_%M')
        filename = self.folderLabel + 'ActuatorLength' + str(self.actuatorLength) + 'cm_N_' + str(
            self.N) + '_m_' + str(self.m) + '_Period_' + str(self.period) + 's_' + dateTime + '.npz'
        return dateTime, filename


class InchwormBackward(inchworm):
    def __init__(self, parameter='parameter 2'):
        inchworm.__init__(self, parameter=parameter)
        self.folderLabel = 'SimpleWalker_ZZ7_Backward_'

    def applyVoltages(self, time, period):
        """
        Default voltages for frictionless inchworm only driving act #5 backward moving
        """
        if time % period < (1.0 / 4.0) * period:
            voltages = (0.0, 0.0, 0.0, 0.0, 0.0)
        elif time % period < (1.0 / 2.0) * period:
            voltages = (0.0, self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
        elif time % period < (3.0 / 4.0) * period:
            voltages = (0.0, self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], self.onVoltages[4])
        else:
            voltages = (0.0, 0.0, 0.0, 0.0, self.onVoltages[4])
        return voltages


class FullFrictionalInchworm(inchworm):
    def __init__(self, parameter='parameter 2'):
        inchworm.__init__(self, parameter=parameter)
        self.frictionHigh = 4.0
        self.folderLabel = 'SimpleWalker_ZZ7_FullFrictionalInchworm_'

    @staticmethod
    def setFrictionalCoefficients(frictionLow, frictionHigh, N, m, fraction=0.5):
        high_friction_segments = int(math.ceil(m * fraction + 0.5))
        low_friction_segments = int(N * m + 1 - 2 * high_friction_segments)
        lateralFrictionCoefficients = high_friction_segments * [frictionHigh] \
                                      + low_friction_segments * [frictionLow] \
                                      + high_friction_segments * [frictionHigh]

        return lateralFrictionCoefficients

    def applyVoltages(self, time, period):
        """
        Default voltages for frictional inchworm and frog motion v1
        """
        if time % period < (1.0 / 5.0) * period:
            voltages = (0.0, 0.0, 0.0, 0.0, 0.0)
        elif time % period < (2.0 / 5.0) * period:
            voltages = (self.onVoltages[0], 0.0, 0.0, 0.0, 0.0)
        elif time % period < (3.0 / 5.0) * period:
            voltages = (self.onVoltages[0], self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
        elif time % period < (4.0 / 5.0) * period:
            voltages = (0.0, self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], self.onVoltages[4])
        else:
            voltages = (0.0, 0.0, 0.0, 0.0, self.onVoltages[4])
        return voltages


class FrictionalFrogMotionV2(FullFrictionalInchworm):

    def applyVoltages(self, time, period):
        """
        Default voltages for frog motion v2
        """
        if time < (1.0 / 5.0) * period:
            voltages = (0.0, 0.0, 0.0, 0.0, 0.0)
            return voltages
        elif time % period < (1.0 / 5.0) * period:
            voltages = (0.0, self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
        elif time % period < (2.0 / 5.0) * period:
            voltages = (self.onVoltages[0], 0.0, 0.0, 0.0, 0.0)
        elif time % period < (3.0 / 5.0) * period:
            voltages = (self.onVoltages[0], self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
        elif time % period < (4.0 / 5.0) * period:
            voltages = (0.0, self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], self.onVoltages[4])
        else:
            voltages = (0.0, 0.0, 0.0, 0.0, self.onVoltages[4])
        return voltages


class InchwormCrawl(inchworm):
    def __init__(self, parameter='parameter 2'):
        inchworm.__init__(self, parameter=parameter)
        self.folderLabel = 'SimpleWalker_ZZ7_InchwormCrawl_'

    def applyVoltages(self, time, period):
        if time % period < (1.0 / 2.0) * period:
            voltages = (0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            voltages = (0.0, self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
        return voltages


class InchwormPeriodScan(inchworm):
    def __init__(self, parameter='parameter 2'):
        inchworm.__init__(self, parameter=parameter)
        self.frequencies = np.concatenate((np.linspace(1.0, 18.0, num=18),
                                           np.linspace(20.0, 30.0, num=6),
                                           np.linspace(35.0, 40.0, num=2)))
        self.scanLabel = '-PeriodScan_'
        self.periods = None

    def reset(self):
        inchworm.reset(self)
        self.periods = 1.0 / self.frequencies
        self.foldername = self.folderLabel + 'N_5_m_' + str(self.m) + self.scanLabel + self.dateTime

    def save(self):
        if not os.path.exists('data/' + self.foldername):
            os.makedirs('data/' + self.foldername)
        self.dataToArray()
        dateTime, self.filename = self.filenameGenerate()
        outfile = 'data/' + self.foldername + '/' + self.filename
        self.saveData(outfile, dateTime)

    def drive(self, arg, saving=True, closing=True):
        period = arg[0]
        simulator = deepcopy(self)
        return simulator.robotDrive([period, ], saving=saving, closing=closing)

    def multiThreadSetup(self, args, saving=True, closing=True):
        poolNumber = os.cpu_count()
        poolNumber = np.minimum(len(args), poolNumber)
        pooler = Pool(poolNumber)
        filenames = pooler.map(partial(self.drive, saving=saving, closing=closing), args)
        pooler.close()
        pooler.join()
        dateTime = datetime.datetime.today().strftime('%m_%d_%Y_%H_%M')
        filename = self.foldername + '/' + self.folderLabel + 'N_5_m_' + str(
            self.m) + '-Scan_' + dateTime + '.npz'
        outfile = 'data/' + filename
        if saving:
            np.savez(outfile, filenames=filenames, foldername=self.foldername, N=self.N, args=args)
        return filenames

    def scan(self, saving=True, closing=True):
        periods = self.periods
        args = [[periods[i]] for i in range(len(periods))]
        filenames = self.multiThreadSetup(args, saving=saving, closing=closing)
        return filenames

    def filenameGenerate(self):
        dateTime = datetime.datetime.today().strftime('%m_%d_%Y_%H_%M')
        filename = self.folderLabel + 'ActuatorLength' + str(self.actuatorLength) + 'cm_N_' + str(
            self.N) + '_m_' + str(self.m) + '_Period_' + str(self.period) + 's_' + '.npz'
        return dateTime, filename


class InchwormDCResponseVoltageScan(InchwormPeriodScan):
    def __init__(self, parameter='parameter 2'):
        InchwormPeriodScan.__init__(self, parameter=parameter)
        self.folderLabel = 'Inchworm_dc_response_'
        self.scanLabel = '-VoltageScan_'
        self.voltages = [(0.0, 520.0, -960.0, 520.0, 0.0), (520.0, 520.0, -960.0, 520.0, 0.0),
                         (520.0, 0.0, 0.0, 0.0, 0.0)]
        self.voltage = None

    def applyVoltages(self, time, period):
        return self.voltage

    def reset(self):
        inchworm.reset(self)
        self.foldername = self.folderLabel + 'N_5_m_' + str(self.m) + self.scanLabel + self.dateTime

    def filenameGenerate(self):
        dateTime = datetime.datetime.today().strftime('%m_%d_%Y_%H_%M')
        filename = self.folderLabel + 'ActuatorLength' + str(self.actuatorLength) + 'cm_N_' + str(
            self.N) + '_m_' + str(self.m) + '_Voltage_' + str(self.voltage) + 'V_' + '.npz'
        return dateTime, filename

    def drive(self, arg, saving=True, closing=True):
        voltage = arg[0]
        simulator = deepcopy(self)
        simulator.voltage = voltage
        return simulator.robotDrive([None, ], saving=saving, closing=closing)

    def scan(self, saving=True, closing=True):
        voltages = self.voltages
        args = [[voltages[i]] for i in range(len(voltages))]
        filenames = self.multiThreadSetup(args, saving=saving, closing=closing)
        return filenames


class InchwormCrawlPeriodScan(InchwormPeriodScan):
    def __init__(self, parameter='parameter 2'):
        InchwormPeriodScan.__init__(self, parameter=parameter)
        self.folderLabel = 'SimpleWalker_ZZ7_InchwormCrawl_'

    def applyVoltages(self, time, period):
        if time % period < (1.0 / 2.0) * period:
            voltages = (0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            voltages = (0.0, self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
        return voltages


class InchwormReversedPeriodScan(InchwormPeriodScan):
    def __init__(self, parameter='parameter 2'):
        InchwormPeriodScan.__init__(self, parameter=parameter)
        self.folderLabel = 'SimpleWalker_ZZ7_Reversed_'

    def applyVoltages(self, time, period):
        if time % period < (1.0 / 4.0) * period:
            voltages = (0.0, 0.0, 0.0, 0.0, 0.0)
        elif time % period < (1.0 / 2.0) * period:
            voltages = (self.onVoltages[0], 0.0, 0.0, 0.0, 0.0)
        elif time % period < (3.0 / 4.0) * period:
            voltages = (self.onVoltages[0], self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
        else:
            voltages = (0.0, self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
        return voltages


class InchwormBackwardPeriodScan(InchwormPeriodScan):
    def __init__(self, parameter='parameter 2'):
        InchwormPeriodScan.__init__(self, parameter=parameter)
        self.folderLabel = 'SimpleWalker_ZZ7_Backward_'

    def applyVoltages(self, time, period):
        if time % period < (1.0 / 4.0) * period:
            voltages = (0.0, 0.0, 0.0, 0.0, 0.0)
        elif time % period < (1.0 / 2.0) * period:
            voltages = (0.0, self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
        elif time % period < (3.0 / 4.0) * period:
            voltages = (0.0, self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], self.onVoltages[4])
        else:
            voltages = (0.0, 0.0, 0.0, 0.0, self.onVoltages[4])
        return voltages


class FullInchwormPeriodScan(InchwormPeriodScan):
    def __init__(self, parameter='parameter 2'):
        InchwormPeriodScan.__init__(self, parameter=parameter)
        self.folderLabel = 'SimpleWalker_ZZ7_Full'

    def applyVoltages(self, time, period):
        """
        Default voltages for frictionless full inchworm forward motion
        """
        if time % period < (1.0 / 5.0) * period:
            voltages = (0.0, 0.0, 0.0, 0.0, 0.0)
        elif time % period < (2.0 / 5.0) * period:
            voltages = (0.0, 0.0, 0.0, 0.0, self.onVoltages[4])
        elif time % period < (3.0 / 5.0) * period:
            voltages = (0.0, self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], self.onVoltages[4])
        elif time % period < (4.0 / 5.0) * period:
            voltages = (self.onVoltages[0], self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
        else:
            voltages = (self.onVoltages[0], 0.0, 0.0, 0.0, 0.0)
        return voltages


class FullFrictionalInchwormPeriodScan(InchwormPeriodScan, FullFrictionalInchworm):
    def __init__(self, parameter='parameter 2'):
        InchwormPeriodScan.__init__(self, parameter=parameter)
        self.frictionHigh = 4.0
        self.folderLabel = 'SimpleWalker_ZZ7_FullFrictionalInchworm_'


class FrictionalFrogMotionV2PeriodScan(InchwormPeriodScan, FrictionalFrogMotionV2):
    def __init__(self, parameter='parameter 2'):
        InchwormPeriodScan.__init__(self, parameter=parameter)
        self.folderLabel = 'SimpleWalker_ZZ7_FrictionalFrogMotionV2_'


class FullFrictionalInchwormPeriodDampingFrictionScan(FullFrictionalInchwormPeriodScan):
    def __init__(self, parameter='parameter 2'):
        FullFrictionalInchwormPeriodScan.__init__(self, parameter=parameter)
        self.scanLabel = '-PeriodDampingFrictionScan_'
        self.dampingEtas = np.linspace(0.008, 0.108, num=5)
        self.frictionLows = np.linspace(0.9, 3.0, num=4)
        self.frictionHighs = np.linspace(3.5, 5.6, num=4)

    def scan(self, saving=True, closing=True):
        periods = self.periods
        dampingEtas = self.dampingEtas
        frictionLows = self.frictionLows
        frictionHighs = self.frictionHighs
        args = [[periods[i], dampingEtas[j], frictionLows[k], frictionHighs[l]]
                for i in range(len(periods))
                for j in range(len(dampingEtas))
                for k in range(len(frictionLows))
                for l in range(len(frictionHighs))]
        filenames = self.multiThreadSetup(args, saving=saving, closing=closing)
        return filenames

    def filenameGenerate(self):
        dateTime = datetime.datetime.today().strftime('%m_%d_%Y_%H_%M')
        filename = self.folderLabel + 'ActuatorLength' + str(self.actuatorLength) + 'cm_N_' + str(
            self.N) + '_m_' + str(self.m) + '_Period_' + str(self.period) + 's_' + 'DampingEta_' + str(
            self.dampingEta) + '_frictionlow_' + str(self.frictionLow) + '_frictionhigh_' + str(
            self.frictionHigh) + '.npz'
        return dateTime, filename

    def drive(self, arg, saving=True, closing=True):
        period, dampingEta, frictionLow, frictionHigh = arg

        simulator = deepcopy(self)
        simulator.dampingEta = dampingEta
        simulator.frictionLow = frictionLow
        simulator.frictionHigh = frictionHigh
        simulator.reset()
        return simulator.robotDrive([period, ], saving=saving, closing=closing)

    def printProgress(self):
        print("Frequency = ", round(1.0 / self.period, 2), "Hz, damping eta = ",
              self.dampingEta, ", friction low = ", self.frictionLow,
              ", friction high", self.frictionHigh, ", step=", self.step + 1)


class FrictionalFrogMotionV2PeriodDampingFrictionScan(FullFrictionalInchwormPeriodDampingFrictionScan,
                                                      FrictionalFrogMotionV2):
    def __init__(self, parameter='parameter 2'):
        FullFrictionalInchwormPeriodDampingFrictionScan.__init__(self, parameter=parameter)
        self.folderLabel = 'SimpleWalker_ZZ7_FrictionalFrogMotionV2_'


class EfficientJumper(InchwormCrawl):
    def __init__(self, parameter='parameter 2'):
        InchwormCrawl.__init__(self, parameter=parameter)
        self.folderLabel = 'EfficientJumper_'

    def applyVoltages(self, time, period):
        voltages = (0.0, self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
        return voltages


class EfficientJumperDelayScan(InchwormPeriodScan, EfficientJumper):
    def __init__(self, parameter='parameter 2', delays=None):
        InchwormPeriodScan.__init__(self, parameter=parameter)
        self.folderLabel = 'EfficientJumper_'
        if delays is None:
            delays = np.arange(-0.059, 0.061, 0.002)
        self.frequencies = 1.0 / delays

    def applyVoltages(self, time, period):
        delay = period
        time3 = 0.059309
        if delay >= 0:
            if time < delay:
                voltages = (0.0, self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
            elif time < time3:
                voltages = (
                self.onVoltages[0], self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], self.onVoltages[4])
            else:
                voltages = (0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            if time < abs(delay):
                voltages = (self.onVoltages[0], 0.0, 0.0, 0.0, self.onVoltages[4])
            elif time < time3 + abs(delay):
                voltages = (
                self.onVoltages[0], self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], self.onVoltages[4])
            else:
                voltages = (0.0, 0.0, 0.0, 0.0, 0.0)
        return voltages


class EfficientJumperActuatorLengthScan(EfficientJumperDelayScan):
    def __init__(self, parameter='parameter 2'):
        EfficientJumperDelayScan.__init__(self, parameter=parameter)
        self.scanLabel = '-ActuatorLengthScan_'
        self.period = 1.0 / 40.0
        self.actuatorLengths = np.arange(2, 10, 0.2)

    def applyVoltages(self, time, period):
        if time < 0.5 * period:
            voltages = (0.0, self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
        else:
            voltages = (0.0, 0.0, 0.0, 0.0, 0.0)
        return voltages

    def drive(self, arg, saving=True, closing=True):
        actuatorLength, = arg
        period = self.period
        simulator = deepcopy(self)
        simulator.actuatorLength = actuatorLength
        simulator.reset()
        return simulator.robotDrive([period, ], saving=saving, closing=closing)

    def scan(self, saving=True, closing=True):
        actuatorLengths = self.actuatorLengths
        args = [[actuatorLengths[i]] for i in range(len(actuatorLengths))]
        filenames = self.multiThreadSetup(args, saving=saving, closing=closing)
        return filenames


class EfficientJumperDelayActuatorLengthScan(EfficientJumperActuatorLengthScan):
    def __init__(self, parameter='parameter 2'):
        EfficientJumperActuatorLengthScan.__init__(self, parameter=parameter)
        self.scanLabel = '-ActuatorLength-Delay-Scan_'
        self.frequencies = np.arange(10.0, 41.0, 1.0)
        self.actuatorLengths = np.arange(2.0, 10.0, 1.0)

    def drive(self, arg, saving=True, closing=True):
        actuatorLength, period = arg
        simulator = deepcopy(self)
        simulator.actuatorLength = actuatorLength
        simulator.reset()
        return simulator.robotDrive([period, ], saving=saving, closing=closing)

    def scan(self, saving=True, closing=True):
        actuatorLengths = self.actuatorLengths
        periods = self.periods
        args = [[actuatorLengths[i], periods[j]] for i in range(len(actuatorLengths)) for j in range(len(periods))]
        filenames = self.multiThreadSetup(args, saving=saving, closing=closing)
        return filenames


class EfficientMover(FullFrictionalInchworm):
    def __init__(self, parameter='parameter 2'):
        FullFrictionalInchworm.__init__(self, parameter=parameter)
        self.folderLabel = 'EfficientMover_'

    def applyVoltages(self, time, period):
        time1 = 0.03437491666666667
        if time < time1:
            voltages = (self.onVoltages[0], 0.0, 0.0, 0.0, 0.0)
        else:
            voltages = (self.onVoltages[0], self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
        return voltages


class EfficientMoverDelayScan(InchwormPeriodScan, EfficientMover):
    def __init__(self, parameter='parameter 2', delays=None):
        InchwormPeriodScan.__init__(self, parameter=parameter)
        self.folderLabel = 'EfficientMover_'
        if delays is None:
            delays = np.arange(0.002, 0.062, 0.002)
        self.frequencies = 1.0 / delays

    def applyVoltages(self, time, period):
        delay = period
        if time < delay:
            voltages = (self.onVoltages[0], 0.0, 0.0, 0.0, 0.0)
        else:
            voltages = (self.onVoltages[0], self.onVoltages[1], self.onVoltages[2], self.onVoltages[3], 0.0)
        return voltages
