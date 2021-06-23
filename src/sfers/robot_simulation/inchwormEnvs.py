import pybullet as p
import pybullet_data
import math
import numpy as np
from sfers.robot_simulation.createActuator import create1DMultiActuators, generate1DMotorVoltages, \
    voltageTorqueControlStep


class inchworm(object):
    def __init__(self):
        self.frictionLow = 1.0
        self.frictionHigh = 2.5
        self.groundFriction = 1.0
        self.isGravity = True
        self.subThickness = 0.00508
        self.actThickness = 0.03
        self.actuatorLength = 10
        self.width = 2
        self.halfEI = 1.2595446893110755e6
        self.drivenFrequency = 1  # in Hz
        self.N = 5
        self.m = 1
        self.thickness = 0.1
        self.simTime = 0
        self.step = 0
        self.beta = 3.6327811654911875e-4
        self.timeStep = 0.01 / 240
        self.dampingEta = 0.2
        self.recordStepInterval = 125
        self.simCycles = 30000
        self.dataTime = []
        self.dataTheta = []
        self.dataAngularVelocities = []
        self.dataTor = []
        self.dataPositions = []
        self.dataMotorVoltages = []
        self.theta = []
        self.angularVelocities = []
        self.positions = []

    def simReset(self):
        self.actuator2DDensity = 8.1 * self.subThickness + 6.6 * self.actThickness
        self.actuator1DDensity = self.actuator2DDensity * self.width
        self.actuatorMass = self.actuator1DDensity * self.actuatorLength
        self.lateralFrictionCoefficients = [self.frictionLow for _ in
                                            range((self.N - 1) * self.m + int(math.ceil(2 * self.m / 3)))]
        for _ in range(self.m + 1 - int(math.ceil(2 * self.m / 3))):
            self.lateralFrictionCoefficients.append(self.frictionHigh)
        self.linkLength = self.actuatorLength / self.m
        self.jointLength = 0.5 * self.linkLength

        p.connect(p.DIRECT)
        p.resetSimulation()
        p.resetDebugVisualizerCamera(self.N * 15, -360, -16, [0, 0, 1])
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.planeId = p.loadURDF('plane.urdf', basePosition=[0, 0, -0.5 * self.thickness], globalScaling=10.0)
        (self.boxId, self.jointNumber) = create1DMultiActuators(self.N, self.m, self.actuatorMass,
                                                                self.actuatorLength, self.width, self.thickness)

        self.jointIndex = range(self.jointNumber)
        self.linkIndex = range(self.jointNumber + 1)
        p.setTimeStep(self.timeStep)
        if self.isGravity:
            p.setGravity(0, 0, -980)
        else:
            p.setGravity(0, 0, -1)

        p.changeDynamics(self.planeId, -1, spinningFriction=0.0, rollingFriction=0.0,
                         lateralFriction=self.groundFriction,
                         linearDamping=0.0, angularDamping=0.0, jointDamping=0.0)
        p.changeDynamics(self.planeId, 0, spinningFriction=0.0, rollingFriction=0.0,
                         lateralFriction=self.groundFriction,
                         linearDamping=0.0, angularDamping=0.0, jointDamping=0.0)
        p.changeDynamics(self.boxId, -1, spinningFriction=0.0, rollingFriction=0.0,
                         lateralFriction=self.lateralFrictionCoefficients[0],
                         linearDamping=0.0, angularDamping=0.0, jointDamping=0.0)
        for joint in self.jointIndex:
            p.changeDynamics(self.boxId, joint, spinningFriction=0.0, rollingFriction=0.0,
                             lateralFriction=self.lateralFrictionCoefficients[joint + 1],
                             linearDamping=0.0, angularDamping=0.0, jointDamping=0.0)

    def reset(self):
        self.simReset()

    def TorVolThe(self, Theta, angularVelocity, Voltage):
        thetaTarget = -self.beta * Voltage
        K = self.halfEI / self.jointLength
        omega = 2 * math.pi * self.drivenFrequency
        Tor = -self.width * K * ((Theta - thetaTarget) + (self.dampingEta / omega) * angularVelocity)
        return Tor

    def simStep(self, actuatorVoltages):
        if (self.step + 1) % 100000 == 0:
            print("step=", self.step + 1)

        self.simTime = self.step * self.timeStep
        [self.theta, self.angularVelocities, self.positions, self.motorVoltages, self.Tor] = voltageTorqueControlStep(
            self.boxId,
            actuatorVoltages,
            self.TorVolThe,
            self.N,
            self.m,
            self.jointNumber,
            self.jointIndex,
            self.linkIndex)

        if (self.step + 1) % self.recordStepInterval == 0 or self.step == 0:
            self.dataTime.append(self.simTime)
            self.dataTheta.append(self.theta)
            self.dataAngularVelocities.append(self.angularVelocities)
            self.dataPositions.append(self.positions)
            self.dataMotorVoltages.append(self.motorVoltages)
            self.dataTor.append(self.Tor)

        p.stepSimulation()

        self.step += 1

    def controlStep(self, actuatorVoltages):
        for simStep in range(self.simCycles):
            self.simStep(actuatorVoltages)

        return [list(-1 * np.array(self.theta)), list(-1 * np.array(self.angularVelocities)), self.positions,
                self.motorVoltages, self.Tor]

    def close(self):
        p.disconnect()
