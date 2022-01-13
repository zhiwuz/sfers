import math
from sfers.robot_models.model_parameters import modelParameter


class RobotBase(modelParameter):
    def __init__(self, parameter='parameter 2'):
        modelParameter.__init__(self, parameter=parameter)
        self._p = None

    def create1DMultiActuators(self,
                               actuatorNumber,
                               unitMotorNumber,
                               actuatorMass,
                               actuatorLength,
                               actuatorWidth,
                               actuatorThickness,
                               basePosition=(0, 0, 0),
                               baseOrientation=(0, 0, 0, 1)):
        N = actuatorNumber
        m = unitMotorNumber
        thickness = actuatorThickness
        width = actuatorWidth
        linkLength = actuatorLength / m
        jointLength = 0.5 * linkLength

        startBoxId = self._p.createCollisionShape(self._p.GEOM_BOX,
                                                  halfExtents=[0.5 * jointLength, 0.5 * width, 0.5 * thickness])
        linkBoxId = self._p.createCollisionShape(self._p.GEOM_BOX,
                                                 halfExtents=[0.5 * linkLength, 0.5 * width, 0.5 * thickness],
                                                 collisionFramePosition=[0.5 * linkLength, 0, 0])
        endBoxId = self._p.createCollisionShape(self._p.GEOM_BOX,
                                                halfExtents=[0.5 * jointLength, 0.5 * width, 0.5 * thickness],
                                                collisionFramePosition=[0.5 * jointLength, 0, 0])

        mass = actuatorMass / (2 * m)
        visualShapeId = -1
        basePosition = basePosition
        baseOrientation = baseOrientation

        link_Masses = [actuatorMass / m for i in range(N * m - 1)]
        link_Masses.append(actuatorMass / (2 * m))

        linkCollisionShapeIndices = [linkBoxId for i in range(N * m - 1)]
        linkCollisionShapeIndices.append(endBoxId)

        linkVisualShapeIndices = [-1 for i in range(N * m)]

        linkPositions = [[0.5 * jointLength, 0, 0]]
        for i in range(N * m - 1):
            linkPositions.append([linkLength, 0, 0])

        linkOrientations = [[0, 0, 0, 1] for i in range(N * m)]

        linkInertialFramePositions = [[0.5 * linkLength, 0, 0] for i in range(N * m - 1)]
        linkInertialFramePositions.append([0.5 * jointLength, 0, 0])

        linkInertialFrameOrientations = [[0, 0, 0, 1] for i in range(N * m)]
        indices = [i for i in range(N * m)]
        jointTypes = [self._p.JOINT_REVOLUTE for i in range(N * m)]

        axis = [[0, 1, 0] for i in range(N * m)]

        boxId = self._p.createMultiBody(mass,
                                        startBoxId,
                                        visualShapeId,
                                        basePosition,
                                        baseOrientation,
                                        linkMasses=link_Masses,
                                        linkCollisionShapeIndices=linkCollisionShapeIndices,
                                        linkVisualShapeIndices=linkVisualShapeIndices,
                                        linkPositions=linkPositions,
                                        linkOrientations=linkOrientations,
                                        linkInertialFramePositions=linkInertialFramePositions,
                                        linkInertialFrameOrientations=linkInertialFrameOrientations,
                                        linkParentIndices=indices,
                                        linkJointTypes=jointTypes,
                                        linkJointAxis=axis)

        jointNumber = self._p.getNumJoints(boxId)
        # Disable the default motors
        for joint in range(jointNumber):
            self._p.setJointMotorControl2(boxId,
                                          joint,
                                          self._p.VELOCITY_CONTROL,
                                          force=0)
        return [boxId, jointNumber]

    @staticmethod
    def generate1DMotorVoltages(actuatorVoltages, actuatorNumber, unitMotorNumber):
        N = actuatorNumber
        m = unitMotorNumber
        motorVoltages = [actuatorVoltages[i] / m for i in range(N) for j in range(m)]
        return motorVoltages

    def voltageTorqueControlStep(self, boxId, actuatorVoltages, TorVolThe, N, m, jointNumber, jointIndex, linkIndex,
                                 jointLength):
        motorVoltages = self.generate1DMotorVoltages(actuatorVoltages, N, m)
        theta = []
        angularVelocities = []
        positions = []
        positionVelocities = []
        jointStates = self._p.getJointStates(boxId, jointIndex)
        linkStates = self._p.getLinkStates(boxId, linkIndex, computeLinkVelocity=1)
        for joint in range(jointNumber):
            theta.append(jointStates[joint][0])
            angularVelocities.append(jointStates[joint][1])
            positions.append(linkStates[joint][4])
            positionVelocities.append(linkStates[joint][6])
        positions, positionVelocities = self.get_positions_and_velocities(positions, positionVelocities, boxId,
                                                                          jointNumber,
                                                                          jointLength)
        Tor = [TorVolThe(theta[joint], angularVelocities[joint], motorVoltages[joint]) for joint in range(jointNumber)]
        self._p.setJointMotorControlArray(boxId,
                                          jointIndex,
                                          self._p.TORQUE_CONTROL,
                                          forces=Tor)
        return [theta, angularVelocities, positions, motorVoltages, Tor, positionVelocities]

    def start_point(self, boxId, jointLength):
        base_state = self._p.getBasePositionAndOrientation(boxId)
        base_position = base_state[0]
        base_orientation = base_state[1]
        base_velocity_state = self._p.getBaseVelocity(boxId)
        base_position_velocity = base_velocity_state[0]
        base_orientation_velocity = base_velocity_state[1]
        _, pitch, _ = self._p.getEulerFromQuaternion(base_orientation)
        _, pitch_velocity, _ = base_orientation_velocity
        length = jointLength / 2.0
        angle = math.pi - pitch
        angle_velocity = -pitch_velocity
        start_position = self.position_transform(base_position, length, angle)
        start_position_velocity = self.velocity_transform(base_position_velocity, length, angle, angle_velocity)
        return start_position, start_position_velocity

    def end_point(self, boxId, jointNumber, jointLength):
        state = self._p.getLinkState(boxId, jointNumber - 1, computeLinkVelocity=1)
        position = state[4]
        orientation = state[5]
        position_velocity = state[6]
        orientation_velocity = state[7]
        _, pitch, _ = self._p.getEulerFromQuaternion(orientation)
        _, pitch_velocity, _ = orientation_velocity
        length = jointLength
        angle = -pitch
        angle_velocity = -pitch_velocity
        end_position = self.position_transform(position, length, angle)
        end_position_velocity = self.velocity_transform(position_velocity, length, angle, angle_velocity)
        return end_position, end_position_velocity

    @staticmethod
    def position_transform(position, length, angle):
        return (position[0] + length * math.cos(angle),
                0.0,
                position[2] + length * math.sin(angle))

    @staticmethod
    def velocity_transform(position_velocity, length, angle, angle_velocity):
        return (position_velocity[0] - length * math.sin(angle) * angle_velocity,
                0.0,
                position_velocity[2] + length * math.cos(angle) * angle_velocity)

    def get_positions_and_velocities(self, positions, positionVelocities, boxId, jointNumber, jointLength):
        start_position, start_position_velocity = self.start_point(boxId, jointLength)
        end_position, end_position_velocity = self.end_point(boxId, jointNumber, jointLength)
        positions.insert(0, start_position)
        positions.append(end_position)
        positionVelocities.insert(0, start_position_velocity)
        positionVelocities.append(end_position_velocity)
        return positions, positionVelocities
