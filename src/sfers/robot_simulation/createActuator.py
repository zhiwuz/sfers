import pybullet as p


def create1DMultiActuators(actuatorNumber, unitMotorNumber,
                           actuatorMass, actuatorLength,
                           actuatorWidth, actuatorThickness,
                           basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1]):
    N = actuatorNumber
    m = unitMotorNumber
    thickness = actuatorThickness
    width = actuatorWidth
    linkLength = actuatorLength / m
    jointLength = 0.5 * linkLength

    startBoxId = p.createCollisionShape(p.GEOM_BOX,
                                        halfExtents=[0.5 * jointLength, 0.5 * width, 0.5 * thickness])
    linkBoxId = p.createCollisionShape(p.GEOM_BOX,
                                       halfExtents=[0.5 * linkLength, 0.5 * width, 0.5 * thickness],
                                       collisionFramePosition=[0.5 * linkLength, 0, 0])
    endBoxId = p.createCollisionShape(p.GEOM_BOX,
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
    jointTypes = [p.JOINT_REVOLUTE for i in range(N * m)]

    axis = [[0, 1, 0] for i in range(N * m)]

    boxId = p.createMultiBody(mass,
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

    jointNumber = p.getNumJoints(boxId)
    # Disable the default motors
    for joint in range(jointNumber):
        p.setJointMotorControl2(boxId,
                                joint,
                                p.VELOCITY_CONTROL,
                                force=0)
    return [boxId, jointNumber]


def generate1DMotorVoltages(actuatorVoltages, actuatorNumber, unitMotorNumber):
    N = actuatorNumber
    m = unitMotorNumber
    motorVoltages = [actuatorVoltages[i] / m for i in range(N) for j in range(m)]
    return motorVoltages


def voltageTorqueControlStep(boxId, actuatorVoltages, TorVolThe, N, m, jointNumber, jointIndex, linkIndex):
    motorVoltages = generate1DMotorVoltages(actuatorVoltages, N, m)
    theta = []
    angularVelocities = []
    positions = []
    jointStates = p.getJointStates(boxId, jointIndex)
    linkStates = p.getLinkStates(boxId, linkIndex)
    for joint in range(jointNumber):
        theta.append(jointStates[joint][0])
        angularVelocities.append(jointStates[joint][1])
        positions.append(linkStates[joint][4])

    Tor = [TorVolThe(theta[joint], angularVelocities[joint], motorVoltages[joint]) for joint in range(jointNumber)]
    p.setJointMotorControlArray(boxId,
                                jointIndex,
                                p.TORQUE_CONTROL,
                                forces=Tor)
    return [theta, angularVelocities, positions, motorVoltages, Tor]
