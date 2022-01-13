import numpy as np
from sfers.robot_simulation.recorder import InchwormRobotShapeRecorder


class BaseSimulatorTest(object):
    def set_parameter(self, m):
        if m == 1:
            timeStep, recordStepInterval = 2e-3 / 240, 1
        else:
            timeStep, recordStepInterval = 2e-5 / 240, 1
        return timeStep, recordStepInterval

    def set_simulator(self, simulatorClass, m, parameter='trimorph parameter 2'):
        simulator = simulatorClass(parameter=parameter)
        simulator.m = m
        simulator.timeStep, simulator.recordStepInterval = self.set_parameter(m)
        simulator.dampingEta = 0.2
        simulator.simCycles = 10
        simulator.reset()
        return simulator

    def set_recorder(self, simulatorName):
        recorder = InchwormRobotShapeRecorder(simulator=simulatorName, record_intervals_in_steps=1, saving=False)
        recorder.simulator.simCycles = 10
        return recorder

    def simulator_testing(self, simulator):
        assert (len(simulator.positions) == simulator.N * simulator.m + 2)

    def recorder_testing(self, recorder):
        assert (len(recorder.positions) == recorder.N * recorder.m + 2)
        assert (abs(recorder.xAxis[0]) < 1e-3)
        assert (abs(recorder.zAxis[0]) < 1e-3)


def get_center_mass_positions(positions):
    positions = np.array(positions)
    shape = positions.shape
    if len(shape) == 3:
        axis = 1
    elif len(shape) == 2:
        axis = 0
    else:
        raise TypeError("Position's shape is not compatible.")
    length = shape[axis]
    total_mass_positions = 0.75 * np.sum(positions, axis=axis) - 0.5 * (np.take(positions, 0, axis=axis) + np.take(positions, -1, axis=axis))
    total_mass = 0.75 * length - 1
    center_mass_positions = total_mass_positions / total_mass
    return center_mass_positions
