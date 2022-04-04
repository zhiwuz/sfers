import pytest
import io
import os

from sfers.robot_simulation.inchworm_envs import SingleActuatorCantilever, \
    SingleActuatorCantileverDCResponseVoltageScan, SingleActuatorCantileverACResponse, \
    Inchworm, InchwormBackward, FullFrictionalInchworm, InchwormCrawl, FrictionalFrogMotionV2, \
    InchwormPeriodScan, InchwormDCResponseVoltageScan, InchwormCrawlPeriodScan, InchwormReversedPeriodScan, \
    InchwormBackwardPeriodScan, FullInchwormPeriodScan, \
    FullFrictionalInchwormPeriodScan, FrictionalFrogMotionV2PeriodScan, FullFrictionalInchwormPeriodDampingFrictionScan

from sfers.robot_simulation.recorder import RobotShapeRecorderFromData
from sfers.utils.simulator_evaluation import BaseSimulatorTest


class TestSingleActuatorSimulator(BaseSimulatorTest):
    simulator_class_set = [SingleActuatorCantilever]
    mSet = [1, 3, 5]

    @pytest.mark.timeout(300)
    @pytest.mark.parametrize("simulator_class", simulator_class_set)
    @pytest.mark.parametrize("m", mSet)
    def test_dc_simulation(self, simulator_class, m):
        simulator = self.set_simulator(simulator_class, m)
        simulator.isGravity = True
        actuatorVoltages = [-1500]
        simulator.actuator_dc_drive([actuatorVoltages], saving=False)
        self.simulator_testing(simulator)

    @pytest.mark.timeout(300)
    @pytest.mark.parametrize("simulator_class", simulator_class_set)
    @pytest.mark.parametrize("m", mSet)
    def test_ac_simulation(self, simulator_class, m):
        simulator = self.set_simulator(simulator_class, m)
        drivenFrequency = 1.0
        actuatorLength = 10.0
        arg = [m, drivenFrequency, actuatorLength]
        simulator.drive(arg, saving=False)
        self.simulator_testing(simulator)

    @pytest.mark.timeout(300)
    @pytest.mark.parametrize("simulator_class", simulator_class_set)
    @pytest.mark.parametrize("m", mSet)
    def test_data_recorder(self, simulator_class, m):
        simulator = self.set_simulator(simulator_class, m)
        io_buf = io.BytesIO()
        simulator.outfilename = io_buf
        simulator.reset()
        drivenFrequency = 1.0
        actuatorLength = 10.0
        arg = [m, drivenFrequency, actuatorLength]
        simulator.drive(arg, saving=True)
        io_buf.seek(0)
        file = io_buf.read()
        recorder = RobotShapeRecorderFromData(data_filename=io.BytesIO(file), record_intervals_in_steps=1, saving=False)
        recorder.reset()
        recorder.run()


class TestInchwormRobotSimulator(BaseSimulatorTest):
    simulator_class_set = [Inchworm, InchwormBackward, FullFrictionalInchworm, InchwormCrawl, FrictionalFrogMotionV2]
    mSet = [1, 3]

    @pytest.mark.timeout(300)
    @pytest.mark.parametrize("simulator_class", simulator_class_set)
    @pytest.mark.parametrize("m", mSet)
    def test_simulation(self, simulator_class, m):
        simulator = self.set_simulator(simulator_class, m)
        period = 1.0
        simulator.drive([period], saving=False)
        self.simulator_testing(simulator)

    @pytest.mark.timeout(300)
    @pytest.mark.parametrize("simulator_class", simulator_class_set)
    @pytest.mark.parametrize("m", mSet)
    def test_data_recorder(self, simulator_class, m):
        simulator = self.set_simulator(simulator_class, m)
        io_buf = io.BytesIO()
        simulator.outfilename = io_buf
        simulator.reset()
        period = 1.0
        simulator.drive([period], saving=True)
        io_buf.seek(0)
        file = io_buf.read()
        recorder = RobotShapeRecorderFromData(data_filename=io.BytesIO(file), record_intervals_in_steps=1, saving=False)
        recorder.reset()
        recorder.run()
        self.recorder_testing(recorder)

    @pytest.mark.timeout(300)
    @pytest.mark.parametrize("simulator_class", simulator_class_set)
    @pytest.mark.parametrize("m", mSet)
    def test_save_and_load_state(self, simulator_class, m):
        simulator = self.set_simulator(simulator_class, m)
        simulator.reset()
        period = 1.0
        simulator.drive([period], saving=False, closing=False)
        state_file = simulator.save_simulator_state()
        state_file_name = state_file.name
        simulator.close()
        simulator.reset()
        simulator.load_simulator_state(state_file_name)
        os.remove(state_file_name)
        simulator.drive([period], saving=False, closing=True)

    @pytest.mark.timeout(300)
    @pytest.mark.parametrize("simulator_class", simulator_class_set)
    @pytest.mark.parametrize("m", mSet)
    def test_simulator_copy(self, simulator_class, m):
        simulator = self.set_simulator(simulator_class, m)
        simulator.reset()
        state_file = simulator.save_simulator_state()
        state_file_name = state_file.name
        simulator.copy(state_file_name)


class TestInchwormRobotScanner(BaseSimulatorTest):
    scanner_class_set = [InchwormPeriodScan,
                         InchwormCrawlPeriodScan,
                         InchwormReversedPeriodScan,
                         InchwormBackwardPeriodScan,
                         FullInchwormPeriodScan,
                         FullFrictionalInchwormPeriodScan,
                         FrictionalFrogMotionV2PeriodScan,
                         FullFrictionalInchwormPeriodDampingFrictionScan,
                         SingleActuatorCantileverDCResponseVoltageScan,
                         SingleActuatorCantileverACResponse,
                         InchwormDCResponseVoltageScan]
    mSet = [1, 3]

    @pytest.mark.timeout(300)
    @pytest.mark.parametrize("scanner_class", scanner_class_set)
    @pytest.mark.parametrize("m", mSet)
    def test_scaning(self, scanner_class, m):
        scanner = self.set_simulator(scanner_class, m)
        scanner.scan(saving=False)


class TestRobotShapeRecorder(BaseSimulatorTest):
    simulator_name_set = ['forward',
                          'backward',
                          'crawl',
                          'full frictional inchworm']

    @pytest.mark.timeout(300)
    @pytest.mark.parametrize("simulator_name", simulator_name_set)
    def test_recording(self, simulator_name):
        recorder = self.set_recorder(simulator_name)
        recorder.run()
        self.recorder_testing(recorder)
