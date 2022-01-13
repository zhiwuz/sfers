import io
import math
import os

import cv2
import matplotlib.pyplot as plt
import numpy as np
from sfers.robot_simulation.inchworm_envs import RobotSimulator, inchworm, InchwormBackward, InchwormCrawl, FullFrictionalInchworm


class Recorder(object):
    def __init__(self, simulator, filename='output.avi', record_intervals_in_steps=None, framerate=12, saving=True):
        self.simulator = simulator
        self.record_intervals_in_steps = record_intervals_in_steps
        self.image = None
        self.filename = filename
        self.time = 0
        self.framerate = framerate
        self.imagesize = (1920, 1080)
        self.dpi = 120
        self.saving = saving

    def reset(self):
        if self.saving:
            self.writer = cv2.VideoWriter(self.filename, cv2.VideoWriter_fourcc(*'DIVX'), self.framerate,
                                          self.imagesize)

    def image_generate(self, **kwargs):
        raise NotImplementedError()

    def record(self, equal_scale=False, **kwargs):
        self.time = self.simulator.simTime
        image = self.image_generate(equal_scale=equal_scale, **kwargs)
        image = image[:, :, 2::-1]
        self.image = image
        if self.saving:
            self.writer.write(self.image)

    def close(self):
        if self.saving:
            self.writer.release()


class RobotShapeRecorder(Recorder):
    def __init__(self, simulator, filename='output.avi', record_intervals_in_steps=None, framerate=12, saving=True):
        Recorder.__init__(self, simulator, filename=filename, record_intervals_in_steps=record_intervals_in_steps,
                          framerate=framerate, saving=saving)
        self.positions = None
        self.xAxis = None
        self.zAxis = None
        self.N = None
        self.m = None
        self.xlim = (-10, 51)
        self.ylim = (-0.1, 1.1)

    def shape(self):
        xAxis, zAxis = self.simulator.shape()
        self.xAxis, self.zAxis = xAxis, zAxis
        self.positions = np.concatenate((self.xAxis[:, np.newaxis], self.zAxis[:, np.newaxis]), axis=1)
        return xAxis, zAxis

    def figure_generate(self, equal_scale=False, **kwargs):
        self.shape()
        figsize = (int(self.imagesize[0] / self.dpi),
                   int(self.imagesize[1] / self.dpi))
        fig = plt.figure(figsize=figsize, dpi=self.dpi, tight_layout=True)
        line = plt.plot(self.xAxis, self.zAxis, 'ko-')
        if len(kwargs.items()) > 0:
            plt.setp(line, **kwargs)
        plt.xlabel("x (cm)")
        plt.ylabel("z (cm)")
        if self.xlim is not None:
            plt.xlim(self.xlim)
        if self.ylim is not None:
            plt.ylim(self.ylim)
        plt.title('t = ' + str(round(self.time, 3)) + ' s')
        axis = fig.axes[0]
        if equal_scale:
            axis.set_aspect('equal', 'box')
        return fig

    def image_generate(self, equal_scale=False, **kwargs):
        fig = self.figure_generate(equal_scale=equal_scale, **kwargs)
        io_buf = io.BytesIO()
        fig.savefig(io_buf, format='raw', transparent=True, dpi=self.dpi)
        io_buf.seek(0)
        image = np.reshape(np.frombuffer(io_buf.getvalue(), dtype=np.uint8),
                           newshape=(int(fig.bbox.bounds[3]), int(fig.bbox.bounds[2]), -1))
        io_buf.close()
        plt.close(fig)
        return image


class RobotShapeRecorderFromData(RobotShapeRecorder, RobotSimulator):
    def __init__(self, data_filename, simulator=None, filename=None, record_intervals_in_steps=4, framerate=12,
                 saving=True, real_time_limit=0.25):
        if filename is None:
            if saving:
                filename = os.path.splitext(data_filename)[0] + '.avi'
        RobotShapeRecorder.__init__(self, simulator, filename=filename,
                                    record_intervals_in_steps=record_intervals_in_steps, framerate=framerate,
                                    saving=saving)
        self.data_filename = data_filename
        self.real_time_limit = real_time_limit
        self.jointLength = None
        self.dataTime = None
        self.dataPositions = None
        self.N = None
        self.m = None
        self.actuatorLength = None
        self.size = None

    def shape(self):
        positions = self.positions
        xAxis, zAxis = self.get_shape(positions)
        self.xAxis, self.zAxis = xAxis, zAxis
        self.positions = np.concatenate((self.xAxis[:, np.newaxis], self.zAxis[:, np.newaxis]), axis=1)
        return xAxis, zAxis

    def reset(self):
        RobotShapeRecorder.reset(self)
        npzFile = np.load(self.data_filename)
        self.dataTime = npzFile['dataTime']
        self.dataPositions = npzFile['dataPositions']
        self.m = npzFile['m']
        self.N = npzFile['N']
        self.actuatorLength = npzFile['actuatorLength']
        self.jointLength = self.actuatorLength / (2 * self.m)
        self.size = len(self.dataTime)

    def run(self, equal_scale=False, **kwargs):
        dataTime = self.dataTime
        dataPositions = self.dataPositions
        size = self.size
        for step, (time, positions) in enumerate(zip(dataTime, dataPositions)):
            self.print_update(step=step)
            if step == 0 or (step + 1) % self.record_intervals_in_steps == 0 or step + 1 == size:
                self.time = time
                if self.real_time_limit is not None:
                    if time > self.real_time_limit:
                        break
                self.positions = positions
                self.record(equal_scale=equal_scale, **kwargs)
        self.close()

    def print_update(self, step):
        if (step + 1) % 10000 == 0 or step + 1 == self.size:
            print("Step = ", step+1, ".")

    def record(self, **kwargs):
        image = self.image_generate(**kwargs)
        image = image[:, :, 2::-1]
        self.image = image
        if self.saving:
            self.writer.write(self.image)

    def position_offset(self, positions):
        jointLength = self.jointLength
        offset = jointLength / 2.0
        for i in range(len(positions)):
            positions[i] = self.position_transform(positions[i], offset, angle=0.0)
        return positions


class InchwormRobotShapeRecorder(RobotShapeRecorder):
    def __init__(self, simulator=None, filename='output.avi', record_intervals_in_steps=500, framerate=12,
                 driving_period=0.05, saving=True):
        if simulator is None or simulator == 'forward':
            simulator = inchworm(parameter='trimorph parameter 2')
        elif simulator == 'backward':
            simulator = InchwormBackward(parameter='trimorph parameter 2')
        elif simulator == 'crawl':
            simulator = InchwormCrawl(parameter='trimorph parameter 2')
        elif simulator == 'full frictional inchworm':
            simulator = FullFrictionalInchworm(parameter='trimorph parameter 2')
        RobotShapeRecorder.__init__(self, simulator, filename=filename,
                                    record_intervals_in_steps=record_intervals_in_steps, framerate=framerate,
                                    saving=saving)
        self.period = driving_period
        self.simulator.period = driving_period

    def run(self, equal_scale=False, **kwargs):
        period = self.period
        simulator = self.simulator
        simulator.reset()
        self.N = simulator.N
        self.m = simulator.m
        for step in range(simulator.simCycles):
            actuatorVoltages = simulator.applyVoltages(simulator.simTime, period)
            simulator.simStep(actuatorVoltages)
            if step == 0 or (step + 1) % self.record_intervals_in_steps == 0:
                self.record(equal_scale=equal_scale, **kwargs)
        simulator.close()
        if self.saving:
            simulator.save()
        self.close()
