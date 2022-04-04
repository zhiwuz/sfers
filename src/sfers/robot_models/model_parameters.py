class TrimorphModel(object):
    def __init__(self):
        self.gravity = 980.0
        self.actuatorLength = None
        self.width = None
        self.piezoPoisson = None
        self.subPoisson = None
        self.piezoModulusNoPoisson = None  # in CGS, 1 Ba = 0.1 Pa
        self.epoxyModulus = None  # in CGS, 1 Ba = 0.1 Pa
        self.subModulusNoPoisson = None
        self.subDensity = None
        self.piezoDensity = None
        self.epoxyDensity = None
        self.subThickness = None
        self.actThickness = None
        self.epoxyThickness = None
        self.d33NoPoisson = None
        self.p2p1ratio = None
        self.pitchDistance = None

    def reset(self):
        self.piezoModulus = self.piezoModulusNoPoisson / (1 - (self.piezoPoisson ** 2))  # in CGS, 1 Ba = 0.1 Pa
        self.subModulus = self.subModulusNoPoisson / (1 - (self.subPoisson ** 2))
        self.d33 = self.d33NoPoisson * (1 + self.piezoPoisson)
        self.actuator2DDensity = self.subDensity * self.subThickness + self.piezoDensity * self.actThickness + self.epoxyDensity * self.epoxyThickness
        self.actuator1DDensity = self.actuator2DDensity * self.width
        self.actuatorMass = self.actuator1DDensity * self.actuatorLength
        self.voltageExpansion = self.d33 / self.pitchDistance
        self.gamma = self.get_curvature(epoxyThickness=self.epoxyThickness)
        self.beta = self.gamma * self.actuatorLength
        self.EI = self.get_flexural_rigidty(epoxyThickness=self.epoxyThickness)
        self.halfEI = self.EI / 2.0
        self.load = self.actuatorMass * self.gravity / (self.actuatorLength * self.width)

    def get_curvature(self, epoxyThickness):
        EI = self.get_flexural_rigidty(epoxyThickness=epoxyThickness)
        zPiezo, _, _ = self.get_relative_position(epoxyThickness=epoxyThickness)
        gamma = self.voltageExpansion * zPiezo * self.piezoModulus * self.actThickness / EI
        return gamma

    def get_relative_position(self, epoxyThickness):
        neutralAxis = self.get_neutral_axis(epoxyThickness=epoxyThickness)
        zPiezo = 0.5 * self.actThickness + epoxyThickness - neutralAxis
        zEpoxy = 0.5 * epoxyThickness - neutralAxis
        zSub = -0.5 * self.subThickness - neutralAxis
        return zPiezo, zEpoxy, zSub

    def get_flexural_rigidty(self, epoxyThickness):
        zPiezo, zEpoxy, zSub = self.get_relative_position(epoxyThickness)
        piezoAreaMoment = 1.0 / 12.0 * (self.actThickness ** 3)
        epoxyAreaMoment = 1.0 / 12.0 * (epoxyThickness ** 3)
        subAreaMoment = 1.0 / 12.0 * (self.subThickness ** 3)
        EI = self.piezoModulus * piezoAreaMoment + self.epoxyModulus * epoxyAreaMoment + self.subModulus * subAreaMoment \
             + self.piezoModulus * self.actThickness * (zPiezo ** 2) + self.epoxyModulus * epoxyThickness * (
                         zEpoxy ** 2) \
             + self.subModulus * self.subThickness * (zSub ** 2)
        return EI

    def get_neutral_axis(self, epoxyThickness):
        neutralAxis = (self.piezoModulus * self.actThickness * (0.5 * self.actThickness + epoxyThickness)
                       + self.epoxyModulus * epoxyThickness * 0.5 * epoxyThickness
                       - self.subModulus * self.subThickness * 0.5 * self.subThickness) \
                      / (
                              self.piezoModulus * self.actThickness + self.epoxyModulus * epoxyThickness + self.subModulus * self.subThickness)
        return neutralAxis


class TrimorphModelParameter2(TrimorphModel):

    def __init__(self):
        TrimorphModel.__init__(self)
        self.actuatorLength = 10.0
        self.width = 2.0
        self.piezoPoisson = 0.31
        self.subPoisson = 0.27
        self.piezoModulusNoPoisson = 3.0e11  # in CGS, 1 Ba = 0.1 Pa
        self.epoxyModulus = 1.489e10  # in CGS, 1 Ba = 0.1 Pa
        self.subModulusNoPoisson = 1.9e12
        self.subDensity = 8.1
        self.piezoDensity = 6.6
        self.epoxyDensity = 1.07
        self.subThickness = 50.8e-4
        self.actThickness = 300.0e-4
        self.epoxyThickness = 10.0e-4
        self.d33NoPoisson = 370.0 * 1e-10
        self.p2p1ratio = 1.733
        self.pitchDistance = 5.0e-2


class TrimorphModelParameter4(TrimorphModelParameter2):
    def __init__(self):
        TrimorphModelParameter2.__init__(self)
        self.piezoModulusNoPoisson = 3.0336e11  # in CGS, 1 Ba = 0.1 Pa
        self.subModulusNoPoisson = 2.03e12
        self.subDensity = 7.85
        self.piezoDensity = 3.20
        self.actThickness = 294.0e-4
        self.d33NoPoisson = 460.0 * 1e-10


class modelParameter(TrimorphModelParameter4):
    def __init__(self, parameter='parameter 2'):
        if parameter == 'trimorph parameter 2':
            superclass = TrimorphModelParameter2
        elif parameter == 'trimorph parameter 4':
            superclass = TrimorphModelParameter4
        else:
            raise NotImplementedError('Model parameter not implemented')
        self.parameter = parameter
        self.superclass = superclass
        superclass.__init__(self)

    def reset(self):
        superclass = self.superclass
        superclass.reset(self)

    def get_curvature(self, *args, **kwargs):
        superclass = self.superclass
        return superclass.get_curvature(self, *args, **kwargs)

    def get_relative_position(self, *args, **kwargs):
        superclass = self.superclass
        return superclass.get_relative_position(self, *args, **kwargs)

    def get_flexural_rigidty(self, *args, **kwargs):
        superclass = self.superclass
        return superclass.get_flexural_rigidty(self, *args, **kwargs)

    def get_neutral_axis(self, *args, **kwargs):
        superclass = self.superclass
        return superclass.get_neutral_axis(self, *args, **kwargs)
