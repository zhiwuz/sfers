class bimorphModel(object):
    def __init__(self):
        self.gravity = 980.0
        self.actuatorLength = None
        self.width = None
        self.piezoModulus = None  # in CGS, 1 Ba = 0.1 Pa
        self.subModulus = None
        self.subDensity = None
        self.piezoDensity = None
        self.subThickness = None
        self.actThickness = None
        self.d33 = None
        self.p2p1ratio = None
        self.pitchDistance = None

        self.actuator2DDensity = None
        self.actuator1DDensity = None
        self.actuatorMass = None
        self.voltageExpansion = None
        self.gamma = None
        self.beta = None
        self.EI = None

    def reset(self):
        """
        All quantities should be in CGS units, unless otherwise specified
        """
        self.actuator2DDensity = self.subDensity * self.subThickness + self.piezoDensity * self.actThickness
        self.actuator1DDensity = self.actuator2DDensity * self.width
        self.actuatorMass = self.actuator1DDensity * self.actuatorLength
        self.voltageExpansion = self.d33 / self.pitchDistance
        self.gamma = self.getCurvature(subThickness=self.subThickness)
        self.beta = self.gamma * self.actuatorLength
        self.EI = self.getFlexuralRigidty()
        self.halfEI = self.EI / 2.0
        self.load = self.actuatorMass * self.gravity / (self.actuatorLength * self.width)

    def getCurvature(self, subThickness):
        gamma = 6 * self.piezoModulus * self.subModulus * (
                self.actThickness + subThickness) * self.actThickness * subThickness * self.voltageExpansion
        gamma = gamma / ((self.piezoModulus ** 2) * (self.actThickness ** 4)
                         + 4 * self.piezoModulus * self.subModulus * (self.actThickness ** 3) * subThickness
                         + 6 * self.piezoModulus * self.subModulus * (self.actThickness ** 2) * (subThickness ** 2)
                         + 4 * self.piezoModulus * self.subModulus * self.actThickness * (subThickness ** 3)
                         + (self.subModulus ** 2) * (subThickness ** 4))
        return gamma

    def getNeutralAxis(self):
        neutralAxis = 0.5 * (self.piezoModulus * (self.actThickness ** 2) - self.subModulus * (self.subThickness ** 2)) \
                      / (self.piezoModulus * self.actThickness + self.subModulus * self.subThickness)
        return neutralAxis

    def getFlexuralRigidty(self):
        """
        Flexural rigidity per unit width
        """
        neutralAxis = self.getNeutralAxis()
        EI = (1.0 / 3.0) * self.piezoModulus * (self.actThickness ** 3) + \
             (1.0 / 3.0) * self.subModulus * (self.subThickness ** 3) + \
             self.piezoModulus * self.actThickness * neutralAxis * (neutralAxis - self.actThickness) + \
             self.subModulus * self.subThickness * neutralAxis * (neutralAxis + self.subThickness)
        return EI


class modelParameter2(bimorphModel):
    """
    Material parameter used for P1 MFC actuator, bonded onto a 2 mil 304 stainless steel foil with 3M double sided tape
    """

    def __init__(self):
        bimorphModel.__init__(self)
        self.actuatorLength = 10.0
        self.width = 6.0
        self.piezoModulus = 3.0e11  # in CGS, 1 Ba = 0.1 Pa
        self.subModulus = 1.9e12
        self.subDensity = 8.1
        self.piezoDensity = 6.6
        self.subThickness = 50.8e-4
        self.actThickness = 290e-4
        self.d33 = 183.0 * 1e-10
        self.pitchDistance = 5.0e-2
        self.p2p1ratio = 1.733


class modelParameter3(bimorphModel):
    def __init__(self):
        bimorphModel.__init__(self)
        self.actuatorLength = 10.0
        self.width = 6.0
        self.piezoModulus = 2.4e11  # in CGS, 1 Ba = 0.1 Pa
        self.subModulus = 1.9e12
        self.subDensity = 8.0
        self.piezoDensity = 5.44
        self.subThickness = 50.8e-4
        self.actThickness = 360.0e-4
        self.d33 = 370.0 * 1e-10
        self.pitchDistance = 5.0e-2
        self.p2p1ratio = 1.733


class modelParameter4(bimorphModel):
    """
    Reference:
    https://www.azom.com/properties.aspx?ArticleID=965
    """

    def __init__(self):
        bimorphModel.__init__(self)
        self.actuatorLength = 10.0
        self.width = 6.0
        self.piezoModulus = 3.0336e11  # in CGS, 1 Ba = 0.1 Pa
        self.subModulus = 2.03e12
        self.subDensity = 7.85
        self.piezoDensity = 3.20
        self.subThickness = 50.8e-4
        self.actThickness = 294.0e-4
        self.d33 = 460.0 * 1e-10
        self.p2p1ratio = 1.733
        self.pitchDistance = 5.0e-2


class trimorphModel(object):
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
        self.gamma = self.getCurvature(epoxyThickness=self.epoxyThickness)
        self.beta = self.gamma * self.actuatorLength
        self.EI = self.getFlexuralRigidty(epoxyThickness=self.epoxyThickness)
        self.halfEI = self.EI / 2.0
        self.load = self.actuatorMass * self.gravity / (self.actuatorLength * self.width)

    def getCurvature(self, epoxyThickness):
        EI = self.getFlexuralRigidty(epoxyThickness)
        zPiezo, _, _ = self.getRelativePosition(epoxyThickness)
        gamma = self.voltageExpansion * zPiezo * self.piezoModulus * self.actThickness / EI
        return gamma

    def getRelativePosition(self, epoxyThickness):
        neutralAxis = self.getNeutralAxis(epoxyThickness)
        zPiezo = 0.5 * self.actThickness + epoxyThickness - neutralAxis
        zEpoxy = 0.5 * epoxyThickness - neutralAxis
        zSub = -0.5 * self.subThickness - neutralAxis
        return zPiezo, zEpoxy, zSub

    def getFlexuralRigidty(self, epoxyThickness):
        zPiezo, zEpoxy, zSub = self.getRelativePosition(epoxyThickness)
        piezoAreaMoment = 1.0 / 12.0 * (self.actThickness ** 3)
        epoxyAreaMoment = 1.0 / 12.0 * (epoxyThickness ** 3)
        subAreaMoment = 1.0 / 12.0 * (self.subThickness ** 3)
        EI = self.piezoModulus * piezoAreaMoment + self.epoxyModulus * epoxyAreaMoment + self.subModulus * subAreaMoment \
             + self.piezoModulus * self.actThickness * (zPiezo ** 2) + self.epoxyModulus * epoxyThickness * (
                         zEpoxy ** 2) \
             + self.subModulus * self.subThickness * (zSub ** 2)
        return EI

    def getNeutralAxis(self, epoxyThickness):
        neutralAxis = (self.piezoModulus * self.actThickness * (0.5 * self.actThickness + epoxyThickness)
                       + self.epoxyModulus * epoxyThickness * 0.5 * epoxyThickness
                       - self.subModulus * self.subThickness * 0.5 * self.subThickness) \
                      / (
                              self.piezoModulus * self.actThickness + self.epoxyModulus * epoxyThickness + self.subModulus * self.subThickness)
        return neutralAxis


class trimorphModelParameter2(trimorphModel):

    def __init__(self):
        trimorphModel.__init__(self)
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


class trimorphModelParameter3(trimorphModelParameter2):
    def __init__(self):
        trimorphModelParameter2.__init__(self)
        self.piezoModulusNoPoisson = 2.4e11  # in CGS, 1 Ba = 0.1 Pa
        self.subDensity = 8.0
        self.piezoDensity = 5.44
        self.actThickness = 360.0e-4


class trimorphModelParameter4(trimorphModelParameter2):
    def __init__(self):
        trimorphModelParameter2.__init__(self)
        self.piezoModulusNoPoisson = 3.0336e11  # in CGS, 1 Ba = 0.1 Pa
        self.subModulusNoPoisson = 2.03e12
        self.subDensity = 7.85
        self.piezoDensity = 3.20
        self.actThickness = 294.0e-4
        self.d33NoPoisson = 460.0 * 1e-10

class trimorphModelParameter5(trimorphModelParameter2):
    """
    Single actuator epoxy bonded cantilever ac response
    """
    def __init__(self):
        trimorphModelParameter2.__init__(self)
        self.piezoModulusNoPoisson = 3.0336e11  # in CGS, 1 Ba = 0.1 Pa
        self.subModulusNoPoisson = 2.03e12
        self.subDensity = 7.85
        self.piezoDensity = 3.20


class modelParameter(modelParameter2, modelParameter3, modelParameter4, trimorphModelParameter3,
                     trimorphModelParameter4, trimorphModelParameter5):
    def __init__(self, parameter='parameter 2'):
        if parameter == 'parameter 2':
            superclass = modelParameter2
        elif parameter == 'parameter 3':
            superclass = modelParameter3
        elif parameter == 'parameter 4':
            superclass = modelParameter4
        elif parameter == 'trimorph parameter 4':
            superclass = trimorphModelParameter4
        elif parameter == 'trimorph parameter 2':
            superclass = trimorphModelParameter2
        elif parameter == 'trimorph parameter 3':
            superclass = trimorphModelParameter3
        elif parameter == 'trimorph parameter 5':
            superclass = trimorphModelParameter5
        else:
            raise NotImplementedError('Model parameter not implemented')
        self.parameter = parameter
        self.superclass = superclass
        superclass.__init__(self)

    def reset(self):
        superclass = self.superclass
        superclass.reset(self)

    def getCurvature(self, *args, **kwargs):
        superclass = self.superclass
        return superclass.getCurvature(self, *args, **kwargs)

    def getRelativePosition(self, *args, **kwargs):
        superclass = self.superclass
        return superclass.getRelativePosition(self, *args, **kwargs)

    def getFlexuralRigidty(self, *args, **kwargs):
        superclass = self.superclass
        return superclass.getFlexuralRigidty(self, *args, **kwargs)

    def getNeutralAxis(self, *args, **kwargs):
        superclass = self.superclass
        return superclass.getNeutralAxis(self, *args, **kwargs)
