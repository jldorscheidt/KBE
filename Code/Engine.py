from __future__ import division

from math import *
from parapy.geom import *
from parapy.core import *

class Engine(GeomBase):
    #: Total thrust per engine
    Thrust = Input(10000.) ## thrust in lbs
    Xf_ratio_c = Input(-0.14) ## horizontal engine position of nacelle with respect to leading edge of wing chord. Negative is fwd, positive is aft
    Chord = Input(2) ##chord length at engine position
    NacelleThickness = Input(0.1)
    NacelleLength = Input(2)
    MountType = Input('Tail')
    EngPosition = Input(Point(0,0,0),Orientation(x=Vector(1,0,0),y=Vector(0,1,0),z=Vector(0,0,1)))
    FuselageDistance = Input(1.5) # lateral distance from nacelle to fuselage for fuselage mounted engines, multiplied with nacelle radius

    @Input
    #: average radius of inner engine in meters
    def engineAvradius(self):
        engineAvradius=0.5*0.0254*(1.0827*self.Thrust**0.4134)
        return engineAvradius

    @Input
    #: inner engine length
    def engineLength(self):
        engineLength=0.0254*(2.4077*self.Thrust**0.3876)
        return engineLength

    @Attribute
    #: Nacelle radius
    def NacelleRadius(self):
        nacelleRad=1.10*self.engineAvradius
        return nacelleRad

    @Part
    #: Inner engine
    def InnerEngine(self):
        return TranslatedShape(Cylinder(radius=self.engineAvradius,height=self.engineLength,hidden=True),displacement=Vector(0,0,-0.5*self.engineLength),hidden=True)

    @Attribute
    #: Nacelle
    def Nacelleshape(self):
        #start of nacelle, nacelle extends 60 percent of maximum nacelle diameter, which is 1.1*engine diameter
        zpos1=0.5*self.engineLength+0.6*(1.1*2*self.engineAvradius)

        crv1 = FittedCurve(points=[Point(self.NacelleRadius,0,zpos1),Point(self.NacelleRadius,0,zpos1-self.NacelleLength),
                                   Point(self.NacelleRadius+self.NacelleThickness,0,zpos1-self.NacelleLength),
                                   Point(self.NacelleRadius+self.NacelleThickness,0,zpos1),
                                   Point(self.NacelleRadius,0,zpos1)],max_degree=1)
        return crv1

    @Part
    def Nacelle(self):
        # Nacelle
        return Revolution(self.Nacelleshape,2*pi,hidden=True)

    @Attribute
    def VerEngineRatio(self):
        # Engine Vertical position to chord ratio, measured from center of engine with respect to wing chord.
        if self.Xf_ratio_c > -0.2 and self.Xf_ratio_c < 0.18:
            H_ratio_c = 0.07+0.03*cos(15*(self.Xf_ratio_c+0.03))
        else:
            H_ratio_c = 0.04
        return H_ratio_c


    @Attribute
    def AttachPoint(self):
        # attachment point of engine with respect to chord
        return Point(self.NacelleRadius+self.NacelleThickness+self.VerEngineRatio*self.Chord if self.MountType=='Wing' else self.FuselageDistance*(self.NacelleRadius+self.NacelleThickness) ,0,(0.5*self.engineLength+0.6*(1.1*2*self.engineAvradius)-self.NacelleLength)+self.Xf_ratio_c*self.Chord if self.MountType=='Wing' else (0.5*self.engineLength+0.6*(1.1*2*self.engineAvradius)-self.NacelleLength))

    @Part
    def Compound(self):
        return Compound(built_from=[self.InnerEngine,self.Nacelle],hidden=True)

    @Part
    def rotatedEngine1(self):
        return RotatedShape(self.Compound,self.AttachPoint,Vector(0,1,0),0.5*pi,hidden=True)

    @Part
    def rotatedEngine2(self):
        return RotatedShape(self.rotatedEngine1,self.AttachPoint,Vector(1,0,0),angle=0*pi if self.MountType == 'Wing' else -0.5*pi,hidden=True)

    @Part
    def TranslatedEngine(self):
        return TranslatedShape(self.rotatedEngine2,displacement=Vector(-self.AttachPoint[0]+self.EngPosition[0],-self.AttachPoint[1]+self.EngPosition[1],-self.AttachPoint[2]+self.EngPosition[2]))

    @Attribute
    def bottomZlocFromWingLE(self):
        return 2*(self.NacelleRadius+self.NacelleThickness)+self.VerEngineRatio*self.Chord


if __name__ == '__main__':
    from parapy.gui import display

    obj = Engine()
    display(obj)

