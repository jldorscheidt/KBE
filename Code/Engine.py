from __future__ import division

from math import *
from parapy.geom import *
from parapy.core import *

class Engine(GeomBase):
    #: Total thrust per engine
    Thrust = Input(10000.) ## thrust in lbs
    Location = Input(Point(0,0,0))
    Config = Input('4') ## engine configuration
    Position = Input(Point(0,0,0)) ##engine location
    NacelleThickness = Input(0.2)
    NacelleLength = Input(1)



    @Input
    #: average radius of inner engine in meters
    def engineAvradius(self):
        engineAvradius=0.0254*(1.0827*self.Thrust**0.4134)
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

    @Attribute
    #: Nacelle extension in front of engine
    def NacelleExtension(self):
        nacelleExtension=nacelleRad*0.6
        return nacelleExtension

    @Part
    #: Inner engine
    def InnerEngine(self):
        return CCylinder(radius=self.engineAvradius,height=self.engineLength,position=self.Position)

    @Attribute(in_tree=True)
    #: Nacelle
    def Nacelleshape(self):
        crv1 = FittedCurve(points=[Point(self.NacelleRadius,0,0),Point(self.NacelleRadius,self.NacelleLength,0),
                                   Point(self.NacelleRadius+self.NacelleThickness,self.NacelleLength,0),
                                   Point(self.NacelleRadius+self.NacelleThickness,0,0),
                                   Point(self.NacelleRadius,0,0)],max_degree=1)
        crv2 = RotatedCurve(crv1,Point(self.NacelleRadius,0,0),Vector(1,0,0),-0.5*pi)
        return crv2

    @Part
    def Nacelle(self):
        return Revolution(self.Nacelleshape,2*pi)




if __name__ == '__main__':
    from parapy.gui import display

    obj = Engine()
    display(obj)

