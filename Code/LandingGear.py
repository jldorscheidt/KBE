from __future__ import division

from math import *
from parapy.geom import *
from parapy.core import *

class LandingGear(GeomBase):
    #: diameter of main landing gear wheel in meters
    gearradius = Input(2)

    #: location of center of main landing gear wheel from nose of airplane
    gearlocation = Input(Point(-10,0,5))

    #: rotation angle at liftoff, input in degrees
    rotangle = Input(25)

    @Part
    def gearwheel(self):
        return RotatedCurve(curve_in=Circle(radius=self.gearradius,position=self.gearlocation),rotation_point=self.gearlocation,vector=Vector(1,0,0),angle=0.5*pi)

    @Attribute(in_tree=True)
    def rotpoint(self):
        line=RotatedCurve(LineSegment(self.gearlocation,Point(self.gearlocation.x,self.gearlocation.y,self.gearlocation.z+3)),self.gearlocation,Vector(0,1,0),radians(-self.rotangle))
        point=line.intersection_point(self.gearwheel)
        return point

if __name__ == '__main__':
    from parapy.gui import display

    obj = LandingGear()
    display(obj)