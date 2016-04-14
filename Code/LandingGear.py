from __future__ import division

from math import *
from parapy.geom import *
from parapy.core import *

class LandingGear(GeomBase):
    #: radius of main landing gear wheel in meters
    gearradius = Input(2)

    #: x location of center of main landing gear wheel from nose of airplane
    xlocgear = Input(-10)

    #: z location of center of main landing gear wheel from ncenterline of fuselage
    zlocgear = Input(3)

    #: z location of bottom of engine with respect to chord of main wing, used for checking if the landing gear is positioned low enough
    zlocengine = Input(0)

    #: z location of root chord of main wing, used for checking if the landing gear is positioned low enough
    zlocwing = Input(0)

    #: z location of wing tip, used for checking if the landing gear is positioned low enough
    zlocwingtip = Input(0)

    #: rotation angle at liftoff, input in degrees
    rot_angle = Input(25)

    @Attribute
    #: check if the gear is positioned low enough
    def ZlocCheck(self):
        if self.zlocengine+self.zlocwing >= self.zlocgear+self.gearradius or self.zlocwingtip >= self.zlocgear+self.gearradius:
            print("WARNING; gear positioned to high, engines  or wing tips touch the ground")
            flag=1
        else:
            flag=0
        return flag

    @Part
    #: location of main gear
    def gearlocation(self):
        return Point(self.xlocgear,0,self.zlocgear)

    @Part
    def gearwheel(self):
        return RotatedCurve(curve_in=Circle(radius=self.gearradius,position=self.gearlocation),rotation_point=self.gearlocation,vector=Vector(1,0,0),angle=0.5*pi)

    @Attribute(in_tree=True)
    #: point of main gear which touches the ground at liftoff. used for the rotation angle check.
    def rotpoint(self):
        line=RotatedCurve(LineSegment(self.gearlocation,Point(self.gearlocation.x,self.gearlocation.y,self.gearlocation.z+3)),self.gearlocation,Vector(0,1,0),radians(-self.rot_angle))
        point=line.intersection_point(self.gearwheel)
        return point

if __name__ == '__main__':
    from parapy.gui import display

    obj = LandingGear()
    display(obj)