from __future__ import division

from math import *
from parapy.geom import *
from parapy.core import *


class Fuselage(GeomBase):

    #: fuselage length, default length taken as the length of a airbus a320-200
    #: :type: float
    fu_length = Input(37.57)

    #: tail cone upsweep angle (degrees)
    #: :type: float
    fu_tail_upsweep = Input(7)

    #: position of main landing gear
    #: :type: float
    rot_point = Input(Point(-30,0,10))

    #: fuselage slenderness, default value taken as the fineness ratio of a airbus a320-200
    #: type: float
    #: length/diameter
    fu_slender = Input(9.51)

    #: fuselage tail slenderness
    #: type: float
    #: length/diameter of tail cone section, note cannot be longer then fu_slender
    fu_tail_slender = Input(1.8)

    #: fuselage nose slenderness
    #: type: float
    #: length/diameter of nose cone section, note cannot be more then fu_slender-fu_tail_slender. Mcruise of 0.8 implies
    #: slenderness of 1.25
    fu_nose_slender = Input(1.25)

    #: nose section radius distribution
    #: :type: collections.Sequence[float]
    fu_nose_radius = Input([10,90,100])

    #: tail section radius distribution
    #: :type: collections.Sequence[float]
    fu_tail_radius = Input([100,90,80,60,40,10])

    @Input
    def fu_radius(self):
        "section radius of cylindrical part of fuselage"

        return self.fu_length/(2*self.fu_slender)

    @Input
    def front_section_totlength(self):
        "total length of nose section"
        return self.fu_nose_slender * (2*self.fu_radius)

    @Input
    def tail_section_totlength(self):
        "total length of rear section"
        return self.fu_tail_slender * (2*self.fu_radius)

    @Input
    def mid_section_totlength(self):
        "total length of mid section"
        return self.fu_length - self.front_section_totlength - self.tail_section_totlength

    @Input
    def front_section_radius(self):
        """front section radius multiplied by the radius distribution
        through the length. Note that the numbers are percentages.

        :rtype: collections.Sequence[float]
        """
        return [i * self.fu_radius / 100 for i in self.fu_nose_radius]

    @Input
    def tail_section_radius(self):
        """tail section radius multiplied by the radius distribution
        through the length. Note that the numbers are percentages.

        :rtype: collections.Sequence[float]
        """
        return [i * self.fu_radius / 100 for i in self.fu_tail_radius]


    @Input
    def front_section_length(self):
        """front section length

        :rtype: float
        """

        return self.front_section_totlength / (len(self.front_section_radius)-1)

    @Input
    def tail_section_length(self):
        """tail section length

        :rtype: float"""

        return self.tail_section_totlength / (len(self.tail_section_radius)-1)

    @Attribute
    def rot_point2(self):
        return self.rot_point

    @Part
    def section_curves_front(self):
        "front section curves"
        return Circle(quantify=len(self.front_section_radius),
                      radius=self.front_section_radius[child.index],
                      position=self.position.translate('z',
                                                       child.index * self.front_section_length), hidden=True)

    @Part
    def section_curves_mid(self):
        "mid section curves"
        return Circle(quantify=2,radius=self.fu_radius,position=self.position.translate('z',self.front_section_totlength +
                                                                                        child.index*self.mid_section_totlength),hidden=True)


    @Part
    def section_curves_tail(self):
        "tail section curves"
        return Circle(quantify=len(self.tail_section_radius),
                      radius=self.tail_section_radius[child.index],
                      position=self.position.translate('z',
                                                       self.front_section_totlength + self.mid_section_totlength +
                                                       child.index * self.tail_section_length,'y' , child.index *
                                                       self.tail_section_length * radians(self.fu_tail_upsweep)),
                                                       hidden=True)


    @Part
    def frontsolid(self):
        "front section solid"
        return LoftedSolid(profiles=self.section_curves_front,hidden=True)

    @Part
    def midsolid(self):
        "mid section solid"
        return LoftedSolid(profiles=self.section_curves_mid,hidden=True)

    @Part
    def tailsolid(self):
        "tail section solid"
        return LoftedSolid(profiles=self.section_curves_tail,hidden=True)

    @Part
    def totsolid(self):
        "total solid"
        return FusedSolid(shape_in=self.midsolid, tool=(self.tailsolid + self.frontsolid),hidden=True)

    @Part
    def rottotsolid(self):
        "solid fuselage rotated in correct axis system"
        return RotatedShape(shape_in=RotatedShape(shape_in=self.totsolid, rotation_point=OXY, vector=Vector(0,1,0), angle=-0.5*pi), rotation_point=OXY, vector=Vector(1,0,0),angle=-0.5*pi)

    @Part
    def rottailsolid(self):
        "solid tail section rotated in correct axis system"
        return RotatedShape(shape_in=RotatedShape(shape_in=self.tailsolid, rotation_point=OXY, vector=Vector(0,1,0), angle=-0.5*pi), rotation_point=OXY, vector=Vector(1,0,0),angle=-0.5*pi,hidden=True)

    @Part
    def plane(self):
        "plane used for intersection"
        return Plane(Point(0,0,0),Vector(0,1,0),hidden=True)

    @Part
    def intersect(self):
        "intersected edges of tail section with xz plane"
        return IntersectedShapes(shape_in=self.rottailsolid, tool=self.plane,hidden=True)

    @Attribute
    def line(self):
        "bottom line of intersected egde"
        return self.intersect.edges[1].equispaced_points(10)

    @Attribute
    def maxrotangle(self):
        "maximum rotation angle"
        rotangles=[degrees(atan2(self.rot_point2.z-i.z,self.rot_point2.x-i.x)) for i in self.line]
        return min(rotangles)


if __name__ == '__main__':
    from parapy.gui import display

    obj = Fuselage()
    display(obj)
