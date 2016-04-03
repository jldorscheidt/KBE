from __future__ import division
from parapy.core import *
from parapy.geom import *
from math import *

from Wing import Wing

class MainWing(GeomBase):
    #Specific inputs to MainWing:
    M_cruise=Input(0.78) #Mach number
    M_techfactor=Input(0.935) #Technology factor for supercritical airfoils. Equal to 0.87 for NACA 6 airfoil and 1 for other conventional airfoils
    wing_configuration=Input("low") #Choose between "high" and "low"

    #General inputs for Wing:
    wing_area=Input(122.4) #in m^2
    aspect_ratio=Input(9.39)
    twist=Input(10.) #in degrees

    #Airfoil options:
    #ClarkX, GOE257, M6, NACA0010, NACA2412, NACA4412, NACA23012, NACA64210, RAF28, TSAGI12
    airfoil_input_root=Input("RAF28") #withouth .dat
    airfoil_input_tip=Input("NACA4412") #withouth .dat

    @Input
    #Dihedral angle (in degrees)
    def dihedral(self):
        if self.wing_configuration=="low":
            angle=3.+2.-self.sweep_qc/10.
        if self.wing_configuration=="high":
            angle=3.-2.-self.sweep_qc/10.
        return angle

    @Input
    #Drag-divergence Mach number
    def M_dd(self):
        return self.M_cruise+0.03

    @Input
    #Quarter-chord sweep angle (in degrees)
    def sweep_qc(self):
        return degrees(acos(0.75*(self.M_techfactor/self.M_dd)))

    @Input
    #Taper ratio from Torenbeek
    def taper_ratio(self):
        return 0.2*(2-self.sweep_qc*(pi/180))

    #Produce the right wing
    @Part
    def mainwing_right(self):
        return Wing(pass_down="wing_area,aspect_ratio,taper_ratio,sweep_qc,dihedral,twist,"
                              "airfoil_input_root,airfoil_input_tip")

    #Mirror to get the left wing
    @Part
    def mainwing_left(self):
        return MirroredShape(self.mainwing_right.solid,reference_point=Point(0, 0, 0), vector1=Vector(1, 0, 0), vector2=Vector(0, 0, 1))

    #Glue left and right together to get the entire wing
    @Part
    def mainwing(self):
        return FusedSolid(shape_in=self.mainwing_right.solid,tool=self.mainwing_left)

if __name__ == '__main__':
    from parapy.gui import display
    obj = MainWing()
    display(obj)