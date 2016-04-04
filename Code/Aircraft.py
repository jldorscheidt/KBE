from __future__ import division

from math import *
from parapy.geom import *
from parapy.core import *
from LandingGear import *
from MainWing import *
from Fuselage import *

class Aircraft(GeomBase):

    ## Input for Main wing
    M_cruise=Input(0.78) #Mach number
    M_techfactor=Input(0.935) #Technology factor for supercritical airfoils. Equal to 0.87 for NACA 6 airfoil and 1 for other conventional airfoils
    wing_configuration=Input("low") #Choose between "high" and "low"

    #General inputs for Wing:
    wing_x_pos=Input(-5)
    wing_z_pos=Input(1)
    wing_area=Input(122.4) #in m^2
    aspect_ratio=Input(9.39)
    twist=Input(10.) #in degrees

    #Airfoil options:
    #ClarkX, GOE257, M6, NACA0010, NACA2412, NACA4412, NACA23012, NACA64210, RAF28, TSAGI12
    airfoil_input_root=Input("RAF28") #withouth .dat
    airfoil_input_tip=Input("NACA4412") #withouth .dat

    ## Input for landinggear
    radius = Input(1) #in meters
    gearlocation = Input(Point(-10,0,3)) #in meters
    rotangle = Input(24) # degrees

    ## Input for Fuselage
    fu_length = Input(37.57) #meters
    fu_tail_upsweep = Input(7) #degrees
    fu_slender = Input(9.51)
    fu_tail_slender = Input(1.8)
    fu_nose_slender = Input(1.25)
    fu_nose_radius = Input([10,90,100]) #percentage
    fu_tail_radius = Input([100,90,80,60,40,10])  #percentage

    @Part
    def mainwing(self):
        return MainWing(pass_down="M_cruise,M_techfactor,wing_configuration,wing_x_pos,wing_z_pos,wing_area,aspect_ratio,twist"
                              "airfoil_input_root,airfoil_input_tip")

    @Part
    def maingear(self):
        return LandingGear(pass_down="radius,gearlocation,rotangle")


    @Part
    def fuselage(self):
        return Fuselage(pass_down="fu_length,fu_tail_upsweep,fu_slender,fu_tail_slender,fu_nose_slender,fu_nose_radius"
                                  "fu_tail_radius", rot_point=self.maingear.rotpoint)
if __name__ == '__main__':
    from parapy.gui import display

    obj = Aircraft()
    display(obj)