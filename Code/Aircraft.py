from __future__ import division

from math import *
from parapy.geom import *
from parapy.core import *
from LandingGear import *
from MainWing import *
from Fuselage import *
from Tail import *

class Aircraft(GeomBase):

    ## Input for Main wing
    M_cruise=Input(0.78) #Mach number
    M_techfactor=Input(0.935) #Technology factor for supercritical airfoils. Equal to 0.87 for NACA 6 airfoil and 1 for other conventional airfoils
    wing_configuration=Input("low") #Choose between "high" and "low"

    #General inputs for Wing:
    wing_x_pos=Input(-20.)
    wing_z_pos=Input(1.)
    wing_area=Input(122.4) #in m^2
    aspect_ratio=Input(9.39)
    twist=Input(10.) #in degrees

    ##Input for Horizontal tail
    h_wing_area=Input(31.) #in m^2
    h_aspect_ratio=Input(5.)
    h_taper_ratio=Input(0.256)
    h_sweep_qc=Input(29.) #in degrees
    h_dihedral=Input(5.) #in degrees
    h_twist=Input(0.) #in degrees. Leave it zero in order to have an exact MAC chord length determination.
    h_wing_x_pos=Input(-60.) #wrt to MAC quarter chord position
    h_wing_z_pos=Input(-1.)
    h_wing_thickness_factor_0=Input(1.)

    @Input#for horizontal tail. Factor=(0.99*thickness of main wing)/thickness of hor tail
    def h_wing_thickness_factor(self):
         tail=Tail(pass_down="h_wing_area,h_aspect_ratio,h_taper_ratio,h_sweep_qc,h_dihedral,h_twist"
                              "h_wing_x_pos,h_wing_z_pos,h_wing_thickness_factor_0,h_airfoil_input_root,h_airfoil_input_tip"
                              "v_wing_area,v_aspect_ratio,v_taper_ratio,v_sweep_qc,v_dihedral,v_twist"
                              "v_wing_x_pos,v_wing_z_pos,v_airfoil_input_root,v_airfoil_input_tip")
         mainwing=MainWing(pass_down="M_cruise,M_techfactor,wing_configuration,wing_x_pos,wing_z_pos,wing_area,aspect_ratio,twist"
                              "airfoil_input_root,airfoil_input_tip")
         return (0.99*mainwing.mainwing_right.airfoil_thickness)/(tail.horwing_right.airfoil_thickness)

    #Airfoil options:
    #ClarkX, GOE257, M6, NACA0010, NACA2412, NACA4412, NACA23012, NACA64210, RAF28, TSAGI12
    h_airfoil_input_root=Input("NACA0010") #withouth .dat
    h_airfoil_input_tip=Input("NACA0010") #withouth .dat

    ##Input for Vertical tail
    v_wing_area=Input(43.) #in m^2. (Twice the area)
    v_aspect_ratio=Input(5.)
    v_taper_ratio=Input(0.303)
    v_sweep_qc=Input(34.) #in degrees
    v_dihedral=Input(0.) #in degrees
    v_twist=Input(0.) #in degrees. Leave it zero in order to have an exact MAC chord length determination.
    v_wing_x_pos=Input(-60.) #wrt to MAC quarter chord position
    v_wing_z_pos=Input(-1.)

    #Airfoil options:
    #ClarkX, GOE257, M6, NACA0010, NACA2412, NACA4412, NACA23012, NACA64210, RAF28, TSAGI12
    v_airfoil_input_root=Input("NACA0010") #withouth .dat
    v_airfoil_input_tip=Input("NACA0010") #withouth .dat


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

    @Part
    def tail(self):
        return Tail(pass_down="h_wing_area,h_aspect_ratio,h_taper_ratio,h_sweep_qc,h_dihedral,h_twist"
                              "h_wing_x_pos,h_wing_z_pos,h_wing_thickness_factor,h_airfoil_input_root,h_airfoil_input_tip"
                              "v_wing_area,v_aspect_ratio,v_taper_ratio,v_sweep_qc,v_dihedral,v_twist"
                              "v_wing_x_pos,v_wing_z_pos,v_airfoil_input_root,v_airfoil_input_tip")

if __name__ == '__main__':
    from parapy.gui import display

    obj = Aircraft()
    display(obj)