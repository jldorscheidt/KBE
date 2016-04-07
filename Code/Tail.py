from __future__ import division
from parapy.core import *
from parapy.geom import *
from math import *

from Wing import Wing

class Tail(GeomBase):
    ##Horizontal tail
    h_wing_area=Input(31.) #in m^2
    h_aspect_ratio=Input(5.)
    h_taper_ratio=Input(0.256)
    h_sweep_qc=Input(29.) #in degrees
    h_dihedral=Input(5.) #in degrees
    h_twist=Input(0.) #in degrees. Leave it zero in order to have an exact MAC chord length determination.
    h_wing_x_pos=Input(-30.) #wrt to MAC quarter chord position
    h_wing_z_pos=Input(-5.)
    h_wing_thickness_factor=Input(1.) #for horizontal tail. Factor=(0.99*thickness of main wing)/thickness of hor tail
    h_wing_pos_factor=Input(0.5) #0 for conventional tail, 1 for T-tail and anything in between for cruciform

    #Airfoil options:
    #ClarkX, GOE257, M6, NACA0010, NACA2412, NACA4412, NACA23012, NACA64210, RAF28, TSAGI12
    h_airfoil_input_root=Input("NACA0010") #withouth .dat
    h_airfoil_input_tip=Input("NACA0010") #withouth .dat

    ##Vertical tail
    v_wing_area=Input(43.) #in m^2. (Twice the area)
    v_aspect_ratio=Input(5.)
    v_taper_ratio=Input(0.303)
    v_sweep_qc=Input(34.) #in degrees
    v_dihedral=Input(0.) #in degrees
    v_twist=Input(0.) #in degrees. Leave it zero in order to have an exact MAC chord length determination.
    v_wing_x_pos=Input(-30.) #wrt to MAC quarter chord position
    v_wing_z_pos=Input(-5.)


    #Airfoil options:
    #ClarkX, GOE257, M6, NACA0010, NACA2412, NACA4412, NACA23012, NACA64210, RAF28, TSAGI12
    v_airfoil_input_root=Input("NACA0010") #withouth .dat
    v_airfoil_input_tip=Input("NACA0010") #withouth .dat

    #MAC chord length for both horizontal and vertical tail
    @Attribute
    def h_MAC_length(self):
        return self.horwing_right.MAC_length

    @Attribute
    def v_MAC_length(self):
        return self.verwing_zero.MAC_length

    #Produce the right horizontal wing
    @Part
    def horwing_right(self):
        return Wing(wing_area=self.h_wing_area,aspect_ratio=self.h_aspect_ratio,taper_ratio=self.h_taper_ratio,
                        sweep_qc=self.h_sweep_qc,dihedral=self.h_dihedral,twist=self.h_twist,
                        airfoil_input_root=self.h_airfoil_input_root,airfoil_input_tip=self.h_airfoil_input_tip,
                        wing_x_pos=self.h_wing_x_pos,wing_z_pos=self.h_wing_z_pos,wing_thickness_factor=self.h_wing_thickness_factor,hidden=True)

    #Make one compound of the MAC airfoil and the point on the horizontal wing
    @Part
    def horwing_right_MAC_tot_zero(self):
        return Compound([self.horwing_right.MAC_airfoil,self.horwing_right.MAC_qc_point_circle_onairfoil],color="red",hidden=True)

    #Mirror to get the left horizontal wing
    @Part
    def horwing_left(self):
        return MirroredShape(self.horwing_right.solid,reference_point=Point(0, 0, 0), vector1=Vector(1, 0, 0), vector2=Vector(0, 0, 1),hidden=True)

    #Combine the left and right horizontal solids
    @Part
    def horwing_totsolid_zero(self):
        return Compound([self.horwing_right.solid,self.horwing_left],hidden=True)

    #Move the horizontal wing to the right position along the vertical tail
    @Part
    def horwing_totsolid(self):
        return TranslatedShape(self.horwing_totsolid_zero,
                               Vector(-self.verwing_zero.span*0.5*self.h_wing_pos_factor*tan(radians(self.verwing_zero.sweep_le)),
                                      0, -self.verwing_zero.span*0.5*self.h_wing_pos_factor),color="green",hidden=True)

    #Move the MAC on the horizontal wing to the right position along the vertical tail
    @Part
    def horwing_right_MAC_tot(self):
        return TranslatedShape(self.horwing_right_MAC_tot_zero,
                                   Vector(-self.verwing_zero.span * 0.5 * self.h_wing_pos_factor * tan(radians(self.verwing_zero.sweep_le)),
                                          0, -self.verwing_zero.span * 0.5 * self.h_wing_pos_factor), color="red")

    #Produce the vertical wing
    @Part
    def verwing_zero(self):
        return Wing(wing_area=self.v_wing_area,aspect_ratio=self.v_aspect_ratio,taper_ratio=self.v_taper_ratio,
                        sweep_qc=self.v_sweep_qc,dihedral=self.v_dihedral,twist=self.v_twist,
                        airfoil_input_root=self.v_airfoil_input_root,airfoil_input_tip=self.v_airfoil_input_tip,
                        wing_x_pos=self.v_wing_x_pos,wing_z_pos=self.v_wing_z_pos,hidden=True)


    # Make one compound of the MAC airfoil and the point on the vertical wing
    @Part
    def verwing_zero_MAC_tot(self):
        return Compound([self.verwing_zero.MAC_airfoil, self.verwing_zero.MAC_qc_point_circle_onairfoil], color="red",hidden=True)

    #Rotate the vertical wing
    @Part
    def verwing(self):
        return RotatedShape(self.verwing_zero.solid,rotation_point=self.verwing_zero.solid.location, vector=Vector(1, 0, 0),angle=radians(-90.),hidden=True)

    #Rotate the MAC airfoil and quarter chord point as well
    @Part
    def verwing_MAC_tot(self):
        return RotatedShape(self.verwing_zero_MAC_tot,rotation_point=self.verwing_zero.solid.location, vector=Vector(1, 0, 0),angle=radians(-90.),color="red")

    #Rotate MAC quarter chord point in the trapezoidial trapezoidal view as well
    @Part
    def verwing_MAC_qc_point(self):
        return RotatedShape(self.verwing_zero.MAC_qc_point,rotation_point=self.verwing_zero.solid.location, vector=Vector(1, 0, 0),angle=radians(-90.),color="red",hidden=True)

    #Combine the three solids into one
    @Part
    def tail_totsolid(self):
        return Compound([self.horwing_totsolid,self.verwing])

if __name__ == '__main__':
    from parapy.gui import display
    obj = Tail()
    display(obj)