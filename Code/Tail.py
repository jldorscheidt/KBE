from __future__ import division
from parapy.core import *
from parapy.geom import *
from math import *

from Wing import Wing

class Tail(GeomBase):
    ##Horizontal tail
    h_wing_pos_factor = Input(0.5)  # 0 for conventional tail, 1 for T-tail and anything in between for cruciform
    h_wing_area=Input(31.) #in m^2
    h_aspect_ratio=Input(5.)
    h_taper_ratio=Input(0.256)
    h_sweep_qc=Input(29.) #in degrees
    h_dihedral=Input(5.) #in degrees
    h_twist=Input(0.) #in degrees. Leave it zero in order to have an exact MAC chord length determination.
    h_wing_thickness_factor=Input(1.) #for horizontal tail. Factor=(0.99*thickness of main wing)/thickness of hor tail
    h_wing_x_pos = Input(-30.)  # wrt to MAC quarter chord position
    h_wing_z_pos = Input(-5.)

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

    #Rudder
    r_factor=Input(0.2) #As percentage of the vertical tail chord
    r_aoa=Input(15.) #Angle of attack for check of blanketed rudder
    spanwise_loc_ratio = Input()

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
                        wing_x_pos=self.h_wing_x_pos,wing_z_pos=self.h_wing_z_pos,wing_thickness_factor=self.h_wing_thickness_factor,
                        spanwise_loc_ratio=self.spanwise_loc_ratio,hidden=True)

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
                               Vector(-self.verwing_zero.span*0.5*self.h_wing_pos_factor*tan(radians(self.verwing_zero.sweep_le))+(self.horwing_right.MAC_qc_point_zero.x-self.verwing_zero.MAC_qc_point_zero.x),
                                      0, -self.verwing_zero.span*0.5*self.h_wing_pos_factor),color="green",hidden=True)

    #Move the MAC on the horizontal wing to the right position along the vertical tail
    @Part
    def horwing_right_MAC_tot(self):
        return TranslatedShape(self.horwing_right_MAC_tot_zero,
                                   Vector(-self.verwing_zero.span * 0.5 * self.h_wing_pos_factor * tan(radians(self.verwing_zero.sweep_le))+(self.horwing_right.MAC_qc_point_zero.x-self.verwing_zero.MAC_qc_point_zero.x),
                                          0, -self.verwing_zero.span * 0.5 * self.h_wing_pos_factor), color="red")

    #Produce the vertical wing
    @Part
    def verwing_zero(self):
        return Wing(wing_area=self.v_wing_area,aspect_ratio=self.v_aspect_ratio,taper_ratio=self.v_taper_ratio,
                        sweep_qc=self.v_sweep_qc,dihedral=self.v_dihedral,twist=self.v_twist,
                        airfoil_input_root=self.v_airfoil_input_root,airfoil_input_tip=self.v_airfoil_input_tip,
                        wing_x_pos=self.v_wing_x_pos,wing_z_pos=self.v_wing_z_pos,spanwise_loc_ratio=self.spanwise_loc_ratio,
                        hidden=True)


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

    # Draw trailing edge line of the vertical wing using a point at the root and tip of the vertical tail
    @Attribute
    def v_root_TE_point(self):
        return Point(self.verwing_zero.solid.location.x-self.verwing_zero.c_root,0,self.verwing_zero.solid.location.z)

    @Attribute
    def v_tip_TE_point(self):
        return Point(self.verwing_zero.solid.location.x - self.verwing_zero.c_tip - self.verwing_zero.span * 0.5 * tan(radians(self.verwing_zero.sweep_le)), 0,
                     self.verwing_zero.solid.location.z - self.verwing_zero.span * 0.5)

    @Part
    def v_TE_line(self):
        return LineSegment(self.v_root_TE_point, self.v_tip_TE_point,hidden=True)

    #Point at the root of vertical wing for the rudder
    @Attribute
    def r_root_point(self):
        return Point(self.verwing_zero.solid.location.x-(1-self.r_factor)*self.verwing_zero.c_root,0,self.verwing_zero.solid.location.z)

    # Point at the tip of vertical wing for the rudder
    @Attribute
    def r_tip_point(self):
        return Point(self.verwing_zero.solid.location.x-(1-self.r_factor)*self.verwing_zero.c_tip-self.verwing_zero.span * 0.5 * tan(radians(self.verwing_zero.sweep_le)),0,
                     self.verwing_zero.solid.location.z-self.verwing_zero.span * 0.5)

    #Drawing hinge line
    @Part
    def r_hinge_line(self):
        return LineSegment(self.r_root_point,self.r_tip_point,hidden=True)

    #Project the line on the vertical tail
    @Part
    def r_hinge_line_on_tail(self):
        return ProjectedCurve(source=self.r_hinge_line, target=self.verwing, direction=Vector(0, 1, 0), color="blue",line_thickness=2, hidden=False)

    # To calculate the total rudder area we need a rudder surface. Therefore we need two more lines: one at the tip and one at the root of the rudder
    @Part
    def r_root_line(self):
        return LineSegment(self.v_root_TE_point, self.r_root_point,hidden=True)

    @Part
    def r_tip_line(self):
        return LineSegment(self.v_tip_TE_point, self.r_tip_point,hidden=True)

    # Draw rudder area
    @Part
    def r_surface(self):
        return FilledSurface(curves=[self.r_root_line, self.r_tip_line, self.r_hinge_line, self.v_TE_line])

    #Create point at root leading edge of horizontal tail to draw the wake line
    @Attribute
    def h_root_LE_point(self):
        return Point(self.horwing_totsolid.location.x+self.h_wing_x_pos-self.horwing_right.MAC_qc_point_zero.x,self.horwing_totsolid.location.y,self.horwing_totsolid.location.z+self.h_wing_z_pos)

    #Draw the wake line
    @Part
    def r_wake_line(self):
        return Line(self.h_root_LE_point, Vector(-1, 0, -tan(radians(self.r_aoa))),hidden=True)

    #Create a point at the root trailing edge of the horizontal tail (needed for the wake area later on)
    @Attribute
    def h_root_TE_point(self):
        return Point(self.horwing_totsolid.location.x + self.h_wing_x_pos - self.horwing_right.MAC_qc_point_zero.x-self.horwing_right.c_root,
                     self.horwing_totsolid.location.y, self.horwing_totsolid.location.z + self.h_wing_z_pos)

    #Create a line segment from leading to trailing edge of the horizontal tail (needed for wake area later on)
    @Part
    def h_root_line(self):
        return Line(self.h_root_LE_point,Vector(-1,0,0),hidden=True)

    #Create point at the crossing of the hinge line and the wake line to draw wake surface
    @Attribute(in_tree=False)
    def r_wake_plane_pt1(self):
        crv1=self.r_hinge_line
        crv2=self.r_wake_line
        return crv1.intersection_point(crv2)

    # Create point at the crossing of the trailing edge line and the wake line to draw wake surface
    @Attribute(in_tree=False)
    def r_wake_plane_pt2(self):
        crv1=self.v_TE_line
        crv2=self.r_wake_line
        return crv1.intersection_point(crv2)

    #Create point at the trailing egde of the vertical wing at the height of the horizontal tail
    @Attribute(in_tree=False)
    def r_wake_plane_pt3(self):
        crv1=self.v_TE_line
        crv2=self.h_root_line
        return crv1.intersection_point(crv2)

    #Create point at the hinge line at the height of the horizontal tail
    @Attribute(in_tree=False)
    def r_wake_plane_pt4(self):
        crv1=self.r_hinge_line
        crv2=self.h_root_line
        return crv1.intersection_point(crv2)

    # Create wake surface of the horizontal tail between the hinge line and the trailing edge of the vertical wing
    @Part
    def r_wake_plane(self):
        return FilledSurface(curves=[LineSegment(self.r_wake_plane_pt1,self.r_wake_plane_pt2),
                                     LineSegment(self.r_wake_plane_pt2,self.r_wake_plane_pt3),
                                     LineSegment(self.r_wake_plane_pt3,self.r_wake_plane_pt4),
                                     LineSegment(self.r_wake_plane_pt4,self.r_wake_plane_pt1)],color="blue")

    #Project this plane on the vertical wing. We have to project the four line segments first and then make a new surface of the correct edges
    @Part
    def r_wake_plane_on_tail_crv1(self):
        return ProjectedCurve(source=LineSegment(self.r_wake_plane_pt1,self.r_wake_plane_pt2), target=self.verwing, direction=Vector(0, 1, 0),hidden=True)

    @Part
    def r_wake_plane_on_tail_crv2(self):
        return ProjectedCurve(source=LineSegment(self.r_wake_plane_pt2,self.r_wake_plane_pt3), target=self.verwing,direction=Vector(0, 1, 0),hidden=True)

    @Part
    def r_wake_plane_on_tail_crv3(self):
        return ProjectedCurve(source=LineSegment(self.r_wake_plane_pt3,self.r_wake_plane_pt4), target=self.verwing,direction=Vector(0, 1, 0),hidden=True)

    @Part
    def r_wake_plane_on_tail_crv4(self):
        return ProjectedCurve(source=LineSegment(self.r_wake_plane_pt4,self.r_wake_plane_pt1), target=self.verwing,direction=Vector(0, 1, 0),hidden=True)

    @Attribute(in_tree=True)
    def r_wake_plane_on_tail(self):
        if self.h_wing_pos_factor==0:
            i3=1
            i4=2
        else:
            i4=0
            i3=0
        return FilledSurface(curves=[self.r_wake_plane_on_tail_crv1.edges[0],self.r_wake_plane_on_tail_crv2.edges[0],self.r_wake_plane_on_tail_crv3.edges[i3],self.r_wake_plane_on_tail_crv4.edges[i4]],color='blue')

    #Percentage of rudder blanketed by horizontal tail
    @Attribute
    def r_unblanketed(self):
        ruddertot=self.r_surface.area
        if self.r_wake_plane.area>0:
            ruddersmall=self.r_wake_plane.area
        else:
            ruddersmall=0.
        return ((ruddertot-ruddersmall)/ruddertot)*100

    #Combine the three tail solids into one
    @Part
    def tail_totsolid(self):
        return Compound([self.horwing_totsolid,self.verwing])

if __name__ == '__main__':
    from parapy.gui import display
    obj = Tail()
    display(obj)