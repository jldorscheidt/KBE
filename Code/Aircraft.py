from __future__ import division

from math import *
from parapy.geom import *
from parapy.core import *
from LandingGear import *
from MainWing import *
from Fuselage import *
from Tail import *
from parapy.lib.xfoil import *
import matplotlib.pyplot as plt

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
    twist=Input(0.) #in degrees

    #Rudder
    r_factor=Input(0.2) #As percentage of the vertical tail chord
    r_aoa=Input(15.) #Angle of attack for check of blanketed rudder

    ##Input for Horizontal tail
    h_wing_pos_factor = Input(0.5)  # 0 for conventional tail, 1 for T-tail and anything in between for cruciform
    h_tail_volume=Input(1.)
    h_aspect_ratio=Input(5.)
    h_taper_ratio=Input(0.4)
    h_dihedral=Input(5.) #in degrees
    h_twist=Input(0.) #in degrees. Leave it zero in order to have an exact MAC chord length determination.
    h_wing_x_pos=Input(-35.) #wrt to MAC quarter chord position
    h_wing_z_pos=Input(-1.)

    @Input
    def h_sweep_qc(self):#in degrees
        return self.mainwing.sweep_qc*1.1

    @Input#for horizontal tail. Factor=(0.99*thickness of main wing)/thickness of hor tail
    def h_wing_thickness_factor(self):
        tail=Tail(h_airfoil_input_root=self.h_airfoil_input_root,h_airfoil_input_tip=self.h_airfoil_input_tip,
                  v_airfoil_input_root=self.v_airfoil_input_root,v_airfoil_input_tip=self.v_airfoil_input_tip)

        mainwing=MainWing(airfoil_input_root=self.airfoil_input_root,airfoil_input_tip=self.airfoil_input_tip)
        return (0.99*mainwing.mainwing_right.airfoil_thickness)/(tail.horwing_right.airfoil_thickness)

    @Input
    def h_wing_area(self):
        Sh=1.
        Sh_increment=1.
        tail = Tail(h_wing_area=Sh, h_aspect_ratio=self.h_aspect_ratio, h_taper_ratio=self.h_taper_ratio,
                    h_sweep_qc=self.h_sweep_qc, h_dihedral=self.h_dihedral, h_twist=self.h_twist,
                    h_wing_x_pos=self.h_wing_x_pos, h_wing_z_pos=self.h_wing_z_pos,
                    h_wing_thickness_factor=self.h_wing_thickness_factor,
                    h_airfoil_input_root=self.h_airfoil_input_root, h_airfoil_input_tip=self.h_airfoil_input_tip,
                    h_wing_pos_factor=self.h_wing_pos_factor)
        S = self.mainwing.wing_area
        c_avg = self.mainwing.mainwing_right.c_avg
        Vh = self.h_tail_volume
        lh = -(tail.horwing_right.MAC_qc_point.x - self.mainwing.mainwing_right.MAC_qc_point.x)
        Sh_new = (Vh * S * c_avg) / lh
        while Sh_new>Sh:
            S=self.mainwing.wing_area
            c_avg=self.mainwing.mainwing_right.c_avg
            Vh=self.h_tail_volume
            tail = Tail(h_wing_area=Sh, h_aspect_ratio=self.h_aspect_ratio,
                        h_taper_ratio=self.h_taper_ratio,
                        h_sweep_qc=self.h_sweep_qc, h_dihedral=self.h_dihedral, h_twist=self.h_twist,
                        h_wing_x_pos=self.h_wing_x_pos, h_wing_z_pos=self.h_wing_z_pos,
                        h_wing_thickness_factor=self.h_wing_thickness_factor,
                        h_airfoil_input_root=self.h_airfoil_input_root, h_airfoil_input_tip=self.h_airfoil_input_tip,
                        h_wing_pos_factor=self.h_wing_pos_factor)
            lh=-(tail.horwing_right.MAC_qc_point.x-self.mainwing.mainwing_right.MAC_qc_point.x)
            Sh_new = (Vh * S * c_avg) / lh
            Sh=Sh+Sh_increment
        return Sh

    #Airfoil options:
    #ClarkX, GOE257, M6, NACA0010, NACA2412, NACA4412, NACA23012, NACA64210, RAF28, TSAGI12
    h_airfoil_input_root=Input("NACA0010") #withouth .dat
    h_airfoil_input_tip=Input("NACA0010") #withouth .dat

    ##Input for Vertical tail
    v_tail_volume=Input(0.083)
    v_sweep_qc=Input(37.5) #in degrees
    v_dihedral=Input(0.) #in degrees
    v_twist=Input(0.) #in degrees. Leave it zero in order to have an exact MAC chord length determination.
    v_wing_x_pos=Input(-35.) #wrt to MAC quarter chord position
    v_wing_z_pos=Input(-1.)
    v_aspect_ratio=Input(5.)
    v_taper_ratio=Input(0.303)

    @Input
    def v_aspect_ratio(self):
        if self.h_wing_pos_factor==0:
            v_AR=1.9
        else:
            v_AR=1.35
        return 2*v_AR #Factor two for different definitions of span and wing area (both factor two)

    @Input
    def v_taper_ratio(self):
        if self.h_wing_pos_factor==0:
            v_taper=0.3
        else:
            v_taper=0.7
        return v_taper

    @Input
    def v_wing_area(self):# (Twice the area)
        Sv = 1.
        Sv_increment = 1.
        tail = Tail(v_wing_area=Sv, v_aspect_ratio=self.v_aspect_ratio, v_taper_ratio=self.v_taper_ratio,
                    v_sweep_qc=self.v_sweep_qc, v_dihedral=self.v_dihedral, v_twist=self.v_twist,
                    v_wing_x_pos=self.v_wing_x_pos, v_wing_z_pos=self.v_wing_z_pos,
                    v_airfoil_input_root=self.v_airfoil_input_root, v_airfoil_input_tip=self.v_airfoil_input_tip)
        S = self.mainwing.wing_area
        b = self.mainwing.mainwing_right.span
        Vv = self.v_tail_volume
        lv = -(tail.verwing_MAC_qc_point.bbox.location.x - self.mainwing.mainwing_right.MAC_qc_point.x)
        Sv_new = (2*Vv * S * b) / lv
        while Sv_new > Sv:
            S = self.mainwing.wing_area
            b = self.mainwing.mainwing_right.span
            Vv = self.v_tail_volume
            tail = Tail(v_wing_area=Sv,v_aspect_ratio=self.v_aspect_ratio,v_taper_ratio=self.v_taper_ratio,
                        v_sweep_qc=self.v_sweep_qc, v_dihedral=self.v_dihedral, v_twist=self.v_twist,
                        v_wing_x_pos=self.v_wing_x_pos, v_wing_z_pos=self.v_wing_z_pos,
                        v_airfoil_input_root=self.v_airfoil_input_root, v_airfoil_input_tip=self.v_airfoil_input_tip)
            lv = -(tail.verwing_MAC_qc_point.bbox.location.x - self.mainwing.mainwing_right.MAC_qc_point.x)
            Sv_new = (2*Vv * S * b) / lv
            Sv = Sv + Sv_increment
        return Sv

    #Airfoil options:
    #ClarkX, GOE257, M6, NACA0010, NACA2412, NACA4412, NACA23012, NACA64210, RAF28, TSAGI12
    v_airfoil_input_root=Input("NACA0010") #withouth .dat
    v_airfoil_input_tip=Input("NACA0010") #withouth .dat


    #Airfoil options:
    #ClarkX, GOE257, M6, NACA0010, NACA2412, NACA4412, NACA23012, NACA64210, RAF28, TSAGI12
    airfoil_input_root=Input("RAF28") #withouth .dat
    airfoil_input_tip=Input("NACA4412") #withouth .dat

    ## Input for landinggear
    gearradius = Input(1.) #in meters
    zlocgear = Input(3.1) # height of gear under fuselage
    xlocgearMAC = Input(90) # x location of gear in percentage of MAC
    rot_angle = Input(14) # degrees


    @Input
    ## location of main gear, percentage of MAC of main wing
    def xlocgear(self):
        return self.wing_x_pos+(0.25-self.xlocgearMAC/100)*self.mainwing.mainwing_right.MAC_length

    ## Input for Fuselage
    fu_length = Input(37.57) #meters
    fu_tail_upsweep = Input(7) #degrees
    fu_slender = Input(9.51)
    fu_tail_slender = Input(1.8)
    fu_nose_slender = Input(1.25)
    fu_nose_radius = Input([10,90,100]) #percentage
    fu_tail_radius = Input([100,90,80,60,40,10])  #percentage

    ## Input for Engines
    TotThrust = Input(10000.) ## Total thrust in lbs
    Xf_ratio_c = Input(-0.14) ## horizontal engine position of nacelle with respect to leading edge of wing chord. Negative is fwd, positive is aft
    NacelleThickness = Input(0.1)
    NacelleLength = Input(2)
    Config = Input(2) ## engine configuration, 1 = 2 engines on main wing, 2 = 4 engines on main wing, 3 = 2 enignes on fuselage

    @Attribute
    def numengines(self):
        if self.Config == 2:
            numen=4
        else:
            numen=2
        return numen

    @Attribute
    def Thrust(self):
        return self.TotThrust/self.numengines

    @Attribute
    def spanwise_loc_ratio(self):
        if self.Config == 1:
            yposengineratio=[0.35]
        if self.Config == 2:
            yposengineratio=[0.4,0.7]
        return yposengineratio


    @Part
    def mainwing(self):
        return MainWing(pass_down="M_cruise,M_techfactor,wing_configuration,wing_x_pos,wing_z_pos,wing_area,aspect_ratio,twist,"
                              "airfoil_input_root,airfoil_input_tip,spanwise_loc_ratio")

    @Part
    def fuselage(self):
        return Fuselage(pass_down="fu_length,fu_tail_upsweep,fu_slender,fu_tail_slender,fu_nose_slender,fu_nose_radius,"
                                  "fu_tail_radius,rot_angle", rot_point=self.maingear.rotpoint)

    @Part
    def tail(self):
        return Tail(pass_down="h_wing_area,h_aspect_ratio,h_taper_ratio,h_sweep_qc,h_dihedral,h_twist,"
                              "h_wing_x_pos,h_wing_z_pos,h_wing_thickness_factor,h_airfoil_input_root,h_airfoil_input_tip,"
                              "v_wing_area,v_aspect_ratio,v_taper_ratio,v_sweep_qc,v_dihedral,v_twist,"
                              "v_wing_x_pos,v_wing_z_pos,v_airfoil_input_root,v_airfoil_input_tip,h_wing_pos_factor,"
                              "r_factor,r_aoa,spanwise_loc_ratio")


    @Attribute
    def engineposition(self):
        if self.Config==3:
            engineposition=[[self.fuselage.bulkhead],[self.fuselage.fu_radius],[0]]
        else:
            engineposition=self.mainwing.mainwing_right.LE_loc
        return engineposition

    @Attribute
    def MountType(self):
        if self.Config==3:
            mounttype='Tail'
        else:
            mounttype='Wing'
        return mounttype


    @Part
    def Engines(self):
        return Engine(quantify=int(self.numengines/2),pass_down='Thrust,Xf_ratio_c,Nacellethickness,NacelleLength,MountType',EngPosition=Point(self.engineposition[0][child.index],self.engineposition[1][child.index],self.engineposition[2][child.index]))
    @Part
    def EngineComp(self):
        return Compound(built_from=([self.Engines[0].TranslatedEngine,self.Engines[1].TranslatedEngine]) if self.numengines == 4 else [self.Engines[0].TranslatedEngine])

    @Part
    def MirroredEngines(self):
        return MirroredShape(self.EngineComp,reference_point=Point(0, 0, 0), vector1=Vector(1, 0, 0), vector2=Vector(0, 0, 1))

    @Attribute
    def zlocwingtip(self):
        mainwing=MainWing(M_cruise=self.M_cruise,M_techfactor=self.M_techfactor,wing_configuration=self.wing_configuration,
                              wing_x_pos=self.wing_x_pos,wing_z_pos=self.wing_z_pos,wing_area=self.wing_area,aspect_ratio=self.aspect_ratio,twist=self.twist,
                              airfoil_input_root=self.airfoil_input_root,airfoil_input_tip=self.airfoil_input_tip,spanwise_loc_ratio=1)
        return mainwing.LE_loc[0]

    @Part
    def maingear(self):
        return LandingGear(pass_down="gearradius,rot_angle,xlocgear,zlocgear", zlocwing=self.mainwing.wing_z_pos, zlocwingtip=self.zlocwingtip, zlocengine=self.Engines[0].bottomZlocFromWingLE)

##Xfoil
    xfoil_wing_select=Input("mainwing") #Choose between "mainwing","hor_tail" and "ver_tail"
    xfoil_spanwise_loc_ratio=Input([0.5]) #has to be a list
    Re=Input(500000) #Reynolds number
    xfoil_aoa_min=Input(-5) #Integer! Minimum angle of attack for the Xfoil analysis
    xfoil_aoa_max=Input(20) #Integer! Maximum angle of attack for the Xfoil analysis

    @Attribute(in_tree=False)
    def xfoil_equipoints_in_plane(self):
        if self.xfoil_wing_select=="mainwing":
            mainwing=MainWing(M_cruise=self.M_cruise,M_techfactor=self.M_techfactor,wing_configuration=self.wing_configuration,
                              wing_x_pos=self.wing_x_pos,wing_z_pos=self.wing_z_pos,wing_area=self.wing_area,aspect_ratio=self.aspect_ratio,twist=self.twist,
                              airfoil_input_root=self.airfoil_input_root,airfoil_input_tip=self.airfoil_input_tip,spanwise_loc_ratio=self.xfoil_spanwise_loc_ratio)
            plane=Plane(Point(mainwing.mainwing_right.LE_loc[0][0],mainwing.mainwing_right.LE_loc[1][0],mainwing.mainwing_right.LE_loc[2][0]),
                        Vector(-sin(radians(mainwing.mainwing_right.sweep_le)),cos(radians(mainwing.mainwing_right.sweep_le)),0))
            intersect=IntersectedShapes(shape_in=self.mainwing.mainwing_right.solid, tool=plane, color='red')
            equipoints=intersect.edges[0].curve.equispaced_points(100)
            equipoints_in_plane = points_in_plane(equipoints,plane.reference,plane.normal,plane.binormal)

        if self.xfoil_wing_select == "hor_tail":
            tail = Tail(h_wing_area=self.h_wing_area,h_aspect_ratio=self.h_aspect_ratio,h_taper_ratio=self.h_taper_ratio,h_sweep_qc=self.h_sweep_qc,h_dihedral=self.h_dihedral,h_twist=self.h_twist,
                              h_wing_x_pos=self.h_wing_x_pos,h_wing_z_pos=self.h_wing_z_pos,h_wing_thickness_factor=self.h_wing_thickness_factor,h_airfoil_input_root=self.h_airfoil_input_root,h_airfoil_input_tip=self.h_airfoil_input_tip,
                              v_wing_area=self.v_wing_area,v_aspect_ratio=self.v_aspect_ratio,v_taper_ratio=self.v_taper_ratio,v_sweep_qc=self.v_sweep_qc,v_dihedral=self.v_dihedral,v_twist=self.v_twist,
                              v_wing_x_pos=self.v_wing_x_pos,v_wing_z_pos=self.v_wing_z_pos,v_airfoil_input_root=self.v_airfoil_input_root,v_airfoil_input_tip=self.v_airfoil_input_tip,h_wing_pos_factor=self.h_wing_pos_factor,
                              r_factor=self.r_factor,r_aoa=self.r_aoa,spanwise_loc_ratio=self.xfoil_spanwise_loc_ratio)
            plane_pos=Point(tail.horwing_right.LE_loc[0][0], tail.horwing_right.LE_loc[1][0],tail.horwing_right.LE_loc[2][0])+Vector(-self.tail.verwing_zero.span*0.5*self.h_wing_pos_factor*tan(radians(self.tail.verwing_zero.sweep_le))+(self.tail.horwing_right.MAC_qc_point_zero.x-self.tail.verwing_zero.MAC_qc_point_zero.x),
                                      0, -self.tail.verwing_zero.span*0.5*self.h_wing_pos_factor)
            plane = Plane(plane_pos,Vector(-sin(radians(tail.horwing_right.sweep_le)),cos(radians(tail.horwing_right.sweep_le)), 0))
            intersect = IntersectedShapes(shape_in=self.tail.horwing_totsolid, tool=plane, color='red')
            equipoints = intersect.edges[0].curve.equispaced_points(100)
            equipoints_in_plane = points_in_plane(equipoints, plane.reference, plane.normal, plane.binormal)

        if self.xfoil_wing_select == "ver_tail":
            tail = Tail(h_wing_area=self.h_wing_area, h_aspect_ratio=self.h_aspect_ratio,h_taper_ratio=self.h_taper_ratio, h_sweep_qc=self.h_sweep_qc, h_dihedral=self.h_dihedral,
                        h_twist=self.h_twist,h_wing_x_pos=self.h_wing_x_pos, h_wing_z_pos=self.h_wing_z_pos,h_wing_thickness_factor=self.h_wing_thickness_factor,
                        h_airfoil_input_root=self.h_airfoil_input_root, h_airfoil_input_tip=self.h_airfoil_input_tip,v_wing_area=self.v_wing_area, v_aspect_ratio=self.v_aspect_ratio,
                        v_taper_ratio=self.v_taper_ratio, v_sweep_qc=self.v_sweep_qc, v_dihedral=self.v_dihedral,v_twist=self.v_twist,
                        v_wing_x_pos=self.v_wing_x_pos, v_wing_z_pos=self.v_wing_z_pos,v_airfoil_input_root=self.v_airfoil_input_root, v_airfoil_input_tip=self.v_airfoil_input_tip,
                        h_wing_pos_factor=self.h_wing_pos_factor,r_factor=self.r_factor, r_aoa=self.r_aoa, spanwise_loc_ratio=self.xfoil_spanwise_loc_ratio)
            plane_pos_zero = Point(tail.verwing_zero.LE_loc[0][0], tail.verwing_zero.LE_loc[1][0],tail.verwing_zero.LE_loc[2][0])
            plane_pos = plane_pos_zero.rotate_around(tail.verwing_zero.solid.location,Vector(1, 0, 0),radians(-90.))
            plane = Plane(plane_pos,Vector(-sin(radians(tail.verwing_zero.sweep_le)),0, -cos(radians(tail.verwing_zero.sweep_le))))
            intersect = IntersectedShapes(shape_in=self.tail.verwing, tool=plane, color='red')
            equipoints = intersect.edges[0].curve.equispaced_points(100)
            equipoints_in_plane = points_in_plane(equipoints, plane.reference, plane.normal, plane.binormal)
        return equipoints_in_plane

    @Attribute
    def xfoil_cl_alpha(self):
        data = run_xfoil(self.xfoil_equipoints_in_plane, self.Re, (self.xfoil_aoa_min, self.xfoil_aoa_max, 1))
        data2 = zip(*data)
        aoa = data2[0]
        cl = data2[1]
        cl_max=max(data2[1])
        for i in range(len(cl)):
            if cl[i]==cl_max:
                aoa_cl_max=aoa[i]
        return aoa, cl, aoa_cl_max, cl_max

    @Attribute
    def xfoil_plot(self):
        plt.plot(self.xfoil_cl_alpha[0], self.xfoil_cl_alpha[1])
        plt.xlabel('Angle of attack [deg]')
        plt.ylabel('Cl')
        plt.grid(b=True, which='both', color='0.65', linestyle='-')
        return plt.show()

    @Attribute
    def xfoil_aoa_cl_max(self):
        return self.xfoil_cl_alpha[2]

    @Attribute
    def xfoil_cl_max(self):
        return self.xfoil_cl_alpha[3]

if __name__ == '__main__':
    from parapy.gui import display
    obj = Aircraft()
    display(obj)