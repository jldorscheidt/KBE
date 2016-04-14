from __future__ import division
import os
from math import *
from parapy.geom import *
from parapy.core import *
from parapy.exchange.step import STEPWriter
from ISA import *
from LandingGear import *
from MainWing import *
from Fuselage import *
from Tail import *
from parapy.lib.xfoil import *
import matplotlib.pyplot as plt

class Aircraft(GeomBase):
##READING OF INPUT FILE##
    fileDir = os.path.dirname(os.path.realpath('__file__'))
    string = '../Code/Input.csv'
    filename = os.path.join(fileDir, string)
    filename = os.path.abspath(os.path.realpath(filename))
    obj = open(filename)
    data = obj.read().splitlines()
    obj.close()
    datalist=[]
    for i in range(len(data)):
        line=data[i]
        words=(line.split(";"))
        datalist.append(words)

    for i in range(len(datalist)):
        my_dict={}
        x=datalist[i][0]
        if datalist[i][1][0] not in ('0', '1', '2', '3', '4', '5', '6', '7', '8', '9','-'):
            my_dict[x]=Input(datalist[i][1])
        else:
            my_dict[x] = Input(float(datalist[i][1]))
        for key,val in my_dict.items():
            exec(key+'=val')


##COMPUTED INPUTS##
#Computed inputs for Wing:
    @Input
    def wing_x_pos(self): #50% of mid fuselage section for wing podded and 60% for fuselage mounted engines
        if self.Config==1 or 2:
            xposwing=self.fuselage.front_section_totlength+0.5*self.fuselage.mid_section_totlength
        if self.Config==3:
            xposwing = self.fuselage.front_section_totlength + 0.6 * self.fuselage.mid_section_totlength
        return -xposwing

    @Input
    def wing_z_pos(self):#At position relative to fuselage radius. Constant of 0.6 is experimentally determined on view
        if self.wing_configuration=="low":
            zposwing=0.6*self.fuselage.fu_radius
        if self.wing_configuration=="high":
            zposwing = -0.6 * self.fuselage.fu_radius
        return zposwing

#Computed inputs and attributes for Horizontal tail
    @Attribute
    def tail_loop(self):# (Twice the area)
        Sv = 0.
        Sv_new = 1.
        Sh = 0.
        Sh_new = 1.
        v_xpos = 0
        v_xpos_new = -self.fu_length
        while abs(Sv_new-Sv) > 0.1 or abs(Sh_new-Sh)>0.1 or abs(v_xpos_new-v_xpos)>0.1:
            Sv=Sv_new
            Sh=Sh_new
            v_xpos = v_xpos_new
            tail = Tail(h_wing_area=Sh, h_aspect_ratio=self.h_aspect_ratio,
                        h_taper_ratio=self.h_taper_ratio, h_sweep_qc=self.h_sweep_qc, h_dihedral=self.h_dihedral,
                        h_twist=self.h_twist, h_wing_x_pos=v_xpos, h_wing_z_pos=self.h_wing_z_pos,
                        h_wing_thickness_factor=self.h_wing_thickness_factor,
                        h_airfoil_input_root=self.h_airfoil_input_root, h_airfoil_input_tip=self.h_airfoil_input_tip,
                        v_wing_area=Sv, v_aspect_ratio=self.v_aspect_ratio,
                        v_taper_ratio=self.v_taper_ratio, v_sweep_qc=self.v_sweep_qc, v_dihedral=self.v_dihedral,
                        v_twist=self.v_twist,
                        v_wing_x_pos=v_xpos, v_wing_z_pos=self.v_wing_z_pos,
                        v_airfoil_input_root=self.v_airfoil_input_root, v_airfoil_input_tip=self.v_airfoil_input_tip,
                        h_wing_pos_factor=self.h_wing_pos_factor, r_factor=self.r_factor, r_aoa=self.r_aoa,
                        spanwise_loc_ratio=self.xfoil_spanwise_loc_ratio)
            S = self.mainwing.wing_area
            b = self.mainwing.mainwing_right.span
            Vv = self.v_tail_volume
            lv = abs(tail.verwing_MAC_qc_point.bbox.location.x - self.mainwing.mainwing_right.MAC_qc_point.x)
            c_avg = self.mainwing.mainwing_right.c_avg
            Vh = self.h_tail_volume
            lh = abs(tail.horwing_right.MAC_qc_point.x - self.mainwing.mainwing_right.MAC_qc_point.x)
            Sv_new = (2*Vv * S * b) / lv
            Sh_new = (Vh * S * c_avg) / lh
            v_xpos_new = -self.fu_length + tail.verwing_zero.c_root + 0.5 + tail.verwing_zero.MAC_qc_point_zero.x
        return Sv_new,Sh_new,v_xpos_new

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
        return self.tail_loop[1]

    @Input
    def h_wing_x_pos(self):  # Horizontal tail (initial) position same as for vertical tail. Position wrt to MAC quarter chord position
        return self.tail_loop[2]

    @Input
    def h_wing_z_pos(self):
        return self.v_wing_z_pos

#Computed inputs for Vertical tail
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
    def v_wing_area(self):  # (Twice the area)
        return self.tail_loop[0]

    @Input
    def v_wing_x_pos(self):  #Trailing edge of vertical 0.5m in front of end of fuselage (wrt to MAC quarter chord position)
        return self.tail_loop[2]

    @Input
    def v_wing_z_pos(self):
        return -self.fuselage.tail_section_totlength*tan(radians(self.fu_tail_upsweep))

#Computed input for fuselage
    @Input #Drag divergence number
    def Mdiv(self):
        return self.mainwing.M_dd

#Attributes for engines
    @Input
    #To make an integer of the Config input
    def Config(self):
        return int(self.Config_float)

    @Attribute
    ## calculates the number of engines corresponding to the Configuration chosen
    def numengines(self):
        if self.Config == 2:
            numen=4
        else:
            numen=2
        return numen

    @Attribute
    ## calculates the thrust per engine and convert to lbs
    def Thrust(self):
        return 0.224808942443*self.TotThrust/self.numengines

    @Attribute
    ## spanwise location of the engine(s) defined as a ratio of the span of the main wing
    def spanwise_loc_ratio(self):
        if self.Config == 1:
            yposengineratio=[0.35]
        if self.Config == 2:
            yposengineratio=[0.4,0.7]
        return yposengineratio

    @Attribute
    # position of the attachment point of the engine, for wing mounted engines this is on the leading edge, for fuselage mounted engines this is the location of the bulkhead
    def engineposition(self):
        if self.Config==3:
            engineposition=[[self.fuselage.bulkhead],[self.fuselage.fu_radius],[0]]
        else:
            engineposition=self.mainwing.mainwing_right.LE_loc
        return engineposition

    @Attribute
    # mount type of the engine
    def MountType(self):
        if self.Config==3:
            mounttype='Tail'
        else:
            mounttype='Wing'
        return mounttype

#Computed input for landing gear
    @Input
    ## location of main gear, percentage of MAC of main wing
    def xlocgear(self):
        return self.wing_x_pos + (0.25 - self.xlocgearMAC / 100) * self.mainwing.mainwing_right.MAC_length

##PARTS##
    @Part
    def mainwing(self):
        return MainWing(pass_down="M_cruise,M_techfactor,wing_configuration,wing_x_pos,wing_z_pos,wing_area,aspect_ratio,twist,"
                              "airfoil_input_root,airfoil_input_tip,spanwise_loc_ratio")
    @Part
    def tail(self):
        return Tail(pass_down="h_wing_area,h_aspect_ratio,h_taper_ratio,h_sweep_qc,h_dihedral,h_twist,"
                              "h_wing_x_pos,h_wing_z_pos,h_wing_thickness_factor,h_airfoil_input_root,h_airfoil_input_tip,"
                              "v_wing_area,v_aspect_ratio,v_taper_ratio,v_sweep_qc,v_dihedral,v_twist,"
                              "v_wing_x_pos,v_wing_z_pos,v_airfoil_input_root,v_airfoil_input_tip,h_wing_pos_factor,"
                              "r_factor,r_aoa,spanwise_loc_ratio")
    @Part
    def fuselage(self):
        return Fuselage(pass_down="fu_length,fu_tail_upsweep,fu_slender,fu_tail_slender,Mdiv,"
                                  "rot_angle", rot_point=self.maingear.rotpoint)

    @Part
    def Engines(self):
        return Engine(quantify=int(self.numengines/2),pass_down='Thrust,Xf_ratio_c,NacelleThickness,NacelleLength,MountType',Chord=self.mainwing.mainwing_right.spanwise_chord[child.index],EngPosition=Point(self.engineposition[0][child.index],self.engineposition[1][child.index],self.engineposition[2][child.index]))
    @Part
    def EngineComp(self):
        return Compound(built_from=([self.Engines[0].TranslatedEngine,self.Engines[1].TranslatedEngine]) if self.numengines == 4 else [self.Engines[0].TranslatedEngine])

    @Part
    def MirroredEngines(self):
        return MirroredShape(self.EngineComp,reference_point=Point(0, 0, 0), vector1=Vector(1, 0, 0), vector2=Vector(0, 0, 1))

    @Part
    def maingear(self):
        return LandingGear(pass_down="gearradius,rot_angle,xlocgear,zlocgear", zlocwing=self.mainwing.wing_z_pos, zlocwingtip=self.zlocwingtip, zlocengine=self.Engines[0].bottomZlocFromWingLE)

    @Part
    def total_compound(self):
        return Compound([self.mainwing.mainwing_totsolid,self.fuselage.rottotsolid,self.tail.tail_totsolid,
                                 self.EngineComp,self.MirroredEngines],hidden=True)

##CHECKS##
    #Attribute that checks all checks#
    @Attribute
    def warning_check(self):
        if self.fuselage.rotanglecheck == 1:
            rot_angle_check = 'rotation angle violated'
        else:
            rot_angle_check = 'rotation angle fullfilled'
        if self.maingear.ZlocCheck == 1:
            main_gear_check = 'main gear z location violated'
        else:
            main_gear_check = 'main gear z location fullfilled'
        if self.deep_stall_warning[5] == 1:
            deep_stall_check = 'deep stall check violated'
        else:
            deep_stall_check = 'deep stall check fullfilled'
        if self.r_warning == 1:
            rudder_check = 'rudder check violated'
        else:
            rudder_check = 'rudder check fullfilled'
        return rot_angle_check, main_gear_check, deep_stall_check, rudder_check

    #Atributes for Rudder
    @Attribute #Unblanketed percentage of rudder
    def r_unblanketed(self):
        return self.tail.r_unblanketed

    @Attribute
    def r_warning(self):
        if self.r_unblanketed<33.33:
            print "WARNING! Too large part of the rudder is blanketed by the horizontal tail!"
            flag = 1
        else:
            flag = 0
        return flag

    @Attribute
    def deep_stall_warning(self):
        dist_hor=abs(self.wing_x_pos-self.h_wing_x_pos)
        dist_hor_c=dist_hor/self.mainwing.mainwing_right.MAC_length
        dist_ver=abs(self.mainwing.mainwing_right.MAC_qc_point.z-(self.tail.horwing_right.MAC_qc_point.z+self.tail.horwing_totsolid.displacement.z))
        dist_ver_c=dist_ver/self.mainwing.mainwing_right.MAC_length
        limit_up=(4/15)*dist_hor_c+(7/15)
        limit_low=(2/15)*dist_hor_c+(1/30)
        if dist_ver_c<limit_up and dist_ver_c>limit_low:
            message="WARNING! Horizontal tail is in wake of main wing!"
            print message
            flag = 1
        else:
            message="Horizontal tail is not in the wake of the main wing. Good job!"
            flag = 0
        return dist_hor_c,dist_ver_c,limit_up,limit_low,message,flag

    @Part
    def deep_stall_low_limit(self):
       return LineSegment(Point(self.mainwing.mainwing_right.MAC_qc_point.x, 0, self.mainwing.mainwing_right.MAC_qc_point.z-(1/30)),
                   (Point(self.mainwing.mainwing_right.MAC_qc_point.x, 0, self.mainwing.mainwing_right.MAC_qc_point.z-(1/30))+Vector(-100, 0, (-200/15))),hidden=True)

    @Part
    def deep_stall_up_limit(self):
        return LineSegment(Point(self.mainwing.mainwing_right.MAC_qc_point.x, 0,self.mainwing.mainwing_right.MAC_qc_point.z - (7 / 15)),
                    (Point(self.mainwing.mainwing_right.MAC_qc_point.x, 0,self.mainwing.mainwing_right.MAC_qc_point.z - (7 / 15))+Vector(-100, 0, (-400 / 15))),hidden=True)

    @Part
    def deep_stall_low_limit_on_tail(self):
        return ProjectedCurve(source=self.deep_stall_low_limit, target=self.tail.verwing.solids[0],
                              direction=Vector(0, 1, 0), color= "green",line_thickness=3, hidden=False)

    @Part
    def deep_stall_up_limit_on_tail(self):
        return ProjectedCurve(source=self.deep_stall_up_limit, target=self.tail.verwing.solids[0],
                              direction=Vector(0, 1, 0), color="green",line_thickness=3,hidden=False)

    @Attribute
    #the z location of the wing tip, used for checking if the landing gear is positioned low enough.
    def zlocwingtip(self):
        mainwing=MainWing(M_cruise=self.M_cruise,M_techfactor=self.M_techfactor,wing_configuration=self.wing_configuration,
                              wing_x_pos=self.wing_x_pos,wing_z_pos=self.wing_z_pos,wing_area=self.wing_area,aspect_ratio=self.aspect_ratio,twist=self.twist,
                              airfoil_input_root=self.airfoil_input_root,airfoil_input_tip=self.airfoil_input_tip,spanwise_loc_ratio=[1])
        return mainwing.mainwing_right.LE_loc[2][0]



##Xfoil##
    @Input #Reynolds number for 11000m (http://www.digitaldutch.com/atmoscalc/)
    def Re(self):
        rho=atmos(self.altitude)[2]
        a=sound(self.altitude)
        T=atmos(self.altitude)[0]
        # rho=0.363918
        # a=295.070
        v=self.M_cruise*a
        L=self.mainwing.mainwing_right.MAC_length
        dyn_viscos=(1.458*10**-6*T*1.5)/(T+110.4) #Sutherland's law
        #dyn_viscos=0.0000143226
        Re=(rho*v*L)/(dyn_viscos)
        return Re

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




##ANALYSIS##
#MAINWING AREAS#
    @Part #Solid of the wing outside the fuselage
    def mainwing_exposed_solid(self):
        return SubtractedSolid(shape_in=self.mainwing.mainwing_totsolid.solids[0], tool=self.fuselage.rottotsolid,hidden=True)

    @Attribute #Reference area main wing [m2]
    def mainwing_reference_area(self):
        return self.wing_area

    @Attribute #Wetted area main wing [m2]
    def mainwing_wetted_area(self):
        return 2*self.mainwing_exposed_solid.area

    @Attribute #Exposed area main wing [m2]. Determined by multiplying the ratio of wetted area of exposed wing and total wing by the total reference area
    def mainwing_exposed_area(self):
        wetted_tot=self.mainwing.mainwing_totsolid.solids[0].area
        wetted_exposed=self.mainwing_exposed_solid.area
        return (wetted_exposed/wetted_tot)*self.mainwing_reference_area

#HORIZONTAL TAIL AREAS#
    @Part  # Solid of the horizontal tail outside the fuselage
    def h_exposed_solid(self):
        return SubtractedSolid(shape_in=self.tail.tail_totsolid.solids[0], tool=self.fuselage.rottotsolid,hidden=True)

    @Attribute  # Reference area horizontal tail [m2]
    def h_reference_area(self):
        return self.h_wing_area

    @Attribute  # Wetted area horizontal tail [m2]
    def h_wetted_area(self):
        return 2 * self.h_exposed_solid.area

    @Attribute  # Exposed area horizontal tail [m2]. Determined by multiplying the ratio of wetted area of exposed h_tail and total h_tail by the total reference area
    def h_exposed_area(self):
        wetted_tot = self.tail.tail_totsolid.solids[0].area
        wetted_exposed = self.h_exposed_solid.area
        return (wetted_exposed / wetted_tot) * self.h_reference_area

#VERTICAL TAIL AREAS#
    @Part  # Solid of the vertical tail outside the fuselage
    def v_exposed_solid(self):
        return SubtractedSolid(shape_in=self.tail.tail_totsolid.solids[2], tool=self.fuselage.rottotsolid,hidden=True)

    @Attribute  # Wetted area vertical tail [m2]
    def v_wetted_area(self):
        return 2 * self.v_exposed_solid.area

    @Attribute  # Exposed area vertical tail [m2]. Determined by multiplying the ratio of wetted area of exposed v_tail and total v_tail by the total reference area
    def v_exposed_area(self):
        wetted_tot = self.tail.tail_totsolid.solids[2].area
        wetted_exposed = self.v_exposed_solid.area
        return (wetted_exposed / wetted_tot) * self.v_wing_area

    @Attribute  # Reference area vertical tail [m2]. Equal to exposed area.
    def v_reference_area(self):
        return self.v_exposed_area

    @Attribute #Speed of flow over tail over aircraft speed
    def Vh_V(self):
        if self.h_wing_pos_factor==0:
            Vh_V=0.85
        if self.h_wing_pos_factor == 1:
            Vh_V = 1.
        else:
            Vh_V=0.95
        return Vh_V

    #DATCOM for main wing. CL_alpha in units [1/rad]
    @Attribute
    def CL_alpha_w(self):
        A=self.aspect_ratio
        beta=sqrt(1-self.M_cruise**2)
        sweep_halfc=self.mainwing.mainwing_right.sweep_halfc #degrees
        eta=0.95
        # A=10^100
        # beta=1
        # eta=1
        # sweep_halfc=0
        CL_alpha=(2*pi*A)/(2+sqrt(4+((A*beta)/eta)**2*(1+(((tan(radians(sweep_halfc)))**2)/(beta**2)))))
        return CL_alpha

    #DATCOM for horizontal tailplane. CL_alpha in units [1/rad]
    @Attribute
    def CL_alpha_h(self):
        A=self.h_aspect_ratio
        M=self.M_cruise*self.Vh_V
        beta=sqrt(1-M**2)
        sweep_halfc=self.tail.horwing_right.sweep_halfc #degrees
        eta=0.95
        CL_alpha=(2*pi*A)/(2+sqrt(4+((A*beta)/eta)**2*(1+(((tan(radians(sweep_halfc)))**2)/(beta**2)))))
        return CL_alpha

    #Lift gradient for wing and fuselage combination. CL_alpha in units [1/rad]
    @Attribute
    def CL_alpha_wf(self):
        CL_alpha_w=self.CL_alpha_w
        bf=2*self.fuselage.fu_radius
        b=self.mainwing.mainwing_right.span
        Snet=self.mainwing_exposed_area
        S=self.wing_area
        CL_alpha_wf=CL_alpha_w*(1+2.15*(bf/b))*(Snet/S)+(pi*(bf**2))/(2*S)
        return CL_alpha_wf

    #Downwash gradient on the tail. Root chord is horizontal so m_tv is determined using the vertical position between two MAC points
    @Attribute
    def downwash(self): #also notated as d_epsilon/d_alpha
        b=self.mainwing.mainwing_right.span
        lh = abs(self.tail.horwing_right.MAC_qc_point.x - self.mainwing.mainwing_right.MAC_qc_point.x)
        lv = abs(self.tail.horwing_right.MAC_qc_point.z - self.mainwing.mainwing_right.MAC_qc_point.z)
        r=(2*lh)/b
        m=(2*lv)/b
        sweep=radians(self.mainwing.sweep_qc)
        Kel=(0.1124+0.1265*sweep+0.1766*sweep**2)/(r**2)+0.1024/r+2.
        Kel0=0.1124/(r**2)+0.1024/r+2.
        Kfactor=Kel/Kel0
        term1=(0.4876*r)/((r**2+m**2)*(sqrt(r**2+0.6319+m**2)))
        term2=1+((r**2)/(r**2+0.7915+5.0734*m**2))**0.3113
        term3=1-sqrt((m**2)/(1+m**2))
        factor2=(self.CL_alpha_w)/(pi*self.aspect_ratio)
        return Kfactor*(term1+term2*term3)*factor2

#AERODYNAMIC CENTER#
    #AC of the wing+fuselage
    @Attribute
    def AC_pos_wf(self):
        AC_pos_w=0.25 #approximation
        bf=2*self.fuselage.fu_radius
        hf=bf #circular fuselage
        lfn=abs(self.mainwing_exposed_solid.bbox.corners[1].x)
        S=self.wing_area
        c_bar=self.mainwing.mainwing_right.MAC_length
        CL_alpha_wf=self.CL_alpha_wf
        cg=self.mainwing.mainwing_right.c_avg
        b=self.mainwing.mainwing_right.span
        taper=self.mainwing.taper_ratio
        sweep_qc=radians(self.mainwing.sweep_qc)
        term1=(bf*hf*lfn)/(CL_alpha_wf*S*c_bar)
        term2=(bf*cg*(b-bf)*tan(sweep_qc))/((1+taper)*(c_bar**2*(b+2.15*bf)))
        return AC_pos_w-1.8*term1+0.273*term2

    #Contribution of the nacelles to AC
    @Attribute
    def AC_pos_n(self):
        S = self.wing_area
        c_bar = self.mainwing.mainwing_right.MAC_length
        CL_alpha_wf = self.CL_alpha_wf
        if self.Config==1: #1 = 2 engines on main wing, 2 = 4 engines on main wing, 3 = 2 enignes on fuselage
            frontnacelle_to_LE=self.Engines[0].NacelleLength-self.Engines[0].Xf_ratio_c*self.Engines[0].Chord
            engposition=self.Engines[0].EngPosition.x
            frontnacelle_xpos=frontnacelle_to_LE+engposition
            MAC_xpos=self.mainwing.mainwing_right.MAC_qc_point.x
            ln=frontnacelle_xpos-MAC_xpos
            bn=2*self.Engines[0].NacelleRadius
            kn=-4.
            AC_pos_n=(2*kn*bn**2*ln)/(S*c_bar*CL_alpha_wf)
        if self.Config==2:
            frontnacelle_to_LE = self.Engines[0].NacelleLength - self.Engines[0].Xf_ratio_c * self.Engines[0].Chord
            engposition1 = self.Engines[0].EngPosition.x
            engposition2 = self.Engines[1].EngPosition.x
            frontnacelle_xpos1 = frontnacelle_to_LE + engposition1
            frontnacelle_xpos2 = frontnacelle_to_LE + engposition2
            MAC_xpos = self.mainwing.mainwing_right.MAC_qc_point.x
            ln1 = frontnacelle_xpos1 - MAC_xpos
            ln2 = frontnacelle_xpos2 - MAC_xpos
            bn = 2 * self.Engines[0].NacelleRadius
            kn = -4.
            AC_pos_n = (2 * kn * bn ** 2 * ln1 + 2 * kn * bn ** 2 * ln2) / (S * c_bar * CL_alpha_wf)
        if self.Config==3:
            #backnacelle_to_engposition = self.Engines[0].NacelleLength - self.Engines[0].Xf_ratio_c * self.Engines[0].Chord-self.Engines[0].NacelleLength
            backnacelle_to_engposition = 0
            engposition = self.Engines[0].EngPosition.x
            backnacelle_xpos = backnacelle_to_engposition + engposition
            MAC_xpos = self.mainwing.mainwing_right.MAC_qc_point.x
            ln = backnacelle_xpos - MAC_xpos
            bn = 2 * self.Engines[0].NacelleRadius
            kn = -2.5
            AC_pos_n = (2 * kn * bn ** 2 * ln) / (S * c_bar * CL_alpha_wf)
        return AC_pos_n

    #AC of fuselage minus tail
    @Attribute
    def AC_pos(self):
        return self.AC_pos_wf+self.AC_pos_n

    #AC point on MAC
    @Attribute
    def AC_point(self):
        return Point(self.mainwing.mainwing_right.MAC_qc_point.x+(0.25-self.AC_pos)*self.mainwing.mainwing_right.MAC_length,
                     self.mainwing.mainwing_right.MAC_qc_point.y,self.mainwing.mainwing_right.MAC_qc_point.z)

    #Create circle at the AC position (in the XY-plane)
    @Part
    def AC_point_circle(self):
        return Circle(radius=0.05,position=self.AC_point,color="blue",hidden=True)

    #Project this circle on the wing solid to get the AC location displayed on the wing
    @Part
    def AC_point_circle_onairfoil(self):
        return ProjectedCurve(source=self.AC_point_circle, target=self.mainwing.mainwing_right.solid, direction=Vector(0, 0, -1), color="blue",line_thickness=3,hidden=False)

#MOST AFT POSITION OF THE CG
    @Attribute
    def aft_cg_pos(self):
        x_ac=self.AC_pos
        CL_ah=self.CL_alpha_h
        CL_a=self.CL_alpha_wf
        downwash=self.downwash
        Sh=self.h_wing_area
        lh = abs(self.tail.horwing_right.MAC_qc_point.x - self.mainwing.mainwing_right.MAC_qc_point.x)
        S=self.wing_area
        c_bar = self.mainwing.mainwing_right.MAC_length
        Vh_V=self.Vh_V
        SM=0.05
        return x_ac+(CL_ah/CL_a)*(1-downwash)*((Sh*lh)/(S*c_bar))*Vh_V**2-SM

    #Most aft cg point on MAC
    @Attribute
    def aft_cg_point(self):
        return Point(self.mainwing.mainwing_right.MAC_qc_point.x+(0.25-self.aft_cg_pos)*self.mainwing.mainwing_right.MAC_length,
                     self.mainwing.mainwing_right.MAC_qc_point.y,self.mainwing.mainwing_right.MAC_qc_point.z)

    #Create circle at the most aft cg position (in the XY-plane)
    @Part
    def aft_cg_point_circle(self):
        return Circle(radius=0.05,position=self.aft_cg_point,color="green",hidden=True)

    #Project this circle on the wing solid to get the most aft cg location displayed on the wing
    @Part
    def aft_cg_point_circle_onairfoil(self):
        return ProjectedCurve(source=self.aft_cg_point_circle, target=self.mainwing.mainwing_right.solid, direction=Vector(0, 0, -1), color="green",line_thickness=3,hidden=False)

##STEP WRITER##
    DIR = Input(os.path.dirname(__file__))
    @Part
    def step_writer(self):
        return STEPWriter(nodes=[self.total_compound],
                          default_directory=self.DIR,
                          filename="aircraft.step")

if __name__ == '__main__':
    from parapy.gui import display
    obj = Aircraft()
    display(obj)