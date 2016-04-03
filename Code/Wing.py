from __future__ import division
from parapy.core import *
from parapy.geom import *
from math import *
import os

class Wing(GeomBase):
    wing_area=Input(122.4) #in m^2
    aspect_ratio=Input(9.39)
    taper_ratio=Input(0.24)
    sweep_qc=Input(25.) #in degrees
    dihedral=Input(10.) #in degrees
    twist=Input(10.) #in degrees
    wing_x_pos=Input(0.)
    wing_z_pos=Input(0.)

    #Airfoil options:
    #ClarkX, GOE257, M6, NACA0010, NACA2412, NACA4412, NACA23012, NACA64210, RAF28, TSAGI12
    airfoil_input_root=Input("GOE257") #withouth .dat
    airfoil_input_tip=Input("GOE257") #withouth .dat

    @Input
    #Leading edge sweep (in degrees
    def sweep_le(self):
        distance=(tan(radians(self.sweep_qc))*0.5*self.span)-0.25*self.c_tip+0.25*self.c_root
        return degrees(atan(distance/(0.5*self.span)))

    @Input
    #Tip to tip span
    def span(self):
        return sqrt(self.aspect_ratio*self.wing_area)

    @Input
    #Average chord
    def c_avg(self):
        return self.span/self.aspect_ratio

    @Input
    #Root chord
    def c_root(self):
        return (2*self.c_avg)/(1+self.taper_ratio)

    @Input
    #Tip chord
    def c_tip(self):
        return self.taper_ratio*self.c_root

    #MAC determination
    #Note: airfoil.start is trailing edge of the airfoil in global axis system
    @Attribute(in_tree=True)
    def MAC(self):
        pt1=self.airfoil3.start+Vector(-self.c_tip,0,0) #point behind root chord at distance c_tip
        pt2=self.airfoil5.start+Vector(self.c_tip+self.c_root,0,0) #point in front of tip chord at distance c_root
        pt3=self.airfoil3.start+Vector(0.5*self.c_root,0,0) #half chord point at root location
        pt4=self.airfoil5.start+Vector(0.5*self.c_tip,0,0) #half chord point at tip location
        crv1=FittedCurve([pt1,pt2])
        crv2=FittedCurve([pt3,pt4])
        cross_point=crv1.intersection_point(crv2)

        return cross_point


    #Opening of airfoil data for root airfoil
    @Attribute
    def airfoil_data_root(self):
        fileDir = os.path.dirname(os.path.realpath('__file__'))
        if self.airfoil_input_root=="ClarkX":
            string= '../Code/airfoils/ClarkX.dat'
        if self.airfoil_input_root=="GOE257":
            string= '../Code/airfoils/GOE257.dat'
        if self.airfoil_input_root=="M6":
            string= '../Code/airfoils/M6.dat'
        if self.airfoil_input_root=="NACA0010":
            string= '../Code/airfoils/NACA0010.dat'
        if self.airfoil_input_root=="NACA2412":
            string= '../Code/airfoils/NACA2412.dat'
        if self.airfoil_input_root=="NACA4412":
            string= '../Code/airfoils/NACA4412.dat'
        if self.airfoil_input_root=="NACA23012":
            string= '../Code/airfoils/NACA23012.dat'
        if self.airfoil_input_root=="NACA64210":
            string= '../Code/airfoils/NACA64210.dat'
        if self.airfoil_input_root=="RAF28":
            string= '../Code/airfoils/RAF28.dat'
        if self.airfoil_input_root=="TSAGI12":
            string= '../Code/airfoils/TSAGI12.dat'
        filename = os.path.join(fileDir, string)
        filename = os.path.abspath(os.path.realpath(filename))
        obj=open(filename)
        data=obj.read().splitlines()
        obj.close()
        return data

    #Opening of airfoil data for tip airfoil
    @Attribute
    def airfoil_data_tip(self):
        fileDir = os.path.dirname(os.path.realpath('__file__'))
        if self.airfoil_input_tip=="ClarkX":
            string= '../Code/airfoils/ClarkX.dat'
        if self.airfoil_input_tip=="M6":
            string= '../Code/airfoils/M6.dat'
        if self.airfoil_input_tip=="GOE257":
            string= '../Code/airfoils/GOE257.dat'
        if self.airfoil_input_tip=="NACA0010":
            string= '../Code/airfoils/NACA0010.dat'
        if self.airfoil_input_tip=="NACA2412":
            string= '../Code/airfoils/NACA2412.dat'
        if self.airfoil_input_tip=="NACA4412":
            string= '../Code/airfoils/NACA4412.dat'
        if self.airfoil_input_tip=="NACA23012":
            string= '../Code/airfoils/NACA23012.dat'
        if self.airfoil_input_tip=="NACA64210":
            string= '../Code/airfoils/NACA64210.dat'
        if self.airfoil_input_tip=="RAF28":
            string= '../Code/airfoils/RAF28.dat'
        if self.airfoil_input_tip=="TSAGI12":
            string= '../Code/airfoils/TSAGI12.dat'
        filename = os.path.join(fileDir, string)
        filename = os.path.abspath(os.path.realpath(filename))
        obj=open(filename)
        data=obj.read().splitlines()
        obj.close()
        return data

    #Creation of list of points from the airfoil data for root airfoil
    @Attribute
    def points_root(self):
        pointlist=[]
        for i in range(2,len(self.airfoil_data_root)):
            x,y=self.airfoil_data_root[i].split()
            pnt=Point(float(x),float(y))
            pointlist.append(pnt)
        return pointlist

    #Creation of list of points from the airfoil data for tip airfoil
    @Attribute
    def points_tip(self):
        pointlist=[]
        for i in range(2,len(self.airfoil_data_tip)):
            x,y=self.airfoil_data_tip[i].split()
            pnt=Point(float(x),float(y))
            pointlist.append(pnt)
        return pointlist

    #Creation of root airfoil
    @Part
    def airfoil_a_root(self):
        return InterpolatedCurve(self.points_root,hidden=True)

    #Creation of tip airfoil
    @Part
    def airfoil_a_tip(self):
        return InterpolatedCurve(self.points_tip,hidden=True)

    #Mirror in YZ-plane
    @Part
    def airfoil_b_root(self):
        return MirroredCurve(self.airfoil_a_root,reference_point=OXY,vector1=Vector(0,1,0),vector2=Vector(0,0,1),hidden=True)

    @Part
    def airfoil_b_tip(self):
        return MirroredCurve(self.airfoil_a_tip,reference_point=OXY,vector1=Vector(0,1,0),vector2=Vector(0,0,1),hidden=True)

    #Rotate around global X-axis negative 90 degrees
    @Part
    def airfoil_root(self):
        return RotatedCurve(self.airfoil_b_root,rotation_point=Point(0, 0, 0), vector=Vector(1, 0, 0), angle=radians(-90),hidden=True)

    @Part
    def airfoil_tip(self):
        return RotatedCurve(self.airfoil_b_tip,rotation_point=Point(0, 0, 0), vector=Vector(1, 0, 0), angle=radians(-90),hidden=True)

    #Moving tip airfoil to tip location
    @Part
    def airfoil2(self):
        return TransformedCurve(self.airfoil_tip,from_position=OXY,to_position=OXY(y=0.5*self.span),hidden=True)

    #Scaling root airfoil to match root chord
    @Part
    def airfoil3(self):
        return ScaledCurve(curve_in=self.airfoil_root,reference_point=self.airfoil_root.center,factor=self.c_root,hidden=True)

    #Scaling tip airfoil to match tip chord
    @Part
    def airfoil4(self):
        return ScaledCurve(curve_in=self.airfoil2,reference_point=self.airfoil2.center,factor=self.c_tip,hidden=True)

    #Moving tip airfoil backwards to match sweep angle
    @Part
    def airfoil5(self):
        return TransformedCurve(self.airfoil4,from_position=OXY,to_position=OXY(x=-tan(radians(self.sweep_le))*0.5*self.span),hidden=True)

    #Moving tip airfoil upwards to match dihedral angle
    @Part
    def airfoil6(self):
        return TransformedCurve(self.airfoil5,from_position=OXY,to_position=OXY(z=-tan(radians(self.dihedral))*0.5*self.span),hidden=True)

    #Rotate tip airfoil around positive Y-axis to match twist angle
    @Part
    def airfoil7(self):
        return RotatedCurve(self.airfoil6,rotation_point=Point(0, 0, 0), vector=Vector(0, 1, 0), angle=radians(self.twist),hidden=True)

    #Creation of solid called "solidwing"
    @Part
    def solid(self):
        return LoftedSolid([self.airfoil3,self.airfoil7])

if __name__ == '__main__':
    from parapy.gui import display
    obj = Wing()
    display(obj)