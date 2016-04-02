from __future__ import division
from parapy.core import *
from parapy.geom import *
from math import *

class Wing(GeomBase):
    w_c_root=Input(6.)
    w_c_tip=Input(2.3)
    w_span=Input(10.)
    w_sweep=Input(35.)

    obj=open('airfoil.dat')
    data=obj.read().splitlines()
    obj.close()

    @Attribute
    def points(self):
        pointlist=[]
        for i in range(0,len(self.data)):
            x,y=self.data[i].split()
            pnt=Point(float(x),float(y))
            pointlist.append(pnt)
        return pointlist

    @Part
    def airfoil(self):
        return FittedCurve(self.points,hidden=True)

    @Part
    def airfoil2(self):
        return TransformedCurve(self.airfoil,from_position=OXY,to_position=OXY(z=0.5*self.w_span),hidden=True)

    @Part
    def airfoil3(self):
        return ScaledCurve(curve_in=self.airfoil,reference_point=self.airfoil.center,factor=self.w_c_root,hidden=True)

    @Part
    def airfoil4(self):
        return ScaledCurve(curve_in=self.airfoil2,reference_point=self.airfoil2.center,factor=self.w_c_tip,hidden=True)

    @Part
    def airfoil5(self):
        return TransformedCurve(self.airfoil4,from_position=OXY,to_position=OXY(x=tan(radians(self.w_sweep))*self.w_span),hidden=True)
    @Part
    def solidwing(self):
        return LoftedSolid([self.airfoil3,self.airfoil5])

if __name__ == '__main__':
    from parapy.gui import display
    obj = Wing()
    display(obj)