from math import exp,sqrt
g=9.80665
R=287.05

def atmos(h):
    if h<=11000:
        T0=288.15
        p0=101325
        a=-0.0065
        h0=0.

        T = T0 + a*(h-h0)
        p = p0*(T/T0)**((-g)/(a*R))
        rho = p/(R*T)

    elif 11000<h<=20000:
        T0=atmos(11000)[0]
        p0=atmos(11000)[1]
        h0=11000.

        T = T0
        p = p0*exp((-g*(h-h0))/(R*T))
        rho = p/(R*T)

    elif 20000<h<=32000:
        T0=atmos(20000)[0]
        p0=atmos(20000)[1]
        h0=20000.
        a=0.001

        T = T0 + a*(h-h0)
        p = p0*(T/T0)**((-g)/(a*R))
        rho = p/(R*T)

    elif 32000<h<=47000:
        T0=atmos(32000)[0]
        p0=atmos(32000)[1]
        h0=32000.
        a=0.0028

        T = T0 + a*(h-h0)
        p = p0*(T/T0)**((-g)/(a*R))
        rho = p/(R*T)

    elif 47000<h<=51000:
        T0=atmos(47000)[0]
        p0=atmos(47000)[1]
        h0=47000.

        T = T0
        p = p0*exp((-g*(h-h0))/(R*T))
        rho = p/(R*T)

    elif 51000<h<=71000:
        T0=atmos(51000)[0]
        p0=atmos(51000)[1]
        h0=51000.
        a=-0.0028

        T = T0 + a*(h-h0)
        p = p0*(T/T0)**((-g)/(a*R))
        rho = p/(R*T)

    elif 71000<h<=84852:
        T0=atmos(71000)[0]
        p0=atmos(71000)[1]
        h0=71000.
        a=-0.002

        T = T0 + a*(h-h0)
        p = p0*(T/T0)**((-g)/(a*R))
        rho = p/(R*T)
    
    return T,p,rho

def sound(h):
    T=atmos(h)[0]
    a=sqrt(1.40*R*T)
    return a
