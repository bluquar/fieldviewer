from visual import *

epsilonNought = 8.85e-12
k = 1 / (4 * pi * epsilonNought)
scale = 4e-2

def eField(source, observation, charge):
    field = observation - source
    field.mag = (k * charge) / (field.mag2)
    return field

# class proton
# class box
# class disk
# class curve
# class surface f(u,v)

class Rod(object):
    def __init__(self, position=vector(0,0,0), axis=vector(1,0,0), q=1e-8):
        self.position = position
        self.axis = axis
        self.q = q
        self.makeCylinder()
    def makeCylinder(self):
        red = 1; blue = 0
        if self.q < 0:
            red = 0; blue = 1
        self.cylinder = cylinder(pos=self.position, axis = self.axis, color=(red,0,blue),
                                 radius = max(0.03, self.axis.mag / 40))
    def field(self, observation, n):
        step = 1./n
        dQ = self.q / n
        field = vector(0,0,0)
        for t in arange(step/2, 1 + step/3, step):
            field += eField(self.position + t*self.axis, observation, dQ)
        return field

class Arc(object):
    def __init__(self, theta = 0, center=vector(0,0,0), radius=1, alpha=pi, normal=vector(1,0,0), q=1e-8):
        self.center = center
        self.alpha = alpha
        self.theta = theta
        self.radius = radius
        self.q = q
        axis = vector(1,1,1)
        if axis.norm() == normal.norm() or -axis.norm() == normal.norm():
            axis = vector(1,0,0)
        self.axis1 = axis.cross(normal)
        self.axis2 = self.axis1.cross(normal)
        self.axis1.mag = radius
        self.axis2.mag = radius
        self.makePath()
    def makePath(self):
        t = arange(self.theta, self.theta + self.alpha, self.alpha / 1000)
        blue = 0
        if self.q < 0:
            blue = 1
        self.curve = curve( x = self.center.x + self.axis1.x * cos(t) + self.axis2.x * sin(t),
                            y = self.center.y + self.axis1.y * cos(t) + self.axis2.y * sin(t),
                            z = self.center.z + self.axis1.z * cos(t) + self.axis2.z * sin(t),
                            radius = max(self.radius / 19, 0.03),
                            red = 1, green = 0, blue=blue)
    def field(self, observation, n):
        field = vector(0,0,0)
        dQ = self.q / n
        step = self.alpha / n
        for theta in arange(self.theta+step/2, self.theta+self.alpha + step/3, step):
            source = self.center + self.axis1*cos(theta) + self.axis2*sin(theta)
            field += eField(source, observation, dQ)
        return field

            

def makeHeart(radius, charges, q):
    q=q/4
    charges.append(Arc(q=q, alpha = pi, theta=pi, radius=radius, center = vector(0.7*radius,1*radius,0), normal = vector(0,0,1)))
    charges.append(Arc(q=q, alpha = pi, theta=pi/2, radius=radius, center = vector(-0.7*radius,1*radius,0), normal = vector(0,0,1)))
    charges.append(Rod(q=q, position = vector(1.41*radius,0.3*radius,0), axis = vector(-1.425*radius, -1.4*radius, 0)))
    charges.append(Rod(q=q, position = vector(-1.41*radius,0.3*radius,0), axis = vector(1.425*radius, -1.4*radius, 0)))

class VectorField(object):
    def __init__(self, origin=vector(0,0,0), phi=arange(0,pi,pi/8), theta=arange(0,2*pi,pi/8), r=[1]):
        self.arrows = []
        fill = (0,1,0)
        for rad in r:
            for phiCurrent in phi:
                z = rad * cos(phiCurrent)
                xyRadius = rad * sin(phiCurrent)
                for thetaCurrent in theta:
                    self.arrows.append(arrow(pos=origin+ vector(xyRadius*cos(thetaCurrent),
                                                  xyRadius*sin(thetaCurrent),
                                                  z), axis=(0,0,0), color=fill))
    def setAxes(self, charges, n):
        for arrow in self.arrows:
            field = vector(0,0,0)
            for charge in charges:
                field += charge.field(arrow.pos, n)
            arrow.axis = scale * field
            arrow.red = field.mag / 300
            arrow.green = 1 - field.mag / 300
            arrow.blue = field.mag / 1000

def main():
    N = 20
    bgColor = (1,1,1)
    scene = display(height=600, width=800, background = bgColor)

    charges = []
    totalCharge = 1e-8
    N = 5
    minRadius = 0
    maxRadius = 1.1
    halfRadius = (maxRadius + minRadius) / 2
    for rad in arange(minRadius, maxRadius, 1./N):
        makeHeart(rad, charges, totalCharge * (rad / halfRadius) / N)

    # text(text='Shae + Chris',align='center', depth=-0.3, color=color.red, height = 0.3, pos = vector(0,0.5,0))
    #charges.append(Arc(alpha = pi/3, q=-1e-8))
    #charges.append(Arc(alpha = pi/3, q=1e-8, radius=0.8))
    #charges.append(Arc(alpha = pi/3, q=1e-8, radius=0.8, normal=vector(-1,0,0)))
    #charges.append(Rod(position=vector(0,0,0.5),axis=vector(0.8,0,0),q=1e-9))
    
    arrows = VectorField(theta=arange(0,2*pi, pi/8), r=[2.1], origin = vector(0.01,0.5,0), phi=arange(0,pi, pi/15))
    arrows.setAxes(charges, N)

if __name__=='__main__':
    main()
    
