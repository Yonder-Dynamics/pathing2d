#!env/bin/python2.7

"""
@author Simon Fong, Aaron Lee, Kevin Lam
@email  simonfong6@gmail.com
@date   12 May 2018

Dubious Curves
"""
import matplotlib.pyplot as plt     # For plotting
from math import pi, sin, cos, atan

class DubiousCurves:
    
    def __init__(self):
        pass
    
        
    def tangent_circle(self,radius,x,y,radian,x_dir):
        """
        Calculates tangent coordinates for a tangent circle.
        
        Args:
            radius: Radius of tangent circle.
            x: X-coordinate of source point.
            y: Y-coordinate of source point.
            x_dir: -1 or 1, left circle if -1, right circle otherwise
            

        Returns:
            circle: Dictionary that holds 'coordinates' as a tuple, '(x,y)' ,
                and 'radius' as a float, '3.0'.
        """
        # Tangent circle
        x_cir = x + x_dir * radius * sin(radian)  # x_cir = x + -1 * r * sin(theta)
        y_cir = y - x_dir * radius * cos(radian)
        
        circle = {'x': x_cir,
                  'y': y_cir,
                  'radius': radius}
        
        return circle
        
    def plot_circle(self,circle, color='b'):
        """
        Plots a circle.
        """
        circle_plt = plt.Circle((circle['x'], circle['y']), circle['radius'], color=color, fill=False)
        ax = plt.gca()
        ax.add_artist(circle_plt)
        
    def tangent_line(self,circle_0,circle_1, x_dir):
        """
        Calculates tangent line points for two circles.
        
        Args:
            circle_0: Source circle.
            circle_1: Destination circle.
            x_dir: -1 or 1, left tangent if -1, right tangent otherwise
        """
        
        # Radius is the same for both circles.
        radius = circle_0['radius']
    
        # Theta in radians
        theta = atan( (circle_1['y'] - circle_0['y']) / ((circle_1['x'] - circle_0['x'])) )
        
        # Point on src circle
        x_0 = radius * cos(theta + -1 * x_dir * pi/2) + circle_0['x']
        y_0 = radius * sin(theta + -1 * x_dir * pi/2) + circle_0['y']
        
        x_1 = radius * cos(theta + -1 * x_dir * pi/2) + circle_1['x']
        y_1 = radius * sin(theta + -1 * x_dir * pi/2) + circle_1['y']
        
        self.plot_line(x_0,y_0,x_1,y_1)
        
        return ((x_0,y_0),(x_1,y_1))
        
    def plot_line(self,x_0,y_0,x_1,y_1,color='ro-'):
        """
        Draws a line between two points.
        """
        
        plt.plot([x_0,x_1],[y_0,y_1], color)
        
    def plot_arrow(self,x,y,radians):
        ax = plt.gca()
        ax.arrow(x, y, cos(radians), sin(radians),
            head_width=1, head_length=1, fc='k', ec='k')

    def tangent_circles(self,radius,x,y,radian):
        """
        Calculates the two tangent circles for a given point trajectory.
        """
    
        # Left circle
        left_circle  = self.tangent_circle(radius,x,y,radian,-1)
        
        # Right circle
        right_circle  = self.tangent_circle(radius,x,y,radian,1)
        
        # Plotting
        plt.plot(x,y,'b.')  # Src point
        self.plot_arrow(x,y,radian)

        
        self.plot_circle(left_circle,'r')
        self.plot_circle(right_circle,'b')
        
        
        
        # Return coordinates for two circles that are tangent to the line. 
        return (left_circle,right_circle) 
        
    def calculate(self,radius,x_0,y_0,radian_0,x_1,y_1,radian_1):
        src = self.tangent_circles(radius,x_0,y_0,radian_0)
        dest = self.tangent_circles(radius,x_1,y_1,radian_1)
        
        #self.plot_line(src[0]['x'],src[0]['y'],dest[0]['x'],dest[0]['y'])
        #self.plot_line(src[1]['x'],src[1]['y'],dest[1]['x'],dest[1]['y'],'bo-')
        
        # Theta in radians
        theta = atan( (y_1 - y_0) / (x_1 - x_0) )
        
        # 0 = left, 1 = right
        first = None
        second = None
        
        if(radian_0 < theta):
            first = 0
        else:
            first = 1
            
        if(radian_1 > theta):
            second = 0
        else:
            second = 1
        
        
        self.tangent_line(src[first],dest[second],-1)
        self.tangent_line(src[first],dest[second],1)
        
        
        
        return "GTFO"
    

def main():
    dc = DubiousCurves()
    
    radius = 2
    
    x_0 = 0
    y_0 = 0
    radian_0 = -pi/2
    
    x_1 = 5
    y_1 = 5
    radian_1 = 0
    
    
    num = dc.calculate(radius,x_0,y_0,radian_0,x_1,y_1,radian_1)
    print(num)
    
    

    ax = plt.gca()                  # Get current axis
    ax.axis('equal')                # Makes circles actually look like circles
    plt.axis([-20, 20, -20, 20])    # Window size.
    plt.show()                      # Show plots on screen.

if(__name__ == '__main__'):
    main()
