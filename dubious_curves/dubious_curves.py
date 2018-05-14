#!env/bin/python2.7

"""
@author Simon Fong, Aaron Lee, Kevin Lam
@email  simonfong6@gmail.com
@date   12 May 2018

Dubious Curves
"""
import matplotlib.pyplot as plt     # For plotting
from math import pi, sin, cos, atan2, acos, sqrt

# Plotting functions
def plot_circle(circle, color='b'):
    """
    Plots a circle.
    """
    circle_plt = plt.Circle((circle['x'], circle['y']), circle['radius'], color=color, fill=False)
    ax = plt.gca()
    ax.add_artist(circle_plt)
    
def plot_line(x_0,y_0,x_1,y_1,color='ro-'):
    """
    Draws a line between two points.
    """
    
    plt.plot([x_0,x_1],[y_0,y_1], color)
    
def plot_arrow(x,y,radians):
    """
    Plots an arrow at given point and in given direction.
    """
    ax = plt.gca()
    ax.arrow(x, y, cos(radians), sin(radians),
        head_width=1, head_length=1, fc='k', ec='k')

def plot():
    ax = plt.gca()                  # Get current axis
    ax.axis('equal')                # Makes circles actually look like circles
    plt.axis([-20, 20, -20, 20])    # Window size.
    plt.show()                      # Show plots on screen.
        
# END Plotting functions

class DubiousCurves:
    
    def __init__(self):
        pass
        
    def tangent_line_inner(self,circle_0,circle_1, x_dir):
        """
        Calculates inner tangent line points for two circles.
        
        Args:
            circle_0: Source circle.
            circle_1: Destination circle.
            x_dir: -1 or 1, left tangent if -1, right tangent otherwise.
            
        Returns:
            A tuple of tuples that represent an inner tangent.
        """
        # Radius is the same for both circles.
        radius = circle_0['radius']
        
        # Calculate distance between the centers
        distance = sqrt( (circle_1['y'] - circle_0['y'])**2 + 
                         (circle_1['x'] - circle_0['x'])**2 )
        
        # Angle between radius and line connecting centers                 
        phi = acos( radius / (distance/2) )
        
        # Angle in radians of line connecting centers
        theta = atan2( (circle_1['y'] - circle_0['y']) ,
                       (circle_1['x'] - circle_0['x']) )
        
        # Sums both to get phi in comparison to x=0
        phi = phi + theta
        
        # Point on source circle
        x_0 = radius * cos(phi) + circle_0['x']
        y_0 = radius * sin(phi) + circle_0['y']
        
        # Point on destination circle
        x_1 = radius * cos(pi + phi) + circle_1['x']
        y_1 = radius * sin(pi + phi) + circle_1['y']
        
        plot_line(x_0,y_0,x_1,y_1)
        
        
        return ((x_0,y_0),(x_1,y_1))
        
    
    def tangent_line_outer(self,circle_0,circle_1, x_dir):
        """
        Calculates outer tangent line points for two circles.
        
        Args:
            circle_0: Source circle.
            circle_1: Destination circle.
            x_dir: -1 or 1, left tangent if -1, right tangent otherwise.
        Returns:
            A tuple of tuples that represent an outer tangent.
        """
        
        # Radius is the same for both circles.
        radius = circle_0['radius']
    
        # Theta in radians
        theta = atan2( (circle_1['y'] - circle_0['y']) ,
                       (circle_1['x'] - circle_0['x']) )
        
        # Point on source circle
        x_0 = radius * cos(theta + -1 * x_dir * pi/2) + circle_0['x']
        y_0 = radius * sin(theta + -1 * x_dir * pi/2) + circle_0['y']
        
        # Point on destination circle
        x_1 = radius * cos(theta + -1 * x_dir * pi/2) + circle_1['x']
        y_1 = radius * sin(theta + -1 * x_dir * pi/2) + circle_1['y']
        
        plot_line(x_0,y_0,x_1,y_1)
        
        return ((x_0,y_0),(x_1,y_1))
        
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
        
    def tangent_circles(self,radius,x,y,radians):
        """
        Calculates the two tangent circles for a given point trajectory.
        
        Args:
            radius: Radius of tangent circle.
            x: X-coordinate of source point.
            y: Y-coordinate of source point.
            
        Returns:
            circles: Tuple of dictionaries that hold 'coordinates' as a tuple, 
                '(x,y)' ,cand 'radius' as a float, '3.0'.
        """
    
        # Left circle
        left_circle  = self.tangent_circle(radius,x,y,radians,-1)
        
        # Right circle
        right_circle  = self.tangent_circle(radius,x,y,radians,1)
        
        # Plotting
        plt.plot(x,y,'b.')      # Tangent point
        
        plot_arrow(x,y,radians) # Trajectory

        plot_circle(left_circle,'r')
        plot_circle(right_circle,'b')
        
        # Return coordinates for two circles that are tangent to the line. 
        return (left_circle,right_circle) 
        
    def calculate(self,radius,x_0,y_0,radians_0,x_1,y_1,radians_1):
        src = self.tangent_circles(radius,x_0,y_0,radians_0)
        dest = self.tangent_circles(radius,x_1,y_1,radians_1)
        
        # Theta in radians
        theta = atan2( (y_1 - y_0) , (x_1 - x_0) )
        
        # 0 = left, 1 = right
        first = None
        second = None
        
        if(radians_0 < theta):
            first = 0
        else:
            first = 1
            
        if(radians_1 > theta):
            second = 0
        else:
            second = 1
        
        
        #self.tangent_line_outer(src[first],dest[second],-1)
        #self.tangent_line_outer(src[first],dest[second],1)
        self.tangent_line_inner(src[first],dest[second],-1)
        self.tangent_line_inner(src[first],dest[second],1)
        plot_line(src[first]['x'],src[first]['y'],dest[second]['x'],dest[second]['y'])
        
        
        
        
        return "GTFO"
        


    

def main():
    dc = DubiousCurves()
    
    radius = 2
    
    x_0 = 0
    y_0 = 0
    radians_0 = -pi/2
    
    x_1 = -10
    y_1 = 10
    radians_1 = 0
    
    num = dc.calculate(radius,x_0,y_0,radians_0,x_1,y_1,radians_1)
    print(num)
    
    plot()

if(__name__ == '__main__'):
    main()
