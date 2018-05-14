#!env/bin/python2.7

"""
@author Simon Fong, Aaron Lee, Kevin Lam
@email  simonfong6@gmail.com
@date   12 May 2018

Dubious Curves

Notes:
    Use atan2 instead of atan because it gives the correct angle.
    Ex:
        atan(-1/-1) = pi/4
        atan2(-1,-1) = -3pi/4
"""
import matplotlib.pyplot as plt     # For plotting
from math import pi, sin, cos, atan2, acos, sqrt

# Plotting functions
def plot_circle(circle, color='b'):
    """
    Plots a circle.
    """
    circle_plt = plt.Circle((circle['x'], circle['y']), 
        circle['radius'], color=color, fill=False)
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
    arrow_length = 3
    ax.arrow(x, y, arrow_length * cos(radians), arrow_length * sin(radians),
        head_width=1, head_length=1, fc='k', ec='k')

def plot(title="",figure_filename=None):
    ax = plt.gca()                  # Get current axis
    ax.axis('equal')                # Makes circles actually look like circles
    plt.axis([-20, 20, -20, 20])    # Window size.
    plt.title(title)
    plt.show()                      # Show plots on screen.
    if(figure_filename is not None):
        plt.savefig(figure_filename)
        print("Saved figure to {}".format(figure_filename))
        
# END Plotting functions

class DubiousCurves:
    
    def __init__(self):
        pass
        
    def circle_to_circle_angle(self,circle_src,circle_dest):
        """
        Calculates the angle from the center of one circle to the center of
        another. Helper function.
        
        Args:
            circle_src: Dictionary for the source circle.
            circle_dest: Dictionary for the destination circle.
            
        Returns:
            double: Angle in radians.
        """
        
        # Angle in radians of line connecting centers
        theta = atan2( (circle_dest['y'] - circle_src['y']) ,
                       (circle_dest['x'] - circle_src['x']) )
                       
        return theta
        
    def line_angle(self,line):
        """
        Calculates the angle from one point to another. Helper function.
        
        Args:
            x_src: X-coordinate of starting location.
            y_src: X-coordinate of starting location.
            x_dest: X-coordinate of destination.
            y_dest: X-coordinate of destination.
            
        Returns:
            double: Angle in radians.
        """
        
        # Angle in radians
        theta = atan2( (line[1][1] - line[0][1]) , (line[1][0] - line[0][0]) )
                       
        return theta
        
    def tangent_line_inner(self,circle_0,circle_1, radian_dir, color='ro-'):
        """
        Calculates inner tangent line points for two circles.
        
        Args:
            circle_0: Source circle.
            circle_1: Destination circle.
            radian_dir: Negative radianed tangent if -1, positive if 1.
            
        Returns:
            A tuple of tuples that represent an inner tangent.
        """
        # Radius is the same for both circles.
        radius = circle_0['radius']
        
        # Calculate distance between the centers
        distance = sqrt( (circle_1['y'] - circle_0['y'])**2 + 
                         (circle_1['x'] - circle_0['x'])**2 )
        
        # Relative angle between radius and center lines                 
        rel_phi = acos( radius / (distance/2) )
        
        # Angle in radians of line connecting centers
        theta = self.circle_to_circle_angle(circle_0,circle_1)
        
        # Sums both to get phi in comparison to x=0
        abs_phi = theta + radian_dir * rel_phi
        
        # Point on source circle
        x_0 = radius * cos(abs_phi) + circle_0['x']
        y_0 = radius * sin(abs_phi) + circle_0['y']
        
        # Point on destination circle
        x_1 = radius * cos(pi + abs_phi) + circle_1['x']
        y_1 = radius * sin(pi + abs_phi) + circle_1['y']
        
        plot_line(x_0,y_0,x_1,y_1,color)
        
        
        return ((x_0,y_0),(x_1,y_1))
        
    
    def tangent_line_outer(self,circle_0,circle_1, radian_dir, color='ro-'):
        """
        Calculates outer tangent line points for two circles.
        
        Args:
            circle_0: Source circle.
            circle_1: Destination circle.
            radian_dir: Negative radianed tangent if -1, positive if 1.
        Returns:
            A tuple of tuples that represent an outer tangent.
        """
        
        # Radius is the same for both circles.
        radius = circle_0['radius']
    
        # Theta in radians
        theta = self.circle_to_circle_angle(circle_0,circle_1)
        
        # Point on source circle
        x_0 = radius * cos(theta + radian_dir * pi/2) + circle_0['x']
        y_0 = radius * sin(theta + radian_dir * pi/2) + circle_0['y']
        
        # Point on destination circle
        x_1 = radius * cos(theta + radian_dir * pi/2) + circle_1['x']
        y_1 = radius * sin(theta + radian_dir * pi/2) + circle_1['y']
        
        plot_line(x_0,y_0,x_1,y_1,color)
        
        return ((x_0,y_0),(x_1,y_1))
        
    def tangent_circle(self,radius,x,y,radian,radian_dir):
        """
        Calculates tangent coordinates for a tangent circle.
        
        Args:
            radius: Radius of tangent circle.
            x: X-coordinate of source point.
            y: Y-coordinate of source point.
            radian_dir: Determines whether circle will be positive radian side.
            

        Returns:
            circle: Dictionary that holds 'coordinates' as a tuple, '(x,y)' ,
                and 'radius' as a float, '3.0'.
        """
        
        # x_cir = x + r * cos(theta +/- pi/2)
        x_cir = x + radius * cos(radian + radian_dir * pi/2)
        
        # y_cir = y + r * sin(theta +/- pi/2)
        y_cir = y + radius * sin(radian + radian_dir * pi/2)
        
        
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
        left_circle  = self.tangent_circle(radius,x,y,radians,1)
        
        # Right circle
        right_circle  = self.tangent_circle(radius,x,y,radians,-1)
        
        # Plotting
        plt.plot(x,y,'b.')      # Tangent point
        
        plot_arrow(x,y,radians) # Trajectory

        plot_circle(left_circle,'r')
        plot_circle(right_circle,'b')
        
        # Return coordinates for two circles that are tangent to the line. 
        return (left_circle,right_circle) 
        
    def calculate(self,radius,x_0,y_0,radians_0,x_1,y_1,radians_1):
        """
        Calculates the circle path and radius that needs to followed.
        
        Args:
            radius: Radius of the circles in m.
            x_0: X-coordinate of starting location.
            y_0: X-coordinate of starting location.
            radians_0: Direction that the rover is pointing in radians.
            x_1: X-coordinate of destination.
            y_1: X-coordinate of destination.
            radians_1: Direction in radians that the rover is pointing at the
                destination.
        Returns:
            Path to be followed.
        """
        src = self.tangent_circles(radius,x_0,y_0,radians_0)
        dest = self.tangent_circles(radius,x_1,y_1,radians_1)
        
        # Theta in radians
        theta = atan2( (y_1 - y_0) , (x_1 - x_0) )
        
        # 0 = pos, 1 = neg
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
        
        """
        Pos on circle: pi/2
        Dir: 0
        3pi/2,pi
        Lets call this negative bc u take the pos on circle - pi/2
        4 tangent lines locations and slopes
        pi/4,-pi/4 Yes
        ?,?         Yes
        ?,?         No
        -3pi/4,-pi/4 No
        Eliminate all tangents on circle that aren't possible for this
        direction.
        
        OR
        left circle = +
        right_circle -
        if left choose -
        
        """
        first1 = None
        if(first == 0):
            first1 = -1
        else:
            first1 = 1
        
        
        if(second == 0):
            tang1 = self.tangent_line_outer(src[first],
                dest[second],first1,'bo-')
            tang2 = self.tangent_line_outer(src[first],
                dest[second],first1,'go-')
        else:
            tang3 = self.tangent_line_inner(src[first],
                dest[second],first1,'bo-')
            tang4 = self.tangent_line_inner(src[first],
                dest[second],first1,'go-')
        
        plot_line(src[first]['x'],src[first]['y'],
            dest[second]['x'],dest[second]['y'])
        
        
        
        
        return "GTFO"
        


    

def main():
    dc = DubiousCurves()
    
    radius = 2
    
    x_0 = 0
    y_0 = 0
    radians_0 = -pi/4
    
    x_1 = 8
    y_1 = 8
    radians_1 = pi
    
    num = dc.calculate(radius,x_0,y_0,radians_0,x_1,y_1,radians_1)
    print(num)
    
    plot()

if(__name__ == '__main__'):
    main()
