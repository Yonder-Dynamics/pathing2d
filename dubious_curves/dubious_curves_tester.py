#!env/bin/python2

"""
@author Simon Fong
@email  simonfong6@gmail.com
@date   14 May 2018

Dubious Curves Tester
"""
import unittest
from math import pi, sin, cos, atan2, acos, sqrt
from dubious_curves import DubiousCurves, plot

class DubiousCurvesTester(unittest.TestCase):
    def test_first_quadrant(self):
        dc = DubiousCurves()

        radius = 2

        x_0 = 0
        y_0 = 0
        radians_0 = -pi/4

        x_1 = 8
        y_1 = 8
        radians_1 = pi

        num = dc.calculate(radius,x_0,y_0,radians_0,x_1,y_1,radians_1)

        plot("First Quadrant")

    def test_second_quadrant(self):
        dc = DubiousCurves()

        radius = 2

        x_0 = 0
        y_0 = 0
        radians_0 = -pi/4

        x_1 = -8
        y_1 = 8
        radians_1 = pi

        num = dc.calculate(radius,x_0,y_0,radians_0,x_1,y_1,radians_1)
        plot("Second  Quadrant")
     
    def test_third_quadrant(self):
        dc = DubiousCurves()

        radius = 2

        x_0 = 0
        y_0 = 0
        radians_0 = -pi/4

        x_1 = -8
        y_1 = -8
        radians_1 = pi

        num = dc.calculate(radius,x_0,y_0,radians_0,x_1,y_1,radians_1)
        plot("Third  Quadrant")
        
    def test_fourth_quadrant(self):
        dc = DubiousCurves()

        radius = 2

        x_0 = 0
        y_0 = 0
        radians_0 = -pi/4

        x_1 = 8
        y_1 = -8
        radians_1 = pi

        num = dc.calculate(radius,x_0,y_0,radians_0,x_1,y_1,radians_1)
    
        
        plot("Fourth  Quadrant")
if(__name__ == '__main__'):
    unittest.main()
