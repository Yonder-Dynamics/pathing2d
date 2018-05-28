from dubious_curves import DubiousCurves
import math

# distance between the wheels in meters
wheelBaseWidth = 0.50

def calculate_path():
    db = DubiousCurves()
    db.calculate(2, 0, 0, -math.pi/4, 8, 8, math.pi)
    #print(db.calculate(2, 0, 0, -math.pi/4, -8, 8, math.pi))
    #print(db.calculate(2, 0, 0, -math.pi/4, -8, -8, math.pi))
    #print(db.calculate(2, 0, 0, -math.pi/4, 8, -8, math.pi))

calculate_path()