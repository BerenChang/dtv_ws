import math
from sympy import symbols, Eq, solve
import numpy as np

if __name__ == "__main__":

    track1 = 849.99
    track2 = 456.92
    track3 = 546.83
    track4 = 136.53
    
    wheel1 = 131
    wheel2 = 152
    wheel3 = 131 # 85
    wheel4 = 131
    
    track12_angle = math.acos(4.8/5.3)
    wheel1_center = [track1/2, wheel1/2]
    wheel2_center = [-track1/2, wheel2/2]
    wheel3_center = [track2*math.cos(track12_angle) - (track2/2-track3/2)*math.sin(track12_angle) - track1/2, track2*math.sin(track12_angle)]
    
    x, y = symbols('x y')
    eq1 = Eq((x - wheel3_center[0])*(x - wheel3_center[0]) + (y - wheel3_center[1])*(y - wheel3_center[1]) - track3*track3, 0)
    eq2 = Eq((x - wheel1_center[0])*(x - wheel1_center[0]) + (y - wheel1_center[1])*(y - wheel1_center[1]) - track4*track4, 0)
    solve_wheel4 = solve((eq1,eq2), (x, y))
    wheel4_center = solve_wheel4[:2]
    
    print(wheel1_center) # 424.995, 65.5
    print(wheel2_center) # -424.995, 65.5
    print(wheel3_center) # 7.880433551199076, 193.73584779972217
    print(wheel4_center) # two solutions, need to pick one (549.824478572476, 120.799567623297)
    wheel4_center = [549.824478572476, 120.799567623297]
    
    track2_center = (np.array([-(track2/2-track3/2)*math.sin(track12_angle) - track1/2, -(track2/2-track3/2)*math.cos(track12_angle) + wheel2/2]) + np.array(wheel3_center)) / 2
    track3_center = (np.array(wheel3_center) + np.array(wheel4_center)) / 2
    track4_center = (np.array(wheel1_center) + np.array(wheel4_center)) / 2
    
    print(track2_center) # -199.02673626  155.22490503
    print(track3_center) # 278.85245606 157.26770771
    print(track4_center) # 487.40973929  93.14978381
    
    track2_angle = math.atan(abs(track2_center[0] - wheel3_center[0]) / abs(track2_center[1] - wheel3_center[1]))
    track3_angle = math.atan(-abs(wheel4_center[0] - wheel3_center[0]) / abs(wheel4_center[1] - wheel3_center[1]))
    track4_angle = math.atan(abs(wheel4_center[0] - wheel1_center[0]) / abs(wheel4_center[1] - wheel1_center[1]))
    
    print(track2_angle)
    print(track3_angle)
    print(track4_angle)
