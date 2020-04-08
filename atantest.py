import math
import numpy as np

link1Len = 150
link2Len = 150

goalx = 10
goaly = 10

goalDist = math.sqrt(goalx**2 + goaly**2)

if goalx > 0:
    alpha1 = math.atan(goaly/goalx)%(2*np.pi)
elif goalx == 0 and abs(goaly) > 0:
    alpha1 = (goaly/abs(goaly) * np.pi/2)%(2*np.pi)
elif goalx < 0:
    alpha1 = (math.atan(goaly/goalx)+np.pi)%(2*np.pi)
else: # goalx == 0 and goaly == 0
    alpha1 = 0

print(alpha1)

# a^2 = b^2 + c^2 -2bc*cosA
# link2Len^2 = link1Len^2 + goalDist^2 -

# a is link2
# b is link1
# c in hypotneuse
# Atheta = first angle
# Ctheta = second angle
# goal1Theta = alpha1 + Atheta (in quad 1 this is concave down) OR goal1Theta = alpha1 - Atheta (in quad1 this is concave up)
# goal2Theta = Ctheta (quad 1 - concave down) OR goal2Theta = 3*np.pi/2 - Ctheta (quad 2 - concave up)
if goalx != 0 or goaly != 0:
    Atheta = math.acos((link1Len**2 + goalDist**2 - link2Len**2)/(2*link1Len*goalDist))
    Ctheta = math.acos((link1Len**2 + link2Len**2 - goalDist**2)/(2*link1Len*link2Len))
    print(Atheta)
    goal1Theta = Atheta + alpha1
    goal2Theta = Ctheta
else:
    goal2Theta = np.pi
print(goal1Theta, goal2Theta)
