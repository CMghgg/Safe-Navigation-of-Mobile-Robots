import sys


from RVO import RVO_update, reach, compute_V_des, reach
from vis import visualize_traj_dynamic


#------------------------------
#define workspace model
ws_model = dict()
#robot radius
ws_model['robot_radius'] = 1.1
#circular obstacles, format [x,y,rad]...
# no obstacles
ws_model['circular_obstacles'] = [[20, 20, 20], [20, 100, 20], [140, 20, 20], [140, 100, 20], [33, 20, 20], [33, 100, 20], [127, 20, 20], [127, 100, 20]]
# with obstacles
# ws_model['circular_obstacles'] = [[-0.3, 2.5, 0.3], [1.5, 2.5, 0.3], [3.3, 2.5, 0.3], [5.1, 2.5, 0.3]]
#rectangular boundary, format [x,y,width/2,heigth/2]
ws_model['boundary'] = [] 

#------------------------------
#initialization for robot 
# position of [x,y]
#X = [[0.5+1.0*i, 1.0] for i in range(7)] + [[0.5+1.0*i, 5.0] for i in range(7)]
X = [[40+80*i, 60] for i in range(2)] + [[40+80*i ,50] for i in range(2)] + [[40+80*i, 70] for i in range(2)] + [[80, 20+80*i] for i in range(2)] + [[70, 20+80*i] for i in range(2)] + [[90, 20+80*i] for i in range(2)]
# velocity of [vx,vy]
V = [[2.5,2.5] for i in range(len(X))]
# maximal velocity norm
V_max = [5.0 for i in range(len(X))]
# goal of [x,y]
#goal = [[5.5-1.0*i, 10*5.0] for i in range(7)] + [[5.5-1.0*i, 1.0] for i in range(7)]
goal = [[140-120*i, 60] for i in range(2)] + [[70+20*i, 10] for i in range(2)] + [[70+20*i, 110] for i in range(2)] + [[80, 110-100*i] for i in range(2)] + [[20, 50+20*i] for i in range(2)] + [[140, 50+20*i] for i in range(2)]

#------------------------------
#simulation setup
# total simulation time (s)
total_time = 50
# simulation step
step = 0.05

#------------------------------
#simulation starts
t = 0
while t*step < total_time:
    # compute desired vel to goal
    V_des = compute_V_des(X, goal, V_max)
    # compute the optimal vel to avoid collision
    V = RVO_update(X, V_des, V, ws_model)
    # update position
    for i in range(len(X)):
        X[i][0] += V[i][0]*step
        X[i][1] += V[i][1]*step
    #----------------------------------------
    # visualization
    if t%10 == 0:
        visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.png'%str(t/10))
        #visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.png'%str(t/10))
    t += 1
    
