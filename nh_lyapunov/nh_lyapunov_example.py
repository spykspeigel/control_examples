#Here I formulate the control problem as a closed loop steering.
# It closely follows the paper



import numpy as np
import math
from scipy.integrate import solve_ivp
import matplotlib.pylab as plt
def main():
	t_eval=np.arange(0,8,0.025)
	x_i=-1
	y_i=1
	theta_i=3*np.pi/4
	goal=np.array([0,0])
	heading_v=np.array([np.cos(theta_i),np.sin(theta_i)])

	x_rel = np.array([x_i,y_i])-goal
	x_rel=x_rel/np.linalg.norm(x_rel)

	e=np.sqrt((x_i-goal[0])**2 + (y_i-goal[1])**2)
	alpha=np.arccos(x_rel.dot(heading_v)/np.linalg.norm(heading_v))
	theta=np.arctan2(x_rel[1],x_rel[0])

	x0=np.array([e,alpha,theta])
	print(x0)

	sol=solve_ivp(nh_ode,[0,8],x0,method='RK45',t_eval=t_eval)

	t_span = sol.t
	X = sol.y
	X=convert_into_cartesian(X)
	return X
#	nh_draw(t_span, X, x_min, x_max, p);

def nh_ode(t,x):
	e=x[0]
	alpha=x[1]
	theta=x[2]

	gamma=3
	h=1
	k=6
	e_dot=-e*(gamma*np.cos(alpha)**2)
	alpha_dot=(-k*alpha -gamma*h*np.cos(alpha)*np.sin(alpha)/alpha)
	theta_dot=gamma*np.cos(alpha)*np.sin(alpha)

	return [e_dot,alpha_dot,theta_dot]


#Convert the polar coordinates into cartesian coordinates
#and call draw_triangle

def convert_into_cartesian(x):
	es=x[0,:]
	alphas=x[1,:]
	thetas=x[2,:]

	xs=es*np.cos(thetas)
	ys=es*np.sin(thetas)
	phis=thetas-alphas
	return np.array([xs,ys,phis])




"""


def draw_triangle():


	T = np.array([
        [np.cos(z3), -np.sin(z3), z1],
        [np.sin(z3), np.cos(z3), z2]
    ])

    # compute cartesian coordinates of the corners of the car
	left_back = T.dot([-length/2, width/2, 1])
	right_back = T.dot([-length/2, -width/2, 1])
	front = T.dot([length/2, 0, 1])
    
    # draw a triangle that symbolizes the robot
	robot = patches.Polygon(
	    np.vstack([left_back, right_back, front]),
	    facecolor='none',
	    **kwargs
	)
	plt.gca().add_patch(robot)
"""


if __name__== '__main__':
	x=main()