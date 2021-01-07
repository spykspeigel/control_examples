

import crocoddyl
import pinocchio
import numpy as np
import matplotlib.animation as animation

class DifferentialActionModelTT(crocoddyl.DifferentialActionModelAbstract):
    def __init__(self,target):
        crocoddyl.DifferentialActionModelAbstract.__init__(self, crocoddyl.StateVector(4), 2,3)  # nu = 1; nr = 6

        a=0.2  
        b=0.2  
        c=0.3  
        d=0.4  
        e=0.4
                
        self.params = (a,b,c,d,e)
        self.costWeights = [1e1, .01, 1e2]  # sin, 1-cos, x, xdot, thdot, f
        self.time_elapsed = 0
        self.target=target
        
    def target_residual(self,x):
        return x-self.target
        
    def calc(self, data, x, u=None):
        if u is None: 
            u = model.unone
        # Getting the state and control variables

        (a,b,c,d,e) = self.params


        x1, y1, theta1, delta = x[0].item(), x[1].item(), x[2].item(), x[3].item()
        theta2=theta1-delta

        v = u[0].item()
        w = u[1].item()
        
        s, c = np.sin(theta1), np.cos(theta1)


        data.xout = np.array([v*c, v*s, w, w-(np.sin(delta)*v*c-np.cos(delta)*(c-b)*w)/(d+e) ])

        residual=self.target_residual(x) 
        # Computing the cost residual and value
        data.r = np.array([self.costWeights[0]*np.linalg.norm(residual),self.costWeights[1]*np.linalg.norm(u),self.costWeights[2]*(abs(abs(delta)-.1))])
        data.cost = .5 * sum(np.asarray(data.r) ** 2).item()




init_state=np.array([0,0,np.pi/2,0])
target=np.array([4,4,np.pi/2,0])



ex_m= DifferentialActionModelTT(target)
ex_d = ex_m.createData()

x = ex_m.state.rand()
u = np.zeros(2)
a=ex_m.calc(ex_d, x, u)
print(ex_d.xout)



ex_nd = crocoddyl.DifferentialActionModelNumDiff(ex_m, True)
ex_d = ex_nd.createData()
ex_nd.calc(ex_d, x, u)
print('h')
ex_nd.calcDiff(ex_d, x, u)
print(ex_d.Fx)



timeStep = 5e-2
ex_iam = crocoddyl.IntegratedActionModelEuler(ex_nd, timeStep)




x0 = np.matrix([ 0., 0, np.pi/2., 0. ]).T
T  = 50
problem = crocoddyl.ShootingProblem(x0, [ ex_iam ]*T, ex_iam)




us = [ pinocchio.utils.zero(ex_iam.differential.nu) ]*T
xs = problem.rollout(us)




import matplotlib.pyplot as plt
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                     xlim=(-2, 6), ylim=(-2, 6))
ax.grid()

line, = ax.plot([], [], 'o-', lw=2)
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)







ddp = crocoddyl.SolverDDP(problem)
ddp.setCallbacks([crocoddyl.CallbackVerbose()])

# Solving this problem
done = ddp.solve([], [], 1000)
print done
print ddp.us

xt=[]
yt=[]




(a,b,c,d,e) = ex_m.params
for i in range(len(ddp.xs)):
    x1=ddp.xs[i][0]
    y1=ddp.xs[i][1]
    theta1=ddp.xs[i][2]
    theta2=theta1-ddp.xs[i][3]

    x = np.array([x1+a*np.cos(theta1),
                x1-c*np.cos(theta1),
               -e*np.cos(theta2)])
    x[2]=x[1]+x[2]
    xt.append(x)

    y = np.array([y1+a*np.sin(theta1),
                y1-c*np.sin(theta1),
               -e*np.sin(theta2)])
    y[2]=y[1]+y[2]
    yt.append(y)

print(yt)

def init():
    """initialize animation"""
    line.set_data([], [])
    time_text.set_text('')
    #energy_text.set_text('')
    return line, time_text

def animate(i):
    """perform animation step"""
    global xt,yt, dt
    
    
    line.set_data(xt[i][:],yt[i][:])
    #time_text.set_text('time = %.1f' % ex.time_elapsed)
    #energy_text.set_text('energy = %.3f J' % pendulum.energy())
    return line, time_text
from time import time
t0 = time()
animate(0)
dt = 1./30 # 30 fps
t1 = time()
interval = 1000 * dt - (t1 - t0)

ani = animation.FuncAnimation(fig, animate, frames=50,
                              interval=interval, blit=True, init_func=init)

