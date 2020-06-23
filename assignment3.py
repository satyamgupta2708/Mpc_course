import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = True

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 15
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 0, 0]
        self.reference2 = None

        self.x_obs = 5
        self.y_obs = 0.1

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        x_t_1 = x_t + v_t*np.cos(psi_t)*dt
        y_t_1 = y_t + v_t*np.sin(psi_t)*dt
        psi_t_1 = psi_t + v_t*dt*np.tan(steering)/2.5
        v_t_1 = v_t + pedal*dt - v_t/25.0 # for air friction 


        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        steering_old = 0
        cost = 0.0    
        # for k in range(0,self.horizon):           #with euclidean distance 
        #         state = self.plant_model(state,0.2,u[2*k],u[(2*k)+1])
        #         distance = (self.x_obs-state[0])**2 + (self.y_obs-state[1])**2
        #         distance = distance**(-1)
        #         steering_new = u[(2*k)+1]
        #         # cost = cost + abs(state[0]-ref[0])**2 + abs(state[1]-ref[1])**2 + 50*distance**2 + 8*abs(state[2]-ref[2])**2 + abs(steering_new-steering_old)**2
        #         cost = cost + 8*abs(state[0]-ref[0])**2 + 8*abs(state[1]-ref[1])**2 + 100*distance + 20*abs(state[2]-ref[2])**2 + abs(steering_new-steering_old)
        #         steering_old = steering_new
         
        for k in range(0,self.horizon):       # with manhattan distance
                state = self.plant_model(state,0.2,u[2*k],u[(2*k)+1])
                distance = abs(self.x_obs-state[0]) + abs(self.y_obs-state[1])
                distance = distance**(-1)
                steering_new = u[(2*k)+1]
                cost = cost + 5*abs(state[0]-ref[0])**2 + 5*abs(state[1]-ref[1])**2 + abs(state[2]-ref[2])**2 + 75*distance**2
                #cost = cost + 5*abs(state[0]-ref[0])**2 + 5*abs(state[1]-ref[1])**2 + abs(state[2]-ref[2])**2 + 75*distance**(3)
               
                steering_old = steering_new
                           

        return cost

sim_run(options, ModelPredictiveControl)
