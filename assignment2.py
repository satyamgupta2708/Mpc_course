import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 10, -1.57]
        self.reference2 = [10, 2 , 3*3.14/2]

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
        cost = 0.0
        steering_old = 0
        pedal_old = 0
        for k in range(0,self.horizon):
            state = self.plant_model(state,0.2,u[2*k],u[(2*k)+1])
            steering_new = u[(2*k)+1]
            pedal_new = u[(2*k)]
            #cost = cost + (state[0]-ref[0])**2 + abs(state[1]-ref[1]) + (state[2]-ref[2])**2 
            # cost = cost + abs(state[0]-ref[0]) + abs(state[1]-ref[1]) + abs(state[2]-ref[2])

            # for self.reference1 = [10, 10, -1.57]
            cost = cost + abs(state[0]-ref[0]) + abs(state[1]-ref[1]) + abs(steering_new-steering_old) + abs(state[2]-ref[2])
            steering_old = steering_new
              
            # cost = cost + (state[0]-ref[0])**2 +  (pedal_new-pedal_old)**2 + (state[2]-ref[2])**2+ abs(steering_new-steering_old)
            # pedal_old = pedal_new
            # steering_old = steering_new



        return cost

sim_run(options, ModelPredictiveControl)
