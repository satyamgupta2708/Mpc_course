import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['FULL_RECALCULATE'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference = [50, 0, 0]

    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        v_t = prev_state[3] # m/s
        x_t_1 = x_t + v_t*dt
        v_t_1 = v_t + pedal*dt - v_t/25.0 # for air friction 
        return [x_t_1, 0, 0, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0

        for k in range(0,self.horizon):
            state = self.plant_model(state,0.2,u[k],0)
            #cost = cost + (state[0]-ref[0])**2 
            cost = cost + 50*(state[0]-ref[0])**2 

            if (state[3]>2.7):
                # cost = cost + 10000*(state[3]-2.7)**2;
                # cost = cost + 10000*(state[3])**2;
                cost = cost + 1000000*(state[3]-2.7)**2;


    
        
        return cost
    

sim_run(options, ModelPredictiveControl)
