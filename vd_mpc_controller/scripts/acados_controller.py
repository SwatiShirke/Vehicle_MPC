from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from ackerman_model import ackerman_model
import numpy as np
import casadi as ca
from scipy.spatial.transform import Rotation as R

#from utils import ackerman_model
#import yaml 
# import ipdb
# import sys

def cal_state_cost(state_vec, ref_vec, weights):
    pos_cost = ca.dot((ref_vec[0:2] - state_vec[0:2])**2, weights[0:2])
    vel_cost = (ref_vec[3] - state_vec[3])**2 * weights[3]
    # #yaw_cost =  ( 1 - np.cos(ref_vec[2] - state_vec[2]))**2  * weights[2]
    cost = (pos_cost + vel_cost )
       
    return cost 



def cal_input_cost(input_vec, ref_vec, weights):
    cost = ca.dot((ref_vec - input_vec)**2, weights)      
    #rate_cost = ca.dot((prev_in[1:3] - input_vec[1:3])**2, control_rate_weight[1:3])
    return cost 

# x, y, qw, qx,qy,qz, v, acc, del1, del2
def calculate_quat_cost(current_yaw, ref_yaw, weight):
    current_quat = ca.vertcat(ca.cos(current_yaw/2), 0, 0, ca.sin(current_yaw/2))
    ref_quat = ca.vertcat(ca.cos(ref_yaw/2), 0, 0, ca.sin(ref_yaw/2))
    
    weights = ca.vertcat(weight, 0,0 )
    qk = ref_quat
    qd = current_quat 
    q_aux = ca.vertcat(
     qd[0] * qk[0] + qd[1] * qk[1] + qd[2] * qk[2] + qd[3] * qk[3],
    -qd[1] * qk[0] + qd[0] * qk[1] + qd[3] * qk[2] - qd[2] * qk[3],
    -qd[2] * qk[0] - qd[3] * qk[1] + qd[0] * qk[2] + qd[1] * qk[3],
    -qd[3] * qk[0] + qd[2] * qk[1] - qd[1] * qk[2] + qd[0] * qk[3]
    )

    q_att_denom = ca.sqrt(q_aux[0] * q_aux[0] + q_aux[3] * q_aux[3] + 1e-3)
    q_att = ca.vertcat(
      q_aux[0] * q_aux[1] - q_aux[2] * q_aux[3],
      q_aux[0] * q_aux[2] + q_aux[1] * q_aux[3],
      q_aux[3]) / q_att_denom
    
   
    cost = ca.transpose(q_att) @ ca.diag(weights) @ q_att
    
    return cost


def get_constraints(u_aaray, prev_in):
    h_list = []    
    #yaw_const = ca.fabs(x_array[2] - prev_state[2])
    yaw_rate_change_const = ca.fabs(u_aaray[1] - prev_in[1])    
    h_list = ca.vertcat( yaw_rate_change_const)
    return h_list

def acados_controller(N, Tf, lf, lr, mass, Cr ):
    #model configs param
    # N = params.N
    # Tf = params.Tf
    
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()
    # set model    
    min_accel = -1
    max_accel = +1    
    vel_min = 0
    vel_max = 85
    yaw_rate_change_rate = 0.0001
    min_yaw_rate = -0.001
    max_yaw_rate = 0.001


    model = ackerman_model(lf, lr, mass, Cr )
    ocp.model = model
    
    ocp.dims.np = ocp.model.p.size()[0]
    ocp.parameter_values = np.zeros(ocp.dims.np)

    #ipdb.set_trace()
    nx = model.x.rows()
    nu = model.u.rows()
        
    # set prediction horizon
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = Tf
    unscale = 1
    #cost matricesq
    # x, y, yaw, pitch, roll, vel
    Q_mat = unscale * ca.vertcat(10e1, 10e1,   100e1, 10e1)
    R_mat = unscale * ca.vertcat( 1e-8, 1e-8)
    Q_emat =  unscale * ca.vertcat(100e1, 100e1,   1000e1,100e1)     
    prev_in = ca.vertcat(0,0)
    prev_state = ca.vertcat(0,0,0,0)

    x_array = model.x
    u_aaray = model.u
    ref_array = model.p  # x, y, qw, qx,qy,qz, v, acc, del1, del2


    state_error = cal_state_cost(x_array, ref_array, Q_mat )
    quat_error = calculate_quat_cost(x_array[2],ref_array[2], Q_mat[2] )
    input_error = cal_input_cost(u_aaray, ref_array[4:6], R_mat)  
    

    ocp.cost.cost_type = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost = state_error + quat_error + input_error 
    ocp.model.cost_expr_ext_cost_0 = state_error + quat_error + input_error     
    
    

    state_error = cal_state_cost(x_array, ref_array, Q_emat)
    quat_error = calculate_quat_cost(x_array[2],ref_array[2], Q_mat[2]  )
    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost_e = state_error + quat_error 

    
    
    # set constraints
    #constraints on control input    
    ocp.constraints.lbu = np.array([min_accel, min_yaw_rate]) #min_yaw_rate
    ocp.constraints.ubu = np.array([max_accel, max_yaw_rate]) #max_yaw_rate
    ocp.constraints.idxbu = np.array([0, 1])

    #initial state contraints
    ocp.constraints.x0 = np.array([0, 0, 0, 0] )

    #lower and upper bound constraints on states - velocity and angular velocities
    ocp.constraints.lbx = np.array([-2* np.pi ,vel_min])
    ocp.constraints.ubx = np.array([2 * np.pi, vel_max])
    ocp.constraints.idxbx = np.array([2,3] )
    
    # # set inequlaity constraints
    # h_list = get_constraints(u_aaray, prev_in)
    # ocp.model.con_h_expr = h_list
    # ocp.dims.nh = h_list.shape[0]
    # ocp.constraints.lh = np.array([0]) #np.array([0,0,0])        # yaw rate, delta rate constraints
    # ocp.constraints.uh =   np.array([yaw_rate_change_rate])#np.array([yaw_rate, steer_rate,state_error])             # Upper bound 
    # ocp.model.lh = np.array([0]) # np.array([0,0,0])            # lower bound
    # ocp.model.uh = np.array([yaw_rate_change_rate]) #np.array([yaw_rate, steer_rate,state_error])  

    ##update last states and input for rate control
    prev_in =  u_aaray 
    prev_state = x_array

    # set QP solver and integration
    ocp.solver_options.tf = Tf
    #ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3

    # create solver
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")
    acados_integrator = AcadosSimSolver(ocp, json_file = "acados_ocp.json")

    return model, acados_solver, acados_integrator



if __name__ == "__main__":
    N = 10
    Tf = 0.2
    lf = 2.56/2
    lr = 2.56/2
    mass = 2000
    Cr =  40000  #N/Rad   #coeff lateral stiffess 
    #L =  2.5654
    model, acados_solver, acados_integrator = acados_controller(N, Tf, lf, lr, mass, Cr )