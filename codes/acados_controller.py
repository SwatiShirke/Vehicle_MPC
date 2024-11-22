from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from ackerman_model import ackerman_model
import numpy as np
import casadi as ca
#from utils import ackerman_model
#import yaml 
# import ipdb
# import sys

def acados_controller(N, Tf, L):
    #model configs param
    # N = params.N
    # Tf = params.Tf
    
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()
    # set model
    
    min_vel = 0
    max_vel = 30
    min_str_angle_rate = -1
    max_str_angle_rate = 1
    dthrottle_min = 0
    dthrottle_max = 5
    ddelta_min = -0.5    
    ddelta_max = 0.5

    model = ackerman_model(L)
    ocp.model = model
    
    #ipdb.set_trace()
    nx = model.x.rows()
    nu = model.u.rows()

    print(nx)
    print(nu)
    
    # set prediction horizon
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = Tf

    #cost matricesq
    Q_mat = 1*np.diag([10e1, 10e1, 100e1, 1e-2, 1e1 ])
    R_mat = 1*np.diag([1e-2, 1e-2])
    Q_emat = 1*np.diag([100e1, 100e1, 100e1, 1e-2, 1e1 ]) 
    unscale = N / Tf

     
    # path cost
    #smooth u transition
    u_last = ca.SX.sym("u_last")
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.yref = np.array([0.,0,0,0,0,0, 0] )
    del_error = ca.vertcat(model.x, model.u)- ocp.cost.yref
    u_delta = u_last - ocp.model.u
    #print(loss)
    #print(u_delta)
    ocp.model.cost_y_expr =  ca.vertcat(model.x, model.u)- ocp.cost.yref #ca.vertcat(del_error, u_delta)
    #ocp.cost.yref = np.zeros((nx+nu,))    
    ocp.cost.W = unscale * ca.diagcat(Q_mat, R_mat).full()

    # terminal cost
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.cost.yref_e = np.array([0., 0, 0, 0, 0])
    cost = model.x - ocp.cost.yref_e
    ocp.model.cost_y_expr_e = cost #model.x # 
    ocp.cost.W_e = unscale * Q_emat
    
    # set constraints
    #constraints on control input
    
    ocp.constraints.lbu = np.array([min_vel, min_str_angle_rate])
    ocp.constraints.ubu = np.array([max_vel, max_str_angle_rate])
    ocp.constraints.idxbu = np.array([0,1])

    #initial state contraints
    ocp.constraints.x0 = np.array([0, 0, 0, 0, 0] )

    #lower and upper bound constraints on states - velocity and angular velocities
    ocp.constraints.lbx = np.array([])
    ocp.constraints.ubx = np.array([])
    ocp.constraints.idxbx = np.array([] )
    ocp.constraints.lbu = np.array([dthrottle_min, ddelta_min])
    ocp.constraints.ubu = np.array([dthrottle_max, ddelta_max])

    # set intial condition
    #ocp.constraints.x0 = model.x0

    # set QP solver and integration
    ocp.solver_options.tf = Tf
    #ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3

    # ocp.solver_options.qp_solver_tol_stat = 1e-2
    # ocp.solver_options.qp_solver_tol_eq = 1e-2
    # ocp.solver_options.qp_solver_tol_ineq = 1e-2
    # ocp.solver_options.qp_solver_tol_comp = 1e-2

    # create solver
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")
    acados_integrator = AcadosSimSolver(ocp, json_file = "acados_ocp.json")

    return model, acados_solver, acados_integrator