from casadi import *
from acados_template import AcadosModel


def ackerman_model(lf, lr, mass, Cr):
    model_name = "ackerman_model"
    model = AcadosModel()   

    #states
    x_pos = SX.sym("x_pos")
    y_pos = SX.sym("y_pos")
    theta = SX.sym("theta")
    Vf = SX.sym("Vf")
    #psi = SX.sym("psi")
    #s = SX.sym("s")
    x = vertcat(x_pos, y_pos, theta,Vf)

    
    #states dt
    x_pos_dt = SX.sym("x_pos_dt")
    y_pos_dt = SX.sym("y_pos_dt")
    theta_dt = SX.sym("theta_dt")
    Vf_dt = SX.sym("Vf_dt")
    #psi_dt = SX.sym("psi_dt")
    #s_dt = SX.sym("s_dt")
    x_dot = vertcat(x_pos_dt, y_pos_dt, theta_dt, Vf_dt)

    #input     
    accel = SX.sym("accel")
    yaw_rate = SX.sym("yaw_rate")    
    u = vertcat(accel,yaw_rate)

    #constants
    K1 = mass * lf /((lf + lr) * Cr)
    K2 = lr

    beta = K1 * Vf * yaw_rate + K2 * yaw_rate /Vf 
    
    #system dynamics/kinematics
    f_expl =vertcat(Vf * np.cos(theta + beta),
                    Vf * np.sin(theta + beta),
                    yaw_rate,
                    accel
                    )

    model.f_impl_expr = x_dot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = x_dot
    model.u = u
    model.x0 = np.array([0.0,0,0,0])
    nx = model.x.rows()
    nu = model.u.rows()    
    reference_param = SX.sym('references', (nx + nu), 1) # instead of yaw angle, we are getting quaternions
    model.p = reference_param
    model.name = model_name

    return model