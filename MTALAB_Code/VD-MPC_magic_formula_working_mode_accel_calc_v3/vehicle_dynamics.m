%This code is for state space of vehicle dynamics
% states: Vx, Vy, r

function dz = vehicle_dynamics(t, z, u)
    % abc = Forces
    % dt;
    %tic;    
    accel = u(1);
    deccel = u(2);
    delta = u(3);
    params = system_parameters();
    Vx =  z(1);
    Vy = z(2);
    r  = z(3);
    x  = z(4);
    y  = z(5);
    psi= z(6);
    ax = -z(2) * r;
    ay = z(1) * r;

    %%input torue to each wheel 
    tau_in = cal_wheel_torque_in(params, accel, deccel);
    tau_in_fl = tau_in(1);
    tau_in_fr = tau_in(2);
    tau_in_rl = tau_in(3);
    tau_in_rr = tau_in(4);
    
    %% calculate forces
    t_forces = tire_forces(params, z, delta, ax, ay, deccel); %8 * 1 vector
    [Fx_sum, Fy_f, Fy_r, Fy_del] = vehicle_forces(params, z, t_forces, delta);
        
    Vx_dt = Vy * r + (Fx_sum / params.m);
    Vy_dt = -Vx * r + ((Fy_f+ Fy_r) / params.m);

    t_forces = tire_forces(params, z, delta, Vx_dt, Vy_dt,deccel); %8 * 1 vector
    [Fx_sum, Fy_f, Fy_r, Fy_del] = vehicle_forces(params, z, t_forces, delta);


    Vx_dt = Vy * r + (Fx_sum / params.m);
    Vy_dt = -Vx * r + ((Fy_f+ Fy_r) / params.m);
    r_dt =  (params.a * Fy_f - params.b *Fy_r + Fy_del) / params.Izz_v;

    x_dt = Vx * cos(psi) - Vy * sin(psi);
    y_dt = Vx * sin(psi) + Vy * cos(psi);
    psi_dt  = r;    

    %set tire forces 
    Fx_fl = t_forces(1);
    Fx_fr = t_forces(2);
    Fx_rl = t_forces(3);
    Fx_rr = t_forces(4); 

    %wheel velocities 
    w_fl_dt = (tau_in_fl - Fx_fl * params.R_f) / params.Iyy_w_f;
    w_fr_dt = (tau_in_fr - Fx_fr * params.R_f) / params.Iyy_w_f;
    w_rl_dt = (tau_in_rl - Fx_rl * params.R_r) / params.Iyy_w_r;
    w_rr_dt = (tau_in_rr - Fx_rr * params.R_r) / params.Iyy_w_r;

    omega_fl_dt = w_fl_dt;
    omega_fr_dt = w_fr_dt;
    omega_rl_dt = w_rl_dt;
    omega_rr_dt = w_rr_dt;
    %tp = toc;
    dz = [Vx_dt; Vy_dt; r_dt; x_dt; y_dt; psi_dt; omega_fl_dt; omega_fr_dt; omega_rl_dt; omega_rr_dt];

    % ax = (Vx_dt -ax)/dt;
    % ay = (Vy_dt - ay)/dt;
    % 
    %dz = [Vx_dt; Vy_dt; r_dt; x_dt; y_dt; psi_dt; omega_fl_dt; omega_fr_dt; omega_rl_dt; omega_rr_dt; ax; ay];


end 


