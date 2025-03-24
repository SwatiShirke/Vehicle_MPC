function z = vehicle_dynamics_dt(z, u, Ts)
    Ts;
    Nd = 10;
    delta = Ts/Nd;
    for ct=1:Nd
        z = z + delta* vehicle_dynamics(z,u);
    end
    %z
    % t_span = [0 Ts];
    % z_init = z;
    % [ t, z ] = ode15s(@(t,z) vehicle_dynamics(t,z, u), t_span , z_init);
    % z = z(end,:);

end