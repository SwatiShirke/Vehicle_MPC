function [Fx_sum, Fy_f, Fy_r,Fy_del] = vehicle_forces(params, z, t_forces, delta)
     
    Fx_sum = longitudinal_forces(params, z, t_forces, delta);
    [Fy_f, Fy_r,Fy_del] = lateral_forces(params, z, t_forces,delta);

end