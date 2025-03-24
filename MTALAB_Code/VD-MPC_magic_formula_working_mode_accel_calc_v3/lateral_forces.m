function [fy_f, fy_r, fy_del] = lateral_forces(params, z, tire_forces,delta)
    f_vector = tire_forces;
    Fx_fl = f_vector(1);
    Fx_fr = f_vector(2);
    Fx_rl = f_vector(3);
    Fx_rr = f_vector(4);

    Fy_fl = f_vector(5);
    Fy_fr = f_vector(6);
    Fy_rl = f_vector(7);
    Fy_rr = f_vector(8);
          
    fy_f =  ((Fx_fr + Fx_fl) * sin(delta) +  (Fy_fr + Fy_fl) * cos(delta));
          
    fy_r =  (Fy_rl + Fy_rr);

    fy_del = params.tf/2 * (Fx_fr - Fx_fl) * cos(delta)  + params.tr * (Fx_rr - Fx_rl) + params.tf/2 * (Fy_fr - Fy_fl)* sin(delta);  
    
    %sum = fy_f - fy_r; %+ fy_del;
end