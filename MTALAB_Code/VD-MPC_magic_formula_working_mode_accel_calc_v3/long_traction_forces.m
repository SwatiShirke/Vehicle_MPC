function f_traction = long_traction_forces( z, tire_forces, delta)
    
    f_vector = tire_forces;
    Fx_fl = f_vector(1);
    Fx_fr = f_vector(2);
    Fx_rl = f_vector(3);
    Fx_rr = f_vector(4);

    Fy_fl = f_vector(5);
    Fy_fr = f_vector(6);
    Fy_rl = f_vector(7);
    Fy_rr = f_vector(8);
    %temp_delta_value = (Fy_fr + Fy_fl) * sin(delta)
     
    f_traction = ((Fx_fr + Fx_fl) * cos(delta) ) - ((Fy_fr + Fy_fl) * sin(delta) ) + (Fx_rl + Fx_rr);
     
    
end