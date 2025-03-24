function torque_in = cal_wheel_torque_in(params, accel, deccel)
    if deccel > 0
        accel_cmd = -deccel;
    else
        accel_cmd = accel;
    end
    
        t_generated = params.max_torque * accel_cmd ; 
        split_ratio = [0.5, 0.5, 0.0, 0.0];
        
        torque_in = split_ratio * t_generated';


    
end
