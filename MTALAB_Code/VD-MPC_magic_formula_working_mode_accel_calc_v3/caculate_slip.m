function slips = caculate_slip(params, z ,deccel)
    %wheel longitudinal velocity and angular rate
    Vx = z(1);
    r = z(3);
    %%calculate linear velocities of each wheel 
       V_fl = Vx - (r * params.tf /2);
       V_fr = Vx + (r * params.tf /2);
       V_rl = Vx - (r * params.tr /2);
       V_rr = Vx + (r * params.tr /2);
       w_fl = z(7);
       w_fr = z(8);
       w_rl = z(9);
       w_rr = z(10); 
      
      if deccel > 0
        D1 = V_fl; 
        D2 = V_fr;
        D3 = V_rl;
        D4 = V_rr;
      else
        D1 = params.R_f * w_fl;
        D2 = params.R_f * w_fr;
        D3 = params.R_r * w_rl;
        D4 = params.R_r * w_rr;
      end
    
     S_fl = (params.R_f * w_fl - V_fl )/ D1; if (w_fl ==0), S_fl = 0; end 
     S_fr = (params.R_f * w_fr - V_fr) / D2; if (w_fr ==0), S_fr = 0; end
     S_rl = (params.R_r * w_rl - V_rl) / D3; if (w_rl ==0), S_rl = 0; end
     S_rr = (params.R_r * w_rr - V_rr )/ D4; if (w_rr ==0), S_rr = 0; end
  
     slips = [S_fl; S_fr; S_rl; S_rr];

end