function angle = angle(t)
    % if t<1
    %     angle_steer = 0;%(pi/180) * sin(2*pi*0.1*t);
    % elseif 16>=t>=1
    %     angle_steer = 0.5*(pi/180)*t;
    % else
    %     angle_steer=0.5*(pi/180)*t;
    % end
    angle_steer = (pi/180) * sin(2*pi*0.1*t);
    angle=0.01%%angle_steer;
end 