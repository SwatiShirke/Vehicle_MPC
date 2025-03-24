Ts = 1;
nx = 10;
ny = 1;
nu = 3;
nlobj = nlmpc(nx,ny,nu);
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 4;
nlobj.Model.IsContinuousTime = true;
nlobj.Model.StateFcn = @(x,u) vehicle_dynamics(x,u);
nlobj.Model.OutputFcn = @(x,u) [x(1)];
nlobj.Weights.OutputVariables =  [0.9, 0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]'; 
nlobj.Weights.ManipulatedVariablesRate = [0.01,0.01,0.01];
nlobj.OV(1).Min = 0;
nlobj.OV(1).Max = 30;
nlobj.ManipulatedVariables(1).Min = 0;    % Min acceleration
nlobj.ManipulatedVariables(1).Max = 1;     % Max acceleration
nlobj.ManipulatedVariables(2).Min = -1; % Min steering angle
nlobj.ManipulatedVariables(2).Max = 0;  % Max steering angle
nlobj.ManipulatedVariables(3).Min = -2*pi; % Min steering angle
nlobj.ManipulatedVariables(3).Max = 2*pi; 

z_init= zeros(10,1);
t_span = [0 1]; %linspace(0,1,2);
u = [0; 0; 0 ];
yref = 10;
[ t, z ] = ode45(@(t,z) vehicle_control(t ,z, u, nlobj, yref), t_span , z_init);
vel_x = z(:,4);
vel_y = z(:, 5);
plot(  vel_x,vel_y , 'b-');
xlabel('time in second');
ylabel('Vx in m/s');
title('longitudinal velocity');
grid on;
axis equal;
