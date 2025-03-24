%velocity control
Ts = 0.1;
nx =10;
ny =1;
nu =3;
nlobj = nlmpc(nx,ny,nu);
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 4;
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 1;
nlobj.Model.StateFcn =  @(x,u, Ts) vehicle_dynamics_dt(x,u, Ts);
nlobj.Model.OutputFcn = @(x,u,Ts) [x(1)];
nlobj.Weights.OutputVariables =  1; 
nlobj.Weights.ManipulatedVariablesRate = [0.1,0.1,0.1];
nlobj.OV(1).Min = 0;
nlobj.OV(1).Max = 30;
nlobj.ManipulatedVariables(1).Min = 0;    % Min acceleration
nlobj.ManipulatedVariables(1).Max = 1;     % Max acceleration
nlobj.ManipulatedVariables(2).Min = 0; % Min steering angle
nlobj.ManipulatedVariables(2).Max = 1;  % Max steering angle
nlobj.ManipulatedVariables(3).Min = -2*pi; % Min steering angle
nlobj.ManipulatedVariables(3).Max = 2*pi; 
nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};
%open_system('VD_mpc');  % Make sure the model is loaded
%createParameterBus(nlobj,'VD_mpc/Nonlinear MPC Controller','myBusObject',{Ts});

%%init
z0 = zeros(10,1);
z0(1) = 30; 
u0 = [0,0,0];
z_state = z0;
u = u0;
validateFcns(nlobj,z0,u0,[], {Ts});
yref = 31;
%yref= 10; %ones(nlobj.PredictionHorizon, 1)* 10
x_vel = [];
t_list = [];

sim_T = 5;
z_history = [];
for i = 1:sim_T / Ts 
    % Solve NMPC for the current state z0    
    [u1, info] = nlmpcmove(nlobj, z_state, u, yref, [], nloptions  );
    info;
    %t_span = linspace(0, Ts, 10);
    u1
    % z_init = z_state;
    % %Simulate system to get next state #ode45
    % [ t, z ] = ode45(@(t,z) vehicle_dynamics(z, u), t_span , z_init);
    % z_state = z(end, :);
    % z_history = [z_history z_state];
    % t_list = [t_list i];
    %ode 
    %z_state = pendulumDT0(z_state, u, Ts);
    z_state = vehicle_dynamics_dt(z_state, u1, Ts);
    z_history = [z_history z_state(1)];
    t_list = [t_list i];
end

disp("a")
plot(  t_list, z_history, 'b-');
xlabel('time in second');
ylabel('Vx in m/s');
title('longitudinal velocity');
grid on;
axis equal;



