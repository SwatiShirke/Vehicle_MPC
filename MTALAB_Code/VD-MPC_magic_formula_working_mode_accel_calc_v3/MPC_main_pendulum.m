%velocity control
Ts = 0.1;
nx =4 ;
ny =2;
nu = 1;
nlobj = nlmpc(nx,ny,nu);
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 4;
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 1;

nlobj.Model.StateFcn =  "pendulumDT0"; %@(x,u, Ts) vehicle_dynamics_dt(x,u, Ts);
nlobj.Model.OutputFcn = @(x,u,Ts) [x(1); x(3)]; %@(x,u,Ts) [x(1)];
% nlobj.Weights.OutputVariables =  [0.9, 0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]'; 
% nlobj.Weights.ManipulatedVariablesRate = [0.01,0.01,0.01];
% nlobj.OV(1).Min = 0;
% nlobj.OV(1).Max = 30;
% nlobj.ManipulatedVariables(1).Min = 0;    % Min acceleration
% nlobj.ManipulatedVariables(1).Max = 1;     % Max acceleration
% nlobj.ManipulatedVariables(2).Min = -1; % Min steering angle
% nlobj.ManipulatedVariables(2).Max = 0;  % Max steering angle
% nlobj.ManipulatedVariables(3).Min = -2*pi; % Min steering angle
% nlobj.ManipulatedVariables(3).Max = 2*pi; 

nlobj.Weights.OutputVariables = [3 3];
nlobj.Weights.ManipulatedVariablesRate = 0.1;
nlobj.OV(1).Min = -10;
nlobj.OV(1).Max = 10;
nlobj.MV.Min = -100;
nlobj.MV.Max = 100;
z0 = [0.1;0.2;-pi/2;0.3];
u0 = 0.4
yref = [0 0];

%%init
% z0 = zeros(10,1);
% u0 = [0,0,0];
% z_state = z0;
% u = u0;
% %params.Ts = Ts;
% nloptions = nlmpcmoveopt;
% nloptions.Parameters = {Ts};
% validateFcns(nlobj,z0,u0,[], {Ts});
% %yref= 10; %ones(nlobj.PredictionHorizon, 1)* 10
% x_vel = [];
% t_list = [];
% % [x, info] = nlmpcmove(nlobj, z_state, u, yref, []);
% % u
% sim_T = 1;
% z_history = [];
% for i = 1:sim_T / Ts 
%     % Solve NMPC for the current state z0    
%     [u, info] = nlmpcmove(nlobj, z_state, u, yref, [], nloptions  );
%     %t_span = linspace(0, Ts, 10);
%     u
%     % z_init = z_state;
%     % %Simulate system to get next state #ode45
%     % [ t, z ] = ode45(@(t,z) vehicle_dynamics(z, u), t_span , z_init);
%     % z_state = z(end, :);
%     % z_history = [z_history z_state];
%     % t_list = [t_list i];
%     %ode 
%     z_state = pendulumDT0(z_state, u, Ts);
%     %z_state = vehicle_dynamics_dt(z_state, u, Ts);
%     % Nd = 10;
%     % delta = Ts/Nd;
%     % for ct=1:Nd
%     %     z_state = z_state + delta* vehicle_dynamics(z_state,u);
%     % end
%     z_history = [z_history z_state(1)];
%     t_list = [t_list i];
% end
% disp("a")
% plot(  t_list, z_history, 'b-');
% xlabel('time in second');
% ylabel('Vx in m/s');
% title('longitudinal velocity');
% grid on;
% axis equal;



