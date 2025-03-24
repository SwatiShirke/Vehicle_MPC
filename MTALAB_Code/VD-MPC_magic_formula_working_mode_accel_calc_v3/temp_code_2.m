% Magic Formula coefficients (example values, modify based on your tire data)
B = 10;  % Stiffness factor
C = 1.9; % Shape factor
D = 1;   % Peak factor (maximum tire force)
E = 0.97;% Curvature factor

%B=11.275; C=1.56; D=1; E=-0.97;
% Vehicle and tire parameters
R = 0.3; % Radius of the tire in meters
Fz = 4000; % Normal load on the tire in Newtons (modify as per your vehicle's weight distribution)

% Slip ratio range (from -1 to 1)
alpha = linspace(-1, 1, 100);

% Calculate the tire force using the Magic Formula
f =  D * sin(C * atan(B * alpha - E * (B * alpha - atan(B * alpha))))

tire_force=f.*Fz;
% Plot the tire force vs slip ratio
figure;
plot(Slips, tire_force, 'LineWidth', 2);
xlabel('Slip Ratio \kappa');
ylabel('Longitudinal Tire Force F_x (N)');
title('Longitudinal Tire Force vs Slip Ratio using Magic Formula');
grid on;
