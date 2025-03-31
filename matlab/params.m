clc; clear;

%% Mass Paper
m_mid_paper = (550 + 740 + 158 + 50 + 16 + 50)*10^-3; % Chasis + battery + Controller + (electronics)
m_arm_paper = (110 + 50)*10^-3; % Part of chasis weight + ESC
m_motors_paper = (75+13)*10^-3; % Flying and Tilting Rotor
m_prop_paper = (24)*10^-3;
m_tot_paper = m_mid_paper + m_arm_paper*6 + m_motors_paper*6 + m_prop_paper*6; %3.1960

%% General Specs
mass = 3.2;
n_prop = 6;
DC_Motor_voltage=0;

% Centre Cylinder
r_ctr_cyl = 0.142/2; % For Frame Size of 600mm (600 - (2*L_arm) = 142mm)
l_ctr_cyl = 0.05;
v_ctr_cyl = pi * r_ctr_cyl^2 * l_ctr_cyl;

% Arms
r_arm = 0.02;
l_arm = 0.229; % For 9 inch propeller -> L_arm = 229 mm
v_arm = pi * r_arm^2 * l_arm;

% Motors
r_motor = 0.02;
l_motor = 0.02;
v_motor = pi * r_motor^2 * l_motor;

% Propeller - Modeled as a cuboid
l_prop = 0.225; %m
% m_prop = 0.024; %kg
w_prop = l_prop/5;
h_prop = 0.010;
v_prop = l_prop * w_prop * h_prop;

t_cw = -2000; %N.m
t_ccw = 2000;
torque = [t_ccw; t_cw; t_ccw; t_cw; t_ccw; t_cw];

% Position of arms

rot_z = @(ang) [cosd(ang) -sind(ang) 0; sind(ang), cosd(ang), 0; 0, 0, 1];

x_pos_arm = zeros(1, n_prop);
y_pos_arm = zeros(1, n_prop);

arm_pos = struct();

for i = 1:n_prop
    arm_pos(i).pos = [r_ctr_cyl * cosd(60*(i-1)), r_ctr_cyl * sind(60*(i-1)), 0];
    arm_pos(i).rot = rot_z(60*(i-1));
end

% Compute Densities
volume = v_ctr_cyl + n_prop* (v_arm + v_motor + v_prop); 

density = mass / volume;

m_ctr_cyl = density * v_ctr_cyl;
m_arm = density * v_arm;
m_motor = density * v_motor;
m_prop = density * v_prop;


%% Transformation Matrices 

% % Transformation Matrices between Arm relative to body
% % Given positions and rotation matrices
% % Number of arms (assuming arm_pos is already defined)
% n_arms = length(arm_pos);
% 
% % Initialize transformation matrices
% T_arm_rel_body = zeros(4, 4, n_arms);
% 
% for i = 1:n_arms
%     % Construct homogeneous transformation matrix from existing struct data
%     T = [arm_pos(i).rot, arm_pos(i).pos'; 
%          0 0 0 1];
%     T_arm_rel_body(:,:,i) = T;
% 
%     % Display transformation matrix
%     fprintf('Transformation matrix for arm %d:\n', i);
%     disp(T);
% end
% 
% % Transformation between Arm Start Frame and Propeller R frame
% R_prop_rel_arm = eye(3,3); % Same Rotation
% P_prop_rel_arm = [0.229;0;0.035]; % in +x we got the 0.229m arm length and in +z is half the motor puck length 0.035 meters
% T_prop_rel_arm_start = [R_prop_rel_arm, P_prop_rel_arm; 
%          0 0 0 1];
% 
% % Transformation Propeller Relative to body
% T_b_p1 = T_arm_rel_body(:, :, 1) * T_prop_rel_arm_start;
% T_b_p2 = T_arm_rel_body(:, :, 2) * T_prop_rel_arm_start;
% T_b_p3 = T_arm_rel_body(:, :, 3) * T_prop_rel_arm_start;
% T_b_p4 = T_arm_rel_body(:, :, 4) * T_prop_rel_arm_start;
% T_b_p5 = T_arm_rel_body(:, :, 5) * T_prop_rel_arm_start;
% T_b_p6 = T_arm_rel_body(:, :, 6) * T_prop_rel_arm_start;
% 
% fprintf('Transformation matrix for propeller 1 relative to body :\n');
% disp(T_b_p1);
% 
% fprintf('Transformation matrix for propeller 2 relative to body :\n');
% disp(T_b_p2);
% 
% fprintf('Transformation matrix for propeller 3 relative to body :\n');
% disp(T_b_p3);
% 
% fprintf('Transformation matrix for propeller 4 relative to body :\n');
% disp(T_b_p4);
% 
% fprintf('Transformation matrix for propeller 5 relative to body :\n');
% disp(T_b_p5);
% 
% fprintf('Transformation matrix for propeller 6 relative to body :\n');
% disp(T_b_p6);