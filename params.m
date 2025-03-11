clc; clear;

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
    % Length: 9.4″.
    % Pitch: 4.3″.
    % Weight: 24 gm.
    % Shaft Diameter: 8 mm.
    % Total Length: 9.4 inch / 225 mm
l_prop = 0.225; %m
% m_prop = 0.024; %kg
w_prop = l_prop/5;
h_prop = 0.010;
v_prop = l_prop * w_prop * h_prop;

prop_torque_cw = -2000; %N.m
prop_torque_ccw = 1500;

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

%% Calculating Inertia (Neglect value below: Incorrect)

I_prop_propframe = diag([7.22115e-05, 0.00172373, 0.00178915]);
I_motor_motorframe = diag([0.000112917, 0.000112917, 6.25e-05]);
I_arm_armframe = diag([0.000529727, 0.000529727, 5.06109e-06]);
I_base = diag([0.00626046, 0.00626046, 0.0123562]);

skew_form = @(d) [0, -d(3) d(2); d(3), 0, -d(1); -d(2), d(1), 0];
do_parallel_axis = @(I_x, m_x, d) I_x + m_x * (d' * d * eye(3) - skew_form(d) * d');

I_arms_baseframe = zeros(3, 3);
I_motors_baseframe = zeros(3, 3);
I_props_baseframe = zeros(3, 3);

for i = 1:n_prop

    R = arm_pos(i).rot;

    d_armframe_baseframe = arm_pos(i).pos + R'*[l_arm/2; 0; 0];
    I_arm_baseframe = do_parallel_axis(R*I_arm_armframe*R', m_arm, d_armframe_baseframe);
    I_arms_baseframe = I_arms_baseframe + I_arm_baseframe;

    d_motorframe_baseframe = arm_pos(i).pos + R'*[l_arm; 0; 0];
    I_motor_baseframe = do_parallel_axis(R*I_motor_motorframe*R', m_motor, d_motorframe_baseframe);
    I_motors_baseframe = I_motors_baseframe + I_motor_baseframe;

    d_propframe_baseframe = arm_pos(i).pos + R'*[l_arm; 0; 0];
    I_prop_baseframe = do_parallel_axis(R*I_prop_propframe*R', m_prop, d_propframe_baseframe);
    I_props_baseframe = I_props_baseframe + I_prop_baseframe;
end

I = I_base + I_motor_baseframe + I_arm_baseframe + I_motor_baseframe