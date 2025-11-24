function tau = contactForceBalancing(t, s, model, params)
% Modify this code to calculate the joint torques
% t - time
% s - state of the robot
% model - struct containing robot properties
% params - user defined parameters in studentParams.m
% tau - 10x1 vector of joint torques

% State vector components ID
q = s(1 : model.n);
dq = s(model.n+1 : 2*model.n);


%% [Control #3] Contact-Force based balancing
% Mass and Gravity
m = model.M; 
g = 9.81; 

% Position Gains (CoM Regulation)
% disp(params);
% Kp_pos = diag([1600, 1600, 1600]); %params.Kp_pos;
% Kd_pos = diag([200, 200, 200]); %params.Kd_pos;
Kp_pos = diag([2500, 2500, 2500]); %params.Kp_pos;
Kd_pos = diag([250, 250, 250]); %params.Kd_pos;

% Orientation Gains (Trunk Regulation)
% Kp_ori = diag([25, 25, 25]); %params.Kp_ori;
% Kd_ori = diag([10, 10, 10]); %params.Kd_ori;
Kp_ori = diag([25, 25, 25]); %params.Kp_ori;
Kd_ori = diag([5, 5, 5]); %params.Kd_ori;

% Friction coefficient
mu = 0.8;

% Initial state
x0 = getInitialState(model); 

%% Kinematics and State Estimation
[p_com, v_com] = computeComPosVel(q, dq, model);
R_body = rot_z(q(4)) * rot_y(q(5)) * rot_x(q(6));
omega_body = dq(4:6);

%% Stage 1: Virtual Wrench Calculation
% Target State (Regulation Setpoint)
[p_des, v_des] = computeComPosVel(x0(1:model.n), x0(model.n+1:2*model.n), model);

% Linear Error
e_pos = p_com - p_des;
e_vel = v_com - v_des;

% Desired Force at CoM (PD + Gravity Compensation)
f_GA_des = - Kp_pose_pos - Kd_pose_vel + [0; 0; m*g];

% Target Orientation (Identity / Upright)
R_des = rot_z(x0(4)) * rot_y(x0(5)) * rot_x(x0(6)); % R_des = eul2rotm(x0(4:6)');
omega_des = [0; 0; 0];

% Orientation Error
R_error = - cross(R_body(:,1), R_des(:,1)) ...
          - cross(R_body(:,2), R_des(:,2)) ...
          - cross(R_body(:,3), R_des(:,3));
omega_error = omega_body - omega_des;

% Desired Torque at CoM
Tau_GA_des = - Kp_oriR_error - Kd_oriflip(omega_error);

% Total Desired Wrench (6x1)
% f_GA_des = clip(f_GA_des, 0, 1000);
% Tau_GA_des = clip(Tau_GA_des, -100, 100);
F_GA = [f_GA_des; Tau_GA_des];

%% Stage 2: Force Distribution via QP

num_contacts = 4;
num_vars = 3 * num_contacts;
[p1, p2, p3, p4] = computeFootPositions(q, model);
p_feet = [p1, p2, p3, p4];  

% Build Contact Grasp G_C (6 x 12)
G_C = zeros(6, num_vars);
for i=1:num_contacts
    r_op = p_feet(:, i) - p_com; % Vector from CoM to contact point
    R_op = R_body' * eye(3); % Rotation from CoM to contact
    G_C(1:3, 3*(i-1)+1 : 3*i) = R_op;
    G_C(4:6, 3*(i-1)+1 : 3*i) = hat_map(r_op) * R_op;
end

% friction cone constraints
num_sigma = 4 * num_contacts; % 4 faces per contact
n1 = [-1/(sqrt(2)*mu); 0; 1];
n2 = [1/(sqrt(2)*mu); 0; 1]; 
n3 = [0; -1/(sqrt(2)*mu); 1];
n4 = [0; 1/(sqrt(2)*mu); 1]; 
N = [n1, n2, n3, n4];

% Formulate QP
% min_{f_c, sigma} ||f_c||^2 s.t.
% F_GA = G_C * f_c
% f_i (of f_c) = sum sigma_ij * n_j (friction cone)
% f_i_z >= 0
% sigma_ij >= 0

% Cost
H = blkdiag(eye(num_vars), zeros(num_sigma));
f = zeros(num_vars + num_sigma, 1);

% Equality 1 
Aeq1 = [G_C, zeros(6, num_sigma)];
beq1 = F_GA;

% Equality 2
Aeq2 = [eye(num_vars), blkdiag(-N,-N,-N,-N)];
beq2 = zeros(num_vars,1);

Aeq = [Aeq1; Aeq2];
beq = [beq1; beq2];

% Lower and Upper Bounds
lb_f = repmat([-inf; -inf; 0], 4, 1);
lb_sigma = zeros(num_sigma, 1);
lb = [lb_f; lb_sigma];
ub = [];

% 5. Solve QP
% No equality constraints (Aeq, beq) because tracking is now in the cost function
options = optimoptions('quadprog', 'Display', 'off');

[f_opt, fval, exitflag] = quadprog(H, f, [], [], Aeq, beq, lb, ub, [], options);
% disp(size(f_opt));
f_opt = reshape(f_opt(1:num_vars), 3, num_contacts); % Each column corresponds to a contact force

if exitflag < 0
    fprintf("%f, %f \n", t, exitflag);
end

%% 5. Map Forces to Joint Torques
% tau = - Sum (J_i' * f_i)
[J1f, J1b, J2f, J2b] = computeFootJacobians([q; dq], model);
% tau_full = J1f(1:3, :)'*f_opt(:,1) + J1b(1:3, :)'*f_opt(:,2) + ...
%            J2f(1:3, :)'*f_opt(:,3) + J2b(1:3, :)'*f_opt(:,4);
tau_full = J1f(4:6, :)'*f_opt(:,1) + J1b(4:6, :)'*f_opt(:,2) + ...
           J2f(4:6, :)'*f_opt(:,3) + J2b(4:6, :)'*f_opt(:,4);

% Map to the actuated coordinates defined by the model constraints
tau = -tau_full(7:16); 

end