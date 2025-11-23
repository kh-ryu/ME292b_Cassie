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
g = -9.81; % Maybe it should be positive?

% Position Gains (CoM Regulation)
Kp_pos = params.contactForceBalancing.Kp_pos;
Kd_pos = params.contactForceBalancing.Kd_pos;

% Orientation Gains (Trunk Regulation)
Kp_ori = params.contactForceBalancing.Kp_ori;
Kd_ori = params.contactForceBalancing.Kd_ori;

% Friction coefficient
mu = 0.8;

x0 = getInitialState(model);

%% 2. Kinematics and State Estimation
[p_com, v_com] = computeComPosVel(q, dq, model);
R_body = eul2rotm(q(4:6)');
omega_body = baseAngularVelocityZXY(q, dq);

%% 3. Stage 1: Virtual Wrench Calculation (Compliance Control)
% Target State (Regulation Setpoint)
[p_des, v_des] = computeComPosVel(x0(1:model.n), x0(model.n+1:2*model.n), model);

% Linear Error
e_pos = p_com - p_des;
e_vel = v_com - v_des;

% Desired Force at CoM (PD + Gravity Compensation)
f_GA_des = -Kp_pos * e_pos - Kd_pos * e_vel + [0; 0; m*g];

% Target Orientation (Identity / Upright)
R_des = eul2rotm(x0(4:6)');
omega_des = [0; 0; 0];

% Orientation Error
R_db = R_des' * R_body;
q_db = rotm2quat(R_db);  % [w, x, y, z]
delta = q_db(1);
epsilon = q_db(2:4)';

tau_r = -2 * (delta * eye(3) + skew(epsilon)) * Kp_ori * epsilon;

% Desired Torque at CoM
e_omega = omega_body - omega_des;
Tau_GA_des = R_body * (tau_r - Kd_ori * e_omega);

% Total Desired Wrench (6x1)
% f_GA_des = clip(f_GA_des, 0, 1000);
% Tau_GA_des = clip(Tau_GA_des, -100, 100);
F_GA = [f_GA_des; Tau_GA_des];

%% 4. Stage 2: Force Distribution via QP
% Solve for forces at 4 contact points (12 variables)
% f = [f1_x; f1_y; f1_z; ... ; f4_x; f4_y; f4_z]

num_contacts = 4;
num_vars = 3 * num_contacts;

% Build Grasp Matrix G (6 x 12)
G_C = zeros(6, num_vars);
[p1, p2, p3, p4] = computeFootPositions(q, model);
p_feet = [p1, p2, p3, p4];  

for i = 1:num_contacts
    % Position of contact point i relative to CoM
    r_i = p_feet(:, i) - p_com;
    
    % Grasp matrix block for this contact
    % Force part: Identity
    % Torque part: skew(r_i)
    col_idx = (i-1)*3 + 1 : i*3;
    G_C(1:3, col_idx) = eye(3);
    G_C(4:6, col_idx) = skew(r_i);
end

% Split G into Linear (top) and Angular (bottom) parts
G_trans = G_C(1:3, :);
G_rot   = G_C(4:6, :);

% 2. Define Weights (alpha3 << alpha2 << alpha1)
alpha1 = params.contactForceBalancing.alpha1; % Priority: Linear Force (Main balance)
alpha2 = params.contactForceBalancing.alpha2;   % Priority: Angular Torque (Orientation)
alpha3 = params.contactForceBalancing.alpha3;  % Priority: Minimize Forces (Regularization)

% 3. Construct QP Matrices
% Cost J = 0.5 * f' * H * f + f_cost' * f
% Expansion of J_total:
% f' * (a1*Gt'*Gt + a2*Gr'*Gr + a3*I) * f  - 2*(a1*Fd'*Gt + a2*Td'*Gr) * f

H = 2 * (alpha1 * (G_trans' * G_trans) + ...
         alpha2 * (G_rot' * G_rot) + ...
         alpha3 * eye(num_vars));

f_linear_term = -2 * (alpha1 * (G_trans' * f_GA_des) + ...
                      alpha2 * (G_rot' * Tau_GA_des));

% 4. Inequality Constraints: Friction Cones (Hard Constraints)
cone_mat = [ 1  0 -mu;
            -1  0 -mu;
             0  1 -mu;
             0 -1 -mu;
             0  0 -1 ]; % fz >= 0

Aineq = [];
for i = 1:num_contacts
    Aineq = blkdiag(Aineq, cone_mat);
end
bineq = zeros(size(Aineq, 1), 1);

% 5. Solve QP
% No equality constraints (Aeq, beq) because tracking is now in the cost function
options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');

f_opt = quadprog(H, f_linear_term, Aineq, bineq, [], [], [], [], [], options);
f_opt = reshape(f_opt, 3, num_contacts); % Each column corresponds to a contact force

%% 5. Map Forces to Joint Torques
% tau = Sum (J_i' * f_i)
[J1f, J1b, J2f, J2b] = computeFootJacobians([q; dq], model);
tau_full = J1f(1:3, :)'*f_opt(:,1) + J1b(1:3, :)'*f_opt(:,2) + J2f(1:3, :)'*f_opt(:,3) + J2b(1:3, :)'*f_opt(:,4);

% Map to the actuated coordinates defined by the model constraints
tau = tau_full(7:16); 

end

%% Helper Functions

function S = skew(v)
    % Returns skew-symmetric matrix for cross product
    S = [0 -v(3) v(2);
         v(3) 0 -v(1);
         -v(2) v(1) 0];
end

function omega = baseAngularVelocityZXY(q, dq)
    % 1. Extract angles based on your specific order (Yaw, Pitch, Roll)
    psi   = q(4);  % Yaw
    theta = q(5);  % Pitch
    phi   = q(6);  % Roll

    % 2. Extract rates based on the same order
    d_psi   = dq(4);
    d_theta = dq(5);
    d_phi   = dq(6);

    % 3. Define the standard Z-Y-X Transformation Matrix
    % This matrix expects the vector to be [Roll_Rate; Pitch_Rate; Yaw_Rate]
    E = [1,  0,           -sin(theta);
        0,  cos(phi),    cos(theta)*sin(phi);
        0, -sin(phi),    cos(theta)*cos(phi)];

    % 4. Compute Angular Velocity
    % IMPORTANT: We must construct the vector as [d_phi; d_theta; d_psi] 
    % to match the columns of matrix E.
    omega = E * [d_phi; d_theta; d_psi];
end