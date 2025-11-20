function tau = studentController(t, s, model, params)
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
g = -9.81;

% Position Gains (CoM Regulation)
Kp_pos = diag([1000, 1000, 2000]);
Kd_pos = diag([100, 100, 200]);

% Orientation Gains (Trunk Regulation)
Kp_ori = diag([1000, 1000, 1000]); 
Kd_ori = diag([100, 100, 100]);

% Friction coefficient
mu = 0.8;

% Target State (Regulation Setpoint)
x0 = getInitialState(model);
x_des = x0(1:3); 
v_des = [0; 0; 0];

% Target Orientation (Identity / Upright)
R_des = eul2rotm(x0(4:6)'); 
omega_des = [0; 0; 0];

%% 2. Kinematics and State Estimation
[p_com, v_com] = computeComPosVel(q, dq, model);
R_body = eul2rotm(q(4:6)');
omega_body = dq(4:6);

% Contact positions (relative to world)
[p1, p2, p3, p4] = computeFootPositions(q, model);
p_feet = [p1, p2, p3, p4];  
[J1f, J1b, J2f, J2b] = computeFootJacobians([q; dq], model);

%% 3. Stage 1: Virtual Wrench Calculation (Compliance Control)

% Linear Error
e_pos = p_com - x_des;
e_vel = v_com - v_des;

% Desired Force at CoM (PD + Gravity Compensation)
f_GA_des = -Kp_pos * e_pos - Kd_pos * e_vel + [0; 0; m*g];

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
F_GA = [f_GA_des; Tau_GA_des];

%% 4. Stage 2: Force Distribution via QP
% Solve for forces at 4 contact points (12 variables)
% f = [f1_x; f1_y; f1_z; ... ; f4_x; f4_y; f4_z]

num_contacts = 4;
num_vars = 3 * num_contacts;

% Build Grasp Matrix G (6 x 12)
G_C = zeros(6, num_vars);

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
alpha1 = 100.0; % Priority: Linear Force (Main balance)
alpha2 = 1.0;   % Priority: Angular Torque (Orientation)
alpha3 = 0.01;  % Priority: Minimize Forces (Regularization)

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

% S1 = [eye(3), zeros(3)]; % Selects Forces
% S2 = [zeros(3), eye(3)]; % Selects Torques
% 
% A1 = S1 * G_C; b1 = S1 * F_GA;
% A2 = S2 * G_C; b2 = S2 * F_GA;
% 
% alpha1 = 100.0; % Priority: Linear Force
% alpha2 = 1.0;   % Priority: Torque
% alpha3 = 0.01;  % Regularization
% 
% % Aggregate Quadratic Form: J(f) = 0.5 * f' * H_total * f + g_total' * f + const
% % Note: The norm ||Ax-b||^2 = x'A'Ax - 2b'Ax + b'b
% % We multiply by 2 in H because derivative of x'Qx is 2Qx, but IPOPT expects Hessian of objective.
% 
% % Hessian of J (Constant)
% H_total = 2 * (alpha1 * (A1' * A1) + ...
%                alpha2 * (A2' * A2) + ...
%                alpha3 * eye(num_vars));
% 
% % Gradient Vector (Constant part)
% g_total_vec = -2 * (alpha1 * (b1' * A1) + ...
%                     alpha2 * (b2' * A2))';
% 
% % 4c. IPOPT Setup
% 
% % Initial guess (Warm start from previous step would be better, using zeros for safety)
% f0 = zeros(num_vars, 1);
% 
% % Variable Bounds (lb <= x <= ub)
% % Enforce fz >= 0 here.
% max_force = 2500;
% lb = -max_force * ones(num_vars, 1);
% ub =  max_force * ones(num_vars, 1);
% % Set fz lower bounds to 0
% for i = 1:num_contacts
%     lb(i*3) = 0; 
% end
% 
% % Constraint Bounds (gl <= g(x) <= gu)
% % Constraints: f_x^2 + f_y^2 - mu^2 * f_z^2 <= 0
% % 4 constraints (one per foot)
% cl = -1e19 * ones(4, 1); % Effectively -infinity
% cu = zeros(4, 1);        % Upper bound 0
% 
% % Aux structure to pass data to callbacks
% auxdata.H = H_total;
% auxdata.g = g_total_vec;
% auxdata.mu = mu;
% 
% % Define IPOPT callbacks
% funcs.objective         = @(x) objective_func(x, auxdata);
% funcs.gradient          = @(x) gradient_func(x, auxdata);
% funcs.constraints       = @(x) constraints_func(x, auxdata);
% funcs.jacobian          = @(x) jacobian_func(x, auxdata);
% funcs.jacobianstructure = @() jacobian_structure(num_vars);
% funcs.hessian           = @(x, sigma, lambda) hessian_func(x, sigma, lambda, auxdata);
% funcs.hessianstructure  = @() hessian_structure(num_vars);
% 
% options.ipopt.print_level = 0; % 0-12
% options.ipopt.max_iter = 50;
% options.ipopt.tol = 1e-4;
% options.ipopt.hessian_approximation = 'exact';
% 
% try
%     [f_opt, info] = ipopt(f0, funcs, options);
% 
%     % Fallback if not solved optimally
%     if info.status ~= 0 && info.status ~= 1
%         % Check if acceptable
%     end
% catch
%     % Fallback in case IPOPT mex is missing or fails
%     f_opt = zeros(num_vars, 1);
% end

f_opt
%% 5. Map Forces to Joint Torques
% tau = Sum (J_i' * f_i)

tau_full = J1f(1:3,:)'*f_opt(1:3) + J1b(1:3,:)'*f_opt(4:6) + J2f(1:3,:)'*f_opt(7:9) + J2b(1:3,:)'*f_opt(10:12);

% Select only actuated joints (Indices 7-16)
tau = tau_full(7:16); 
tau
end

%% Helper Functions

function S = skew(v)
    % Returns skew-symmetric matrix for cross product
    S = [0 -v(3) v(2);
         v(3) 0 -v(1);
         -v(2) v(1) 0];
end