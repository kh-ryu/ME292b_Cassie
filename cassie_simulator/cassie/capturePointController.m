function tau = capturePointController(t, s, model, params)
% capturePointController: calculates joint torques for Cassie balance control
% t - time
% s - state of the robot (q, dq)
% model - struct containing robot properties
% params - user defined parameters
% tau - 10x1 vector of joint torques

% State vector components
q = s(1 : model.n);
dq = s(model.n+1 : 2*model.n);

[r_com, v_com] = computeComPosVel(q, dq, model);

[p_foot_L_front, p_foot_L_back, p_foot_R_front, p_foot_R_back] = computeFootPositions(q, model);
support_polygon_center = (p_foot_L_front + p_foot_L_back + p_foot_R_front + p_foot_R_back) / 4;
xc_desired = support_polygon_center(1:2);


g_vec = [0; 0; -9.81]; 
xc = capturePoint(r_com, v_com, g_vec);

xc_error = xc - xc_desired;

K_xc_hip = -0.1; 
K_xc_toe = -0.05;

delta_q_hip_pitch = K_xc_hip * xc_error(1);   % Sagittal plane correction
delta_q_toe_pitch = K_xc_toe * xc_error(1);   % Sagittal plane correction
delta_q_hip_roll = K_xc_hip * xc_error(2);    % Frontal plane correction

kp = 500;
kd = 50;
x0 = getInitialState(model);
q0 = x0(1:model.n);

q_desired = q0;

% Apply corrections for sagittal plane (forward/backward balance)
q_desired(model.jidx.hip_flexion_left) = q_desired(model.jidx.hip_flexion_left) + delta_q_hip_pitch;
q_desired(model.jidx.hip_flexion_right) = q_desired(model.jidx.hip_flexion_right) + delta_q_hip_pitch;
q_desired(model.jidx.toe_joint_left) = q_desired(model.jidx.toe_joint_left) + delta_q_toe_pitch;
q_desired(model.jidx.toe_joint_right) = q_desired(model.jidx.toe_joint_right) + delta_q_toe_pitch;

% Apply corrections for frontal plane (left/right balance)
q_desired(model.jidx.hip_abduction_left) = q_desired(model.jidx.hip_abduction_left) + delta_q_hip_roll;
q_desired(model.jidx.hip_abduction_right) = q_desired(model.jidx.hip_abduction_right) - delta_q_hip_roll; % Opposite for right leg

% PD control law
tau = -kp * (q(model.actuated_idx) - q_desired(model.actuated_idx)) - kd * dq(model.actuated_idx);

end

function xc = capturePoint(r_com, v_com, g)
% capturePoint: calculates the capture point for the LIPM
% r_com: 3x1 vector of center of mass position
% v_com: 3x1 vector of center of mass velocity
% g: 3x1 vector of gravity

if nargin < 3
    g = [0; 0; -9.81];
end

g_mag = norm(g);
z_com = r_com(3);

% Avoid division by zero or negative z_com
if z_com <= 0
    % Return a safe value (e.g., current CoM horizontal position)
    xc = r_com(1:2);
    return;
end

omega_0 = sqrt(g_mag / z_com);

xc = r_com(1:2) + v_com(1:2) / omega_0;

end