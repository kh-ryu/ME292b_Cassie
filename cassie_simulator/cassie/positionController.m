function tau = positionController(t, s, model, params)
% Modify this code to calculate the joint torques
% t - time
% s - state of the robot
% model - struct containing robot properties
% params - user defined parameters in studentParams.m
% tau - 10x1 vector of joint torques

% State vector components ID
q = s(1 : model.n);
dq = s(model.n+1 : 2*model.n);

%% [Control #1] zero control
% tau = zeros(10,1);

%% [Control #2] High Gain Joint PD control on all actuated joints
kp = params.positionController.kp ;
kd = params.positionController.kd ;
x0 = getInitialState(model);
q0 = x0(1:model.n) ;
dq0 = zeros(size(dq), like=dq);
tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*(dq(model.actuated_idx) - dq0(model.actuated_idx));

%% [Control #3] Position control based balancing
% roll control
e_roll  = q(6)  - q0(6);
ed_roll = dq(6) - 0;

Kp_roll = params.positionController.Kp_roll; 
Kd_roll = params.positionController.Kd_roll; 

T_roll = Kp_roll*e_roll + Kd_roll*ed_roll;

% If roll is positive (tilt left), push left hip up, right hip down:
idxL_hipAbd = 1;
idxR_hipAbd = 2;

tau(idxL_hipAbd) = tau(idxL_hipAbd) - T_roll;
tau(idxR_hipAbd) = tau(idxR_hipAbd) + T_roll;

% If roll is positive (tilt left), push left knee up, right knee down:
idxL_knee = 7;
idxR_knee = 8;

tau(idxL_knee) = tau(idxL_knee) - T_roll;
tau(idxR_knee) = tau(idxR_knee) + T_roll;

% x control
e_x  = q(1)  - q0(1);
ed_x = dq(1) - 0;

Kp_pitch = params.positionController.Kp_pitch; 
Kd_pitch = params.positionController.Kd_pitch; 

T_t = Kp_pitch*e_x + Kd_pitch*ed_x;

idxL_toe = 9;
idxR_toe = 10;

% If e_x is positive (leaning forward), increased toe joint:
tau(idxL_toe) = tau(idxL_toe) - T_t;
tau(idxR_toe) = tau(idxR_toe) - T_t;

% Height control
e_z  = q(3)  - q0(3);    % torso height
ed_z = dq(3) - 0;

Kp_z = params.positionController.Kp_z; 
Kd_z = params.positionController.Kd_z; 

T_z = Kp_z*e_z + Kd_z*ed_z;

idxL_knee = 7;
idxR_knee = 8;

% Get shorter if z is high, longer if z is low:
tau(idxL_knee) = tau(idxL_knee) - T_z;
tau(idxR_knee) = tau(idxR_knee) - T_z;
