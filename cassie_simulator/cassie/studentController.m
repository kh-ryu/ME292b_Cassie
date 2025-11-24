function tau = studentController(t, s, model, params)
% Modify this code to calculate the joint torques
% t - time
% s - state of the robot
% model - struct containing robot properties
% params - user defined parameters in studentParams.m
% tau - 10x1 vector of joint torques

% State vector components ID
params = studentParams(model);
tau = capturePointController(t, s, model, params);
% tau = positionController(t, s, model, params);
% tau = contactForceBalancing(t, s, model, params);
tau = contactForceBalancing_SY(t, s, model, params);
% tau = momentumBasedBalancing(t, s, model, params);