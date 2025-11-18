function [x0, model] = getInitialState(model)

% Points of contact
model.gc.body(1:2) = model.idx.foot1 ;
model.gc.point(:,1) = model.p1 ;
model.gc.point(:,2) = model.p2 ;
model.gc.body(3:4) = model.idx.foot2 ;
model.gc.point(:,3) = model.p1 ;
model.gc.point(:,4) = model.p2 ;


% define the actuated idx
idx_all = 1:20;
idx_unactuated = [1:6, model.dependent_idx];
idx_all(idx_unactuated) = [];
model.actuated_idx  = idx_all;

x0 = zeros(2*model.NB,1);
x0(model.jidx.hip_abduction_left) = 0 ;
x0(model.jidx.hip_rotation_left) = 0 ;
x0(model.jidx.hip_flexion_left) = 0.5 ;
x0(model.jidx.knee_joint_left) = -1.2 ;
x0(model.jidx.toe_joint_left) = -1.6 ;

x0(model.jidx.hip_abduction_right) = 0 ;
x0(model.jidx.hip_rotation_right) = 0 ;
x0(model.jidx.hip_flexion_right) = 0.5 ;
x0(model.jidx.knee_joint_right) = -1.2 ;
x0(model.jidx.toe_joint_right) = -1.6 ;

% Impose constraints
x0(model.jidx.ankle_joint_left) = deg2rad(13) - x0(model.jidx.knee_joint_left) ;
x0(model.jidx.ankle_joint_right) = deg2rad(13) - x0(model.jidx.knee_joint_right) ;

% correction for z
ground_pos = X_to_r(bodypos(model, model.idx.foot1, x0)) ;
x0(3) = -ground_pos(3) ;


% correction for feet angles
foot_pos = gcPosVel(model, x0, 0*x0) ;
foot_pos_min = min(foot_pos(3,:)) ;
foot_pos_max = max(foot_pos(3,:)) ;
x0(3) = x0(3) - foot_pos_min ;


% Initialisation for ground model
x0((2*model.NB+1) : (2*model.NB + 2*length(model.gc.body))) = 0 ;

end