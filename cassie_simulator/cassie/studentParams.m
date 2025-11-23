function params = studentParams(model)
% define any parameters here 
% params - struct

params = struct();


%% positionController tunable gains

params.positionController.kp = 500 ;
params.positionController.kd = 100 ;

params.positionController.Kp_roll = 5000 ;
params.positionController.Kd_roll = 500 ;

params.positionController.Kp_pitch = 3000 ;
params.positionController.Kd_pitch = 300 ;

params.positionController.Kp_z = 30000 ;
params.positionController.Kd_z = 3000 ;


%% contactForceBalancing tunable gains

params.contactForceBalancing.Kp_pos = diag([500, 500, 1000]);
params.contactForceBalancing.Kd_pos = diag([100, 100, 400]);

params.contactForceBalancing.Kp_ori = diag([200, 200, 200]);
params.contactForceBalancing.Kd_ori = diag([50, 50, 50]);

% alpha3 << alpha2 << alpha1
params.contactForceBalancing.alpha1 = 100.0 ; % Priority: Linear Force (Main balance)
params.contactForceBalancing.alpha2 = 1.0 ;   % Priority: Angular Torque (Orientation)
params.contactForceBalancing.alpha3 = 0.01 ;  % Priority: Minimize Forces (Regularization)


%% momentumBasedBalancing tunable gains

params.momentumBasedBalancing.Kp_com = 500 ;
params.momentumBasedBalancing.Kd_com = 50 ;

params.momentumBasedBalancing.Kp_ang = 300 ;
params.momentumBasedBalancing.Kd_ang = 30 ;


end