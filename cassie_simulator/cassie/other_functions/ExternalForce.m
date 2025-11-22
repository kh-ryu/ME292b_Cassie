function f_ext = ExternalForce(t, q,model)
% F_pert 6x1 - roll, pitch, yaw, x,y,z
F_pert =       [3*sin(t) 3*sin(t) 3*sin(t) ...
                15*sin(t) 15*sin(t) 15*sin(t)]';


% apply perturbation force on torso
f_ext = cell(1,model.NB);
bXw_curr = bodypos(model, model.idx.torso, q) ;
f_ext{model.idx.torso} = bXw_curr' * F_pert;