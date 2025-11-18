% Function to impose planar constraints on the atlas model
% Expressions:
% [1] Find set of independent variables y
% [2] Find Gamma - Express q = gamma(y)
% [3] Find G - Express dq = G*dy
% [4] Find g - Express g = dG*dy, (dG=G')
%
% Computations:
% [1] y = gamma^-1(q0)
% [2] dy = G^-1 * dq0
% [3] q = gamma(y)
% [4] dq = G*dy
% [5] gs = g + g_stab
%
% Template:
% function  [q,qd,G,gs] = gamma_q( model, qo, qdo )
% 
%   y = formula for calculating y from qo;
%   q = formula for gamma(y);
% 
%   G = Jacobian of gamma;
% 
%   yd = formula for calculating yd from qo (or y) and qdo;
%   qd = G * yd;  (or a formula equivalent to this expression)
% 
%   g = formula for dG/dt * yd;
% 
%   Tstab = some suitable value, such as 0.1;
%   gstab = 2/Tstab * (qd - qdo) + 1/Tstab^2 * (q - qo);
% 
%   gs = g + gstab;
% end
function [q dq G gs] = planar_stance_constraints(model, q0, dq0)
    red_model = get_planar_stance_constraints_model(model) ;
    
    G = red_model.gamma ;
    g = red_model.g ;
    
    y = red_model.gamma_inv*q0 ;
    dy = red_model.G_inv*dq0 ;

    q = red_model.gamma*y ;
    dq = red_model.G*dy ;
    
    pHip = -X_to_r(bodypos(model, model.idx.l_lleg, q)) ;
    vHip = bodyJac_vel(model, model.idx.l_lleg, q)*dq ;
    q(model.idx.pelvis_x) = pHip(1) ;
    q(model.idx.pelvis_z) = pHip(3) ;
    dq(model.idx.pelvis_x) = vHip(1) ;
    dq(model.idx.pelvis_z) = vHip(3) ;
    
    Tstab = red_model.Tstab ;
    gstab = 2/Tstab * (dq - dq0) + 1/Tstab^2 * (q - q0);

    gs = g + gstab ;
end