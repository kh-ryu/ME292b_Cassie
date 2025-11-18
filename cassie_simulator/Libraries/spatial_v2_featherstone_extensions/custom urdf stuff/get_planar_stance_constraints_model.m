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
function red_model = get_planar_stance_constraints_model(model)
    red_model.NB = 5 ;
    N = red_model.NB ;

%     independent_DOF = [model.idx.l_llglut, model.idx.l_uleg, model.idx.r_lglut, model.idx.r_uleg, model.idx.ltorso] ;
    independent_DOF = [model.idx.l_lglut, model.idx.l_uleg, model.idx.r_lglut, model.idx.r_uleg, model.idx.pelvis_Ry] ;
    red_model.idx.l_llglut = 1 ;
    red_model.idx.l_uleg = 2 ;
    red_model.idx.r_lglut = 3 ;
    red_model.idx.r_uleg = 4 ;
    red_model.idx.pelvis_Ry = 5 ;

    red_model.gamma_inv = zeros(N, model.NB) ;
    red_model.gamma = zeros(model.NB, N) ;
    for j=1:N
        red_model.gamma_inv(j, independent_DOF(j)) = 1 ;
        red_model.gamma(independent_DOF(j), j) = 1 ;
    end
    red_model.G_inv = red_model.gamma_inv ;
    red_model.G = red_model.gamma ;
    red_model.g = zeros(model.NB, 1) ;
    red_model.Tstab = 0.1 ;
end