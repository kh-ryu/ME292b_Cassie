% urdf_to_featherstone('drc_robot.urdf') ;
%urdf_to_featherstone('atlas_shorten.urdf') ;
% urdf_to_featherstone('atlas_sandia_hands_drc2_6_1.urdf') ;
urdf_to_featherstone(urdf_file) ;
gen_model_autogen ;


nb = model.NB ;
taper = 0.9;
skew = 0;


model.appearance.base = ...
  { };

% p0 = -1;
% for i = 1:nb
%   p1 = model.parent(i);
%   tap = taper^(i-1);
%   if p1 == 0
%     ptap = 1;
%   else
%     ptap = taper^(p1-1);
%   end
%   if ( p1 > p0 )
%     model.appearance.body{i} = ...
%       { 'cyl', [0 0 0; 1 0 0]*tap, 0.05*tap, ...
%         'cyl', [0 0 -0.07; 0 0 0.07]*ptap, 0.08*ptap };
%     p0 = p1;
%   else
%     model.appearance.body{i} = ...
%       { 'cyl', [0 0 0; 1 0 0]*tap, 0.05*tap };
%   end
% end

for j=1:length(model.link_id)
    eval(['model.idx.' model.link_names{j} ' = ' num2str(model.link_id(j)) ';']) ;
end

% A better appearance
model = gen_kappa_mu(model) ;
model = model_app(model);


% Floating the base
fbmodel = floatbase(model) ;
fbmodel.link_id = [1:5 model.link_id+5];
fbmodel.link_names = {'pelvis_x','pelvis_y','pelvis_z','pelvis_Rx','pelvis_Ry','pelvis_Rz', model.link_names{2:end}} ;
% fbmodel.link_map = containers.Map(fbmodel.link_names, fbmodel.link_id) ;
for j=1:length(fbmodel.link_id)
    eval(['fbmodel.idx.' fbmodel.link_names{j} ' = ' num2str(fbmodel.link_id(j)) ';']) ;
end

fbmodel = gen_kappa_mu(fbmodel) ;

% Constraint function
% fbmodel.gamma_q = @planar_stance_constraints ;