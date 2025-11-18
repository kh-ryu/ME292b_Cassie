function model = model_app(model)

% Use Xtree to give a good appearance field  
    
    if(isfield(model.appearance, 'body'))
        model.appearance = rmfield(model.appearance, 'body') ;
    end
      for i = 1:model.NB
          model.appearance.body{i} = {};
      end

      for i = find(model.parent ~= 0)%2:model.NB
          prev_i = model.parent(i);
          prev_cyl = X_to_r(model.Xtree{i});
%           prev_cyl'
          if ~isequal(prev_cyl,zeros(3,1))
            cell_contents = model.appearance.body{prev_i};
            if isempty(cell_contents)
                   model.appearance.body{prev_i} = ...
                       {'cyl',[0 0 0; prev_cyl'],0.01};
            else
                  model.appearance.body{prev_i} = ...
                {[cell_contents,...
                'cyl',[0 0 0; prev_cyl'],0.01]};
            end
          end
      end
      
      
% % Add leaf link appearances based on COM
% leaf_idx = setdiff(1:model.NB, model.parent) ;
% for j=1:length(leaf_idx)
%     [m c] = mcI_inv(model.I{leaf_idx(j)}) ;
%     model.appearance.body{leaf_idx(j)} = ...
%         {'cyl',[0 0 0; 0 0 -0.422],0.01};
% %        {'cyl',[0 0 0; 2*c'],0.01};
% end
%       
% %       Add the head, hands and feet
% names = model.link_names ;
% 
% id_head = find(strcmp(names, 'head')) ;
% id_lhand = find(strcmp(names, 'l_hand')) ;
% id_rhand = find(strcmp(names, 'r_hand')) ;
% id_lfoot = find(strcmp(names, 'l_foot')) ;
% id_rfoot = find(strcmp(names, 'r_foot')) ;
% 
% if(id_head)
%     model.appearance.body{id_head} = {'box',[-0.005 -0.05 0; 0.005 0.05 0.1] };
% end
% if(id_lhand)
%     model.appearance.body{id_lhand} = {'box',[-0.005 0 -0.05; 0.005 0.1 0.05 ]};
% end
% if(id_rhand)
%     model.appearance.body{id_rhand} = {'box',[-0.005 -0.1 -0.05; 0.005 0 0.05 ]};
% end
% if(id_lfoot)
%     model.appearance.body{id_lfoot} = {'box',[-0.025 -0.05 0;0.075 0.05 -0.01 ]};
% end
% if(id_rfoot)
%     model.appearance.body{id_rfoot} = {'box',[-0.025 -0.05 0;0.075 0.05 -0.01 ]};
% end