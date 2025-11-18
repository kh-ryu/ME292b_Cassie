% Function to traverse model.parent breadth first
function model = update_model_breadth_first(model)
    new_idx = breadth_first(model.parent, 0) ;
    
    model.jtype = model.jtype(new_idx) ;
    model.Xtree = model.Xtree(new_idx) ;
    model.I = model.I(new_idx) ;

    mp = model.parent(new_idx) ; mp(1) = [] ;
    old_parent_names = model.link_names(mp) ;
    
    model.link_names = model.link_names(new_idx) ;
    model.link_id = 1:length(new_idx) ;
    model.link_map = containers.Map(model.link_names, model.link_id) ;
    
    if(isfield(model, 'jnames'))
        model.jnames = model.jnames(new_idx) ;
    end
    
    model.parent = 0*model.parent ;
    for j = 1:length(old_parent_names)
        model.parent(j+1) = model.link_map(old_parent_names{j}) ;
    end
end

function new_idx = breadth_first(parent_array, base)
    Q = base ;
    j = 1 ;
    
    while(j <= length(Q))
        idx = find(parent_array == Q(j)) ;
        Q = [Q idx] ;
        j = j+1 ;
    end
    new_idx = Q(2:end) ;
end
% 
% function new_idx = depth_first(parent_array, curr_base, curr_idx)
%     idx = find(parent_array == curr_base) ;
%     if(~isempty(idx))
%         curr_idx = [curr_idx idx] ;
%         for j=1:length(idx)
%             curr_idx = breadth_first(parent_array, idx(j), curr_idx) ;
%         end
%     end
%     new_idx = curr_idx ;
% end