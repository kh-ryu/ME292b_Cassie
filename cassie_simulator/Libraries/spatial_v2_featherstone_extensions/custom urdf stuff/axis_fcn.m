% Function to convert axis to joint type
function axis_text = axis_fcn(axis_array)
    if(axis_array(1) == 1)
        axis_text = 'Rx' ;
    elseif(axis_array(2) == 1)
        axis_text = 'Ry' ;
    elseif(axis_array(3) == 1)
        axis_text = 'Rz' ;
    elseif(norm(axis_array) <= eps)
        axis_text = 'F' ;
    else
        %disp('Error in axis_fcn()') ; keyboard ;
        axis_text.code = 'Ra' ;
        axis_text.pars.axis = axis_array' ;
    end
end