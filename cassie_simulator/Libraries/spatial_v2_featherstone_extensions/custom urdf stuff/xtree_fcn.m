% Function to convert urdf xyz, rpy to xTree
function xTree = xtree_fcn(xyz, rpy)
%     xTree = xlt(xyz) * rotz(rpy(3)) * roty(rpy(2)) * rotx(rpy(1)) ;
    xTree = rotx(rpy(1)) * roty(rpy(2)) * rotz(rpy(3)) *  xlt(xyz) ;
end