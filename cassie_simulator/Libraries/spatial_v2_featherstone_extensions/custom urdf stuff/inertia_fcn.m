function I = inertia_fcn(ixx,ixy,ixz,iyy,iyz,izz)
    I = [ixx ixy ixz; ixy iyy iyz; ixz iyz izz] ;
end