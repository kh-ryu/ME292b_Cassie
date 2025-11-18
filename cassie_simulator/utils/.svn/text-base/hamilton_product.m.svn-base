function qprod = hamilton_product(q1,q2)
% q1 q2 and qprod are quaternions, ie 4-by-1 vectors
s1 = q1(1);
v1 = [q1(2);q1(3);q1(4)];

s2 = q2(1);
v2 = [q2(2);q2(3);q2(4)];

sprod = s1*s2 - v1'*v2;
qprod = s1*v2 + s2*v1 + cross (v1,v2);
qprod = [sprod;qprod];
end