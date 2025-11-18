function [value,isterminal,direction] = falldetect(t,y, model, params, externalForce)

value = y(3) - 0.1;
isterminal= 1;
direction = 0;