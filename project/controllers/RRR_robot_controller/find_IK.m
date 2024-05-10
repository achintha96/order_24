function [theta1,theta2,theta3] = find_IK(X,Y,L_1,L_2)
%calculating angles for given cartesian coordinate
%% simplified using matlab
XE = abs(X);
YE = abs(Y+0.01);

theta1 = 2*atan2((2*L_1*YE + (- L_1^4 + 2*L_1^2*L_2^2 + 2*L_1^2*XE^2 + 2*L_1^2*YE^2 - L_2^4 + 2*L_2^2*XE^2 + 2*L_2^2*YE^2 - XE^4 - 2*XE^2*YE^2 - YE^4)^(1/2)),(L_1^2 + 2*L_1*XE - L_2^2 + XE^2 + YE^2));
theta2 = 2*atan2(((- L_1^2 + 2*L_1*L_2 - L_2^2 + XE^2 + YE^2)*(L_1^2 + 2*L_1*L_2 + L_2^2 - XE^2 - YE^2))^(1/2),(- L_1^2 + 2*L_1*L_2 - L_2^2 + XE^2 + YE^2));

theta3 = (0.5*pi) - theta2 + theta1;
theta1 = (0.5*pi) - theta1

if X<0
  theta1 = pi-theta1
  theta2 = -1*theta2
  theta3 = -1*theta3
end



end

