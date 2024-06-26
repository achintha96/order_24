function [theta1,theta2,theta3] = find_IK(X,Y,L_1,L_2)
%calculating angles for given cartesian coordinate
%% simplified using matlab
XE = abs(X);
YE = abs(Y+0.15);

denominator1 = (L_1^2 + 2*L_1*XE - L_2^2 + XE^2 + YE^2)
sigma1 = (- L_1^4 + 2*L_1^2*L_2^2 + 2*L_1^2*XE^2 + 2*L_1^2*YE^2 - L_2^4 + 2*L_2^2*XE^2 + 2*L_2^2*YE^2 - XE^4 - 2*XE^2*YE^2 - YE^4)^(1/2)
numerator1 = (2*L_1*YE + sigma1)

denominator2 = (- L_1^2 + 2*L_1*L_2 - L_2^2 + XE^2 + YE^2)

theta1 = 2*atan2(numerator1,denominator1);
theta2 = -2*atan2(((- L_1^2 + 2*L_1*L_2 - L_2^2 + XE^2 + YE^2)*(L_1^2 + 2*L_1*L_2 + L_2^2 - XE^2 - YE^2))^(1/2),denominator2);
theta3 = (3*pi/2) - theta2 - theta1 - (2*pi);

if X<0
  theta1 = pi-theta1
  theta2 = theta2
  theta3 = -1*theta3
end



end

