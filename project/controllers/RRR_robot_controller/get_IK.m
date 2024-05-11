function [theta1,theta2,theta3] = get_IK(X,Y,L_1,L_2,L_3)
XE = abs(X);
YE = abs(Y+L_3);

sigma1 = (- L_1^4 + 2*L_1^2*L_2^2 + 2*L_1^2*XE^2 + 2*L_1^2*YE^2 - L_2^4 + 2*L_2^2*XE^2 + 2*L_2^2*YE^2 - XE^4 - 2*XE^2*YE^2 - YE^4)^(1/2)
num1 = (2*L_1*YE + sigma1)
den1 = (L_1^2 + 2*L_1*XE - L_2^2 + XE^2 + YE^2)
theta1 = 2*atan2(num1,den1);

num2 = ((- L_1^2 + 2*L_1*L_2 - L_2^2 + XE^2 + YE^2)*(L_1^2 + 2*L_1*L_2 + L_2^2 - XE^2 - YE^2))^(1/2)
den2 = (- L_1^2 + 2*L_1*L_2 - L_2^2 + XE^2 + YE^2)
theta2 = 2*atan2(num2,den2);

theta3 = (0.5*pi) - theta2 + theta1;
theta1 = (0.5*pi) - theta1;

if X<0
  theta1 = -1*theta1
  theta2 = -1*theta2
  theta3 = -1*theta3
end



end

