function [alpha,beta] = IK(l1, l2, l3, l4, l5, x, y)

c = sqrt(x^2+y^2); % Position magntiude from Motor A to End-Effector

e = sqrt((l5-x)^2+y^2); % Positional magnitude from End-Effector to Motor B

alpha1 = rad2deg(atan(y/x)); % Angle between the ground link and the straight line formed between Motor A and End-effector

alpha2 = rad2deg(acos((l2^2-c^2-l1^2) / (-2*l1*c))); % Angle between straight line formed between Motor A and End-effector and active link 1

alpha = alpha1 + alpha2;

beta1 = rad2deg(atan(y/(l5-x))); % Angle between ground link and straight line formed between End-effector and Motor B

beta2 = rad2deg(acos((l1^2-l2^2-e^2) / (-2*l1^2*e))); % Angle between straight formed between End-effector and Motor B and active link 2

beta = 180 - (beta1 + beta2);

end