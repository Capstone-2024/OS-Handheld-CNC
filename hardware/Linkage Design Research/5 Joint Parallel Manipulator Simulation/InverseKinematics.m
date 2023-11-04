%% Known Variables
% Link Lengths [mm]

l1 = 7.5; % Active Link - Left
l2 = 110; % Passive Link - Long
l3 = 100; % Passive Link - Short
l4 = 7.5; % Active Link - Right
l5 = 170; % Ground Link


%% Singularities
% % S1: L1 and L2 are colinear, L3 and L4 are colinear, J1 = J2 and J3 = J4
% 
% alphaS1 = rad2deg(acos((l5/2)/(l1+l2)))
% betaS1 = 180 - alphaS1
% 
% %S2: L1 L2 are colinear, L3 L4 are colinear, J1 = J2 + 180
% 


%% Solving for Joint Angles
% Define End-Effector Position
x = 87;
y = 65;

c = sqrt(x^2+y^2); % Position magntiude from Motor A to End-Effector

e = sqrt((l5-x)^2+y^2); % Positional magnitude from End-Effector to Motor B

alpha1 = (atan(y/x)); % Angle between the ground link and the straight line formed between Motor A and End-effector

alpha2 = (acos((l2^2-c^2-l1^2) / (-2*l1*c))); % Angle between straight line formed between Motor A and End-effector and active link 1

alpha = rad2deg(alpha1 + alpha2)

beta1 = (atan(y/(l5-x))); % Angle between ground link and straight line formed between End-effector and Motor B

beta2 = (acos((l3^2-l4^2-e^2) / (-2*l4*e))); % Angle between straight formed between End-effector and Motor B and active link 2

beta = 180 - rad2deg(beta1 + beta2);
beta = 180 + beta

%% Solving for Joint Angles
% Define End-Effector Position
x = 89.42;
y = 54.85;

A1 = x;
B1 = y;
C1 = (l1^2-l2^2+x^2+y^2)/(2*l1);
theta1 = 2*atan((-B1-sqrt(A1^2+B1^2-C1^2))/(-A1-C1));
theta1 = rad2deg(theta1);
alpha = theta1

A2 = x-l5;
B2 = y;
C2 = (l4^2+l5^2-l3^2-2*x*l5+x^2+y^2)/(2*l4);
theta4 = 2*atan((-B2+sqrt(A2^2+B2^2-C2^2))/(-A2-C2));
theta4 = rad2deg(theta4);
beta = theta4

%% Plotting
Link1 = line([0, l1*(cos(alpha*pi/180))], [0, l1*(sin(alpha*pi/180))]);
Link4 = line([l5,l5+l4*(cos(beta*pi/180))], [0, l4*(sin(beta*pi/180))]);
Link2 = line([l1*(cos(alpha*pi/180)), x], [l1*(sin(alpha*pi/180)), y]);
Link3 = line([l5+l4*(cos(beta*pi/180)), x], [l4*(sin(beta*pi/180)), y]);

% Joint1 = viscircles([0, 0], 2)
% Joint1 = viscircles([l1*(cos(alpha*pi/180)), l1*(sin(alpha*pi/180))], 1)
% Joint3 = viscircles([x, y], 1)
% Joint4 = viscircles([l5+l4*(cos(beta*pi/180)), l1*(sin(beta*pi/180))], 1)
% Joint5 = viscircles([l5, 0], 2)


%% Cycling Through Link Positions
clear plot

x=95;
while true

    y = 53;
    [alpha, beta] = IK(l1,l2,l3,l4,l5,x,y);

    Link1 = line([0, l1*(cos(alpha*pi/180))], [0, l1*(sin(alpha*pi/180))]);
    Link4 = line([l5,l5+l4*(cos(beta*pi/180))], [0, l4*(sin(beta*pi/180))]);
    Link2 = line([l1*(cos(alpha*pi/180)), x], [l1*(sin(alpha*pi/180)), y]);
    Link3 = line([l5+l4*(cos(beta*pi/180)), x], [l4*(sin(beta*pi/180)), y]);

    x=x-1;

    Joint1 = viscircles([0, 0], 1)
    Joint1 = viscircles([l1*(cos(alpha*pi/180)), l1*(sin(alpha*pi/180))], 1)
    Joint3 = viscircles([x, y], 1)
    Joint4 = viscircles([l5+l4*(cos(beta*pi/180)), l1*(sin(beta*pi/180))], 1)
    Joint5 = viscircles([l5, 0], 1)

    if(x == 75)

        break;

    end

end    