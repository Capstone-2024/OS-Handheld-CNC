%% Link Lengths - Linkage

l1 = 7.5; % Left Link
l2 = 110; % Passive Link
l3 = 100; % End Effector
l4 = 7.5; % Right Link
l5 = 170; % Ground Link

theta1 = linspace(0,360,360);
theta4= linspace(0,360,360);

%theta1 = 180 - 17.119;
%theta4 = 180 + 36.986;


theta1 = deg2rad(theta1);
theta4 = deg2rad(theta4);

%xc_points= zeros(360,1);
%yc_points = zeros(360,1);

%% Forward Kinematics

        xb = l1*(cos(theta1));
        yb = l1*(sin(theta1));

        xd = l5 + l4*(cos(theta4));
        yd = l4*(sin(theta4));

        A = 2*l3*l4*sin(theta4) - 2*l1*l3*cos(theta1);
        B = 2*l3*l5 - 2*l1*l3*cos(theta1) + 2*l3*l4*cos(theta4);
        C = l1^2-l2^2+l3^2+l4^2+l5^2-l1*l4*sin(theta1)*sin(theta4)-2*l1*l5*cos(theta1)+2*l4*l5*cos(theta4)-2*l1*l4*cos(theta1)*cos(theta4);

        theta3_1 = 2*atan((A+sqrt(A^2+B^2-C^2))/(B-C));
        theta3_2 = 2*atan((A-sqrt(A^2+B^2-C^2))/(B-C));    

        theta2 = asin((l3*sin(theta3_1) + l4*sin(theta4) - l1*sin(theta1)) / l2);

        xc = l1*(cos(theta1)) + l2*(cos(theta2));
        yc = l1*(sin(theta1)) + l2*(sin(theta2));


%% Jacobian 

l1 = 7.5/1000; % Left Link
l2 = 110/1000; % Passive Link
l3 = 100/1000; % End Effector
l4 = 7.5/1000; % Right Link
l5 = 170/1000; % Ground Link

x = 89.42/1000;
y = 54.85/1000;

A1 = x;
B1 = y;
C1 = (l1^2-l2^2+x^2+y^2)/(2*l1);
theta1 = 2*atan((-B1-sqrt(A1^2+B1^2-C1^2))/(-A1-C1));
theta1 = rad2deg(theta1);

A2 = x-l5;
B2 = y;
C2 = (l4^2+l5^2-l3^2-2*x*l5+x^2+y^2)/(2*l4);
theta4 = 2*atan((-B2+sqrt(A2^2+B2^2-C2^2))/(-A2-C2));
theta4 = rad2deg(theta4);

theta2 = asin((y - l1*sin(theta1))/l2);

theta3 = acos((x - l5 - l4*cos(theta4))/l5);

theta5 = 180 - theta4;

theta6 = theta1 - theta2;

theta7 = theta3 - theta4;

theta1 = deg2rad(theta1);
theta2 = deg2rad(theta2);
theta3 = deg2rad(theta3);
theta4 = deg2rad(theta4);
theta5 = deg2rad(theta5);
theta6 = deg2rad(theta6);
theta7 = deg2rad(theta7);

A11 = l1*sin(theta1) + l2*sin(theta1+theta6);
A12 = l2*sin(theta1+theta6);
A21 = l2*cos(theta1)+l2*cos(theta1+theta6);
A22 = l2*cos(theta1+theta6);

B11 = l4*sin(theta4) + l3*sin(theta4+theta7);
B12 = l4*sin(theta4+theta7)
B21 = l4*cos(theta4) + l3*cos(theta4+theta7);
B22 = l3*cos(theta4+theta7)

C11 = A11 + A21*((B12*A21-A11*B22)/(A12*B22-B12*A22))
C12 = A12*((B11*B22-B21*B12)/(A12*B22-B12*A22))
C21 = A21+A22*((B12*A21-A11*B22)/(A12*B22-B12*A22))
C22 = A22*((B11*B22-B22*B12)/(A12*B22-B12*A22))

J = [-C11 -C12; C21 C22];
JT = transpose(J)

n = 20000; % 20000 RPM Router
D = 6.35; % 1/4 inch diameter toolbit
Kc = 146; % Specific cutting force constant
f = 5; % 5mm/s feedrate

Ftot = pi*D*n*f*Kc / 30000000
Fx = sqrt(((Ftot)^2)/2);
Fy = Fx;

F = [Fx; Fy];

tau = JT * F;

Kt = 0.63 / 2; % 0.63 Nm holding torque for double stack Nema 17 / 2 A stall torque

current = abs(tau./Kt)

%% Calculations

clear plot

for i = 1:length(theta1)
    for j = 1:length(theta4)
            
        xb = l1*(cos(theta1(i)));
        yb = l1*(sin(theta1(i)));

        xd = l5 + l4*(cos(theta4(j)));
        yd = l4*(sin(theta4(j)));

        A = 2*l3*l4*(sin(theta4(j))) - 2*l1*l3*(sin(theta1(i)));
        B = 2*l3*l5 - 2*l1*l3*(cos(theta1(i))) + 2*l3*l4*(cos(theta4(j)));
        C = l1*l1 - l2*l2 + l3*l3 + l4*l4 + l5*l5 - 2*l1*l4*(sin(theta1(i)))*(sin(theta4(j))) - 2*l1*l5*(cos(theta1(i))) + 2*l4*l5*(cos(theta4(j))) -2*l1*l4*(cos(theta1(i)))*(cos(theta4(j)));

        theta3 = 2*(atan((A + sqrt(A*A+B*B-C*C)) / (B-C)));

        theta2 = (asin((l3*(sin(theta3)) + l4*(sin(theta4(j)))  - l1*(sin(theta1(i)))) / l2));
        
        xc = l1*(cos(theta1(i))) + l2*(cos(theta2));
        yc = l1*(sin(theta1(i))) + l2*(sin(theta2));
        
        xc_points(j) = xc;
        yc_points(j) = yc;

    end
    i
    plot(xc_points, yc_points)
    hold on
end
th = 0:pi/50:2*pi;
xunit = 5 * cos(th) + 91;
yunit = 5 * sin(th) + 60;
plot(xunit, yunit, 'color', 'k', 'LineWidth', 4);
hold off
xlabel("X [mm]", 'FontSize', 14);
ylabel("Y [mm]", 'FontSize', 14);
title("Desired Path on Manipulator Workspace", 'FontSize', 20);
axis([82.5, 107.5, 47.5, 72.5])
axis square

%% Plotting
Link1 = line([0, xb], [0, yb]);
Link4 = line([l5, xd], [0, yd]);
Link2 = line([xb, xc], [yb, yc]);
Link3 = line([xd, xc], [yd, yc]);
set(gca, 'xlim', [-50, 200], 'ylim', [-20, 50])
