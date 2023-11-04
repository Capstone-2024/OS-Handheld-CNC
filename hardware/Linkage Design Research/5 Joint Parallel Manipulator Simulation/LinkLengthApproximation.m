%% Link Lengths - Eccentric CAMs

l1 = 7.5; % Left CAM
l2 = 110; % Passive Link
l3 = 110; % End Effector
l4 = 7.5; % Right CAM
l5 = 188; % Ground Link

theta1 = linspace(0,360,360);
theta2 = linspace(0,360,360);

theta1 = theta1'
theta2 = theta2'

xc_points= zeros(1,1);
yc_points = zeros(1,1);

%% Calculations

clear plot

% minVals3 = zeros(1,10);
% minVals2 = zeros(1,10);
% aa = 0
% bb = 0;

% for l2 = 90:2:110
% for l3 = 90:2:110

for i = 1:length(theta1)
    for j = 1:length(theta2)
            
        xb = l1*rad2deg(cos(theta1(i)));
        yb = l1*rad2deg(sin(theta1(i)));

        xd = l5 + l4*rad2deg(cos(theta2(j)));
        yd = l4*rad2deg(sin(theta2(j)));

        A = 2*l3*l4*(sin(theta2(j))) - 2*l1*l3*(sin(theta1(i)));
        B = 2*l3*l5 - 2*l1*l3*(cos(theta1(i))) + 2*l3*l4*(cos(theta2(j)));
        C = l1*l1 - l2*l2 + l3*l3 + l4*l4 + l5*l5 - 2*l1*l4*(sin(theta1(i)))*(sin(theta2(j))) - 2*l1*l5*(cos(theta1(i))) + 2*l4*l5*(cos(theta2(j))) -2*l1*l4*(cos(theta1(i)))*(cos(theta2(j)));

        theta3 = 2*(atan((A + sqrt(A*A+B*B-C*C)) / (B-C)));

        theta4 = (asin((l3*(sin(theta3)) + l4*(sin(theta2(j)))  - l1*(sin(theta1(i)))) / l2));
        
        xc = l1*(cos(theta1(i))) + l2*(cos(theta4));
        yc = l1*(sin(theta1(i))) + l2*(sin(theta4));
        
        xc_points(j) = xc;
        yc_points(j) = yc;

    end
        if i == 1
            totalMaxX = max(xc_points)
            totalMaxY = max(yc_points)
            totalMinX = min(xc_points)
            totalMinY = min(yc_points)
        else
            if max(xc_points) > totalMaxX
                totalMaxX = max(xc_points);
            end

            if min(xc_points) < totalMinX
                totalMinX = min(xc_points);
            end

            if max(yc_points) > totalMaxY
                totalMaxY = max(yc_points);
            end

            if min(yc_points) < totalMinY
                totalMinY = min(yc_points);
            end
        end
    i
    plot(xc_points, yc_points)
    hold on
end

% [l3val,l3idx]=min(abs(xc_points-90));
% minVall3=xc_points(l3idx)
% aa = aa + 1;
% minVals3(aa) = minVall3
% end
% [l2val,l2idx]=min(abs(xc_points-90));
% minVall2=xc_points(l2idx)
% bb = bb + 1;
% minVals2(bb) = minVall2
% end

horzWorkspace = totalMaxX - totalMinX;
vertWorkspace = totalMaxY - totalMinY;
% 
% [l2bestval,l2bestidx]=min(abs(minVals2-90));
% bestminVall2=minVals2(l2bestidx)
