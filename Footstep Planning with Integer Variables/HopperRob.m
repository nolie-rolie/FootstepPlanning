%close all; 
clear; clc

%% Configure for tests
solveTime = 10;
nHops = 8;

%% make environment

% bounds
xLB = 0;
xUB = 10;
yLB = 0;
yUB = 8;

% points
stPoint = [xLB + 0.5; yLB + 0.5];
endPoint = [xUB - 0.5; yUB - 0.5; 0;0];

% obstacle
plat = [0.5 1.5 0.5 1.5;
    9 10 7 8;
    2 3 3 4;
    7 8 4 5;
    4 5 5 6;
    4 5 2 3;
    7 8 2 3;
    6 7 6 7;
    8.5 9.5 5.5 6.5;
    5.5 6.5 4.5 5.5]'; %[xLB xUB yLB yUB]
nPlatforms = length(plat(1,:));

%% plot environment for binary and integer performance
figure(1)
for fix = 1:2
subplot(1,2,fix)
% points
plot3([stPoint(1) endPoint(1)],[stPoint(2) endPoint(2)],[0 0],'k*')
axis equal
hold on

%bounds
plot([xLB xUB],[yLB yLB],'k','LineWidth',5)
hold on
plot([xLB xUB],[yUB yUB],'k','LineWidth',5)
hold on
plot([xLB xLB],[yLB yUB],'k','LineWidth',5)
hold on
plot([xUB xUB],[yLB yUB],'k','LineWidth',5)

% obstacle
hold on
for ix = 1:nPlatforms
fill([plat(1,ix) plat(1,ix) plat(2,ix) plat(2,ix)], ...
        [plat(3,ix) plat(4,ix) plat(4,ix) plat(3,ix)],'r')
end
end

%% formulate and solve MIP problem with integers
subplot(1,2,1)
N = nHops; % number of hops

z = intvar(1,N);
x = sdpvar(5,N); % [x y vx vy vz]'
M = 1e6;
safety = 0;


constr = [ z(1,:) <= nPlatforms;
           z(1,:) >= 1;
           x(1:2,1) == stPoint, ... % Start point
           x(1:4,N) == endPoint, ... % End point
           x(3,1:N-1).*x(5,1:N-1) == 4.9*(x(1,2:N)- x(1,1:N-1)),... % Constrain X vel
           x(4,1:N-1).*x(5,1:N-1) == 4.9*(x(2,2:N)- x(2,1:N-1)),...
           x(5,:) >= 0]; % Constrain Z vel
       
for ix = 1:nPlatforms
    constr = [constr,...
           x(1,:) >= plat(1,ix) - M*(z(1,:)-ix).^2, ... % Obstacle
           x(1,:) <= plat(2,ix) + M*(z(1,:)-ix).^2, ... 
           x(2,:) >= plat(3,ix) - M*(z(1,:)-ix).^2, ... 
           x(2,:) <= plat(4,ix) + M*(z(1,:)-ix).^2];
end

VZ = 1;
VXY = 1;
B = diag([5 5 1 1 1]);

objective = 2*VZ*x(5,1:N-1)*x(5,1:N-1)'...
    + VXY * (  (x(3,2:N) - x(3,1:N-1)) * (x(3,2:N) - x(3,1:N-1))'  ...
              +(x(4,2:N) - x(4,1:N-1)) * (x(4,2:N) - x(4,1:N-1))'  )...
    + VXY*( x(3,1)^2 + x(4,1)^2 );
    


options = sdpsettings('verbose',0,'solver','GUROBI','debug',1,'GUROBI.TimeLimit',solveTime);
sol = optimize(constr,objective,options);

% plot with size of marker proportional to z velocity
plot(value(x(1,1:N/2+1)),value(x(2,1:N/2+1)),'b*-');
plot(value(x(1,N/2+1:N)),value(x(2,N/2+1:N)),'k*-');

vel_scale = 0.1;
for ix = 1:N
   plot3([value(x(1,ix)), value(x(1,ix)) + vel_scale*value(x(3,ix))] ,...
       [value(x(2,ix)), value(x(2,ix)) + vel_scale*value(x(4,ix))] ,...
       [0             , 0              + vel_scale*value(x(5,ix))] , 'g-');
end

title(strcat("z=1 int     cost = ",num2str(value(objective))));
hold off

%% formulate and solve MIP problem with four binary variables
subplot(1,2,2)

x = sdpvar(5,N);
z = binvar(nPlatforms,N);

constr = [ x(1:2,1) == stPoint, ... % Start point
           x(1:4,N) == endPoint, ... % End point
           x(3,1:N-1).*x(5,1:N-1) == 4.9*(x(1,2:N)- x(1,1:N-1)),... % Constrain X vel
           x(4,1:N-1).*x(5,1:N-1) == 4.9*(x(2,2:N)- x(2,1:N-1)),...
           x(5,:) >= 0,...
           sum(z) == nPlatforms-1]; % Constrain Z vel
       
for ix = 1:nPlatforms
    constr = [constr,...
           x(1,:) >= plat(1,ix) - M*z(ix,:), ... % Obstacle
           x(1,:) <= plat(2,ix) + M*z(ix,:), ... 
           x(2,:) >= plat(3,ix) - M*z(ix,:), ... 
           x(2,:) <= plat(4,ix) + M*z(ix,:)];
end

objective = 2*VZ*x(5,1:N-1)*x(5,1:N-1)'...
    + VXY * (  (x(3,2:N) - x(3,1:N-1)) * (x(3,2:N) - x(3,1:N-1))'  ...
              +(x(4,2:N) - x(4,1:N-1)) * (x(4,2:N) - x(4,1:N-1))'  )...
    + VXY*( x(3,1)^2 + x(4,1)^2 );

sol = optimize(constr,objective,options);

% plot with size of marker proportional to z velocity
plot(value(x(1,1:N/2+1)),value(x(2,1:N/2+1)),'b*-');
plot(value(x(1,N/2+1:N)),value(x(2,N/2+1:N)),'k*-');

vel_scale = 0.1;
for ix = 1:N
   plot3([value(x(1,ix)), value(x(1,ix)) + vel_scale*value(x(3,ix))] ,...
       [value(x(2,ix)), value(x(2,ix)) + vel_scale*value(x(4,ix))] ,...
       [0             , 0              + vel_scale*value(x(5,ix))] , 'g-');
end
title(strcat("z=bins      cost = ",num2str(value(objective))));
hold off