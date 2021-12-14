clc
clear
close all

%% lay out environment

% bounds
xLB = 0;
xUB = 10;
yLB = 0;
yUB = 8;

% points
stPoint = [xLB + 0.5, yLB + 0.5];
endPoint = [xUB - 0.5, yUB - 0.5];

% obstacle
obs_xLB = 3.5;
obs_xUB = 6.5;
obs_yLB = 2.5;
obs_yUB = 5.5;

%% plot environment

figure(1)
% points
plot([stPoint(1) endPoint(1)],[stPoint(2) endPoint(2)],'k*')
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
fill([obs_xLB obs_xLB obs_xUB obs_xUB], ...
        [obs_yLB obs_yUB obs_yUB obs_yLB],'r')

%% formulate and solve MIP problem

nPoints = 100;
nBoundaries = 4;

z = binvar(nPoints,nBoundaries);
x = sdpvar(nPoints,2);
M = 1e2;
safety = 0.08;

constr = [x(:,1) <= obs_xLB - safety + z(:,1)*M, ...
          x(:,1) >= obs_xUB + safety - z(:,2)*M, ...
          x(:,2) <= obs_yLB - safety + z(:,3)*M, ... 
          x(:,2) >= obs_yUB + safety - z(:,4)*M, ...
          z(:,1) + z(:,2) + z(:,3) + z(:,4) <= 3, ...
          x(1,:) == stPoint, ...
          x(nPoints,:) == endPoint];

objective = 0;
for i = 1:nPoints-1
    objective = objective + norm(x(i+1,:) - x(i,:),2)^2;
end

options = sdpsettings('verbose',2,'solver','GUROBI','debug',1);
sol = optimize(constr,objective,options);



%% plot trajectory

plot(value(x(:,1)),value(x(:,2)))