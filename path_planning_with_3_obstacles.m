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

% obstacles
obs1_xLB = 1;
obs1_xUB = 3;
obs1_yLB = 2;
obs1_yUB = 5;

obs2_xLB = 5;
obs2_xUB = 8;
obs2_yLB = 5;
obs2_yUB = 7;

obs3_xLB = 6;
obs3_xUB = 7;
obs3_yLB = 1;
obs3_yUB = 3;

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
fill([obs1_xLB obs1_xLB obs1_xUB obs1_xUB], ...
        [obs1_yLB obs1_yUB obs1_yUB obs1_yLB],'r')
fill([obs2_xLB obs2_xLB obs2_xUB obs2_xUB], ...
        [obs2_yLB obs2_yUB obs2_yUB obs2_yLB],'r')
fill([obs3_xLB obs3_xLB obs3_xUB obs3_xUB], ...
        [obs3_yLB obs3_yUB obs3_yUB obs3_yLB],'r')

%% formulate and solve MIP problem

nPoints = 100;
nBoundaries = 4;

z1 = binvar(nPoints,nBoundaries);
z2 = binvar(nPoints,nBoundaries);
z3 = binvar(nPoints,nBoundaries);
x = sdpvar(nPoints,2);
M = 1e2;
safety = 0.08;

constr = [x(:,1) <= obs1_xLB - safety + z1(:,1)*M, ...
          x(:,1) >= obs1_xUB + safety - z1(:,2)*M, ...
          x(:,2) <= obs1_yLB - safety + z1(:,3)*M, ... 
          x(:,2) >= obs1_yUB + safety - z1(:,4)*M, ...
          z1(:,1) + z1(:,2) + z1(:,3) + z1(:,4) <= 3, ...
          x(:,1) <= obs2_xLB - safety + z2(:,1)*M, ...
          x(:,1) >= obs2_xUB + safety - z2(:,2)*M, ...
          x(:,2) <= obs2_yLB - safety + z2(:,3)*M, ... 
          x(:,2) >= obs2_yUB + safety - z2(:,4)*M, ...
          z2(:,1) + z2(:,2) + z2(:,3) + z2(:,4) <= 3, ...
          x(:,1) <= obs3_xLB - safety + z3(:,1)*M, ...
          x(:,1) >= obs3_xUB + safety - z3(:,2)*M, ...
          x(:,2) <= obs3_yLB - safety + z3(:,3)*M, ... 
          x(:,2) >= obs3_yUB + safety - z3(:,4)*M, ...
          z3(:,1) + z3(:,2) + z3(:,3) + z3(:,4) <= 3, ...
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