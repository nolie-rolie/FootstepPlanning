close all; clear; clc

%% make environment

% bounds
xLB = 0;
xUB = 10;
yLB = 0;
yUB = 8;

% points
stPoint = [xLB + 0.5, yLB + 0.5];
endPoint = [xUB - 0.5, yUB - 0.5];

% obstacle
obs1_xLB = 3.5;
obs1_xUB = 6.5;
obs1_yLB = 2.5;
obs1_yUB = 5.5;

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

%% formulate and solve MIP problem
nPoints = 10;
nBoundaries = 4;

z1 = binvar(nPoints,nBoundaries);
x = sdpvar(nPoints,3);
M = 1e6;
safety = 0;


constr = [x(:,1) <= obs1_xLB - safety + z1(:,1)*M, ...
          x(:,1) >= obs1_xUB + safety - z1(:,2)*M, ...
          x(:,2) <= obs1_yLB - safety + z1(:,3)*M, ... 
          x(:,2) >= obs1_yUB + safety - z1(:,4)*M, ...
          z1(:,1) + z1(:,2) + z1(:,3) + z1(:,4) <= 3, ...
          x(1,1:2) == stPoint, ...
          x(nPoints,1:2) == endPoint];

A = 2;
B = 3;

vz_inv = x(3);
objective = B * (x(1,3)^2 + ...
                        (x(2,1) - x(1,1))^2 + ...
                        (x(2,2) - x(1,2))^2);
%                         ((x(2,1) - x(1,1))/(x(1,3)/4.8))^2 + ...
%                         ((x(2,2) - x(1,2))/(x(1,3)/4.8))^2);

for i = 2:nPoints-1
    objective = objective + A * x(i-1,3)^2 + ...
                    B * (x(i,3)^2 - x(i-1,3)^2 + ...
                        (x(i+1,1) - x(i,1))^2 - ... 
                          (x(i,1) - x(i-1,1))^2 + ... 
                          (x(i+1,2) - x(i,2))^2 - ... 
                          (x(i,2) - x(i-1,2))^2);
%                         ((x(i+1,1) - x(i,1))/(x(i,3)/4.8))^2 + ...
%                         ((x(i,1) - x(i-1,1))/(x(i-1,3)/4.8))^2 + ...
%                         ((x(i+1,2) - x(i,2))/(x(i,3)/4.8))^2 + ...
%                         ((x(i,2) - x(i-1,2))/(x(i-1,3)/4.8))^2);
                            
    % smoothness 
    objective = objective + norm(x(i+1,:) - x(i,:),2)^2;
end


options = sdpsettings('verbose',2,'solver','GUROBI','debug',1);
sol = optimize(constr,objective,options);

% plot with size of marker proportional to z velocity