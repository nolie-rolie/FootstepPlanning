clc; close all; clear;
x = sdpvar(1,1);

constr = [];
objective = x;

options = sdpsettings('verbose',2,'solver','GUROBI','debug',1);
sol = optimize(constr,objective,options);