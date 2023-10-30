clear all
close all
addpath('./Resources')
rng(2141444)
I = diag([1 2 3]);
w0 = [pi/10 pi/6 pi/8]'; %Initial body-frame angular velocity
q0 = [0 0 0 1]'; %Initial attitude quaternion (body from to inertial frame rotation)
r0 = [-.5 .3 .1]'; %Initial rotor momentum in body frame
x0 = [q0; w0]; %Initial state vector for ODE45

u=r0;
[t,y]=ode45(@(t,x) dys(t,x,I,u),[0 0.05],x0);
x0=y(end,:);
a=norm(y(end,1:4));