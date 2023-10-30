clear all
close all
addpath('./Resources')
rng(2141444)
I = diag([1 2 3]);
n=7; %number of states
m=3; %number of control inputs

%% ************************** Collect data ********************************
tic
disp('Starting data collection')
deltaT=0.01;
Nsim = 10;%200次轨迹
Ntraj=1000;%1000次迭代
X = []; Y = []; U = [];
for  j = 1:Nsim
    % Random initial conditions
    Xcurrent = (rand(n,1)*2 - 1);
    Xcurrent(1:4)=Xcurrent(1:4)/norm(Xcurrent(1:4));
    Xcurrent(5:7)=Xcurrent(5:7)*pi;
    for  i = 1:Ntraj
        u=(rand(m,1)*2 - 1); % Random forcing
        [t,y]=ode45(@(t,x) dys(t,x,I,u),[0 deltaT],Xcurrent);
        Xnext=y(end,:)';
        X = [X Xcurrent];
        Y = [Y Xnext];
        U = [U u];
        Xcurrent=Xnext;
    end
end
fprintf('Data collection DONE, time = %1.2f s \n', toc);







