clear all
close all
addpath('./Resources')
rng(2141444)

deltaT=0.01;
I = diag([1 2 3]);
n=7; %number of states
m=3; %number of control inputs
%% ************************** Basis functions *****************************

basisFunction = 'rbf';
% RBF centers
Nrbf = 100;
cent = rand(n,Nrbf)*2 - 1;
rbf_type = 'thinplate'; 
% Lifting mapping - RBFs + the state itself
liftFun = @(xx)( [xx;rbf(xx,cent,rbf_type)] );
Nlift = Nrbf + n;

%% ************************** Collect data ********************************
tic
disp('Starting data collection')
Nsim = 200;%200次轨迹
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
%% ******************************* Lift ***********************************
disp('Starting LIFTING')
tic
Xlift = liftFun(X);
Ylift = liftFun(Y);
fprintf('Lifting DONE, time = %1.2f s \n', toc);
%% ********************** Build predictor *********************************

disp('Starting REGRESSION')
tic
W = [Ylift ; X];
V = [Xlift; U];
VVt = V*V';
WVt = W*V';
M = WVt * pinv(VVt); % Matrix [A B; C 0]
Alift = M(1:Nlift,1:Nlift);
Blift = M(1:Nlift,Nlift+1:end);
Clift = M(Nlift+1:end,1:Nlift);

fprintf('Regression done, time = %1.2f s \n', toc);

%% *********************** Predictor visualization ***************************
Tmax = 3;
Nsim = Tmax/deltaT;

% Initial condition
x0 = (rand(n,1)*2 - 1);
x0(1:4)=x0(1:4)/norm(x0(1:4));
x0(5:7)=x0(5:7)*pi;
x_true = x0;
% Lifted initial condition
xlift = liftFun(x0);

% Simulate
for i = 0:Nsim-1
     u=(rand(m,1)*2 - 1); % Random forcing
     % Lifted dynamics
     xlift = [xlift, Alift*xlift(:,end) + Blift*u]; 
     % True dynamics
     [t,y]=ode45(@(t,x) dys(t,x,I,u),[0 deltaT],x_true(:,end));
     x_true = [x_true, y(end,:)' ];
end

x_koop = Clift * xlift; % Koopman predictions
%% ****************************  Plots  ***********************************
lw = 4;
for i=1:n
    figure
    plot([0:Nsim]*deltaT,x_true(i,:),'linewidth',lw); hold on
    plot([0:Nsim]*deltaT,x_koop(i,:), '--r','linewidth',lw)
    axis([0 Tmax min(x_koop(i,:))-0.1 max(x_koop(i,:))+0.1])
    title('Predictor comparison - $x_1$','interpreter','latex'); xlabel('Time [s]','interpreter','latex');
    set(gca,'fontsize',20)
    LEG = legend('True','Koopman','Local at $x_0$','Local at 0','location','southwest');
    set(LEG,'interpreter','latex')
end





