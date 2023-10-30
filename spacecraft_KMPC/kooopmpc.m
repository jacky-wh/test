clear all
close all
addpath('./Resources')
addpath('./Resources/qpOASES-3.1.0/interfaces/matlab') 
%% ********************** Feedback control ********************************
deltaT=0.01;
Tmax = 3; % Simlation legth
Nsim = Tmax/deltaT;
REF = 'space'; % 'step' or 'cos'

switch REF
    case 'space'
        ymin = [-1;-1;-1;-1;-pi; -pi; -pi;];
        ymax = -ymin;
        x0=load('.\mpcAB\Xinit.txt');
        yrr = load('.\mpcAB\Xref.txt');   % reference
    case 'step'
        ymin = -0.6;
        ymax = 0.6;
        x0 = [0;0.6];
        yrr = 0.3*( -1 + 2*([1:Nsim] > Nsim/2)  ); % reference
    case 'cos'
        ymin = -0.4;
        ymax = 0.4;
        x0 = [-0.1;0.1];
        yrr = 0.5*cos(2*pi*[1:Nsim] / Nsim); % reference
end
Alift=load('.\mpcAB\A.txt');
Blift=load('.\mpcAB\B.txt');
s_A=size(Blift);

Nlift=s_A(1);

% Define Koopman controller
C1=diag([1 1 1 1 1 1 1]);
C = zeros(7,Nlift); 
C(1:7,1:7)=C1;
% Weight matrices
Q = diag([1 1 1 1 1 1 1]);
R = diag([0.01 0.01 0.01]);
% Prediction horizon
Tpred = 1;
Np = round(Tpred / deltaT);
% Constraints
xlift_min = [ymin ; nan(Nlift-7,1)];
xlift_max = [ymax ; nan(Nlift-7,1)];

% Build Koopman MPC controller
koopmanMPC  = getMPC(Alift,Blift,C,0,Q,R,Q,Np,-[1;1;1], [1;1;1], xlift_min, xlift_max,'qpoases');

Nsim=500;
% Current value of the reference signal
yr = yrr;
xlift = load('.\mpcAB\Xinit.txt');
for i = 0:Nsim-1
    if(mod(i,10) == 0)
        fprintf('Closed-loop simulation: iterate %i out of %i \n', i, Nsim)
    end
    x1=size(xlift(:,end));
    x2=size(yr);
    u_koop = koopmanMPC(xlift(:,end),yr(1:7));
    xlift = [xlift, Alift*xlift(:,end) + Blift*u_koop]; 
end

x_koop = C * xlift; % Koopman predictions

%% ****************************  Plots  ***********************************
lw = 4;
for i=1:7
    figure
    yline(yr(i)); hold on
    plot([0:Nsim]*deltaT,x_koop(i,:), '--r','linewidth',lw)

    axis([0 Nsim*deltaT -1 1])
    title('Predictor comparison - $x_1$','interpreter','latex'); xlabel('Time [s]','interpreter','latex');
    set(gca,'fontsize',20)
    LEG = legend('True','Koopman','Local at $x_0$','Local at 0','location','southwest');
    set(LEG,'interpreter','latex')
end



