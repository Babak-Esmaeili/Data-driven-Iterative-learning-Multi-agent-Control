clc;
clear;
close all;

tic;
%% Time domain
T0 = 0;
Ts = 1;
Tf = 100;

t = T0:Ts:Tf;

nSteps = numel(t);

%% Iteration domain
nIter = 100;

%% MAS matrices
A = [ 0  0  0  1
      1  0  1  0
      0  1  0  0 
      1  0  1  0 ];
  
D = diag(sum(A,2));
  
L = D - A;

W = diag([1 1 1 1]);

%% Controller parameters
eta = 1;
mu = 05;

epsilon = 1e-4;

c_1 = 1*diag([1 1 1 1]);
c_2 = 0.00001*diag([1 1 1 1]);

c_s = 0.1*diag([1 1 1 1]);

%% Initialization
nInputs = 1;
nOutputs = 1;
nAgents = 4;

I = eye(nAgents*nOutputs,nAgents*nOutputs);

u_eq = zeros(nAgents*nInputs,nSteps,nIter);
u_sw = zeros(nAgents*nInputs,nSteps,nIter);
u = zeros(nAgents*nInputs,nSteps,nIter);
delta_u = zeros(nAgents*nInputs,nSteps,nIter);

a = zeros(nAgents,nSteps);
a(:,1) = [1 1 1 1]';
a(:,2) = [1 1 1 1]';

y = zeros(nAgents*nOutputs,nSteps+1,nIter);
y(:,1,1) = 0.01*randn(nAgents,1);
y(:,2,1) = y(:,1,1);
y(:,3,1) = y(:,1,1);
delta_y = zeros(nAgents*nOutputs,nSteps+1,nIter);

w = zeros(nAgents*nOutputs,nSteps,nIter);

y_d = zeros(nOutputs,nSteps+1);
y_d(1) = 0.2*sin(pi*1/40);
y_d(2) = 0.2*sin(pi*2/40);
y_d(3) = 0.2*sin(pi*3/40);
    
d = zeros(nAgents*nOutputs,nSteps+1);
d(:,1) = [ 0.2*sin(pi*1/40) + 0.6
           0.2*sin(pi*1/40) + 0.4
           0.2*sin(pi*1/40) - 0.6
           0.2*sin(pi*1/40) - 0.4 ];
d(:,2) = [ 0.2*sin(pi*2/40) + 0.6
           0.2*sin(pi*2/40) + 0.4
           0.2*sin(pi*2/40) - 0.6
           0.2*sin(pi*2/40) - 0.4 ];
d(:,3) = [ 0.2*sin(pi*3/40) + 0.6
           0.2*sin(pi*3/40) + 0.4
           0.2*sin(pi*3/40) - 0.6
           0.2*sin(pi*3/40) - 0.4 ];

e = zeros(nAgents*nOutputs,nSteps+1,nIter);
e(:,1,1) = y_d(1)*ones(nAgents,1) + d(:,1) - y(:,1,1);
e(:,2,1) = y_d(2)*ones(nAgents,1) + d(:,2) - y(:,2,1);
e(:,3,1) = y_d(3)*ones(nAgents,1) + d(:,3) - y(:,3,1);

zeta = zeros(nAgents*nOutputs,nSteps,nIter);
zeta(:,1,1) = (L+W)*e(:,1,1);
zeta(:,2,1) = (L+W)*e(:,2,1);
zeta(:,3,1) = (L+W)*e(:,3,1);

int_ZETA = zeros(nAgents*nOutputs,nSteps+1);
int_ZETA(:,1,1) = zeta(:,1,1);
int_ZETA(:,2,1) = zeta(:,2,1);
int_ZETA(:,3,1) = zeta(:,3,1);

s = zeros(nAgents*nOutputs,nSteps,nIter);
s(:,1,1) = zeta(:,1,1);
s(:,2,1) = zeta(:,2,1);
s(:,3,1) = zeta(:,3,1);
delta_s = zeros(nAgents*nOutputs,nSteps+1,nIter);

PHI_hat = cell(nSteps,nIter);
PHI_hat{1,1} = 02*diag([0.5 0.5 0.5 0.5]);
PHI_hat{2,1} = PHI_hat{1,1};

e_max = zeros(nAgents*nOutputs,nIter);
e_MSE = zeros(nAgents*nOutputs,nIter);
e_IAE = zeros(nAgents*nOutputs,nIter);

%% Initial iteration simulation
for k = 3:nSteps
    
    %% PPD estimation
    PHI_hat{k,1} = PHI_hat{1,1};
%     for j = 1:nAgents
%         if(abs(PHI_hat{k,i}(j,j))<=epsilon || abs(delta_u(j,k,i-1))<=epsilon || sign(PHI_hat{k,i}(j,j))~=sign(PHI_hat{k,1}(j,j)))
%             PHI_hat{k,i}(j,j) = PHI_hat{k,1}(j,j);
%         end
%     end
    
    %% Leader's trajectory
    y_d(k+1) = 0.2*sin(pi*(k+1)/30);
    
    %% Reference deviations from leader
    d(:,k+1) = [ 0.2*sin(pi*(k+1)/30) + 0.6
                 0.2*sin(pi*(k+1)/30) + 0.4
                 0.2*sin(pi*(k+1)/30) - 0.6
                 0.2*sin(pi*(k+1)/30) - 0.4 ];

    %% Consensus protocols
%     H = diag(diag(PHI_hat{k,i}));
%     u_eq(:,k,i) = u_eq(:,k,i-1) + (H^-1)*(y_d(k+1)*ones(nAgents,1)+d(:,k+1)-...
%                                           y(:,k+1,i-1)-(((c_1+c_2)^-1)*((L+W)^-1)*(-c_2*int_ZETA(:,k)+s(:,k+1,i-1))));
%     u_sw(:,k,i) = u_sw(:,k,i-1) + c_s*(H^-1)*sign(s(:,k+1,i-1));
%     u_sw(:,k,i) = u_sw(:,k,i-1) + c_s*(H^-1)*tanh(s(:,k+1,i-1)./001);
% 
%     u(:,k,i) = u_eq(:,k,i) + u_sw(:,k,i);
    u(:,k,1) = zeros(nAgents*nInputs,1);
    
    %% Iteration-dependent disturbances
%     w(1,k,1) = 0.2 + 0.1*cos(1*pi/15) + 0.2*sin(2*k/1) + 0.3*cos(k);
%     w(2,k,1) = (1+0.1*cos(1*pi/15))*exp(-0.05*k) + (0.2+0.1*sin(pi/1))*sin(2*k);
%     w(3,k,1) = 0.5 + 0.1*sin(1*pi/15) + 0.1*cos(k/5);
%     w(4,k,1) = 0.5 + 0.2*cos(2*pi/1)*((-1)^(-5*1*k));

    w(1,k,1) = 1*(1+0.1*cos(1*pi/15))*exp(-05*k) + 0*(0.2+0.1*sin(pi/1))*sin(2*k);
    w(2,k,1) = 1*(1+0.1*cos(1*pi/15))*exp(-05*k) + 0*(0.2+0.1*sin(pi/1))*sin(2*k);
    w(3,k,1) = 1*(1+0.1*cos(1*pi/15))*exp(-05*k) + 0*(0.2+0.1*sin(pi/1))*sin(2*k);
    w(4,k,1) = 1*(1+0.1*cos(1*pi/15))*exp(-05*k) + 0*(0.2+0.1*sin(pi/1))*sin(2*k);
    
    %% Time-dependent disturbances
%     w(:,k,1) = ((01*sin(2*k*pi/10))*(k>=40 && k<=60))*ones(nAgents,1);
%     w(:,k,1) = ((01)*(k>=40 && k<=60))*ones(nAgents,1);
    
    %% MAS simulation
    a(:,k) = (1+round(k/50))*ones(nAgents,1);
    
    y(:,k+1,1) = ((y(:,k,1)./(1*ones(nAgents,1)+y(:,k,1).^2))+(u(:,k,1).^3)).*(k<=30) + ((y(:,k,1).*y(:,k-1,1).*y(:,k-2,1).*u(:,k-1,1).*(y(:,k-2,1)-1)+a(:,k).*u(:,k,1))./(1*ones(nAgents,1)+(y(:,k-1,1).^2)+(y(:,k-2,1)).^2)).*(k>30) + w(:,k,1);
    
    %% Tracking errors
    e(:,k+1,1) = y_d(k+1)*ones(nAgents,1) + d(:,k+1) - y(:,k+1,1);
    
    %% Formation errors
    zeta(:,k+1,1) = (L+W)*e(:,k+1,1);
    
    %% Iterative sliding surfaces
%     int_ZETA(:,k+1) = zeta(:,k+1,1);
    s(:,k+1,1) = zeta(:,k+1,1);
    
    %% Difference of iterative sliding surfaces
    delta_s(:,k+1,1) = s(:,k+1,1) - s(:,k+1,1);
    
    %% Incremental I/O data
    delta_u(:,k,1) = zeros(nAgents*nInputs,1);
    delta_y(:,k+1,1) = zeros(nAgents*nOutputs,1);
    
end

e_max(:,1) = max(abs(e(:,5:end,1)),[],2);
e_MSE(:,1) = mse(e(:,:,1)')';
e_IAE(:,1) = sum(abs(e(:,:,1)),2);

%% Simulation
for i = 2:nIter
    
    PHI_hat{1,i} = PHI_hat{1,1};
    PHI_hat{2,i} = PHI_hat{1,1};
    
    y(:,1,i) = 0.01*randn(nAgents,1);
    y(:,2,i) = y(:,1,i);
    y(:,3,i) = y(:,1,i);
    
    e(:,1,i) = y_d(1)*ones(nAgents,1) + d(:,1) - y(:,1,i);
    e(:,2,i) = y_d(2)*ones(nAgents,1) + d(:,2) - y(:,2,i);
    e(:,3,i) = y_d(3)*ones(nAgents,1) + d(:,3) - y(:,3,i);
     
    zeta(:,1,i) = (L+W)*e(:,1,i);
    zeta(:,2,i) = (L+W)*e(:,2,i);
    zeta(:,3,i) = (L+W)*e(:,3,i);
    
    int_ZETA(:,1,i) = int_ZETA(:,1,i-1) + zeta(:,1,i);
    int_ZETA(:,2,i) = int_ZETA(:,2,i-1) + zeta(:,2,i);
    int_ZETA(:,3,i) = int_ZETA(:,3,i-1) + zeta(:,3,i);
    
    s(:,1,i) = zeta(:,1,i) + c_2*int_ZETA(:,1,i-1);
    s(:,2,i) = zeta(:,2,i) + c_2*int_ZETA(:,2,i-1);
    s(:,3,i) = zeta(:,3,i) + c_2*int_ZETA(:,3,i-1);
    
    for k = 3:nSteps
        
        %% PPD estimation
        PHI_hat{k,i} = PHI_hat{k,i-1} + diag((eta*delta_u(:,k,i-1)./(mu*ones(nAgents,1)+(abs(delta_u(:,k,i-1)).^2))).*(delta_y(:,k+1,i-1)-PHI_hat{k,i-1}*delta_u(:,k,i-1)));
        for j = 1:nAgents
            if(abs(PHI_hat{k,i}(j,j))<=epsilon || abs(delta_u(j,k,i-1))<=epsilon || sign(PHI_hat{k,i}(j,j))~=sign(PHI_hat{k,1}(j,j)))
                PHI_hat{k,i}(j,j) = PHI_hat{k,1}(j,j);
            end
        end
        
        %% Leader's trajectory
        y_d(k+1) = 0.2*sin(pi*(k+1)/40);
        
        %% Reference deviations from leader
        d(:,k+1) = [ 0.2*cos(pi*(k+1)/40) + 0.6
                     0.2*cos(pi*(k+1)/40) + 0.4
                     0.2*cos(pi*(k+1)/40) - 0.6
                     0.2*cos(pi*(k+1)/40) - 0.4 ];
        
        %% Formation protocols
        H = diag(diag(PHI_hat{k,i}));
        u_eq(:,k,i) = u_eq(:,k,i-1) + (H^-1)*(y_d(k+1)*ones(nAgents,1)+d(:,k+1)-...
                                              y(:,k+1,i-1)-(((c_1+c_2)^-1)*((L+W)^-1)*(-c_2*int_ZETA(:,k+1,i-1)+s(:,k+1,i-1))));
%         u_sw(:,k,i) = u_sw(:,k,i-1) + c_s*(H^-1)*sign(s(:,k+1,i-1));
        u_sw(:,k,i) = u_sw(:,k,i-1) + c_s*(H^-1)*tanh(s(:,k+1,i-1)./001);
        
        u(:,k,i) = u_eq(:,k,i) + u_sw(:,k,i);
        
        %% Iteration-dependent disturbances
%         w(1,k,i) = 0.2 + 0.1*cos(i*pi/15) + 0.2*sin(2*k/i) + 0.3*cos(k);
%         w(2,k,i) = (1+0.1*cos(i*pi/15))*exp(-0.05*k) + (0.2+0.1*sin(pi/i))*sin(2*k);
%         w(3,k,i) = 0.5 + 0.1*sin(i*pi/15) + 0.1*cos(k/5);
%         w(4,k,i) = 0.5 + 0.2*cos(2*pi/i)*((-1)^(-5*i*k));

        w(1,k,i) = 10*(1+0.1*cos(i*pi/15))*exp(-05*k) + 0*(0.2+0.1*sin(pi/i))*sin(2*k);
        w(2,k,i) = 10*(1+0.1*cos(i*pi/15))*exp(-05*k) + 0*(0.2+0.1*sin(pi/i))*sin(2*k);
        w(3,k,i) = 10*(1+0.1*cos(i*pi/15))*exp(-05*k) + 0*(0.2+0.1*sin(pi/i))*sin(2*k);
        w(4,k,i) = 10*(1+0.1*cos(i*pi/15))*exp(-05*k) + 0*(0.2+0.1*sin(pi/i))*sin(2*k);

        %% Time-dependent disturbances
%         w(:,k,i) = ((01*sin(2*k*pi/10))*(k>=40 && k<=60))*ones(nAgents,1);
%         w(:,k,i) = ((01)*(k>=40 && k<=60))*ones(nAgents,1);
        
        %% MAS simulation
        a(:,k) = (1+round(k/50))*ones(nAgents,1);
        
        y(:,k+1,i) = ((y(:,k,i)./(1*ones(nAgents,1)+y(:,k,i).^2))+(u(:,k,i).^3)).*(k<=30) + ((y(:,k,i).*y(:,k-1,i).*y(:,k-2,i).*u(:,k-1,i).*(y(:,k-2,i)-1)+a(:,k).*u(:,k,i))./(1*ones(nAgents,1)+(y(:,k-1,i).^2)+(y(:,k-2,i)).^2)).*(k>30) + w(:,k,i);
        
        %% Tracking errors
        e(:,k+1,i) = y_d(k+1)*ones(nAgents,1) + d(:,k+1) - y(:,k+1,i);
        
        %% Formation errors
        zeta(:,k+1,i) = (L+W)*e(:,k+1,i);
        
        %% Iterative sliding surfaces
%         int_ZETA(:,k+1) = int_ZETA(:,k) + zeta(:,k+1,i);
        s(:,k+1,i) = zeta(:,k+1,i) + c_2*int_ZETA(:,k+1,i-1);
        
        %% Difference of iterative sliding surfaces
        delta_s(:,k+1,i) = s(:,k+1,i) - s(:,k+1,i-1);
        
        %% Incremental I/O data
        delta_u(:,k,i) = u(:,k,i) - u(:,k,i-1);
        delta_y(:,k+1,i) = y(:,k+1,i) - y(:,k+1,i-1);
        
    end
    
    int_ZETA(:,:,i) = int_ZETA(:,:,i-1) + zeta(:,:,i);
    
    e_max(:,i) = max(abs(e(:,5:end,i)),[],2);
    e_MSE(:,i) = mse(e(:,:,i)')';
    e_IAE(:,i) = sum(abs(e(:,:,i)),2);
    
end

y_d = y_d(:,1:end-1);
d = d(:,1:end-1);

y = y(:,1:end-1,:);

e = e(:,1:end-1,:);
zeta = zeta(:,1:end-1,:);

s = s(:,1:end-1,:);
delta_s = delta_s(:,1:end-1,:);

myIter_1 = 20;

PHI_hat_mat = cell2mat(PHI_hat);
phi_hat_myIter_1 = PHI_hat_mat(:,nAgents*(myIter_1-1)+1:nAgents*myIter_1);
phi_hat_1_myIter_1 = phi_hat_myIter_1(1:4:end,1);
phi_hat_2_myIter_1 = phi_hat_myIter_1(2:4:end,2);
phi_hat_3_myIter_1 = phi_hat_myIter_1(3:4:end,3);
phi_hat_4_myIter_1 = phi_hat_myIter_1(4:4:end,4);

phi_hat_nIter = PHI_hat_mat(:,nAgents*(nIter-1)+1:nAgents*nIter);
phi_hat_1_nIter = phi_hat_nIter(1:4:end,1);
phi_hat_2_nIter = phi_hat_nIter(2:4:end,2);
phi_hat_3_nIter = phi_hat_nIter(3:4:end,3);
phi_hat_4_nIter = phi_hat_nIter(4:4:end,4);

toc;
%% Plot results
LineWidth = 1.5;
LineWidth_d = 2;

answ = questdlg('Which index would you like to choose for errors?', ...
                'Index choice', ...
                'Max','MSE','IAE','IAE');
switch answ
    case 'Max'
        iterative_index = e_max;
    case 'MSE'
        iterative_index = e_MSE;
    case 'IAE'
        iterative_index = e_IAE;
end

%% PPD estimations
% figure(1);
% subplot(1,2,1);
% plot(t,phi_hat_1_myIter_1,'b-','LineWidth',LineWidth);
% hold on;
% plot(t,phi_hat_2_myIter_1,'r--','LineWidth',LineWidth);
% plot(t,phi_hat_3_myIter_1,'m-.','LineWidth',LineWidth);
% plot(t,phi_hat_4_myIter_1,'g:','LineWidth',LineWidth);
% grid on;
% xlabel('Time step');
% ylabel('Value');
% legend('Agent #1','Agent #2','Agent #3','Agent #4');
% title(['Iteration #',num2str(myIter_1)]);
% 
% subplot(1,2,2);
% plot(t,phi_hat_1_nIter,'b-','LineWidth',LineWidth);
% hold on;
% plot(t,phi_hat_2_nIter,'r--','LineWidth',LineWidth);
% plot(t,phi_hat_3_nIter,'m-.','LineWidth',LineWidth);
% plot(t,phi_hat_4_nIter,'g:','LineWidth',LineWidth);
% grid on;
% xlabel('Time step');
% ylabel('Value');
% legend('Agent #1','Agent #2','Agent #3','Agent #4');
% title(['Iteration #',num2str(nIter)]);
% 
% suptitle('PPD estimation');

%% Difference of iterative sliding surfaces in last iteration number
figure(2);
subplot(1,2,1);
plot(t,delta_s(1,:,myIter_1),'b-','LineWidth',LineWidth);
hold on;
plot(t,delta_s(2,:,myIter_1),'r--','LineWidth',LineWidth);
plot(t,delta_s(3,:,myIter_1),'m-.','LineWidth',LineWidth);
plot(t,delta_s(4,:,myIter_1),'g:','LineWidth',LineWidth);
grid on;
xlabel('Time step');
ylabel('\Delta{s}');
legend('Agent #1','Agent #2','Agent #3','Agent #4');
title(['Iteration #',num2str(myIter_1)]);

subplot(1,2,2);
plot(t,delta_s(1,:,nIter),'b-','LineWidth',LineWidth);
hold on;
plot(t,delta_s(2,:,nIter),'r--','LineWidth',LineWidth);
plot(t,delta_s(3,:,nIter),'m-.','LineWidth',LineWidth);
plot(t,delta_s(4,:,nIter),'g:','LineWidth',LineWidth);
grid on;
xlabel('Time step');
ylabel('\Delta{s}');
legend('Agent #1','Agent #2','Agent #3','Agent #4');
title(['Iteration #',num2str(nIter)]);

%% Formation protocols
figure(3);
subplot(1,2,1);
plot(t,u(1,:,myIter_1),'b-','LineWidth',LineWidth);
hold on;
plot(t,u(2,:,myIter_1),'r--','LineWidth',LineWidth);
plot(t,u(3,:,myIter_1),'m-.','LineWidth',LineWidth);
plot(t,u(4,:,myIter_1),'g:','LineWidth',LineWidth);
grid on;
xlabel('Time step');
ylabel('Value');
legend('Agent #1','Agent #2','Agent #3','Agent #4');
title(['Iteration #',num2str(myIter_1)]);

subplot(1,2,2);
plot(t,u(1,:,nIter),'b-','LineWidth',LineWidth);
hold on;
plot(t,u(2,:,nIter),'r--','LineWidth',LineWidth);
plot(t,u(3,:,nIter),'m-.','LineWidth',LineWidth);
plot(t,u(4,:,nIter),'g:','LineWidth',LineWidth);
grid on;
xlabel('Time step');
ylabel('Value');
legend('Agent #1','Agent #2','Agent #3','Agent #4');
title(['Iteration #',num2str(nIter)]);

%% All errors
figure(4);
subplot(1,2,1);
plot(t,e(1,:,myIter_1),'b-','LineWidth',LineWidth);
hold on;
plot(t,e(2,:,myIter_1),'r--','LineWidth',LineWidth);
plot(t,e(3,:,myIter_1),'m-.','LineWidth',LineWidth);
plot(t,e(4,:,myIter_1),'g:','LineWidth',LineWidth);
grid on;
xlabel('Time step');
ylabel('Value');
legend('Agent #1','Agent #2','Agent #3','Agent #4');
title(['Iteration #',num2str(myIter_1)]);

subplot(1,2,2);
plot(t,e(1,:,nIter),'b-','LineWidth',LineWidth);
hold on;
plot(t,e(2,:,nIter),'r--','LineWidth',LineWidth);
plot(t,e(3,:,nIter),'m-.','LineWidth',LineWidth);
plot(t,e(4,:,nIter),'g:','LineWidth',LineWidth);
grid on;
xlabel('Time step');
ylabel('Value');
legend('Agent #1','Agent #2','Agent #3','Agent #4');
title(['Iteration #',num2str(nIter)]);

%% Formation errors
figure(5);
subplot(1,2,1);
plot(t,zeta(1,:,myIter_1),'b-','LineWidth',LineWidth);
hold on;
plot(t,zeta(2,:,myIter_1),'r--','LineWidth',LineWidth);
plot(t,zeta(3,:,myIter_1),'m-.','LineWidth',LineWidth);
plot(t,zeta(4,:,myIter_1),'g:','LineWidth',LineWidth);
grid on;
xlabel('Time step');
ylabel('Value');
legend('Agent #1','Agent #2','Agent #3','Agent #4');
title(['Iteration #',num2str(myIter_1)]);

subplot(1,2,2);
plot(t,zeta(1,:,nIter),'b-','LineWidth',LineWidth);
hold on;
plot(t,zeta(2,:,nIter),'r--','LineWidth',LineWidth);
plot(t,zeta(3,:,nIter),'m-.','LineWidth',LineWidth);
plot(t,zeta(4,:,nIter),'g:','LineWidth',LineWidth);
grid on;
xlabel('Time step');
ylabel('Value');
legend('Agent #1','Agent #2','Agent #3','Agent #4');
title(['Iteration #',num2str(nIter)]);

%% Iterative index
figure(6);
plot(iterative_index(1,:),'b-','LineWidth',LineWidth);
hold on;
plot(iterative_index(2,:),'r--','LineWidth',LineWidth);
plot(iterative_index(3,:),'m-.','LineWidth',LineWidth);
plot(iterative_index(4,:),'g:','LineWidth',LineWidth);
grid on;
xlabel('Iteration number');
ylabel('Value');
legend('Agent #1','Agent #2','Agent #3','Agent #4');

title('Iterative index');

%% Outputs
figure(7);
subplot(2,1,1);
plot(t,y_d,'k-o','LineWidth',LineWidth_d,'MarkerSize',5);
hold on;
plot(t,y(1,:,myIter_1),'b-','LineWidth',LineWidth);
plot(t,y(2,:,myIter_1),'r--','LineWidth',LineWidth);
plot(t,y(3,:,myIter_1),'m-.','LineWidth',LineWidth);
plot(t,y(4,:,myIter_1),'g:','LineWidth',LineWidth);
grid on;
xlabel('Time step');
ylabel('Value');
legend('Leader','Agent #1','Agent #2','Agent #3','Agent #4','Orientation','Horizontal','Location','best');
title(['Iteration #',num2str(myIter_1)]);

subplot(2,1,2);
plot(t,y_d,'k-o','LineWidth',LineWidth_d,'MarkerSize',5);
hold on;
plot(t,y(1,:,nIter),'b-','LineWidth',LineWidth);
plot(t,y(2,:,nIter),'r--','LineWidth',LineWidth);
plot(t,y(3,:,nIter),'m-.','LineWidth',LineWidth);
plot(t,y(4,:,nIter),'g:','LineWidth',LineWidth);
grid on;
xlabel('Time step');
ylabel('Value');
legend('Leader','Agent #1','Agent #2','Agent #3','Agent #4','Orientation','Horizontal','Location','best');
title(['Iteration #',num2str(nIter)]);
