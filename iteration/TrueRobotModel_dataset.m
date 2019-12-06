%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%created by Solmaz S. Kia%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised April 2014%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%UCSD%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%modified by Qi Yan%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised Nov. 2019%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [X_trueProp,phi_true_out,phi_est_out,sigma_V,sigma_phi_out] = TrueRobotModel_dataset(X_k,k)
%% load dt and N when this function is firstly used
persistent dt N sigma_phi dataset;
if isempty (dt)
    [dt,~] = IterationInit();
    sigma_phi = 2/57.3;     % standard deviation for absolute orientation measurement
    dataset = load('MRCLAMdata.mat');
end
if isempty(N)
%     [~,~,~,N] = RobotInit();
    N = length(X_k);
end
%% computation for true robot motion
% [V,W] = TrajGen(k);
V = zeros(N,1); W = zeros(N,1);
for i=1:N
    V(i) = eval(['dataset.Robot',num2str(i),'_Odometry(k,2);']);
    W(i) = eval(['dataset.Robot',num2str(i),'_Odometry(k,3);']);
end

[sigma_V,~] = ProprioVar(V,k);
X_trueProp = cell(1,N);
phi_true_out = zeros(1,N); phi_est_out = zeros(1,N);
for i=1:N
    X_trueProp{1,i} = eval(['dataset.Robot',num2str(i),'_Groundtruth(k+1,2:3);']);
    X_trueProp{1,i} = X_trueProp{1,i}';
    phi_true_out(i) = eval(['dataset.Robot',num2str(i),'_Groundtruth(k+1,4);']);
    phi_est_out(i) = phi_true_out(i) + randn*sigma_phi;
end
sigma_phi_out = repmat(sigma_phi,[1 N]);
end
