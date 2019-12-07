%%%%%%%%%%created by Qi Yan%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised Nov. 2019%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [X0,X_hat0,P0,N] = RobotInit()
%% Robots initialization
% X0     : initial true poses
% X_hat0 : initial estimated poses and covariances
% P0     : initial collective covariances
% N      : number of robots

%% Robot number
N = 9; % for simulation
% N = 5; % for dataset
R2D = 180/pi;
D2R = pi/180;

%% True system pose X{ID}
X0 = cell(1,N);
X0{1} = [3,9]';
X0{2} = [6,9]';
X0{3} = [9,9]';
X0{4} = [3,3]';
X0{5} = [6,3]';
X0{6} = [9,3]';
X0{7} = [3,6]';
X0{8} = [6,6]';
X0{9} = [9,6]';
X0{10}= [20,-20]';

%% Robot Covariance P{ID}
P_coll = cell(1,N);
sigma_init = 0.1;
for i = 1:N
    P_coll{i} = diag([sigma_init^2,sigma_init^2]);
end
P0 = blkdiag(P_coll{:});
%% Estimated pose Xhat{ID}
X_hat0=cell(1,N);
for i = 1:N
    X_hat0{i}=X0{i} + [sigma_init*randn;sigma_init*randn];
end
end