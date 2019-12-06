%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%created by Solmaz S. Kia%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised April 2014%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%UCSD%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%modified by Qi Yan%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised Nov. 2019%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [X_trueProp,phi_true_out,phi_est_out,sigma_V,sigma_phi_out] = TrueRobotModel(X_k,phi_true_in,k,noise_v,noise_w)
%% load dt and N when this function is firstly used
persistent dt N sigma_phi
if isempty (dt)
    [dt,~] = IterationInit();
    sigma_phi = 2/57.3;     % standard deviation for absolute orientation measurement
end
if isempty(N)
%     [~,~,~,N] = RobotInit();
    N = length(X_k);
end
%% computation for true robot motion
[V,W] = TrajGen(k);
[sigma_V,sigma_W] = ProprioVar(V,k);
X_trueProp = cell(1,N);
phi_true_out = zeros(1,N); phi_est_out = zeros(1,N);
for i=1:N
    V_true = V(i) - sigma_V(i)*noise_v(i);
    W_true = W(i) - sigma_W(i)*noise_w(i);
    cur_phi = phi_true_in(i);
    X_trueProp{1,i} = X_k{1,i} + [V_true*dt*cos(cur_phi); V_true*dt*sin(cur_phi)];
    phi_true_out(i) = phi_true_in(i) + W_true*dt;
    phi_est_out(i) = phi_true_out(i) + randn*sigma_phi;
end
sigma_phi_out = repmat(sigma_phi,[1 N]);
end
