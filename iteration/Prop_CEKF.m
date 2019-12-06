%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%created by Solmaz S. Kia%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised April 2014%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%UCSD%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%modified by Qi Yan%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised Nov. 2019%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [X_Prop,P_Prop] = Prop_CEKF(X_hat_k,phi_est_in,P_k,k,sigma_V,sigma_phi)
%% load dt and N when this function is firstly used
persistent dt N fun_F fun_G;
if isempty(N)
%     [~,~,~,N] = RobotInit();
    N = length(X_hat_k);
end
if isempty (dt)
    [dt,~] = IterationInit();
end
if isempty(fun_F)
%     fun_F = @(x,u) [1 0 -u(1)*sin(x(3))*dt; 0 1 u(1)*cos(x(3))*dt; 0 0 1]; % Jacobian matrix for system function (states)
%     fun_G = @(x,u) [dt*cos(x(3)) 0;dt*sin(x(3)) 0;0 dt]; % Jacobian matrix for system function (inputs)
    fun_F = eye(2);
%     fun_G = @(v,phi)dt*[cos(phi) -v*sin(phi);sin(phi) v*cos(phi)];
    fun_G = @(phi)[cos(phi) -sin(phi);sin(phi) cos(phi)];
end
%%
[V,W] = TrajGen(k);
Q = cell(1,N);
for i = 1:N
    Q{i} = dt^2*diag([sigma_V(i)^2,V(i)^2*sigma_phi(i)^2]);
end

%% EKF Propagation robot-wisely
X_Prop = cell(1,N); this_F = cell(1,N); this_G = cell(1,N);
for i = 1:N
    % after going through the motion equation
    cur_phi = phi_est_in(i);
    X_Prop{i} = X_hat_k{i} + [V(i)*dt*cos(cur_phi);V(i)*dt*sin(cur_phi)];
    this_F{i} = fun_F;
    this_G{i} = fun_G(cur_phi);
end
this_F = blkdiag(this_F{:});
this_G = blkdiag(this_G{:});
P_Prop = this_F*P_k*this_F' + this_G*blkdiag(Q{:})*this_G';
end