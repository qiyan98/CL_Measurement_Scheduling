%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%created by Solmaz S. Kia%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised April 2014%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%UCSD%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%modified by Qi Yan%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised August 2018%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [K,S_ab,r_a] = Update_CEKF(a,b,X_a,X_b,phi_true_in,X_hat_a,X_hat_b,phi_est_in,P_k,k,noises)
%% load dt and N when this function is firstly used
persistent dt N n_x fun_H_a fun_H_b C_mat;
if isempty(dt)
    [dt,~] = IterationInit();
end
if isempty(N)
%     [~,~,~,N] = RobotInit();
    N = length(phi_est_in);
    n_x=2*N;
end
if (isempty(fun_H_a) || isempty(fun_H_b))
    C_mat = @(x)[cos(x) -sin(x);sin(x) cos(x)];
%     fun_H_a = @(x1,x2)[-cos(x1(3)) -sin(x1(3)) -(x2(1)-x1(1))*sin(x1(3))+(x2(2)-x1(2))*cos(x1(3));...
%                     sin(x1(3)) -cos(x1(3)) -(x2(1)-x1(1))*cos(x1(3))-(x2(2)-x1(2))*sin(x1(3));...
%                     0 0 -1];
%     fun_H_b = @(x1,x2)[cos(x1(3)) sin(x1(3)) 0; -sin(x1(3)) cos(x1(3)) 0; 0 0 1];
    fun_H_a = @(phi)C_mat(phi)'*-eye(2);
    fun_H_b = @(phi)C_mat(phi)'*eye(2);
end
%%
[R_rel,R_abs] = ExteroVar(k); % measurement noise variance

x_true_a = X_a(1);
y_true_a = X_a(2);
phi_true_a = phi_true_in(a);

x_true_b = X_b(1);
y_true_b = X_b(2);
phi_true_b = phi_true_in(b);

if a~=b
    R_a=R_rel{1,a};
%     v_z_a=[sqrt(R_a(1,1))*noises(1);sqrt(R_a(2,2))*noises(2)];
elseif a == b
    clear R_a v_z_a;
    R_a = R_abs{1,a};
    v_z_a = [sqrt(R_a(1,1))*noises(4);sqrt(R_a(2,2))*noises(5)];
end

if a~=b
    x_est_a   =    X_hat_a(1);     %   X_hat_a(1);
    y_est_a   =    X_hat_a(2);     %   X_hat_a(2);
    phi_est_a =    phi_est_in(a);  %   phi_a;
    
    x_est_b   =    X_hat_b(1);     %   X_hat_b(1);
    y_est_b   =    X_hat_b(2);     %   X_hat_b(2);
    phi_est_b =    phi_est_in(b);  %   phi_b;
    
    C = [cos(phi_true_a)  -sin(phi_true_a)
        sin(phi_true_a)   cos(phi_true_a)];
%     z_ab=[C'*([x_true_b-x_true_a;y_true_b-y_true_a])] + v_z_a;
%     z_ab=[C'*([x_true_b-x_true_a;y_true_b-y_true_a])];
    
    rho_true = sqrt((x_true_b-x_true_a)^2 + (y_true_b-y_true_a)^2);
    theta_true = atan2(y_true_b-y_true_a,x_true_b-x_true_a) - phi_true_a;
    
    rho_noise = sqrt(R_a(1,1))*noises(1);
    theta_noise = sqrt(R_a(2,2))*noises(2);
    
    rho_est = sqrt((x_est_b-x_est_a)^2 + (y_est_b-y_est_a)^2);
    theta_est = atan2(y_est_b-y_est_a,x_est_b-x_est_a) - phi_est_a;
    v_z_a = [cos(theta_est)     -rho_est*sin(theta_est);...
             sin(theta_est)      rho_est*cos(theta_est)]...
            *[rho_noise;theta_noise];
    z_ab = rho_true*[cos(theta_true); sin(theta_true)] + v_z_a;
%     z_ab = rho_true*[cos(theta_true); sin(theta_true)];
    
    C_est = [cos(phi_est_a)  -sin(phi_est_a)
             sin(phi_est_a)   cos(phi_est_a)];
    z_ab_est = [C_est'*([x_est_b-x_est_a;y_est_b-y_est_a])];
    
elseif a == b
    z_ab = [x_true_a;y_true_a] + v_z_a;
    
    x_est_a   =    X_hat_a(1);
    y_est_a   =    X_hat_a(2);
    z_ab_est = [x_est_a;y_est_a];
end

r_a = z_ab - z_ab_est;

if a~=b % inter-robot measurement
    H_a = fun_H_a(phi_est_in(a));
    H_b = fun_H_b(phi_est_in(a));
    P_a = P_k(2*a-1:2*a,2*a-1:2*a); P_b = P_k(2*b-1:2*b,2*b-1:2*b);
    P_ab = P_k(2*a-1:2*a,2*b-1:2*b); P_ba = P_k(2*b-1:2*b,2*a-1:2*a);
    S_ab = R_a + H_a*P_a*H_a' + H_b*P_b*H_b' + H_a*P_ab*H_b' + H_b*P_ba*H_a';
    K = cell(1,N);
    for j = 1:N
        P_ja = P_k(2*j-1:2*j,2*a-1:2*a);
        P_jb = P_k(2*j-1:2*j,2*b-1:2*b); 
        K{j} = (P_ja*H_a' + P_jb*H_b')*S_ab^-1;
    end
    K = cell2mat(K');
    k=k;
elseif a == b
    H_a = eye(2);
    P_a = P_k(2*a-1:2*a,2*a-1:2*a);
    S_ab = R_a + H_a*P_a*H_a';
    K = cell(1,N);
    for j = 1:N
        P_ja = P_k(2*j-1:2*j,2*a-1:2*a);
        K{j} = (P_ja*H_a')*S_ab^-1;
    end
    K = cell2mat(K');
end
end

