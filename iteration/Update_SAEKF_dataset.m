%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%created by Solmaz S. Kia%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised April 2014%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%UCSD%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%modified by Qi Yan%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised August 2018%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [r_a_bar_send,Gamma_col_send] = Update_SAEKF(a,b,X_a,X_b,phi_true_in,X_hat_a,P_a,Phi_a,X_hat_b,P_b,Phi_b,phi_est_in,k,noises)
%% load dt and N when this function is firstly used
persistent dt N n_x fun_H_a fun_H_b C_mat PI;
if isempty(N)
%     [~,~,~,N] = RobotInit();
    N = length(phi_true_in);
    n_x = 2*N;
    PI = cell(1,N*(N-1)/2);
    for i=1:N*(N-1)/2
        PI{i} = zeros(2);
    end
end
if isempty(dt)
    [dt,~,~,~,~,~,~,coef_Z] = IterationInit();
    coef_Z = coef_Z(end);
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
%% collect measurement data
[R_rel,R_abs] = ExteroVar(k); % measurement noise variance

x_true_a = X_a(1);
y_true_a = X_a(2);
phi_true_a = phi_true_in(a);

x_true_b = X_b(1);
y_true_b = X_b(2);
phi_true_b = phi_true_in(b);

if a~=b
    R_a=R_rel{1,a};
    v_z_a=[sqrt(R_a(1,1))*noises(1);sqrt(R_a(2,2))*noises(2)];
elseif a == b
    clear R_a v_z_a;
    R_a = R_abs{1,a}; 
    v_z_a=[sqrt(R_a(1,1))*noises(4);sqrt(R_a(2,2))*noises(5)];
end

if a~=b
    x_est_a   =    X_hat_a(1);     %   X_hat_a(1);
    y_est_a   =    X_hat_a(2);     %   X_hat_a(2);
    phi_est_a =    phi_est_in(a);  %   phi_a;
    
    x_est_b   =    X_hat_b(1);     %   X_hat_b(1);
    y_est_b   =    X_hat_b(2);     %   X_hat_b(2);
    phi_est_b =    phi_est_in(b);  %   phi_b;
    
%     C = [cos(phi_true_a)  -sin(phi_true_a)
%         sin(phi_true_a)   cos(phi_true_a)];
%     z_ab=[C'*([x_true_b-x_true_a;y_true_b-y_true_a])] + v_z_a;
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
    
    C_est = [cos(phi_est_a)  -sin(phi_est_a)
             sin(phi_est_a)   cos(phi_est_a)];
    z_ab_est = [C_est'*([x_est_b-x_est_a;y_est_b-y_est_a])];
    
elseif a == b
    z_ab = [x_true_a;y_true_a] + v_z_a;
    
    x_est_a   =    X_hat_a(1);
    y_est_a   =    X_hat_a(2);
    z_ab_est  = [x_est_a;y_est_a];
end
r_a = z_ab - z_ab_est;

%% Calcualte S_ab
if a ~= b % inter-robot measurement
    H_a = fun_H_a(phi_est_in(a));
    H_b = fun_H_b(phi_est_in(a));
    if a > b
        Pi_ba = PI{find_index(b,a)};
        Pi_ab = Pi_ba';
    elseif a < b
        Pi_ab = PI{find_index(a,b)};
        Pi_ba = Pi_ab';
    end
    S_ab = R_a + H_a*P_a*H_a' + H_a*Phi_a*Pi_ab*Phi_b'*H_b' ...
               + H_b*P_b*H_b' + H_b*Phi_b*Pi_ba*Phi_a'*H_a';
elseif a == b
    H_a = eye(2);
    S_ab = R_a + H_a*P_a*H_a';
end
%% calculation at the server
Gamma_col = cell(1,N);
if a ~= b % inter-robot measurement
    for i = 1:N
        if i == a
            Gamma_col{i} = (Phi_a^-1*P_a*H_a' + Pi_ab*Phi_b'*H_b')*S_ab^-0.5;
        elseif i == b
            Gamma_col{i} = (Phi_b^-1*P_b*H_b' + Pi_ba*Phi_a'*H_a')*S_ab^-0.5;
        else % not a or b
            if i > a
                Pi_ia = PI{find_index(i,a)}';
            elseif i < a
                Pi_ia = PI{find_index(i,a)};
            end
            if i > b
                Pi_ib = PI{find_index(i,b)}';
            elseif i < b
                Pi_ib = PI{find_index(i,b)};
            end
            Gamma_col{i} = (Pi_ia*Phi_a'*H_a' + Pi_ib*Phi_b'*H_b')*S_ab^-0.5;
        end
    end
elseif a == b % absolute positioning
    for i = 1:N
        if i == a
            Gamma_col{i} = (Phi_a^-1*P_a*H_a')*S_ab^-0.5;
        else % not a or b
            if i > a
                Pi_ia = PI{find_index(i,a)}';
            elseif i < a
                Pi_ia = PI{find_index(i,a)};
            end
            Gamma_col{i} = (Pi_ia*Phi_a'*H_a')*S_ab^-0.5;
        end
    end
end
r_a_bar = S_ab^-0.5*r_a;
%% update the variables stored at server
Gamma_col_send = Gamma_col;
r_a_bar_send = r_a_bar;

for i=1:N
    for j=i+1:N
        PI{find_index(i,j)} = PI{find_index(i,j)} - Gamma_col{i}*Gamma_col{j}';
    end
end

end

function index = find_index(a,b)
persistent N data_index;
if isempty(N)
    [~,~,~,N] = RobotInit_dataset();
    data_index = zeros(N);k = 1;
    for i = 1:N
        for j = i+1:N
            data_index(i,j) = k;
            k = k + 1;
        end
    end
end
% find the PI index for correlation between a,b
if a == b
    disp('a = b is not allowed!')
    return;
elseif a>b % so that a < b always holds
    tmp = a; a = b; b = tmp;
end
index = data_index(a,b);
end
