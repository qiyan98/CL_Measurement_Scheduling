%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%created by Solmaz S. Kia%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised July 2015%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%UCI%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%modified by Qi Yan%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised Nov. 2019%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% at this moment, data set 7 is used
addpath('dataset');
% loadMRCLAMdataSet;
% sampleMRCLAMdataSet;

tic;
% clear;
clear all
close all;
addpath('initialization');
addpath('iteration');
addpath('visualization');
load('MRCLAMdata.mat');
rng(1,'twister');

NoiseGen(); % generate noise once before running simulation
%%%%%%%%%%%%%%%%%%%%%%k%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization
%Missed_on_off = 0: Every one updates
%Missed_on_off = 1: Only two robots making relative measurement update
%Missed_on_off = 2: No CL
Missed_on_off = 0;
% cross_on = 1;
% 
% color_plot = 'r';
% if Missed_on_off == 0 && cross_on==1
%     color_plot = 'r';
% end
% if Missed_on_off == 0 && cross_on==0
%     color_plot = 'b';
% end
% 
% if Missed_on_off==2
%     color_plot = 'k';
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%Iteration Settings%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[delta,k_f,~,~,~,RelMea_Table,UpdateOrder,mea_para] = IterationInit_dataset(Missed_on_off);
% delta                 : stepsize
% k_f                   : total number of steps
% RelMea_Table          : relative measurements occurrence table
% UpdateOrder           : update order for sequential updating
% mea_para              : some legacy parameters

delta = sample_time;
% k_f = timesteps-1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%Robot Prameters%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[~,~,P0,N] = RobotInit_dataset();
% X0     : initial true poses
% X_hat0 : initial estimated poses and covariances
% P0     : initial collective covariances
% N      : number of robots

N = n_robots;

P_coll = cell(1,N);
for i = 1:N
    P_coll{i} = diag([0.1^2,0.1^2]);
end
P0 = blkdiag(P_coll{:});

for i=1:N
    X0{i} = eval(['Robot',num2str(i),'_Groundtruth(1,2:3);']);
    X0{i} = X0{i}';
    X_hat0{i} = X0{i} + [0.1*randn;0.1*randn];
end


n_x = 2*N; %total number of states
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X_true = cell(k_f,N);               % true states
X_hat_DR = cell(k_f,N);             % states by dead reckoning only
X_hat_CEKF_subMOD = cell(k_f,N);    % states by standard EKF with submodular method
X_hat_SAEKF_subMOD = cell(k_f,N);   % states by server-assisted EKF with submodular method
X_hat_CEKF_Dense = cell(k_f,N);     % states by standard EKF with dense graph
X_hat_CEKF_Random = cell(k_f,N);    % states by standard EKF with random graph
X_hat_CEKF_subOPT = cell(k_f,N);    % states by standard EKF with sub-optimal scheduling
X_hat_CEKF_subMOD_multi = cell(k_f,N);       % states by standard EKF with OPT (q^i > 1)
X_hat_CEKF_subOPT_multi = cell(k_f,N);    % states by standard EKF with sub-optimal scheduling (q^i > 1)

phi_true = zeros(k_f,N);            % true orientations
phi_true(1,:) = rand(1,N)*360/57.3;
for i=1:N
    phi_true(1,i) = eval(['Robot',num2str(i),'_Groundtruth(1,4);']);
end  
phi_est = phi_true;                 % estimated orientations
phi_est(1,:) = phi_true(1,:) + randn(1,N)*2/57.3; 

P_DR = cell(k_f,1);               % collective covariance for dead reckoning
P_CEKF_subMOD = cell(k_f,1);      % collective covariance matrix for standard EKF with submodular method
P_SAEKF_subMOD = cell(k_f,N);     % robot-wise covariance matrix for SA-EKF with submodular method
Phi_SAEKF_subMOD = cell(k_f,N);   % robot-wise auxiliary matrix for SA-EKF with submodular method
P_CEKF_Dense = cell(k_f,1);       % collective covariance matrix for standard EKF with dense graph
P_CEKF_Random = cell(k_f,1);      % collective covariance matrix for standard EKF with random graph
P_CEKF_subOPT = cell(k_f,1);      % collective covariance matrix for standard EKF with sub-optimal graph
P_CEKF_subMOD_multi = cell(k_f,1);    % collective covariance matrix for standard EKF with OPT (q^i > 1)
P_CEKF_subOPT_multi = cell(k_f,1); % collective covariance matrix for standard EKF with sub-optimal graph (q^i > 1)

flag_robot = cell(N,1); % flags for each robot observation
mea_schedule = cell(k_f,1);
mea_schedule_OPT = cell(k_f,1);
mea_schedule_Random = cell(k_f,1);
mea_schedule_subOPT = cell(k_f,1);
mea_schedule_subMOD_multi = cell(k_f,1);
mea_schedule_subOPT_multi = cell(k_f,1);

% load initial information when k = 0
for i=1:N
    X_true{1,i} = X0{i};
    X_hat_DR{1,i} = X_hat0{i};
    X_hat_CEKF_subMOD{1,i} = X_hat0{i};
    X_hat_SAEKF_subMOD{1,i} = X_hat0{i};
    X_hat_CEKF_Dense{1,i} = X_hat0{i};
    X_hat_CEKF_Random{1,i} = X_hat0{i};
    X_hat_CEKF_subOPT{1,i} = X_hat0{i};
    X_hat_CEKF_subMOD_multi{1,i} = X_hat0{i};
    X_hat_CEKF_subOPT_multi{1,i} = X_hat0{i};
    flag_robot{i} = 1;
end
P_DR{1} = P0; P_CEKF_subMOD{1} = P0;
P_CEKF_Dense{1} = P0; P_CEKF_Random{1} = P0; P_CEKF_subOPT{1} = P0;
P_CEKF_subMOD_multi{1} = P0; P_CEKF_subOPT_multi{1} = P0;
for i=1:N
    P_SAEKF_subMOD{1,i} = P0(2*i-1:2*i,2*i-1:2*i);
    Phi_SAEKF_subMOD{1,i} = eye(2);
end
clear X0 X_hat0 P0;

load noise_profile.mat;
flag_mea = 0; % flag for measurement (relative or absolute)
flag_seq = 0; % flag for sequantial updating
j_mea = 1;    % times of measurement period

% number of communication times (= measurement times)
comm_CEKF_subMOD = zeros(k_f,1);
comm_SAEKF_subMOD = zeros(k_f,1);
comm_CEKF_Dense = zeros(k_f,1);
comm_CEKF_Random = zeros(k_f,1);
comm_CEKF_subOPT = zeros(k_f,1);
comm_CEKF_subMOD_multi = zeros(k_f,1);
comm_CEKF_subOPT_multi = zeros(k_f,1);

% collective 3 sigma boundaries
Three_sigma_DR = zeros(2,k_f,N);
Three_sigma_CEKF_subMOD = zeros(2,k_f,N);
Three_sigma_SAEKF_subMOD = zeros(2,k_f,N);
Three_sigma_CEKF_Dense = zeros(2,k_f,N);
Three_sigma_CEKF_Random = zeros(2,k_f,N);
Three_sigma_CEKF_subOPT = zeros(2,k_f,N);
Three_sigma_CEKF_subMOD_multi = zeros(2,k_f,N);
Three_sigma_CEKF_subOPT_multi = zeros(2,k_f,N);

% some anonymous functions
f_logdet = @(X)log(det(X));
fun_Hab_a = @(phi_a)[cos(phi_a) -sin(phi_a);sin(phi_a) cos(phi_a)]*-eye(2);
fun_Hab_b = @(phi_a)[cos(phi_a) -sin(phi_a);sin(phi_a) cos(phi_a)]*eye(2);

% some other useful variables
index_noise_extero = 1;
timing_subMOD = zeros(k_f,1);
timing_subOPT = zeros(k_f,1);
timing_subMOD_multi = zeros(k_f,1);
timing_subOPT_multi = zeros(k_f,1);

bytes_subMOD = zeros(k_f,1);
bytes_subOPT = zeros(k_f,1);
bytes_subMOD_multi = zeros(k_f,1);
bytes_subOPT_multi = zeros(k_f,1);
%% Iteration
for k=1:k_f
    disp([num2str(k),' out of ',num2str(k_f)]);
    for j=1:N % index of robot
        for i=1:2 % index of state in each robot
            Three_sigma_DR(i,k,j) = 3*sqrt(P_DR{k}(2*j+i-2,2*j+i-2));
            Three_sigma_CEKF_subMOD(i,k,j) = 3*sqrt(P_CEKF_subMOD{k}(2*j+i-2,2*j+i-2));
            Three_sigma_SAEKF_subMOD(i,k,j) = 3*sqrt(P_SAEKF_subMOD{k,j}(i,i));
            Three_sigma_CEKF_Dense(i,k,j) = 3*sqrt(P_CEKF_Dense{k}(2*j+i-2,2*j+i-2));
            Three_sigma_CEKF_Random(i,k,j) = 3*sqrt(P_CEKF_Random{k}(2*j+i-2,2*j+i-2));
            Three_sigma_CEKF_subOPT(i,k,j) = 3*sqrt(P_CEKF_subOPT{k}(2*j+i-2,2*j+i-2));
            Three_sigma_CEKF_subMOD_multi(i,k,j) = 3*sqrt(P_CEKF_subMOD_multi{k}(2*j+i-2,2*j+i-2));
            Three_sigma_CEKF_subOPT_multi(i,k,j) = 3*sqrt(P_CEKF_subOPT_multi{k}(2*j+i-2,2*j+i-2));
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % True robot model (pose propagation with noise)
    [X_true(k+1,:),phi_true(k+1,:),phi_est(k+1,:),sigma_V,sigma_phi] = TrueRobotModel_dataset(X_true(k,:),k);
    
    % dead reckoning propagation for pose and covariance
    [X_hatProp_DR,P_Prop_DR] = Prop_CEKF_dataset(X_hat_DR(k,:),phi_est(k,:),P_DR{k},k,sigma_V,sigma_phi);
    [X_hatProp_CEKF_subMOD,P_Prop_CEKF_subMOD] = Prop_CEKF_dataset(X_hat_CEKF_subMOD(k,:),phi_est(k,:),P_CEKF_subMOD{k},k,sigma_V,sigma_phi);
    [X_hatProp_SAEKF_subMOD,P_Prop_SAEKF_subMOD,Phi_Prop_SAEKF_subMOD,Q_Prop_SAEKF_subMOD] = Prop_SAEKF_dataset(X_hat_SAEKF_subMOD(k,:),phi_est(k,:),P_SAEKF_subMOD(k,:),Phi_SAEKF_subMOD(k,:),k,sigma_V,sigma_phi);
    [X_hatProp_CEKF_Dense,P_Prop_CEKF_Dense] = Prop_CEKF_dataset(X_hat_CEKF_Dense(k,:),phi_est(k,:),P_CEKF_Dense{k},k,sigma_V,sigma_phi);
    [X_hatProp_CEKF_Random,P_Prop_CEKF_Random] = Prop_CEKF_dataset(X_hat_CEKF_Random(k,:),phi_est(k,:),P_CEKF_Random{k},k,sigma_V,sigma_phi);
    [X_hatProp_CEKF_subOPT,P_Prop_CEKF_subOPT] = Prop_CEKF_dataset(X_hat_CEKF_subOPT(k,:),phi_est(k,:),P_CEKF_subOPT{k},k,sigma_V,sigma_phi);
    [X_hatProp_CEKF_subMOD_multi,P_Prop_CEKF_subMOD_multi] = Prop_CEKF_dataset(X_hat_CEKF_subMOD_multi(k,:),phi_est(k,:),P_CEKF_subMOD_multi{k},k,sigma_V,sigma_phi);
    [X_hatProp_CEKF_subOPT_multi,P_Prop_CEKF_subOPT_multi] = Prop_CEKF_dataset(X_hat_CEKF_subOPT_multi(k,:),phi_est(k,:),P_CEKF_subOPT_multi{k},k,sigma_V,sigma_phi);
    
    X_hat_DR(k+1,:) = X_hatProp_DR; P_DR{k+1} = P_Prop_DR;
    X_hat_CEKF_subMOD(k+1,:) = X_hatProp_CEKF_subMOD; P_CEKF_subMOD{k+1} = P_Prop_CEKF_subMOD;
    X_hat_SAEKF_subMOD(k+1,:) = X_hatProp_SAEKF_subMOD; P_SAEKF_subMOD(k+1,:) = P_Prop_SAEKF_subMOD; Phi_SAEKF_subMOD(k+1,:) = Phi_Prop_SAEKF_subMOD;
    X_hat_CEKF_Dense(k+1,:) = X_hatProp_CEKF_Dense; P_CEKF_Dense{k+1} = P_Prop_CEKF_Dense;
    X_hat_CEKF_Random(k+1,:) = X_hatProp_CEKF_Random; P_CEKF_Random{k+1} = P_Prop_CEKF_Random;
    X_hat_CEKF_subOPT(k+1,:) = X_hatProp_CEKF_subOPT; P_CEKF_subOPT{k+1} = P_Prop_CEKF_subOPT;
    X_hat_CEKF_subMOD_multi(k+1,:) = X_hatProp_CEKF_subMOD_multi; P_CEKF_subMOD_multi{k+1} = P_Prop_CEKF_subMOD_multi;
    X_hat_CEKF_subOPT_multi(k+1,:) = X_hatProp_CEKF_subOPT_multi; P_CEKF_subOPT_multi{k+1} = P_Prop_CEKF_subOPT_multi;
   
    comm_CEKF_subMOD(k+1) = comm_CEKF_subMOD(k);
    comm_SAEKF_subMOD(k+1) = comm_SAEKF_subMOD(k);
    comm_CEKF_Dense(k+1) = comm_CEKF_Dense(k);
    comm_CEKF_Random(k+1) = comm_CEKF_Random(k);
    comm_CEKF_subOPT(k+1) = comm_CEKF_subOPT(k);
    comm_CEKF_subMOD_multi(k+1) = comm_CEKF_subMOD_multi(k);
    comm_CEKF_subOPT_multi(k+1) = comm_CEKF_subOPT_multi(k);
    
    bytes_subMOD(k+1) = bytes_subMOD(k);
    bytes_subOPT(k+1) = bytes_subOPT(k);
    bytes_subMOD_multi(k+1) = bytes_subMOD_multi(k);
    bytes_subOPT_multi(k+1) = bytes_subOPT_multi(k);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % declaring the master and landmark at the beginning of measurement period
    if RelMea_Table{1,j_mea}(1,1) == k % if it is the starting moment of a measurement period
        cur_start_k = RelMea_Table{1,j_mea}(1,1);
        a_set_subMOD = RelMea_Table{1,j_mea}(2:end,1); % master robot identities
        b_set_subMOD = RelMea_Table{1,j_mea}(2:end,2); % landmark robot identities
        size_RelMea_Table = size(RelMea_Table{1,j_mea});
        
        % elements in MeaMat:
        % update sequence - robot_a - robot_b - robot_missed
%         MeaMat_subMOD = zeros(length(a_set_subMOD),size_RelMea_Table(2)+1);
%         for ii=1:length(a_set_subMOD)
%             MeaMat_subMOD(ii,1) = find(UpdateOrder == a_set_subMOD(ii));
%         end
%         MeaMat_subMOD(:,2:end) = RelMea_Table{1,j_mea}(2:end,1:end);
%         % Sorting the order of the update order based on the prespecified guidline saved in UpdateOrder
%         MeaMat_subMOD = sortrows(MeaMat_subMOD,1);
        
        %%%%%%%%% measurement-related flag settings %%%%%%%%%
        %         if size_RelMea_Table(1) > 2 % see if sequential measurement is on
        %             flag_seq = 1;
        %         end
        flag_seq = 1;
        flag_mea=1;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%if there is a reletive measurement%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if flag_mea == 1
        if mod(k-cur_start_k,mea_para(1)) == 0
            flag_update = 1;
        else
            flag_update = 0;
        end
        
        %%%%% optimize for measurement scheduling by changing MeaMat %%%%%
        if mod(k-cur_start_k,delta/delta) == 0 % change topology per some seconds
            P_col_subMOD = P_CEKF_subMOD{k+1};
            P_col_subMOD_multi = P_CEKF_subMOD_multi{k+1};
            
            cell_C = cell(1,N); [R_rel,~] = ExteroVar_dataset();
            ab_set = zeros(10,2);
            ii_sensor = 1;
            for i=1:N  % for each master robot
                cell_C{i} = zeros(2*(N-1),2*N); % all possible measurement matrix for each robot
                cur_row = 1;
                for j = [1:i-1,i+1:N] % for each landmark robot
                    H_a = fun_Hab_a(phi_est(k+1,i));
                    H_b = fun_Hab_b(phi_est(k+1,i));
                    cell_C{i}(2*cur_row-1:2*cur_row,2*i-1:2*i) = H_a;
                    cell_C{i}(2*cur_row-1:2*cur_row,2*j-1:2*j) = H_b;
                    cur_row=cur_row+1;
                    ab_set(ii_sensor,:) = [i,j];
                    ii_sensor = ii_sensor+1;
                end
            end
            for i = 1:N
                ab_set(end+1,:) = [i i]; % absolute measurement
            end
            clear cur_row ii_sensor H_a H_b
            
            mat_C = vertcat(cell_C{:});
            fun_mat_O = @(S) OPT_get_O(S,mat_C);
            cov_V = repmat(R_rel,1,(N-1));
            cov_V = blkdiag(cov_V{:});
            cov_Z = P_col_subMOD;
            cov_Z_post = @(S)cov_Z - cov_Z*fun_mat_O(S)'*(fun_mat_O(S)*cov_Z*fun_mat_O(S)' + cov_V)^-1*fun_mat_O(S)*cov_Z;
            f_H = @(S)f_logdet(cov_Z_post(S));
            
            cov_Z = P_col_subMOD_multi;
            cov_Z_post = @(S)cov_Z - cov_Z*fun_mat_O(S)'*(fun_mat_O(S)*cov_Z*fun_mat_O(S)' + cov_V)^-1*fun_mat_O(S)*cov_Z;
            f_H_multi = @(S)f_logdet(cov_Z_post(S));
            
%             if k > 0.1*k_f && k <= 0.2*k_f
%                 set_abs_mea = 1;
%             elseif k > 0.35*k_f && k <= 0.4*k_f
%                 set_abs_mea = 3;
%             elseif k > 0.6*k_f && k <= 0.65*k_f
%                 set_abs_mea = 2;
%             elseif k > 0.95*k_f && k <= k_f
%                 set_abs_mea = 5;
%             else
%                 set_abs_mea = [];
%             end
            set_abs_mea = [];
            
            num_sensor = N*(N-1);
            if k <= 0.2*k_f
                master_robot_allowed = [1 3 5];
            elseif k <= 0.35*k_f
                master_robot_allowed = [2 4];
            elseif k <= 0.4*k_f
                master_robot_allowed = [1 5];
            elseif k <= 0.6*k_f
                master_robot_allowed = [3 4];
            elseif k <= 0.65*k_f
                master_robot_allowed = 1;
            elseif k <= 0.8*k_f
                master_robot_allowed = [2 3];
            elseif k <= 0.95*k_f
                master_robot_allowed = [1 4];
            elseif k <= k_f
                master_robot_allowed = [4];
            end
%             master_robot_allowed = 1:N;
            r_sensor = length(master_robot_allowed);
            
            sensor_allowed = cell(length(master_robot_allowed),1);
            for i = 1:length(master_robot_allowed)
                cur_sensor_allowed = [];
                for aa = 1:num_sensor
                    cur_ab_set = ab_set(aa,:);
                    if cur_ab_set(1) == master_robot_allowed(i)
                        cur_sensor_allowed = [cur_sensor_allowed; aa];
                    end
                end
                sensor_allowed{i} = cur_sensor_allowed;
            end
            clear cur_set cur_sensor_allowed;
            
            q_i = 1; % number of measurements allowed per time step
            q_i_multi = 2; % q_i_multi must be larger than q_i_multi !!!
            
                        % for submodular greedy sensor selection, q_i
            tic;
            set_subMOD = cell(length(master_robot_allowed),1);
            for aa = 1:length(master_robot_allowed)
                cur_gain = -1e6;
                set_subMOD{aa} = [];
                cur_set_subMOD = [];
                cur_remain = q_i;
                while(cur_remain > 0)
                    tmp_gain = zeros(length(sensor_allowed{aa}),1);
                    for i = 1:length(sensor_allowed{aa})
                        cur_sensor = sensor_allowed{aa}(i);
                        if isempty(intersect(cur_set_subMOD,cur_sensor)) % if this sensor is not selected yet
                            tmp = [cur_set_subMOD cur_sensor]; % schedule after selecting sensor i
                            tmp_gain(i) = f_H(cur_set_subMOD) - f_H(tmp); % calculate the gain of selecting this sensor
                        else
                            tmp_gain(i) = -1; % if the sensor is selected, there is no gain
                        end
                    end
                    % choose the best sensor greedily
                    [~,tmp_I] = sort(tmp_gain,'descend');
                    cur_best_sensor = tmp_I(1);
                    tmp = sensor_allowed{aa};
                    cur_best_sensor = tmp(cur_best_sensor)';
                    cur_set_subMOD = [cur_set_subMOD cur_best_sensor];
                    cur_remain = cur_remain - 1;
                end
                if (length(cur_set_subMOD) == q_i)
                    set_subMOD{aa} = cur_set_subMOD';
                else
                    error('bad!');
                end
            end
            clear cur_best_snesor cur_set_subMOD cur_remain tmp tmp_gain tmp_I;
            set_subMOD = cell2mat(set_subMOD)';
            t_tmp = 1e3*toc/r_sensor;
            timing_subMOD(k) = t_tmp;
            disp(['Time per robot for submodualr method = ',num2str(t_tmp)]);
            comm_CEKF_subMOD(k+1) = comm_CEKF_subMOD(k+1) + (N-1);
            comm_SAEKF_subMOD(k+1) = comm_SAEKF_subMOD(k+1) + (N-1);
            bytes_subMOD(k+1) = bytes_subMOD(k+1) + q_i*(N-1)*4*8;
            % end of submodular greedy sensor selection, q_i
            
            % for submodular greedy sensor selection, q_i_multi
            tic;
            set_subMOD_multi = cell(length(master_robot_allowed),1);
            for aa = 1:length(master_robot_allowed)
                cur_gain = -1e6;
                set_subMOD_multi{aa} = [];
                cur_set_subMOD_multi = [];
                cur_remain = q_i_multi;
                while(cur_remain > 0)
                    tmp_gain = zeros(length(sensor_allowed{aa}),1);
                    for i = 1:length(sensor_allowed{aa})
                        cur_sensor = sensor_allowed{aa}(i);
                        if isempty(intersect(cur_set_subMOD_multi,cur_sensor)) % if this sensor is not selected yet
                            tmp = [cur_set_subMOD_multi cur_sensor]; % schedule after selecting sensor i
                            tmp_gain(i) = f_H_multi(cur_set_subMOD_multi) - f_H_multi(tmp); % calculate the gain of selecting this sensor
                        else
                            tmp_gain(i) = -1; % if the sensor is selected, there is no gain
                        end
                    end
                    % choose the best sensor greedily
                    [~,tmp_I] = sort(tmp_gain,'descend');
                    cur_best_sensor = tmp_I(1);
                    tmp = sensor_allowed{aa};
                    cur_best_sensor = tmp(cur_best_sensor)';
                    cur_set_subMOD_multi = [cur_set_subMOD_multi cur_best_sensor];
                    cur_remain = cur_remain - 1;
                end
                if (length(cur_set_subMOD_multi) == q_i_multi)
                    set_subMOD_multi{aa} = cur_set_subMOD_multi';
                else
                    error('bad!');
                end
            end
            clear cur_best_snesor cur_set_subMOD_multi cur_remain tmp tmp_gain tmp_I;
            set_subMOD_multi = cell2mat(set_subMOD_multi)';
            t_tmp = 1e3*toc/r_sensor;
            timing_subMOD_multi(k) = t_tmp;
            disp(['Time per robot for submodualr method (multi) = ',num2str(t_tmp)]);
            comm_CEKF_subMOD_multi(k+1) = comm_CEKF_subMOD_multi(k+1) + (N-1);
            bytes_subMOD_multi(k+1) = bytes_subMOD_multi(k+1) + q_i_multi*(N-1)*4*8;
            % end of submodular greedy sensor selection, q_i_multi
            
            % for random sensor selection
            set_random = randi(num_sensor,1,r_sensor);
            
            % for our proposed sub-optimal selection, q_i
            tic;
            P_col_subOPT = P_CEKF_subOPT{k+1};
            obj_vec = zeros(1,num_sensor);
            for aa = 1:num_sensor
                cur_i = ab_set(aa,1);
                cur_j = ab_set(aa,2);
                P_ii = P_col_subOPT(2*cur_i-1:2*cur_i,2*cur_i-1:2*cur_i);
                P_ij = P_col_subOPT(2*cur_i-1:2*cur_i,2*cur_j-1:2*cur_j);
                P_ji = P_col_subOPT(2*cur_j-1:2*cur_j,2*cur_i-1:2*cur_i);
                P_jj_hat = P_ji*P_ii^-1*P_ij;
                obj_vec(aa) = trace(P_ii + P_jj_hat - P_ij - P_ji);
            end
            [obj_sorted,idx_obj_sorted] = sort(obj_vec,'descend'); % choose the largest J
            
            idx_obj_allowed_sorted = cell(length(master_robot_allowed),1);
            for aa = 1:length(master_robot_allowed)
                cur_idx = idx_obj_sorted;
                for bb = 1:num_sensor
                    cur_ab_set = ab_set(cur_idx(bb),:);
                    if isempty(intersect(cur_ab_set(1),master_robot_allowed(aa))) % this measurement is not allowed
                        cur_idx(bb) = 0;
                    end
                end
                cur_idx(cur_idx==0) = []; % delete not allowed measurements
                idx_obj_allowed_sorted{aa} = cur_idx;
            end
            set_subopt = [];
            for aa = 1:length(master_robot_allowed)
                set_subopt = [set_subopt idx_obj_allowed_sorted{aa}(1:q_i)];
            end
            t_tmp = 1e3*toc/r_sensor;
            timing_subOPT(k) = t_tmp;
            disp(['Time per robot for subopt method = ',num2str(t_tmp)]);
            clear t_tmp;
            % end of our proposed sub-optimal selection, q_i
            
            % for our proposed sub-optimal selection, q_i_multi
            tic;
            P_col_subOPT_multi = P_CEKF_subOPT_multi{k+1};
            obj_vec = zeros(1,num_sensor);
            for aa = 1:num_sensor
                cur_i = ab_set(aa,1);
                cur_j = ab_set(aa,2);
                P_ii = P_col_subOPT_multi(2*cur_i-1:2*cur_i,2*cur_i-1:2*cur_i);
                P_ij = P_col_subOPT_multi(2*cur_i-1:2*cur_i,2*cur_j-1:2*cur_j);
                P_ji = P_col_subOPT_multi(2*cur_j-1:2*cur_j,2*cur_i-1:2*cur_i);
                P_jj_hat = P_ji*P_ii^-1*P_ij;
                obj_vec(aa) = trace(P_ii + P_jj_hat - P_ij - P_ji);
            end
            [obj_sorted,idx_obj_sorted] = sort(obj_vec,'descend'); % choose the largest J
            
            idx_obj_allowed_sorted = cell(length(master_robot_allowed),1);
            for aa = 1:length(master_robot_allowed)
                cur_idx = idx_obj_sorted;
                for bb = 1:num_sensor
                    cur_ab_set = ab_set(cur_idx(bb),:);
                    if isempty(intersect(cur_ab_set(1),master_robot_allowed(aa))) % this measurement is not allowed
                        cur_idx(bb) = 0;
                    end
                end
                cur_idx(cur_idx==0) = []; % delete not allowed measurements
                idx_obj_allowed_sorted{aa} = cur_idx;
            end
            set_subopt_multi = [];
            for aa = 1:length(master_robot_allowed)
                set_subopt_multi = [set_subopt_multi idx_obj_allowed_sorted{aa}(1:q_i_multi)];
            end
            t_tmp = 1e3*toc/r_sensor;
            timing_subOPT_multi(k) = t_tmp;
            disp(['Time per robot for subopt (multi) method = ',num2str(t_tmp)]);
            clear t_tmp;
            % end of our proposed sub-optimal selection, q_i_multi
            
            set_subMOD = [set_subMOD N*(N-1)+set_abs_mea];
            set_subMOD_multi = [set_subMOD_multi N*(N-1)+set_abs_mea];
            set_random = [set_random N*(N-1)+set_abs_mea];
            set_dense = [1:N*(N-1) N*(N-1)+set_abs_mea];
            set_subopt = [set_subopt N*(N-1)+set_abs_mea];
            set_subopt_multi = [set_subopt_multi N*(N-1)+set_abs_mea];
            
            %             set_selected = set_dense;
            %             set_random = set_dense;
            %             set_subopt = set_dense;
            
            MeaMat_subMOD = []; MeaMat_random = []; MeaMat_dense = []; MeaMat_subOPT = [];
            MeaMat_subMOD_multi = []; MeaMat_subOPT_multi = [];
            for i=1:length(set_subMOD)
                MeaMat_subMOD = [MeaMat_subMOD; i ab_set(set_subMOD(i),:)];
            end
            for i=1:length(set_subMOD_multi)
                MeaMat_subMOD_multi = [MeaMat_subMOD_multi; i ab_set(set_subMOD_multi(i),:)];
            end
            for i=1:length(set_random)
                MeaMat_random = [MeaMat_random;i ab_set(set_random(i),:)];
            end
            for i=1:length(set_dense)
                MeaMat_dense = [MeaMat_dense;i ab_set(set_dense(i),:)];
            end
            for i=1:length(set_subopt)
                MeaMat_subOPT = [MeaMat_subOPT;i ab_set(set_subopt(i),:)];
            end
            for i=1:length(set_subopt_multi)
                MeaMat_subOPT_multi = [MeaMat_subOPT_multi;i ab_set(set_subopt_multi(i),:)];
            end
            a_set_subMOD = MeaMat_subMOD(:,2);
            b_set_subMOD = MeaMat_subMOD(:,3);
            a_set_subMOD_multi = MeaMat_subMOD_multi(:,2);
            b_set_subMOD_multi = MeaMat_subMOD_multi(:,3);
            a_set_random = MeaMat_random(:,2);
            b_set_random = MeaMat_random(:,3);
            a_set_dense = MeaMat_dense(:,2);
            b_set_dense = MeaMat_dense(:,3);
            a_set_subOPT = MeaMat_subOPT(:,2);
            b_set_subOPT = MeaMat_subOPT(:,3);
            a_set_subOPT_multi = MeaMat_subOPT_multi(:,2);
            b_set_subOPT_multi = MeaMat_subOPT_multi(:,3);
        end
        %%%%% end %%%%%
        
        %%%%% optimize for measurement scheduling by changing MeaMat %%%%%

        mea_schedule{k} = MeaMat_subOPT;
        %%%%% ii_a - relative measurment order for submodular edges %%%%%
        for ii_a = 1:length(a_set_subMOD)
            cur_a = MeaMat_subMOD(ii_a,2);
            cur_b = MeaMat_subMOD(ii_a,3);
            if size_RelMea_Table(2) > 2 % more than 2 columns
                miss_set = MeaMat_subMOD(ii_a,4:end);
            else
                miss_set = 0;
            end
            % double check the measurement period
            if (k>=RelMea_Table{1,j_mea}(1,1) && k<=RelMea_Table{1,j_mea}(1,2))
                if flag_seq == 1 && ii_a > 1 % not the first measurement at time step k
                    if flag_robot{cur_a} == 1
                        [K_CEKF_subMOD,S_ab_CEKF_subMOD,r_a_CEKF_subMOD] = Update_CEKF(cur_a,cur_b,X_true{k+1,cur_a},X_true{k+1,cur_b},phi_true(k+1,:),X_hat_CEKF_subMOD{k+1,cur_a},X_hat_CEKF_subMOD{k+1,cur_b},phi_est(k+1,:),P_CEKF_subMOD{k+1},k,Noise_extero(index_noise_extero,:));
                        comm_CEKF_subMOD(k+1) = comm_CEKF_subMOD(k+1) + N - 1;
                        bytes_subMOD(k+1) = bytes_subMOD(k+1) + 16 + 58*(N-1);
                        [r_a_bar,Gamma_col] = Update_SAEKF_dataset(cur_a,cur_b,X_true{k+1,cur_a},X_true{k+1,cur_b},phi_true(k+1,:),X_hat_SAEKF_subMOD{k+1,cur_a},P_SAEKF_subMOD{k+1,cur_a},Phi_SAEKF_subMOD{k+1,cur_a},X_hat_SAEKF_subMOD{k+1,cur_b},P_SAEKF_subMOD{k+1,cur_b},Phi_SAEKF_subMOD{k+1,cur_b},phi_est(k+1,:),k,Noise_extero(index_noise_extero,:));
                        comm_SAEKF_subMOD(k+1) = comm_SAEKF_subMOD(k+1) + 1;
                    end
                    index_noise_extero = index_noise_extero + 1;
                elseif ii_a == 1 % the first measurement at time step k
                    if flag_robot{cur_a} == 1
                        [K_CEKF_subMOD,S_ab_CEKF_subMOD,r_a_CEKF_subMOD] = Update_CEKF(cur_a,cur_b,X_true{k+1,cur_a},X_true{k+1,cur_b},phi_true(k+1,:),X_hat_CEKF_subMOD{k+1,cur_a},X_hat_CEKF_subMOD{k+1,cur_b},phi_est(k+1,:),P_CEKF_subMOD{k+1},k,Noise_extero(index_noise_extero,:));
                        comm_CEKF_subMOD(k+1) = comm_CEKF_subMOD(k+1) + N - 1;
                        bytes_subMOD(k+1) = bytes_subMOD(k+1) + 16 + 58*(N-1);
                        [r_a_bar,Gamma_col] = Update_SAEKF_dataset(cur_a,cur_b,X_true{k+1,cur_a},X_true{k+1,cur_b},phi_true(k+1,:),X_hat_SAEKF_subMOD{k+1,cur_a},P_SAEKF_subMOD{k+1,cur_a},Phi_SAEKF_subMOD{k+1,cur_a},X_hat_SAEKF_subMOD{k+1,cur_b},P_SAEKF_subMOD{k+1,cur_b},Phi_SAEKF_subMOD{k+1,cur_b},phi_est(k+1,:),k,Noise_extero(index_noise_extero,:));
                        comm_SAEKF_subMOD(k+1) = comm_SAEKF_subMOD(k+1) + 1;
                    end
                    index_noise_extero = index_noise_extero + 1;
                end
            end
            
            % update for CEKF_subMOD
            if flag_robot{cur_a} == 1 % robot-to-server update is allowed
                KK_CEKF_subMOD = cell(1,N);
                for i=1:N
                    KK_CEKF_subMOD{i} = K_CEKF_subMOD(2*i-1:2*i,:);
                    if sum(i == miss_set) == 0 % robot i doesn't miss update
                        X_hat_CEKF_subMOD{k+1,i} = X_hat_CEKF_subMOD{k+1,i} + KK_CEKF_subMOD{i}*r_a_CEKF_subMOD;
                    end
                end
                P_CEKF_subMOD{k+1} = P_CEKF_subMOD{k+1} - K_CEKF_subMOD*S_ab_CEKF_subMOD*K_CEKF_subMOD';
            end
            % update for SAEKF
            if flag_robot{cur_a} == 1 % robot-to-server update is allowed
                for i = 1:N
                    if sum(i == miss_set) == 0 % robot i doesn't miss update
                        X_hat_SAEKF_subMOD{k+1,i} = X_hat_SAEKF_subMOD{k+1,i} + Phi_SAEKF_subMOD{k+1,i}*Gamma_col{i}*r_a_bar;
                        P_SAEKF_subMOD{k+1,i} = P_SAEKF_subMOD{k+1,i} - Phi_SAEKF_subMOD{k+1,i}*(Gamma_col{i}*Gamma_col{i}')*Phi_SAEKF_subMOD{k+1,i}';
                    end
                end
            end
        end
        % end of ii_a with respect to submodular MeaMat
        
        %%%%% ii_a - relative measurment order for subMOD_multi edges %%%%%
        for ii_a = 1:length(a_set_subMOD_multi)
            cur_a = MeaMat_subMOD_multi(ii_a,2);
            cur_b = MeaMat_subMOD_multi(ii_a,3);
            if size_RelMea_Table(2) > 2 % more than 2 columns
                miss_set = MeaMat_subMOD_multi(ii_a,4:end);
            else
                miss_set = 0;
            end
            % double check the measurement period
            if (k>=RelMea_Table{1,j_mea}(1,1) && k<=RelMea_Table{1,j_mea}(1,2))
                if flag_seq == 1 && ii_a > 1 % not the first measurement at time step k
                    if flag_robot{cur_a} == 1
                        [K_CEKF_subMOD_multi,S_ab_CEKF_subMOD_multi,r_a_CEKF_subMOD_multi] = Update_CEKF(cur_a,cur_b,X_true{k+1,cur_a},X_true{k+1,cur_b},phi_true(k+1,:),X_hat_CEKF_subMOD_multi{k+1,cur_a},X_hat_CEKF_subMOD_multi{k+1,cur_b},phi_est(k+1,:),P_CEKF_subMOD_multi{k+1},k,Noise_extero(index_noise_extero,:));
                        comm_CEKF_subMOD_multi(k+1) = comm_CEKF_subMOD_multi(k+1) + N - 1;
                        bytes_subMOD_multi(k+1) = bytes_subMOD_multi(k+1) + 16 + 58*(N-1);
                    end
                    index_noise_extero = index_noise_extero + 1;
                elseif ii_a == 1 % the first measurement at time step k
                    if flag_robot{cur_a} == 1
                        [K_CEKF_subMOD_multi,S_ab_CEKF_subMOD_multi,r_a_CEKF_subMOD_multi] = Update_CEKF(cur_a,cur_b,X_true{k+1,cur_a},X_true{k+1,cur_b},phi_true(k+1,:),X_hat_CEKF_subMOD_multi{k+1,cur_a},X_hat_CEKF_subMOD_multi{k+1,cur_b},phi_est(k+1,:),P_CEKF_subMOD_multi{k+1},k,Noise_extero(index_noise_extero,:));
                        comm_CEKF_subMOD_multi(k+1) = comm_CEKF_subMOD_multi(k+1) + N - 1;
                        bytes_subMOD_multi(k+1) = bytes_subMOD_multi(k+1) + 16 + 58*(N-1);
                    end
                    index_noise_extero = index_noise_extero + 1;
                end
            end
            
            % update for CEKF_subMOD_multi
            if flag_robot{cur_a} == 1 % robot-to-server update is allowed
                KK_CEKF_subMOD_multi = cell(1,N);
                for i=1:N
                    KK_CEKF_subMOD_multi{i} = K_CEKF_subMOD_multi(2*i-1:2*i,:);
                    if sum(i == miss_set) == 0 % robot i doesn't miss update
                        X_hat_CEKF_subMOD_multi{k+1,i} = X_hat_CEKF_subMOD_multi{k+1,i} + KK_CEKF_subMOD_multi{i}*r_a_CEKF_subMOD_multi;
                    end
                end
                P_CEKF_subMOD_multi{k+1} = P_CEKF_subMOD_multi{k+1} - K_CEKF_subMOD_multi*S_ab_CEKF_subMOD_multi*K_CEKF_subMOD_multi';
            end
        end
        % end of ii_a with respect to submodular selection MeaMat_multi
        
        %%%%% ii_a - relative measurment order for random edges %%%%%
        for ii_a = 1:length(a_set_random)
            cur_a = MeaMat_random(ii_a,2);
            cur_b = MeaMat_random(ii_a,3);
            if size_RelMea_Table(2) > 2 % more than 2 columns
                miss_set = MeaMat_random(ii_a,4:end);
            else
                miss_set = 0;
            end
            % double check the measurement period
            if (k>=RelMea_Table{1,j_mea}(1,1) && k<=RelMea_Table{1,j_mea}(1,2))
                if flag_seq == 1 && ii_a > 1 % not the first measurement at time step k
                    if flag_robot{cur_a} == 1
                        [K_CEKF_Random,S_ab_CEKF_Random,r_a_CEKF_Random] = Update_CEKF(cur_a,cur_b,X_true{k+1,cur_a},X_true{k+1,cur_b},phi_true(k+1,:),X_hat_CEKF_Random{k+1,cur_a},X_hat_CEKF_Random{k+1,cur_b},phi_est(k+1,:),P_CEKF_Random{k+1},k,Noise_extero(index_noise_extero,:));
                        comm_CEKF_Random(k+1) = comm_CEKF_Random(k+1) + N - 1;
                    end
                    index_noise_extero = index_noise_extero + 1;
                elseif ii_a == 1 % the first measurement at time step k
                    if flag_robot{cur_a} == 1
                        [K_CEKF_Random,S_ab_CEKF_Random,r_a_CEKF_Random] = Update_CEKF(cur_a,cur_b,X_true{k+1,cur_a},X_true{k+1,cur_b},phi_true(k+1,:),X_hat_CEKF_Random{k+1,cur_a},X_hat_CEKF_Random{k+1,cur_b},phi_est(k+1,:),P_CEKF_Random{k+1},k,Noise_extero(index_noise_extero,:));
                        comm_CEKF_Random(k+1) = comm_CEKF_Random(k+1) + N - 1;
                    end
                    index_noise_extero = index_noise_extero + 1;
                end
            end
            % update for CEKF_Random
            if flag_robot{cur_a} == 1 % robot-to-server update is allowed
                KK_CEKF_Random = cell(1,N);
                for i = 1:N
                    KK_CEKF_Random{i} = K_CEKF_Random(2*i-1:2*i,:);
                    if sum(i == miss_set) == 0 % robot i doesn't miss update
                        X_hat_CEKF_Random{k+1,i} = X_hat_CEKF_Random{k+1,i} + KK_CEKF_Random{i}*r_a_CEKF_Random;
                    end
                end
                P_CEKF_Random{k+1} = P_CEKF_Random{k+1} - K_CEKF_Random*S_ab_CEKF_Random*K_CEKF_Random';
            end
        end
        %%%%% end of ii_a with respect to randomized MeaMat %%%%%
        
        %%%%% ii_a - relative measurment order for dense graph%%%%%
        for ii_a = 1:length(a_set_dense)
            cur_a = MeaMat_dense(ii_a,2);
            cur_b = MeaMat_dense(ii_a,3);
            if size_RelMea_Table(2) > 2 % more than 2 columns
                miss_set = MeaMat_dense(ii_a,4:end);
            else
                miss_set = 0;
            end
            % double check the measurement period
            if (k>=RelMea_Table{1,j_mea}(1,1) && k<=RelMea_Table{1,j_mea}(1,2))
                if flag_seq == 1 && ii_a > 1 % not the first measurement at time step k
                    if flag_robot{cur_a} == 1
                        [K_CEKF_Dense,S_ab_CEKF_Dense,r_a_CEKF_Dense] = Update_CEKF(cur_a,cur_b,X_true{k+1,cur_a},X_true{k+1,cur_b},phi_true(k+1,:),X_hat_CEKF_Dense{k+1,cur_a},X_hat_CEKF_Dense{k+1,cur_b},phi_est(k+1,:),P_CEKF_Dense{k+1},k,Noise_extero(index_noise_extero,:));
                        comm_CEKF_Dense(k+1) = comm_CEKF_Dense(k+1) + N - 1;
                    end
                    index_noise_extero = index_noise_extero + 1;
                elseif ii_a == 1 % the first measurement at time step k
                    if flag_robot{cur_a} == 1
                        [K_CEKF_Dense,S_ab_CEKF_Dense,r_a_CEKF_Dense] = Update_CEKF(cur_a,cur_b,X_true{k+1,cur_a},X_true{k+1,cur_b},phi_true(k+1,:),X_hat_CEKF_Dense{k+1,cur_a},X_hat_CEKF_Dense{k+1,cur_b},phi_est(k+1,:),P_CEKF_Dense{k+1},k,Noise_extero(index_noise_extero,:));
                        comm_CEKF_Dense(k+1) = comm_CEKF_Dense(k+1) + N - 1;
                    end
                    index_noise_extero = index_noise_extero + 1;
                end
            end
            % update for Dense-EKF
            if flag_robot{cur_a} == 1 % robot-to-server update is allowed
                KK_CEKF_Dense = cell(1,N);
                for i = 1:N
                    KK_CEKF_Dense{i} = K_CEKF_Dense(2*i-1:2*i,:);
                    if sum(i == miss_set) == 0 % robot i doesn't miss update
                        X_hat_CEKF_Dense{k+1,i} = X_hat_CEKF_Dense{k+1,i} + KK_CEKF_Dense{i}*r_a_CEKF_Dense;
                    end
                end
                P_CEKF_Dense{k+1} = P_CEKF_Dense{k+1} - K_CEKF_Dense*S_ab_CEKF_Dense*K_CEKF_Dense';
            end
        end
        %%%%% end of ii_a with respect to dense MeaMat %%%%%
        
        %%%%% ii_a - relative measurment order for subOPT graph%%%%%
        for ii_a = 1:length(a_set_subOPT)
            cur_a = MeaMat_subOPT(ii_a,2);
            cur_b = MeaMat_subOPT(ii_a,3);
            if size_RelMea_Table(2) > 2 % more than 2 columns
                miss_set = MeaMat_subOPT(ii_a,4:end);
            else
                miss_set = 0;
            end
            % double check the measurement period
            if (k>=RelMea_Table{1,j_mea}(1,1) && k<=RelMea_Table{1,j_mea}(1,2))
                if flag_seq == 1 && ii_a > 1 % not the first measurement at time step k
                    if flag_robot{cur_a} == 1
                        [K_CEKF_subOPT,S_ab_CEKF_subOPT,r_a_CEKF_subOPT] = Update_CEKF(cur_a,cur_b,X_true{k+1,cur_a},X_true{k+1,cur_b},phi_true(k+1,:),X_hat_CEKF_subOPT{k+1,cur_a},X_hat_CEKF_subOPT{k+1,cur_b},phi_est(k+1,:),P_CEKF_subOPT{k+1},k,Noise_extero(index_noise_extero,:));
                        comm_CEKF_subOPT(k+1) = comm_CEKF_subOPT(k+1) + N - 1;
                        bytes_subOPT(k+1) = bytes_subOPT(k+1) + 48 + 58*(N-1);
                    end
                    index_noise_extero = index_noise_extero + 1;
                elseif ii_a == 1 % the first measurement at time step k
                    if flag_robot{cur_a} == 1
                        [K_CEKF_subOPT,S_ab_CEKF_subOPT,r_a_CEKF_subOPT] = Update_CEKF(cur_a,cur_b,X_true{k+1,cur_a},X_true{k+1,cur_b},phi_true(k+1,:),X_hat_CEKF_subOPT{k+1,cur_a},X_hat_CEKF_subOPT{k+1,cur_b},phi_est(k+1,:),P_CEKF_subOPT{k+1},k,Noise_extero(index_noise_extero,:));
                        comm_CEKF_subOPT(k+1) = comm_CEKF_subOPT(k+1) + N - 1;
                        bytes_subOPT(k+1) = bytes_subOPT(k+1) + 48 + 58*(N-1);
                    end
                    index_noise_extero = index_noise_extero + 1;
                end
            end
            % update for subOPT-EKF
            if flag_robot{cur_a} == 1 % robot-to-server update is allowed
                KK_CEKF_subOPT = cell(1,N);
                for i = 1:N
                    KK_CEKF_subOPT{i} = K_CEKF_subOPT(2*i-1:2*i,:);
                    if sum(i == miss_set) == 0 % robot i doesn't miss update
                        X_hat_CEKF_subOPT{k+1,i} = X_hat_CEKF_subOPT{k+1,i} + KK_CEKF_subOPT{i}*r_a_CEKF_subOPT;
                    end
                end
                P_CEKF_subOPT{k+1} = P_CEKF_subOPT{k+1} - K_CEKF_subOPT*S_ab_CEKF_subOPT*K_CEKF_subOPT';
            end
        end
        %%%%% end of ii_a with respect to subOPT MeaMat %%%%%
        
        %%%%% ii_a - relative measurment order for subOPT_multi graph%%%%%
        for ii_a = 1:length(a_set_subOPT_multi)
            cur_a = MeaMat_subOPT_multi(ii_a,2);
            cur_b = MeaMat_subOPT_multi(ii_a,3);
            if size_RelMea_Table(2) > 2 % more than 2 columns
                miss_set = MeaMat_subOPT_multi(ii_a,4:end);
            else
                miss_set = 0;
            end
            % double check the measurement period
            if (k>=RelMea_Table{1,j_mea}(1,1) && k<=RelMea_Table{1,j_mea}(1,2))
                if flag_seq == 1 && ii_a > 1 % not the first measurement at time step k
                    if flag_robot{cur_a} == 1
                        [K_CEKF_subOPT_multi,S_ab_CEKF_subOPT_multi,r_a_CEKF_subOPT_multi] = Update_CEKF(cur_a,cur_b,X_true{k+1,cur_a},X_true{k+1,cur_b},phi_true(k+1,:),X_hat_CEKF_subOPT_multi{k+1,cur_a},X_hat_CEKF_subOPT_multi{k+1,cur_b},phi_est(k+1,:),P_CEKF_subOPT_multi{k+1},k,Noise_extero(index_noise_extero,:));
                        comm_CEKF_subOPT_multi(k+1) = comm_CEKF_subOPT_multi(k+1) + N - 1;
                        bytes_subOPT_multi(k+1) = bytes_subOPT_multi(k+1) + 48 + 58*(N-1);
                    end
                    index_noise_extero = index_noise_extero + 1;
                elseif ii_a == 1 % the first measurement at time step k
                    if flag_robot{cur_a} == 1
                        [K_CEKF_subOPT_multi,S_ab_CEKF_subOPT_multi,r_a_CEKF_subOPT_multi] = Update_CEKF(cur_a,cur_b,X_true{k+1,cur_a},X_true{k+1,cur_b},phi_true(k+1,:),X_hat_CEKF_subOPT_multi{k+1,cur_a},X_hat_CEKF_subOPT_multi{k+1,cur_b},phi_est(k+1,:),P_CEKF_subOPT_multi{k+1},k,Noise_extero(index_noise_extero,:));
                        comm_CEKF_subOPT_multi(k+1) = comm_CEKF_subOPT_multi(k+1) + N - 1;
                        bytes_subOPT_multi(k+1) = bytes_subOPT_multi(k+1) + 48 + 58*(N-1);
                    end
                    index_noise_extero = index_noise_extero + 1;
                end
            end
            % update for subOPT-EKF
            if flag_robot{cur_a} == 1 % robot-to-server update is allowed
                KK_CEKF_subOPT_multi = cell(1,N);
                for i = 1:N
                    KK_CEKF_subOPT_multi{i} = K_CEKF_subOPT_multi(2*i-1:2*i,:);
                    if sum(i == miss_set) == 0 % robot i doesn't miss update
                        X_hat_CEKF_subOPT_multi{k+1,i} = X_hat_CEKF_subOPT_multi{k+1,i} + KK_CEKF_subOPT_multi{i}*r_a_CEKF_subOPT_multi;
                    end
                end
                P_CEKF_subOPT_multi{k+1} = P_CEKF_subOPT_multi{k+1} - K_CEKF_subOPT_multi*S_ab_CEKF_subOPT_multi*K_CEKF_subOPT_multi';
            end
        end
        %%%%% end of ii_a with respect to subOPT MeaMat_multi %%%%%
        
        % at the end of each measurement period, set flag variables
        if RelMea_Table{1,j_mea}(1,2) == k
            j_mea = j_mea+1;
            flag_mea = 0;
            %             clear MeaMat a_set b_set miss_set;
        end
    end % end of relative flag
    
end  % end of k
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualization
XX = zeros(2,k_f+1,N); XX_hat_DR = XX;
XX_hat_CEKF_subMOD = XX; XX_hat_SAEKF_subMOD = XX;
XX_hat_CEKF_Dense = XX; XX_hat_CEKF_Random = XX;
XX_hat_CEKF_subOPT = XX;
XX_hat_CEKF_subMOD_multi = XX; XX_hat_CEKF_subOPT_multi = XX;
for i=1:N
    XX(:,:,i) = cell2mat(X_true(:,i)');
    XX_hat_DR(:,:,i) = cell2mat(X_hat_DR(:,i)');
    XX_hat_CEKF_subMOD(:,:,i) = cell2mat(X_hat_CEKF_subMOD(:,i)');
    XX_hat_SAEKF_subMOD(:,:,i) = cell2mat(X_hat_SAEKF_subMOD(:,i)');
    XX_hat_CEKF_Dense(:,:,i) = cell2mat(X_hat_CEKF_Dense(:,i)');
    XX_hat_CEKF_Random(:,:,i) = cell2mat(X_hat_CEKF_Random(:,i)');
    XX_hat_CEKF_subOPT(:,:,i) = cell2mat(X_hat_CEKF_subOPT(:,i)');
    XX_hat_CEKF_subMOD_multi(:,:,i) = cell2mat(X_hat_CEKF_subMOD_multi(:,i)');
    XX_hat_CEKF_subOPT_multi(:,:,i) = cell2mat(X_hat_CEKF_subOPT_multi(:,i)');
end
k=k_f+1;
for j=1:N % index of robot
    for i=1:2 % index of state in each robot
        Three_sigma_DR(i,k,j) = 3*sqrt(P_DR{k}(2*j+i-2,2*j+i-2));
        Three_sigma_CEKF_subMOD(i,k,j) = 3*sqrt(P_CEKF_subMOD{k}(2*j+i-2,2*j+i-2));
        Three_sigma_SAEKF_subMOD(i,k,j) = 3*sqrt(P_SAEKF_subMOD{k,j}(i,i));
        Three_sigma_CEKF_Dense(i,k,j) = 3*sqrt(P_CEKF_Dense{k}(2*j+i-2,2*j+i-2));
        Three_sigma_CEKF_Random(i,k,j) = 3*sqrt(P_CEKF_Random{k}(2*j+i-2,2*j+i-2));
        Three_sigma_CEKF_subOPT(i,k,j) = 3*sqrt(P_CEKF_subOPT{k}(2*j+i-2,2*j+i-2));
        Three_sigma_CEKF_subMOD_multi(i,k,j) = 3*sqrt(P_CEKF_subMOD_multi{k}(2*j+i-2,2*j+i-2));
        Three_sigma_CEKF_subOPT_multi(i,k,j) = 3*sqrt(P_CEKF_subOPT_multi{k}(2*j+i-2,2*j+i-2));
    end
end

NEES_DR = zeros(1,k_f+1);
NEES_CEKF_subMOD = NEES_DR; NEES_SAEKF_subMOD = NEES_DR;
NEES_CEKF_Dense = NEES_DR; NEES_CEKF_Random = NEES_DR;
NEES_CEKF_subOPT = NEES_DR;
NEES_CEKF_subMOD_multi = NEES_DR; NEES_CEKF_subOPT_multi = NEES_DR;

Tr_DR = NEES_DR;
Tr_CEKF_subMOD = Tr_DR; Tr_SAEKF_subMOD = Tr_DR;
Tr_CEKF_Dense = Tr_DR; Tr_CEKF_Random = Tr_DR;
Tr_CEKF_subOPT = Tr_DR;
Tr_CEKF_subMOD_multi = Tr_DR; Tr_CEKF_subOPT_multi = Tr_DR;

Logdet_DR = NEES_DR;
Logdet_CEKF_subMOD = Logdet_DR; Logdet_SAEKF_subMOD = Logdet_DR;
Logdet_CEKF_Dense = Logdet_DR; Logdet_CEKF_Random = Logdet_DR;
Logdet_CEKF_subOPT = Logdet_DR;
Logdet_CEKF_subMOD_multi = Logdet_DR; Logdet_CEKF_subOPT_multi = Logdet_DR;

RMSE_DR = zeros(k_f+1,N);
RMSE_CEKF_subMOD = RMSE_DR; RMSE_SAEKF_subMOD = RMSE_DR;
RMSE_CEKF_Dense = RMSE_DR; RMSE_CEKF_Random = RMSE_DR;
RMSE_CEKF_subOPT = RMSE_DR;
RMSE_CEKF_subMOD_multi = RMSE_DR; RMSE_CEKF_subOPT_multi = RMSE_DR;

D_ACC_DR = zeros(k_f+1,1);
D_ACC_CEKF_subMOD = D_ACC_DR; D_ACC_SAEKF_subMOD = D_ACC_DR;
D_ACC_CEKF_Dense = D_ACC_DR; D_ACC_CEKF_Random = D_ACC_DR;
D_ACC_CEKF_subOPT = D_ACC_DR;
D_ACC_CEKF_subMOD_multi = D_ACC_DR; D_ACC_CEKF_subOPT_multi = D_ACC_DR;

for j=1:k_f+1
    X_wave = cat(1,X_true{j,:}) - cat(1,X_hat_DR{j,:});
    NEES_DR(j) = X_wave'*P_DR{j}^-1*X_wave/n_x;
    Tr_DR(j) = trace(P_DR{j});
    tmp = sqrt(reshape(X_wave.^2,2,N));
    RMSE_DR(j,:) = sum(tmp(1:2,:),1);
    Logdet_DR(j) = f_logdet(P_DR{j});
    if j > 1
        D_ACC_DR(j) = 1/j*(D_ACC_DR(j-1)*(j-1) + X_wave'*X_wave);
    else
        D_ACC_DR(j) = X_wave'*X_wave;
    end
    
    X_wave = cat(1,X_true{j,:}) - cat(1,X_hat_CEKF_subMOD{j,:});
    NEES_CEKF_subMOD(j) = X_wave'*P_CEKF_subMOD{j}^-1*X_wave/n_x;
    Tr_CEKF_subMOD(j) = trace(P_CEKF_subMOD{j});
    tmp = sqrt(reshape(X_wave.^2,2,N));
    RMSE_CEKF_subMOD(j,:) = sum(tmp(1:2,:),1);
    Logdet_CEKF_subMOD(j) = f_logdet(P_CEKF_subMOD{j});
    if j > 1
        D_ACC_CEKF_subMOD(j) = 1/j*(D_ACC_CEKF_subMOD(j-1)*(j-1) + X_wave'*X_wave);
    else
        D_ACC_CEKF_subMOD(j) = X_wave'*X_wave;
    end
    
    X_wave = cat(1,X_true{j,:}) - cat(1,X_hat_CEKF_subMOD_multi{j,:});
    NEES_CEKF_subMOD_multi(j) = X_wave'*P_CEKF_subMOD_multi{j}^-1*X_wave/n_x;
    Tr_CEKF_subMOD_multi(j) = trace(P_CEKF_subMOD_multi{j});
    tmp = sqrt(reshape(X_wave.^2,2,N));
    RMSE_CEKF_subMOD_multi(j,:) = sum(tmp(1:2,:),1);
    Logdet_CEKF_subMOD_multi(j) = f_logdet(P_CEKF_subMOD_multi{j});
    if j > 1
        D_ACC_CEKF_subMOD_multi(j) = 1/j*(D_ACC_CEKF_subMOD_multi(j-1)*(j-1) + X_wave'*X_wave);
    else
        D_ACC_CEKF_subMOD_multi(j) = X_wave'*X_wave;
    end
    
    X_wave = cat(1,X_true{j,:}) - cat(1,X_hat_CEKF_Dense{j,:});
    NEES_CEKF_Dense(j) = X_wave'*P_CEKF_Dense{j}^-1*X_wave/n_x;
    Tr_CEKF_Dense(j) = trace(P_CEKF_Dense{j});
    tmp = sqrt(reshape(X_wave.^2,2,N));
    RMSE_CEKF_Dense(j,:) = sum(tmp(1:2,:),1);
    Logdet_CEKF_Dense(j) = f_logdet(P_CEKF_Dense{j});
    if j > 1
        D_ACC_CEKF_Dense(j) = 1/j*(D_ACC_CEKF_Dense(j-1)*(j-1) + X_wave'*X_wave);
    else
        D_ACC_CEKF_Dense(j) = X_wave'*X_wave;
    end
    
    X_wave = cat(1,X_true{j,:}) - cat(1,X_hat_CEKF_Random{j,:});
    NEES_CEKF_Random(j) = X_wave'*P_CEKF_Random{j}^-1*X_wave/n_x;
    Tr_CEKF_Random(j) = trace(P_CEKF_Random{j});
    tmp = sqrt(reshape(X_wave.^2,2,N));
    RMSE_CEKF_Random(j,:) = sum(tmp(1:2,:),1);
    Logdet_CEKF_Random(j) = f_logdet(P_CEKF_Random{j});
    if j > 1
        D_ACC_CEKF_Random(j) = 1/j*(D_ACC_CEKF_Random(j-1)*(j-1) + X_wave'*X_wave);
    else
        D_ACC_CEKF_Random(j) = X_wave'*X_wave;
    end
    
    X_wave = cat(1,X_true{j,:}) - cat(1,X_hat_SAEKF_subMOD{j,:});
    %     NEES_SAEKF_OPT(j) = X_wave'*blkdiag(P_SAEKF_OPT{j,:})^-1*X_wave/n_x;
    NEES_SAEKF_subMOD(j) = NEES_CEKF_subMOD(j);
    Tr_SAEKF_subMOD(j) = trace(blkdiag(P_SAEKF_subMOD{j,:}));
    tmp = sqrt(reshape(X_wave.^2,2,N));
    RMSE_SAEKF_subMOD(j,:) = sum(tmp(1:2,:),1);
    Logdet_SAEKF_subMOD(j) = Logdet_CEKF_subMOD(j);
    if j > 1
        D_ACC_SAEKF_subMOD(j) = 1/j*(D_ACC_SAEKF_subMOD(j-1)*(j-1) + X_wave'*X_wave);
    else
        D_ACC_SAEKF_subMOD(j) = X_wave'*X_wave;
    end
    
    X_wave = cat(1,X_true{j,:}) - cat(1,X_hat_CEKF_subOPT{j,:});
    NEES_CEKF_subOPT(j) = X_wave'*P_CEKF_subOPT{j}^-1*X_wave/n_x;
    Tr_CEKF_subOPT(j) = trace(P_CEKF_subOPT{j});
    tmp = sqrt(reshape(X_wave.^2,2,N));
    RMSE_CEKF_subOPT(j,:) = sum(tmp(1:2,:),1);
    Logdet_CEKF_subOPT(j) = f_logdet(P_CEKF_subOPT{j});
    if j > 1
        D_ACC_CEKF_subOPT(j) = 1/j*(D_ACC_CEKF_subOPT(j-1)*(j-1) + X_wave'*X_wave);
    else
        D_ACC_CEKF_subOPT(j) = X_wave'*X_wave;
    end
    
    X_wave = cat(1,X_true{j,:}) - cat(1,X_hat_CEKF_subOPT_multi{j,:});
    NEES_CEKF_subOPT_multi(j) = X_wave'*P_CEKF_subOPT_multi{j}^-1*X_wave/n_x;
    Tr_CEKF_subOPT_multi(j) = trace(P_CEKF_subOPT_multi{j});
    tmp = sqrt(reshape(X_wave.^2,2,N));
    RMSE_CEKF_subOPT_multi(j,:) = sum(tmp(1:2,:),1);
    Logdet_CEKF_subOPT_multi(j) = f_logdet(P_CEKF_subOPT_multi{j});
    if j > 1
        D_ACC_CEKF_subOPT_multi(j) = 1/j*(D_ACC_CEKF_subOPT_multi(j-1)*(j-1) + X_wave'*X_wave);
    else
        D_ACC_CEKF_subOPT_multi(j) = X_wave'*X_wave;
    end
end
Tk = (0:delta:k_f*delta);
t_f = k_f*delta;

disp(['Dense-EKF Number of communications: ',num2str(comm_CEKF_Dense(end))]);
disp(['subMod-EKF Number of communications: ',num2str(comm_SAEKF_subMOD(end))]);

save CL_data_dataset.mat;
% save P_CEKF.mat P_CEKF_subMOD;
% 
% q_i
% mean(timing_subMOD)
% mean(timing_subOPT)
Plot_Results_dataset;
% Traj_Plot; % plot results
%%
toc;