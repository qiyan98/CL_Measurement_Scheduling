%%%%%%%%%%Created by Qi Yan%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised Nov. 2019%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [delta,k_f,w_m,w_c,lambda,RelMea_Table,UpdateOrder,mea_para] = IterationInit_dataset(missed_onoff)
% STATIC iteration parameters initialization, DYNAMIC parameters such as
% variance of noise are determined elsewhere.
% delta                 : stepsize
% k_f                   : total number of steps
% w_m                   : weights for mean calculation (UKF)
% w_c                   : weights for covarance calculation (UKF)
% lambda                : parameter for UKF
% RelMea_Table          : relative measurements occurrence table
% UpdateOrder           : update order for sequential updating
% mea_para              : some legacy parameters

%% computation settings
delta = 0.1; % stepsize

%%% for dataset %%%
t_s = 0; % start time
t_f = 300; % end time, large enough such that CL is always enabled

k_f = floor(t_f/delta); % total number of steps
mea_para = 2; % observation interval
%% UKF weights settings
[~,~,~,N] = RobotInit();
n_x = 2*N; % number of all state variables

alpha = 1; kappa = (3-n_x); % determining parameters for UKF weights
beta = 2;
lambda = alpha^2*(n_x+kappa)-n_x;

w_m = zeros(1,2*n_x+1); w_c = zeros(1,2*n_x+1);
w_m(1) = lambda/(n_x+lambda);
w_c(1) = lambda/(n_x+lambda) + (1-alpha^2+beta);
for i=2:2*n_x+1
    w_m(i) = 1/(2*(n_x+lambda));
    w_c(i) = 1/(2*(n_x+lambda));
end

%% relative measurement times
% missed_onoff=0: Every one updates
% missed_onoff=1: Only two robots making relative measurements update (loosely coupled)
% missed_onoff=2: No CL

if nargin == 0
    missed_onoff = 0;
end

UpdateOrder = [1:N]; %this is the priority list for updating (sequential updating)

% each element(matrix) in RelMea_Table:
% [ mea. start iteration   , mea. end iteration   ;
%   identity of robot_a_1  , identity of robot_b_1;
%   identity of robot_a_2  , identity of robot_b_2;]

if missed_onoff == 0
% All the robots recieve the update message, by default

% RelMea_Table={[-1 -1;0 0]};

% RelMea_Table={[floor(3/delta)+1 floor(5/delta);1 1],...
%     [floor(8/delta)+1 floor(10/delta);1 1],...
%     [floor(13/delta)+1 floor(15/delta);1 1],...
%     [floor(18/delta)+1 floor(20/delta);1 1],...
%     [-1 -1;0 0]};

RelMea_Table={[floor(t_s/delta)+1 floor(t_f/delta);1 2],...
    [-1 -1;0 0]};

elseif missed_onoff == 1
    %Only master and landmark robots update
    t_zero = zeros(1,N-1);
    RelMea_Table={[floor(10/delta)+1     floor(60/delta)  t_zero; 2 1 setdiff(1:N,[2 1]) 0 ; 5 1 setdiff(1:N,[5 1]) 0],...
                  [floor(60/delta)+1     floor(110/delta) t_zero; 3 2 setdiff(1:N,[3 2]) 0],...
                  [floor(110/delta)+1    floor(160/delta) t_zero; 1 1 setdiff(1:N,[1 1])],...
                  [floor(160/delta)+1    floor(210/delta) t_zero; 2 2 setdiff(1:N,[2 2])],...
                  [floor(210/delta)+1    floor(260/delta) t_zero; 3 1 setdiff(1:N,[3 1]) 0],...
                  [-1 -1;0 0]};
elseif missed_onoff == 2
    % No CL
    clear RelMea_Table;
    RelMea_Table={[-1 -1;0 0]};
end

end