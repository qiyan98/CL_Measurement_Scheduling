%%%%%%%%%%created by Qi Yan%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised Nov. 2019%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [R_rel,R_abs] = ExteroVar(k)
%% uncertainty for measurement (exteroceptive sensor)
D2R = pi/180;

persistent N;
if isempty(N)
    [~,~,~,N] = RobotInit();
end
%the values can be time-varying
R_rel = cell(1,N);
R_abs = cell(1,N);

%robot 1 measures relative pose
sigma_r11 = sqrt(0.0215); % relative distance
sigma_r12 = sqrt(0.01);   % relative bearing

% sigma_r11 = 0.1;
% sigma_r12 = 2/57.3;   % relative bearing

% sigma_r11 = 0.01;     % relative distance
% sigma_r12 = 0.0349;   % relative bearing

% sigma_r11 = 1e-6;     % relative distance
% sigma_r12 = 1e-6;   % relative bearing

%robot 1 measures absolute position
sigma_a11=sqrt(0.0215);     
sigma_a12=sqrt(0.0215);
% sigma_a13=sqrt(0.01);

% sigma_a11=0.147;     
% sigma_a12=0.147;

R_rel{1,1}=diag([sigma_r11^2,sigma_r12^2]);
for i = 2:N
    R_rel{1,i}=diag([sigma_r11^2,sigma_r12^2]);
end

R_abs{1,1}=diag([sigma_a11^2,sigma_a12^2]);
for i = 2:N
    R_abs{1,i}=diag([sigma_a11^2,sigma_a12^2]);
end
end