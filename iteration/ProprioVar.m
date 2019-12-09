%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%created by Solmaz S. Kia%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised April 2014%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%UCSD%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%modified by Qi Yan%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised Nov. 2019%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [sigma_V,sigma_W] = ProprioVar(V,k)
%% uncertatinty for true robot motion (proprioceptive sensor)
persistent dt N;
if isempty(dt) && isempty(N)
    [dt,~] = IterationInit();
%     [~,~,~,N] = RobotInit();
    N = length(V);
%     [~,W] = TrajGen(0);
end
%%
% linear velocity noise (standard deviation)
sigma_V = zeros(1,N);
for i = 1:N
    sigma_V(i) = sqrt(5.075*V(i)^2);
%     sigma_V(i) = 0.0125;
%     sigma_V(i) = 0.05*sqrt(V(i)^2);
end

% angular velocity noise (standard deviation)
% [~,W] = TrajGen(k);
sigma_W = zeros(1,N);
for i = 1:N
    sigma_W(i) = sqrt(0.345);
%     sigma_W(i) = 0.0384;
%     sigma_W(i) = 0.2*sqrt(W(i)^2);2.
end
end