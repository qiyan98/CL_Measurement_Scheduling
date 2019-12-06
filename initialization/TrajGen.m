%%%%%%%%%%created by Qi Yan%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised Nov. 2019%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [V,W] = TrajGen(k)
%V  : true linear velocities, 1xN
%W  : true angular velocities, 1xN
%% Load necessary variables
persistent N dt k_f;
if isempty(N) && isempty(dt) && isempty(k_f)
    [~,~,~,N] = RobotInit();
    [dt,k_f,~] = IterationInit();
end
%% Trajectory setting
R2D=180/pi;
D2R=pi/180;

V = zeros(N,1); W = zeros(N,1);
% V(1) = (k/k_f)*0.05 + 0.1;
V(1) = 0.25;
if k < floor(k_f/4)
    W(1) = 0;
elseif k < floor(k_f/3)
    W(1) = -0.2;
elseif k < floor(k_f/2)
    W(1) = -0.2;
elseif k < 3*floor(k_f/4)
    W(1) = -0.3;
else
    W(1) = 0.25;
end
W(1) = (rand-0.5)*0.5;

W(1) = 0.1;
V(1) = 0.1;

for i = 2:N
    V(i) = V(1);
    W(i) = W(1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Motion of the robots are specified by their linear
%%and angular velocities
%these values can be time-varying

% if k<15/dt
%     W(1)=0*D2R;
%     V(1)=1.8;
% elseif k>=15/dt && k<45/dt
%     W(1)=1.8*D2R;
%     V(1)=1.75;
% elseif k>=45/dt && k<70/dt
%     W(1)=1.8*D2R;
%     V(1)=1.65;
% elseif k>=70/dt && k<90/dt
%     W(1)=2.1*D2R;
%     V(1)=0.5;
% elseif k>=90/dt && k<105/dt
%     W(1)=3.2*D2R;
%     V(1)=2.05;
% elseif k>=105/dt && k<135/dt
%     W(1)=1.8*D2R;
%     V(1)=2.25;
% else
%     W(1)=2.5*D2R;
%     V(1)=2;
% end
% 
% if k<20/dt
%     W(2)=0*D2R;
%     V(2)=1.7;
% elseif k>=20/dt && k<45/dt
%     W(2)=0.5*D2R;
%     V(2)=1.75;
% elseif k>=45/dt && k<90/dt
%     W(2)=3*D2R;
%     V(2)=1.9;
% elseif k>=90/dt && k<105/dt
%     W(2)=2.4*D2R;
%     V(2)=2.6;
% elseif k>=105/dt && k<130/dt
%     W(2)=2.4*D2R;
%     V(2)=2.45;
% else
%     W(2)=1*D2R;
%     V(2)=1.9;
% end
% 
% if k<20/dt
%     W(3)=0*D2R;
%     V(3)=1.7;
% elseif k>=20/dt && k<50/dt
%     W(3)=0.5*D2R;
%     V(3)=1.75;
% elseif k>=50/dt && k<65/dt
%     W(3)=7*D2R;
%     V(3)=2.25;
% elseif k>=65/dt && k<80/dt
%     W(3)=-1*D2R;
%     V(3)=2.5;
% elseif k>=80/dt && k<85/dt
%     W(3)=8*D2R;
%     V(3)=0.8;
% elseif k>=85/dt && k<95/dt
%     W(3)=2*D2R;
%     V(3)=2.8;
% elseif k>=95/dt && k<120/dt
%     W(3)=2.5*D2R;
%     V(3)=2.05;
% elseif k>=120/dt
%     W(3)=1.8*D2R;
%     V(3)=2;
% end
% % 
% if k<150/dt
%     V(4)=-0.2;
%     W(4)=1*D2R;
%     V(5)=-0.2;
%     W(5)=1*D2R;
% elseif k>=150/dt && k<250/dt
%     V(4)=-0.25;
%     W(4)=1.5*D2R;
%     V(5)=-0.25;
%     W(5)=1.5*D2R;
% elseif k>=250/dt && k<350/dt
%     V(4)=-0.25;
%     W(4)=2*D2R;
%     V(5)=-0.25;
%     W(5)=2*D2R;
% else
%     V(4)=-0.2;
%     W(4)=1*D2R;
%     V(5)=-0.2;
%     W(5)=1*D2R;
% end
% 
% V(1)=0.45;
% W(1)=1*D2R;
% V(2)=-0.35;
% W(2)=0.8*D2R;
% V(3)=-0.35;
% W(3)=1.1*D2R;
% V(4)=-0.35;
% W(4)=0.8*D2R;
% V(5)=-0.35;
% W(5)=1.1*D2R;
% 
% 
% 
% 
% if k<20/dt
%     W(1)=0;
%     V(1)=0.3;
%     elseif k>=20/dt &&k<=21/dt
%       W(1)=pi/2*exp(-0.1*(k-20)*dt);
%     V(1)=0.3;
%     elseif k>21/dt &&k<60/dt
%       W(1)=0;
%     V(1)=0.4;  
% elseif k>=60/dt &&k<=61/dt
%       W(1)=pi/2*exp(-0.1*(k-60)*dt);
%     V(1)=0.4;
%     elseif k>61/dt &&k<80/dt
%       W(1)=0;
%     V(1)=0.4;
%     elseif k>=80/dt &&k<81/dt
%       W(1)=pi/2*exp(-0.1*(k-80)*dt);
%     V(1)=0.4;
%     elseif k>=81/dt &&k<110/dt
%         W(1)=0;
%         V(1)=0.5;
% else
%      W(1)=0;
%         V(1)=0.5;
% end

end