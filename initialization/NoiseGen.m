%%%%%%%%%%created by Qi Yan%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised Nov. 2019%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function NoiseGen()
% generate Gaussian random numbers as noise

[~,k_f] = IterationInit(0);
length_noise=floor(2e2*k_f+1);
[~,~,~,N] = RobotInit();

Noise_extero = zeros(N*length_noise,6); % first 3 for rel-mea., last 3 columns for abs-mea
Noise_propa_v = zeros(length_noise,N);
Noise_propa_w = zeros(length_noise,N);
for i=1:length_noise
    Noise_extero(i,:) = randn(1,6);
    Noise_propa_v(i,:) = randn(1,N);
    Noise_propa_w(i,:) = randn(1,N);
end
save .\noise_profile.mat Noise_extero Noise_propa_v Noise_propa_w;

end