function mat_O = OPT_get_O(on_sensors,mat_C)
% decide the matrix O given selected measurements S so as to greedily
% choose the measurements
if length(on_sensors) < 0
    disp('Wrong number of turned-on sensors!');
    return;
end

len_m = size(mat_C,1);      % dimension of measurement vector, 2*M (2*#edges)
len_n = size(mat_C,2);      % dimension of all states, 2*N
mat_S = zeros(len_m);       % initialize by zeros matrix, 2M*2M

for i=1:length(on_sensors) % modify the selection matrix mat_S by list of turned-on sensors
    pos = on_sensors(i);
    mat_S(2*pos-1:2*pos,2*pos-1:2*pos) = eye(2);
end
mat_SC = mat_S*mat_C;     % 2M*2M x 2M*2N = 2M*2N
% mat_SC = repmat({mat_SC},1,1);
% mat_SC = blkdiag(mat_SC{:});
% L_0 = [eye(len_n) zeros(len_n)];
% L_1 = [eye(len_n) eye(len_n)];
% mat_O = [mat_SC*L_0;mat_SC*L_1];
mat_O = mat_SC;
end