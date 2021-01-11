
% Initial estimate info from vehicle config YAML file
RPY_M = [0 -0.0049916 0.7827801355];
RPY_F = [0 0.445058959 -0.005236];
RPY_R = [0 0.428 3.15555525359];

rotm_M = eul2rotm(flip(RPY_M)); % flip to put into ZYX format
rotm_F = eul2rotm(flip(RPY_F));
rotm_R = eul2rotm(flip(RPY_R));

t_M = [0.038 0 0.405]';
t_F = [0.138 -0.01 0.285]';
t_R = [-0.47 0 0.21]';

T_AM = vertcat(horzcat(rotm_M, t_M), [0 0 0 1]);
T_AF = vertcat(horzcat(rotm_F, t_F), [0 0 0 1]);
T_AR = vertcat(horzcat(rotm_R, t_R), [0 0 0 1]);

T_MF = T_AM \ T_AF;
T_MR = T_AM \ T_AR;

q_MF = rotm2quat(T_MF(1:3,1:3))'; % [w,x,y,z] format -- this looks good
q_MR = rotm2quat(T_MR(1:3,1:3))';


%% For lidar-align -- needs the inverse as initial guess
T_FM = inv(T_MF);
rot_FM = T_FM(1:3,1:3);
rot_xyz_FM = rotm2eul(rot_FM, 'XYZ');
t_FM = T_FM(1:3,4);



%% For NDT

% Best MF guess:  0.0738489 -0.0466149 -0.0219538  2.39064  2.89781 -3.10294
% Format: [x y z rz ry rx] (yaw pitch roll)  

t_ndt_MF = [0.0738489 -0.0466149 -0.0219538]';
YPR_ndt_MF = [2.39064  2.89781 -3.10294];
rotm_ndt_MF = eul2rotm(YPR_ndt_MF);
T_ndt_MF = vertcat(horzcat(rotm_ndt_MF, t_ndt_MF), [0 0 0 1]);