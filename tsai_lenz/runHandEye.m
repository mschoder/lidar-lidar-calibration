% Run handEye on lidar-lidar data for DARPA Sub-T
% Using Tsai-Lenz implementation in MATLAB written by Zoran Lazarevic:
% http://lazax.com/www.cs.columbia.edu/~laza/html/Stewart/matlab/handEye.m

%% Load input data

load_odom_data();

%% Run Calibration

consecutive = true;               % true: use consecutive pose pairs only (n-1 total pairs). false: use every combination of poses (n*(n-1) pairs)
use_nth_pose = 1;                 % use every nth pose (eg. every 5th pose). Set to 1 to use every consecutive pair and skip nothing

T_MF = handEye(mainMF, frontMF, consecutive, use_nth_pose);  % returns T from input1 frame to input2 frame (gripper to camera)
T_MR = handEye(mainMR, rearMR, consecutive, use_nth_pose);

t_MF = T_MF(1:3,4);
t_MR = T_MR(1:3,4);
q_MF = rotm2quat(T_MF(1:3,1:3));  % [w,x,y,z] format
q_MR = rotm2quat(T_MR(1:3,1:3));


%%
fprintf("Main to Front Lidar Transform: \n Rotation [qw qx qy qz]: \n");
disp(q_MF');
fprintf("Translation: [x y z]\n");
disp(t_MF);

fprintf("Main to Rear Lidar Transform: \n Rotation [qw qx qy qz]: \n");
disp(q_MR');
fprintf("Translation: [x y z]\n");
disp(t_MR);


%% Time execution

% f = @() handEye(mainMF, frontMF);
% t = timeit(f);
% disp(t);

%% Time a single run

% tic
% handEye(mainMF, frontMF);
% toc