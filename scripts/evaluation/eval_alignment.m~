
close all; clear; clc;

load_odom_data();
addpath('./data/he_data', './tsai_lenz', './data/la_data');
posesA = mainMF; posesB = frontMF;
M = size(posesA,3);

%% Set HE Cal Solution to evaluate - load from json file

fsol = 'calibration_MF_TL.json';  % full version with 3M pairs
T_TL = load_Tcal(fsol);

% fsol_2575 = 'calibration_MF_TL_2575.json';
% T_2575 = load_Tcal(fsol_2575);
% 
% fsol_515 = 'calibration_MF_TL_515.json';
% T_515 = load_Tcal(fsol_515);
% 
% fsol_102 = 'calibration_MF_TL_102.json';
% T_102 = load_Tcal(fsol_102);
% 
% fsol_20 = 'calibration_MF_TL_20.json';
% T_20 = load_Tcal(fsol_20);
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
fsol_HE = 'calibration_MF_HE.json';
T_HE =load_Tcal(fsol_HE);
% 
% fsol_HE_opt = 'calibration_optimized_MF.json';
% T_HE_opt =load_Tcal(fsol_HE_opt);
% 
% fsol_TL_opt = 'calibration_optimized_MF_TL.json';
% T_TL_opt =load_Tcal(fsol_TL_opt);
% 
% fsol_base_opt = 'calibration_optimized_MF_base.json';
% T_base_opt =load_Tcal(fsol_base_opt);
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

fsol_LA = 'calibration_LA_MF_500.json';
T_LA = inv(load_Tcal(fsol_LA));

fbase = 'calibration_estimate_base_MF.json';
T_base = load_Tcal(fbase);

% Best NDT solution - single frame
t_ndt_MF = [0.0738489 -0.0466149 -0.0219538]';
YPR_ndt_MF = [2.39064  2.89781 -3.10294];
rotm_ndt_MF = eul2rotm(YPR_ndt_MF);
T_NDT = vertcat(horzcat(rotm_ndt_MF, t_ndt_MF), [0 0 0 1]);

%% Compute errors for loop completions
ev_base = eval_calib(posesA, posesB, T_base);
ev_TL = eval_calib(posesA, posesB, T_TL);
ev_HE = eval_calib(posesA, posesB, T_HE);

ev_LA = eval_calib(posesA, posesB, T_LA);
ev_NDT = eval_calib(posesA, posesB, T_NDT);

% ev_HE_opt = eval_calib(posesA, posesB, T_HE_opt);
% ev_TL_opt = eval_calib(posesA, posesB, T_TL_opt);
% ev_base_opt = eval_calib(posesA, posesB, T_base_opt);


% ev_2575 = eval_calib(posesA, posesB, T_2575);
% ev_515 = eval_calib(posesA, posesB, T_515);
% ev_102 = eval_calib(posesA, posesB, T_102);
% ev_20 = eval_calib(posesA, posesB, T_20);

%% Plot Computed Pose Error
figure()
subplot(2,1,1);
hold on
% plot(ev_HE.err_nt, 'c')
% plot(ev_TL.err_nt, 'r')
plot(ev_base.err_nt, 'k')
plot(ev_LA.err_nt, 'g')
% plot(ev_NDT.err_nt, 'm')
% plot(ev_2575.err_nt, 'g')
% plot(ev_515.err_nt, 'c')
% plot(ev_102.err_nt, 'm')
% plot(ev_20.err_nt, '--r')
hold off
ylabel('Translation error norm (m)');
title('Error Evaluation');
txt1 = 'Position RMSE (m): ' + string(ev_base.rmse_t);
% text(100,0.55,txt1)
% legend('Base transform', '3314025 - all pose combinations', '2575 - all consecutive poses', ...
%     '515 - rand consecutive poses', '102 - rand consecutive poses', '20 - rand consecutive poses')
% legend('ETHZ HE-DQ','Tsai-Lenz','Base estimate', 'ETHZ Lidar-Align', 'Multi-Lidar NDT');
legend('Base estimate', 'ETHZ Lidar-Align');

subplot(2,1,2); 
hold on
% plot(ev_HE.err_nr, 'c')
% plot(ev_TL.err_nr, 'r')
plot(ev_base.err_nr, 'k')
plot(ev_LA.err_nr, 'g')
% plot(ev_NDT.err_nr, 'm')
% plot(ev_2575.err_nr, 'g');
% plot(ev_515.err_nr, 'c')
% plot(ev_102.err_nr, 'm')
% plot(ev_20.err_nr, '--r')
hold off
ylabel('Rotation error norm (deg)');
xlabel('Frame')
txt2 = 'Rotation RMSE (deg): ' + string(ev_base.rmse_r);
% text(100,13,txt2)



%% Reference Plots
% err_t_norm = zeros(1,M-1);
% for i = 1:M-1
%     err_t_norm(i) = norm(ev.err(1:3,4,i));
% end
% plot(err_t_norm);

%% Plot traj 2D
% T_sol_no_t = T_20;
% T_sol_no_t(:,4) = [0 0 0 1]';
% T_base_no_t = T_base;
% T_base_no_t(:,4) = [0 0 0 1]';

% HATB_TL20 = zeros(4,4,M);
% HATB_TL102 = zeros(4,4,M);
% HATB_TL515 = zeros(4,4,M);
% HATB_TL2575 = zeros(4,4,M);

HATB_TL = zeros(4,4,M);
HATB_base = zeros(4,4,M);
HATB_HE = zeros(4,4,M);
HATB_LA = zeros(4,4,M);
HATB_NDT = zeros(4,4,M);

for i = 1:M
%    HATB_TL20(:,:,i) = inv(T_20) * posesA(:,:,i); 
%    HATB_TL102(:,:,i) = inv(T_102) * posesA(:,:,i); 
%    HATB_TL515(:,:,i) = inv(T_515) * posesA(:,:,i); 
%    HATB_TL2575(:,:,i) = inv(T_2575) * posesA(:,:,i); 
   HATB_TL(:,:,i) = inv(T_TL) * posesA(:,:,i); 
   HATB_base(:,:,i) = inv(T_base) * posesA(:,:,i);
   HATB_HE(:,:,i) = inv(T_HE) * posesA(:,:,i);
   HATB_LA(:,:,i) = inv(T_LA) * posesA(:,:,i);
   HATB_NDT(:,:,i) = inv(T_NDT) * posesA(:,:,i);
end

% HATB_cum = zeros(4,4,M);
% HATB_cum(:,:,1) = eye(4);
% for i = 1:M-1
%     HATB_cum(:,:,i+1) = HATB_cum(:,:,i) * HATB_step(:,:,i);
% end

figure
hold on
% plot(permute(posesA(1,4,:),[3 1 2]), permute(posesA(2,4,:),[3 1 2]), 'r') % orig frame A
plot(permute(posesB(1,4,:),[3 1 2]), permute(posesB(2,4,:),[3 1 2]), '--b') % frame B
plot(permute(HATB_TL(1,4,:),[3 1 2]), permute(HATB_TL(2,4,:),[3 1 2]), 'r') % frame B
plot(permute(HATB_base(1,4,:),[3 1 2]), permute(HATB_base(2,4,:),[3 1 2]), 'k')
plot(permute(HATB_HE(1,4,:),[3 1 2]), permute(HATB_HE(2,4,:),[3 1 2]), 'c')
plot(permute(HATB_LA(1,4,:),[3 1 2]), permute(HATB_LA(2,4,:),[3 1 2]), 'g')
plot(permute(HATB_NDT(1,4,:),[3 1 2]), permute(HATB_NDT(2,4,:),[3 1 2]), 'm')
hold off
xlabel('x (m)')
ylabel('y (m)')
title('2D Odometry Matching')
legend('Lidar B', 'Base estimate', 'ETHZ HE-DQ','Tsai-Lenz', 'ETHZ Lidar-Align', 'Multi-Lidar NDT');

%% 3D trajectory plots
figure
plot3(permute(posesB(1,4,:),[3 1 2]), ...
      permute(posesB(2,4,:),[3 1 2]), ...
      permute(posesB(3,4,:),[3 1 2]), '--b');
hold on
grid on

plot3(permute(HATB_base(1,4,:),[3 1 2]), ...
      permute(HATB_base(2,4,:),[3 1 2]), ...
      permute(HATB_base(3,4,:),[3 1 2]), 'k');

plot3(permute(HATB_HE(1,4,:),[3 1 2]), ...
      permute(HATB_HE(2,4,:),[3 1 2]), ...
      permute(HATB_HE(3,4,:),[3 1 2]), 'c');
  
plot3(permute(HATB_TL(1,4,:),[3 1 2]), ...
      permute(HATB_TL(2,4,:),[3 1 2]), ...
      permute(HATB_TL(3,4,:),[3 1 2]), 'r');
  
plot3(permute(HATB_LA(1,4,:),[3 1 2]), ...
      permute(HATB_LA(2,4,:),[3 1 2]), ...
      permute(HATB_LA(3,4,:),[3 1 2]), 'g');
  
plot3(permute(HATB_NDT(1,4,:),[3 1 2]), ...
      permute(HATB_NDT(2,4,:),[3 1 2]), ...
      permute(HATB_NDT(3,4,:),[3 1 2]), 'm');
  
% plot3(permute(HATB_TL2575(1,4,:),[3 1 2]), ...
%       permute(HATB_TL2575(2,4,:),[3 1 2]), ...
%       permute(HATB_TL2575(3,4,:),[3 1 2]), 'g');
%   
% plot3(permute(HATB_TL515(1,4,:),[3 1 2]), ...
%       permute(HATB_TL515(2,4,:),[3 1 2]), ...
%       permute(HATB_TL515(3,4,:),[3 1 2]), 'c');
%   
% plot3(permute(HATB_TL102(1,4,:),[3 1 2]), ...
%       permute(HATB_TL102(2,4,:),[3 1 2]), ...
%       permute(HATB_TL102(3,4,:),[3 1 2]), 'm');
%   
% plot3(permute(HATB_TL20(1,4,:),[3 1 2]), ...
%       permute(HATB_TL20(2,4,:),[3 1 2]), ...
%       permute(HATB_TL20(3,4,:),[3 1 2]), '--r');
  
hold off
title('Trajectory Matching');
xlabel('x (m)');ylabel('y (m)'); zlabel('z (m)');
% legend('Lidar B Odometry', 'Lidar A - Base Estimate Transform', ...
%     'Lidar A - TL 3M Transform', 'Lidar A - TL 2575 Transform', ...
%     'Lidar A - TL 515 Transform', 'Lidar A - TL 102 Transform', ...
%     'Lidar A - TL 20 Transform');
legend('Lidar B Odometry', 'Lidar A - Base Estimate Transform', ...
    'Lidar A - HE Transform', ...
    'Lidar A - TL Transform', 'Lidar A - Lidar Align Transform', ...
    'Lidar A - Mult-Lidar NDT Transform')
axis('equal')





