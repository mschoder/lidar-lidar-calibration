% Load Lidar odometry estimates for Sub-T data
% 3 datasets, Main, Front, Rear
% MF = main-front
% MR = main-rear
% The above pairs are already time-aligned using filtering/correlation
% analysis from ETHZ Hand-Eye Calibration package

%% Load Data from File
addpath('./data/he_data', './tsai_lenz', './data/la_data');
main_MF  = readmatrix('main_aligned_MF.csv');   % Quaternion format [qx qy qz qw]
front_MF = readmatrix('front_aligned_MF.csv');

main_MR  = readmatrix('main_aligned_MR.csv');
rear_MR  = readmatrix('rear_aligned_MR.csv');

mainMF   = build3M(main_MF);
frontMF  = build3M(front_MF);
mainMR   = build3M(main_MR);
rearMR   = build3M(rear_MR);

%% Build the 3D matrix

function result = build3M(inputMat)
    dims = size(inputMat);
    M    = dims(1);
    for i = 1:M
       T       = inputMat(i,:);
       trans   = T(2:4)';
       rotq    = horzcat(T(8),T(5:7));  % expects [w x y z] format
       rotm    = quat2rotm(rotq);
       T_h     = vertcat(horzcat(rotm, trans), [0 0 0 1]);
       
       if i == 1  % initialize
           result = T_h;
       else
           result = cat(3,result,T_h);
       end
    end
end
