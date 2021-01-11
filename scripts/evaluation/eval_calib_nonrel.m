
% non-relative version

function res = eval_calib_nonrel(posesA, posesB, Tcal)

    % Tcal is the estimated Hand-Eye transformation matrix from B to A

    assert(size(posesA,3) == size(posesB,3), 'Poses must be same size');
    M = size(posesA,3);
    
    % Initialize matrices
    HA = posesA;
    HB = posesB;
    HA_step = zeros(4,4,M-1);
    HB_step = zeros(4,4,M-1);
    HATB_step = zeros(4,4,M-1);
    err = zeros(4,4,M-1);
    trans_err = zeros(1,M-1);
    rot_err = zeros(1,M-1);
    
    % Compute closed loop error for acutal poses at every timestep
    for i = 1:M-1
%         j = i+1;
        HAij = inv(HA(:,:,j))*HA(:,:,i);
        HA_step(:,:,i) = HAij;

        HBij = inv(HB(:,:,j))*HB(:,:,i);
        HB_step(:,:,i) = HBij;

        HATB_step(:,:,i) = HAij*Tcal;
        T_err = Tcal * HAij * inv(Tcal) * inv(HBij); 

        err(:,:,i) = T_err;

        t_norm = norm(T_err(1:3,4));
        trans_err(i) = t_norm;

        q_err = rotm2quat(T_err(1:3,1:3));
        rod_err = quat2rod(q_err);
        rod_norm = norm(rod_err);
        rot_err(i) = rod_norm*360/(2*pi);

    end
    

    % Compute RMSE
    rmse_r = sqrt(sum(rot_err.^2)/size(rot_err,2));
    rmse_t = sqrt(sum(trans_err.^2)/size(trans_err,2));
    
    % Build struct for output
    res.err = err; res.err_nt = trans_err; res.err_nr = rot_err;
    res.rmse_t = rmse_t; res.rmse_r = rmse_r;
    
end