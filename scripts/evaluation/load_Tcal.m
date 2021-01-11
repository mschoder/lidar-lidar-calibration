% Takes in a filename string for json formatted HE calibration transform
% Returns 4x4 homogenous transformation matrix

function T = load_Tcal(fname)

base = jsondecode(fileread(fname));
t = [str2double(base.translation.x), ...
         str2double(base.translation.y), ...
         str2double(base.translation.z)];  % [x y z] format
q = [str2double(base.rotation.w), ...
         str2double(base.rotation.i), ...
         str2double(base.rotation.j), ...
         str2double(base.rotation.k)];     % [w x y z] format
rot = quat2rotm(q);
T = vertcat(horzcat(rot, t'), [0 0 0 1]);

end