function [R] = euler2R_XYZ(euler_xyz)
%#codegen
%$cgargs {zeros(3,1)}
assert(length(euler_xyz) == 3, 'Function euler2R_XYZ error: input dimension is incorrect');

phix = zeros(1);
phiy = zeros(2);
phiz = zeros(3);
R = zeros(3,3);

phix = euler_xyz(1);
phiy = euler_xyz(2);
phiz = euler_xyz(3);

R = [cos(phiy)*cos(phiz),   -cos(phiy)*sin(phiz),  sin(phiy);
    cos(phix)*sin(phiz)+cos(phiz)*sin(phix)*sin(phiy), cos(phix)*cos(phiz)-sin(phiz)*sin(phix)*sin(phiy), -cos(phiy)*sin(phix);
    sin(phix)*sin(phiz)-cos(phiz)*cos(phix)*sin(phiy), sin(phix)*cos(phiz)+sin(phiz)*cos(phix)*sin(phiy), cos(phiy)*cos(phix)];


end

