% to use with the old world file
% p0 : the center in y and z for each wheel
% p : the pose in y and z of the nut
% factor : the scaling used for the wheel

( factor * eye(2) * ( p - p0 )' + p0' )'
