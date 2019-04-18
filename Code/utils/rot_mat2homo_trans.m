function [transformation_matrix] = rot_mat2homo_trans(rotation_matrix,translation_vector)
%ROT_MAT2HOMO_TRANS Builds the transformation matrix (3x4) from the rotation
%matrix and the translation vector

%Input checking
if size(rotation_matrix,1) ~= 3 || size(rotation_matrix,2) ~= 3
    error('Not a valid rotation matrix.');
elseif size(translation_vector,1) ~=3 || size(translation_vector,2) ~= 1
    error('Not a valid translation vector.');
end
transformation_matrix =...
    [rotation_matrix translation_vector];
end

