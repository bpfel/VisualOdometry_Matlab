function [rotation_matrix,translation_vector] =...
    homo_trans2rot_mat(transformation_matrix)
%HOMO_TRANS2ROT_MAT Extract the rotation matrix and translation vector from
%a homogeneous transformation matrix

rotation_matrix = transformation_matrix(1:3,1:3);
translation_vector = transformation_matrix(1:3,4);
end

