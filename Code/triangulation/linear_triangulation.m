function [W_landmarks] = linear_triangulation(matched_keypoints_img1,...
    matched_keypoints_img2,M1,M2)
%LINEAR_TRIANGULATION Triangulate the location of the landmarks in world
%coordinates using the keypoint matches and the relative poses of the two
%frames
%   matched_keypoints_img1 - 2xN matrix
%   matched_keypoints_img2 - 2xN matrix
%   M1 - 3x4 matrix = K*[R|t]
%   M2 - 3x4 matrix
%OUTPUT W_landmarks - 3xN matrix

% transfrom to homogenous coordinates, since linearTriangulation uses
% homogenous coordinates
matched_keypoints_img1_homo = [matched_keypoints_img1; ones(1,size(matched_keypoints_img1,2))];
matched_keypoints_img2_homo = [matched_keypoints_img2; ones(1,size(matched_keypoints_img2,2))];

% Sanity checks
[dim,NumPoints] = size(matched_keypoints_img1_homo);
[dim2,NumPoints2] = size(matched_keypoints_img2_homo);
assert(dim==dim2,'Size mismatch of input points');
assert(NumPoints==NumPoints2,'Size mismatch of input points');
assert(dim==3,'Arguments x1, x2 should be 3xN matrices (homogeneous coords)');

[rows,cols] = size(M1);
assert(rows==3 && cols==4,'Projection matrices should be of size 3x4');
[rows,cols] = size(M2);
assert(rows==3 && cols==4,'Projection matrices should be of size 3x4');

% 3D Points that we triangulate now
P = zeros(4,NumPoints);

% Linear algorithm
for j=1:NumPoints
    % Built matrix of linear homogeneous system of equations
    A1 = cross2Matrix(matched_keypoints_img1_homo(:,j))*M1;
    A2 = cross2Matrix(matched_keypoints_img2_homo(:,j))*M2;
    A = [A1; A2];
    
    % Solve the linear homogeneous system of equations
    [~,~,v] = svd(A,0);
    P(:,j) = v(:,4);
end

P = P./repmat(P(4,:),4,1); % Dehomogeneize (P is expressed in homogeneous coordinates)


W_landmarks = P(1:3,:);
end

