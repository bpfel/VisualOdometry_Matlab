function [W_landmarks] = debug_linear_triangulation(matched_keypoints_img1,...
    matched_keypoints_img2,M1,M2, img1 , img2 , K)
%LINEAR_TRIANGULATION Triangulate the location of the landmarks in world
%coordinates using the keypoint matches and the relative poses of the two
%frames
%   matched_keypoints_img1 - 2xN matrix
%   matched_keypoints_img2 - 2xN matrix
%   M1 - 3x4 matrix = K*[R|t]
%   M2 - 3x4 matrix
%OUTPUT W_landmarks - 3xN matrix

pose1 = invert_homo_trans( K\M1 );
pose2 = invert_homo_trans( K\M2 );

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


figure;
distance = 0.5;
plot_frame(img1, pose1(1:3,4), pose1(1:3,1:3),'y+',distance,K); % set frame in 3D room
hold on;
plot_frame(img2, pose2(1:3,4), pose2(1:3,1:3),'y+',distance,K); % set frame in 3D room
quiver3(0,0,0,0.1,0,0,'r');
quiver3(0,0,0,0,0.1,0,'g');
quiver3(0,0,0,0,0,0.1,'b');

for j = 1:size(matched_keypoints_img1,2)
    if norm(W_landmarks(:,j))<30 & W_landmarks(3,j)>0 & j<3
    % ray from positions to landmarks
        init_ray = W_landmarks(:,j)-pose1(:,4);
        curr_ray = W_landmarks(:,j)-pose2(:,4);
        quiver3(pose2(1,4),pose2(2,4), pose2(3,4), curr_ray(1), curr_ray(2), curr_ray(3), 'r');
        quiver3(pose1(1,4), pose1(2,4), pose1(3,4), init_ray(1), init_ray(2), init_ray(3), 'g');
        % plot landmarks
        scatter3(W_landmarks(1,j), W_landmarks(2,j), W_landmarks(3,j), 25, 'bo', 'LineWidth', 3)
        plot_landmark(size(img1),pose1(1:3,4), pose1(1:3,1:3), matched_keypoints_img1(:,j),'ro',distance,K);
        plot_landmark(size(img2),pose2(1:3,4), pose2(1:3,1:3), matched_keypoints_img2(:,j),'ro',distance,K);
    end
end
% axis([-5 5 -5 5 -2 20])
view(0,-60)
axis equal
hold off
end

