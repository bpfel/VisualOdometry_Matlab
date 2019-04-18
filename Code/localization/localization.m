function [T_WC, best_inlier_mask, num_iterations] = localization(p_query, W_landmarks, K,max_num_iterations,max_dist)
%LOCALISATION calculates the pose (w.r.t world frame) of the current camera position
%   INPUTS:
%   p_query: current keypoints - 2xn matrix
%   W_landmarks: current landmarks, same order as keypoints - 2xn matrix
%   K: intrinsic matrix of used camera 3x3 matrix
%   OUTPUTS:
%   T_WC: pose of camera in world frame - 4x3 matrix
%   best_inlier_mask: logic array, true for inliers
%   num_iterations: number of RANSAC iterations needed

% Use of Matlab built in function
% NOT working with Lecture 7 dataset
% 
% cameraParams = cameraParameters('IntrinsicMatrix',K);
% [R_WC,t_WC,inlierIdx] = estimateWorldCameraPose(keypoints',W_landmarks',...
%     cameraParams,'MaxReprojectionError',10,'Confidence',90,...
%     'MaxNumTrials',2000);
% T_WC = [R_WC,t_WC'];

%Set parameters
% max_num_iterations = 1000;
% max_dist = 10;

%Check dimensions of input variables
assert(size(W_landmarks,2) == size(p_query,2),'Dimensions of input do not agree!');

%Create variables
T_WC = eye(3,4);
best_inlier_mask = [];
max_num_inlier = 0;
num_iterations = 0;
    
% convert p_query to bearing vectors
p_query_homo = K \ [p_query ; ones(1,size(p_query,2))];

% normalize bearing vectors
p_query_homo = p_query_homo ./ vecnorm(p_query_homo,2,1);

%run ransac using PnP
for i = 1:max_num_iterations

    %choose 3 points    
    [query_samples, indx] = datasample(p_query_homo, 3, 2, 'Replace', false);
    landmark_samples = W_landmarks(:,indx);

    %run p3p of points against matches in 3d space
    M_p3p = real(p3p(landmark_samples, query_samples));       
    M_p3p = reshape(M_p3p,3,4,4);

    %check the four solutions of P3P
    M_test = eye(3,4);
    for j = 1:4
        M_test(:,1:3) = M_p3p(:,2:4,j)';
        M_test(:,4) = - M_p3p(:,2:4,j)'*M_p3p(:,1,j);

        %rotate landmarkes into camera frame
        C_landmarks = M_test * [W_landmarks; ones(1,size(W_landmarks,2))];

        %project all matched query features to 3d and remove outliers
        %EDIT: project matched 3D landmarks to 2D camera plane
        p_landmarks_proj = project_points(C_landmarks, K);

        %get inliers
        inlier_mask = (sum((p_landmarks_proj - p_query).^2,1) < (max_dist^2));
        num_inliners = sum(inlier_mask);

        %check if we could get more inliers
        if num_inliners > max_num_inlier
            max_num_inlier = num_inliners;
            best_inlier_mask = inlier_mask;
            num_iterations = i;
            
            
        end
    end        
end

%calculate pose estimation based on inliers using dlt
T_CW = dlt(p_query(:,best_inlier_mask),...
    W_landmarks(:,best_inlier_mask), K);

T_WC = invert_homo_trans(T_CW);

% ************* apply pose refinement *************
% only use inliers
initial_guess = HomogMatrix2twist(T_WC);
error_terms = @(x) pose_refinement_error(x, W_landmarks(:,best_inlier_mask) , double(p_query(:,best_inlier_mask)), K);
% options = optimoptions(@lsqnonlin, 'Display', 'iter', ...
%     'MaxIter', 50);
options = optimoptions(@lsqnonlin, 'Display','Off','MaxIter', 50);
refined_twist = lsqnonlin(error_terms, initial_guess, [], [], options);

T_WC = twist2HomogMatrix(refined_twist);
T_WC = T_WC(1:3,:);

end