function [is_valid_landmark, is_wrong_triangulated, landmark_keypoints] = check_candidates(state_curr, K, ang_thr , age_thr, px_thr)
%
% IS_VALID_LANDMARK determines which matched keypoints are usable as landmarks by calculating
% the change of angle between the first and current observation of the keypoint
%
% INPUT:
%     state_curr: current state - state struct
%     K: camera intrinsics
%     ang_thr: threshold of angle for valid landmark - scalar, degree
%     age_thr: introduce long tracked keypoints that moved through frame as landmarks
%     px_thr: introduce long tracked keypoints that moved through frame as landmarks
% OUTPUT:
%     is_valid_landmark: boolean array of valid landmarks - 1xn matrix

Tau = state_curr.Tau; % pose of camera when landmark was first observed
F = state_curr.F; % coordinates of keypoints when first obsevred, ordered
first_obs = state_curr.first_observation_frame;

T_C_W_curr = invert_homo_trans(state_curr.Pose);
C = state_curr.C; % current coordinates of keypoints, ordered

angles = zeros([1 size(Tau,3)]);

landmark_keypoints = zeros([3 size(Tau,3)]);

init_ray = zeros([3 size(Tau,3)]);
curr_ray = zeros([3 size(Tau,3)]);

for i = 1:size(angles,2)
    % triangulate keypoints (2xN, 3x4)
    landmark_keypoints(:,i) = linear_triangulation( C(:,i),F(:,i), K*T_C_W_curr,K*invert_homo_trans(Tau(:,:,i))  );
    
    % create connecting lines
    init_ray(:,i) = landmark_keypoints(:,i)-Tau(:,4,i);
    curr_ray(:,i) = landmark_keypoints(:,i)-state_curr.Pose(:,4);
    
    % calculate cosine
    %anglescos(i) = abs( dot(init_ray,curr_ray)/( norm(init_ray) * norm(curr_ray) ) );
    
    % calulate angle
    angles(i) = abs(atan2d(norm(cross(init_ray(:,i),curr_ray(:,i))),dot(init_ray(:,i),curr_ray(:,i))));
end

% Check if the reprojection error is too big
landmarks_keypoints_camera = T_C_W_curr*[landmark_keypoints;ones(1,size(landmark_keypoints,2))];
landmarks_keypoints_camera = landmarks_keypoints_camera(1:3,:);
projected_points = project_points(landmarks_keypoints_camera,K);

reprojection_error = vecnorm(projected_points - C,2,1);

px_dist = vecnorm(F-C,2,1);

%disp( ['Angles:  Max: ',num2str(max(angles)),' Min: ',num2str(min(angles)),' Mean: ',num2str(mean(angles)) ])
% apply threshold
is_wrong_triangulated = reprojection_error >= 10;
sum((px_dist > px_thr & state_curr.frame-first_obs > age_thr & reprojection_error < 10));
is_valid_landmark = (angles > ang_thr & reprojection_error < 10) | (px_dist > px_thr & state_curr.frame-first_obs > age_thr & reprojection_error < 10);
%is_valid_landmark = (angles > ang_thr & reprojection_error < 10);


end
