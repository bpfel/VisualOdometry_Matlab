function [is_valid_landmark, landmark_keypoints] = check_candidates(state_curr, K, ang_thr)%, time_thr, dist_thr)
%
% IS_VALID_LANDMARK determines which matched keypoints are usable as landmarks by calculating
% the change of angle between the first and current observation of the keypoint
%
% INPUT:
%     state_curr: current state - state struct
%     K: camera intrinsics
%     ang_thr: threshold of angle for valid landmark - scalar, degree
%     bootstrap_index: index of last image used for bootstrap, only
%     temporary
%     time_thr: introduce "old" keypoints as landmarks ???
%     dist_thr: introduce distant keypoints as landmarks ???
% OUTPUT:
%     is_valid_landmark: boolean array of valid landmarks - 1xn matrix
global dataset;

Tau = state_curr.Tau; % pose of camera when landmark was first observed
F = state_curr.F; % coordinates of keypoints when first obsevred, ordered

T_C_W_curr = invert_homo_trans(state_curr.Pose);
C = state_curr.C; % current coordinates of keypoints, ordered

angles = zeros([1 size(Tau,3)]);

landmark_keypoints = zeros([3 size(Tau,3)]);

init_ray = zeros([3 size(Tau,3)]);
curr_ray = zeros([3 size(Tau,3)]);

for i = 1:size(angles,2)
    % triangulate keypoints (2xN, 3x4)
    landmark_keypoints(:,i) = debug_linear_triangulation( C(:,i),F(:,i), K*T_C_W_curr,K*invert_homo_trans(Tau(:,:,i)),...
            get_frame(dataset,state_curr.frame) ,get_frame(dataset,state_curr.first_observation_frame(i)) ,dataset.K);
    
    % create connecting lines
    init_ray(:,i) = landmark_keypoints(:,i)-Tau(:,4,i);
    curr_ray(:,i) = landmark_keypoints(:,i)-state_curr.Pose(:,4);
    
    % calculate cosine
    %anglescos(i) = abs( dot(init_ray,curr_ray)/( norm(init_ray) * norm(curr_ray) ) );
    
    % calulate angle
    angles(i) = abs(atan2d(norm(cross(init_ray(:,i),curr_ray(:,i))),dot(init_ray(:,i),curr_ray(:,i))));
end

%disp( ['Angles:  Max: ',num2str(max(angles)),' Min: ',num2str(min(angles)),' Mean: ',num2str(mean(angles)) ])
% apply threshold
is_valid_landmark = angles > ang_thr;

end