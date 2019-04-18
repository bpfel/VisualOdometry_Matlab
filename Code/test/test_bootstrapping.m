clc;
clear all;
close all;

dataset = Dataset('kitti'); % options: 'kitti', 'malaga', 'parking'

%% Bootstrap / Keyframing

first_frame_index = dataset.first_frame;

%1. Get frames
[keyframe_1] = get_frame(dataset ,first_frame_index);

candidate_index = first_frame_index + 1;
found_next_keyframe = false;

while(~found_next_keyframe)
    
    [keyframe_candidate] = get_frame(dataset , candidate_index);

    % Find keypoints in images
    [keypoints_keyframe_1] = some_keypoint_finding_algorithm(keyframe_1);
    [keypoints_candidate] = some_keypoint_finding_algorithm(keyframe_candidate);

    %Track keypoints across frames
    [matched_index_boot_img1, matched_index_boot_img2,...
        matched_logic_boot_img1, matched_logic_boot_img2,all_KLT_tracked_points] =...
        matching(keyframe_1,keyframe_candidate,...
        keypoints_keyframe_1,keypoints_candidate);

    % extract the matched keypoints with help of the index vectors
    matched_keypoints_boot_img1 = keypoints_keyframe_1(: , matched_index_boot_img1);
    matched_keypoints_boot_img2 = all_KLT_tracked_points(: , matched_index_boot_img1);

    % plotMatches(keyframe_1,keyframe_candidate,matched_keypoints_boot_img1,matched_keypoints_boot_img2)

    %Find relative pose
    %Use the index-vectors matched_keypoints_database/query
    %to extract the keypoint correspondences from the full keypoint vectors
    %keypoints_query/database
    %3x3 matrix, 2xo matrix (o<=m)
    [F, inliers] = estimateFundamentalMatrix(...
        matched_keypoints_boot_img1',matched_keypoints_boot_img2');
    inliers = inliers'; % convert to our convention;

    [R_C_W, t_C_W] = decompose_fundamental_matrix(...
        F,dataset.K,matched_keypoints_boot_img1,matched_keypoints_boot_img2);

    % Transform to homogenous transformation
    T_C_W = rot_mat2homo_trans(R_C_W, t_C_W);

    %Projection matrices for images 1 and 2
    M_1 = dataset.K * rot_mat2homo_trans(eye(3),zeros(3,1));   % set first camera frame as world frame
    M_2 = dataset.K * T_C_W;

    % Triangulate landmarks in inertial frame
    % 3xo matrix
    [W_landmarks] = linear_triangulation(matched_keypoints_boot_img1(: , inliers ),...
        matched_keypoints_boot_img2( : , inliers ) , M_1 , M_2 );

    % Remove landmark that are not reasonable
    % e.g. behind camera or to far away
    is_reasonable_landmark = sanity_check_landmarks(T_C_W,W_landmarks);
    W_landmarks = W_landmarks( : , is_reasonable_landmark );
    
    
    % Decide if we should take candiate as an keyframe
    average_depth = mean( W_landmarks(3,:) );       % Compute average depth of triangulated landmarks
    keyframe_distance = norm(t_C_W);                % Compute distance between keyframe and candidate keyframe
    ratio = keyframe_distance / average_depth;

    if(ratio > 0.1)
        found_next_keyframe = true;
        display('***************** BOOTSTRAPPING **********************');
        display(['Found new keyframe with index ', num2str(candidate_index) , newline, ...
                  'Average depth = ', num2str(average_depth), '  ratio = ', num2str(ratio)]);
        display('***************************************');
    else
        found_next_keyframe = false;
        candidate_index = candidate_index + 1;
    end
    
end

% Plot matches and generated point cloud
plotMatchesAndPointCloud(rot_mat2homo_trans(eye(3),zeros(3,1)),...
    invert_homo_trans(T_C_W),W_landmarks(:,W_landmarks(3,:)<200),...
    keyframe_1,keyframe_candidate,matched_keypoints_boot_img1(:,inliers),...
    matched_keypoints_boot_img2(:,inliers),average_depth,ratio);
% Conclusion: RANSAC does a good job in removing outliers from the matches

%%