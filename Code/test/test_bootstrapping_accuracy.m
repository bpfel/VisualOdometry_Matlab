clc;
clear all;
close all;

global dataset;
dataset = Dataset('kitti'); % options: 'kitti', 'malaga', 'parking'

plot_bootstrap = true;
plot_continous_operation = true;

% %% Tune feature detection
% first_frame_index = dataset.first_frame;
% 
% %1. Get frames
% [first_frame] = get_frame(dataset ,first_frame_index);
% % Find keypoints in first image
% [keypoints_first_frame] = localized_harris_maxima_supression(...
%     first_frame,dataset.options.localized_harris{:});

%% Bootstrap

first_frame_index = dataset.first_frame;

%1. Get frames
[first_frame] = get_frame(dataset ,first_frame_index);
% Find keypoints in first image
[keypoints_first_frame] = localized_harris_maxima_supression(...
    first_frame,dataset.options.localized_harris{:});


candidate_index = first_frame_index + dataset.options.triangulation.bootframe_distance;
found_next_keyframe = false;

while(~found_next_keyframe)
    
    [candidate_frame] = get_frame(dataset , candidate_index);
    
    %Track keypoints across frames
    [matched_logic_keyframe, all_KLT_keypoints, all_candidates] =matching(...
        first_frame,candidate_frame,keypoints_first_frame,dataset.options.matching{:});
    
    % extract the matched keypoints with help of the logic vectors
    matched_keypoints_first_frame = keypoints_first_frame(: , matched_logic_keyframe);
    matched_keypoints_candidate_frame = all_KLT_keypoints(: , matched_logic_keyframe);
    
    % plotMatches(keyframe_1,keyframe_candidate,matched_keypoints_first_frame,matched_keypoints_candidate_frame)
    
    %Find relative pose
    %3x3 matrix, 1xo matrix (o<=m)
    [F, inliers] = estimateFundamentalMatrix(matched_keypoints_first_frame',matched_keypoints_candidate_frame');
    inliers = inliers'; % convert to our convention;
    
    [R_C_W, t_C_W] = decompose_fundamental_matrix(...
        F,dataset.K,matched_keypoints_first_frame(:,inliers),matched_keypoints_candidate_frame(:,inliers));
    
    % Transform to homogenous transformation
    T_C_W = rot_mat2homo_trans(R_C_W, t_C_W);
    
    %Projection matrices for images 1 and 2
    M_1 = dataset.K * rot_mat2homo_trans(eye(3),zeros(3,1));   % set first camera frame as world frame
    M_2 = dataset.K * T_C_W;
    
    % Triangulate landmarks in inertial frame, also outliers
    [W_landmarks] = linear_triangulation( matched_keypoints_first_frame(:,:),...
        matched_keypoints_candidate_frame(:,:) , M_1 , M_2 );
    
    % Remove landmark that are not reasonable
    % e.g. behind camera or to far away or outliers
    is_reasonable_landmark = sanity_check_landmarks( T_C_W , W_landmarks , dataset.options.triangulation.sanity_check_factor );
    W_landmarks = W_landmarks( : , is_reasonable_landmark & inliers );
    
    
    % Decide if we should take candiate as an keyframe
    average_depth = mean( W_landmarks(3,:) );       % Compute average depth of triangulated landmarks
    keyframe_distance = norm(t_C_W);                % Compute distance between keyframe and candidate keyframe
    ratio = keyframe_distance / average_depth;

    if( ratio > 0.05 | true)
        found_next_keyframe = true;
        disp('***************** BOOTSTRAPPING **********************');
        display(['Found new keyframe with index ', num2str(candidate_index) , newline, ...
            'Average depth = ', num2str(average_depth), '  ratio = ', num2str(ratio)]);
        display('******************************************************');
    else
        found_next_keyframe = false;
        candidate_index = candidate_index + 1;
    end
    
end

% Plot matches and generated point cloud
if plot_bootstrap
    plotMatchesAndPointCloud(rot_mat2homo_trans(eye(3),zeros(3,1)),invert_homo_trans(T_C_W),W_landmarks(:,W_landmarks(3,:)<200),...
        first_frame,candidate_frame,matched_keypoints_first_frame(:,inliers),matched_keypoints_candidate_frame(:,inliers), average_depth, ratio);
    % Conclusion: RANSAC does a good job in removing outliers from the matches
end



%% Check accuracy

disp('Check accuracy of Bootstrapping:')
[R_error, t_error] = check_accuracy(first_frame_index,candidate_frame,T_C_W);
    
    