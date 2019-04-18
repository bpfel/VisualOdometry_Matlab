clc;
clear all;
close all;

dataset = Dataset('kitti'); % options: 'kitti', 'malaga', 'parking'
new_candidate_threshold = 2;
klt_match_score = 0.997;
angel_threshold = 20;

%% Bootstrap

plot_bootstrap = false;

first_frame_index = dataset.first_frame;

%1. Get frames
[first_frame] = get_frame(dataset ,first_frame_index);
% Find keypoints in first image
[keypoints_first_frame] = some_keypoint_finding_algorithm(first_frame);

candidate_index = first_frame_index + 1;
found_next_keyframe = false;

while(~found_next_keyframe)
    
    [candidate_frame] = get_frame(dataset , candidate_index);

    %Track keypoints across frames
    [matched_logic_keyframe, all_KLT_keypoints, all_candidates] =matching(...
            first_frame,candidate_frame,keypoints_first_frame,new_candidate_threshold,klt_match_score);

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
    is_reasonable_landmark = sanity_check_landmarks( T_C_W , W_landmarks );
    W_landmarks = W_landmarks( : , is_reasonable_landmark & inliers );
    
    
    % Decide if we should take candiate as an keyframe
    average_depth = mean( W_landmarks(3,:) );       % Compute average depth of triangulated landmarks
    keyframe_distance = norm(t_C_W);                % Compute distance between keyframe and candidate keyframe
    ratio = keyframe_distance / average_depth;

    if( ratio > 0.1 )
        found_next_keyframe = true;
        display('***************** BOOTSTRAPPING **********************');
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

% **************************** CONVENTION *********************************
% state_prev.P - Previously matched keypoints
% state_prev.X - Landmarks corresponding to matched keypoints
% state_prev.C - Candidate keypoints
% state_prev.F - Image coordinates of first occurence of candidate keypoint
% state_prev.Tau - Pose at first occurence of candidate keypoint 
% T in R^{3x4xN_T}
% extend T -> T(:,:,end+1:end+n_t) = repmat(Pose,1,1,n_t);
% state_prev.frame - number of treated image frame
% state_prev.Pose - Pose as homogeneous transformation in R^{3x4} = [R|t]
% omitting the last line.
% *************************************************************************
state_prev.P = matched_keypoints_candidate_frame( : , inliers & is_reasonable_landmark );
state_prev.X = W_landmarks;
state_prev.C = [ all_candidates , matched_keypoints_candidate_frame(:,~inliers) ];
state_prev.F = state_prev.C;
state_prev.Pose = invert_homo_trans(T_C_W);
state_prev.Tau = repmat( state_prev.Pose , 1 , 1 , size(state_prev.C,2) );  % 3x4xsize(state_prev.C,2)
state_prev.frame = candidate_index;

first_state = state_prev;

%% Continuous operation

range = (candidate_index+1):dataset.last_frame;
for i = (candidate_index+1):20

    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    
    % Get new framenumber
    state_curr.frame = i;
    current_frame = get_frame(dataset, state_curr.frame);
    prev_frame = get_frame(dataset, state_prev.frame);
    
    % Match features from current frame against keypoints from prev state
    [matched_logic_keypoints, all_KLT_keypoints, all_candidates] = matching(...
                        prev_frame , current_frame , state_prev.P , new_candidate_threshold , klt_match_score );
   
    % Fill keypoints and landmarks of current state
    state_curr.P = all_KLT_keypoints( : , matched_logic_keypoints );
    state_curr.X = state_prev.X( : , matched_logic_keypoints );
    
    % Localization with P3P & RANSAC to detect inlier, followed by DLT on
    % inliers
    [T_WCcurr, inlier_mask_localization, num_iterations] = localization(state_curr.P, state_curr.X, dataset.K);
    state_curr.Pose = T_WCcurr;
    
    plotLocationTracking(state_curr.frame, get_frame(dataset, state_curr.frame), ...
        inlier_mask_localization, state_curr.Pose, state_curr.X, state_curr.P, num_iterations);

    % TODO later : add BUNDLE ADJUSTMENT here

    % matching against candidate feature of previous state
    [matched_logic_candidates, all_KLT_candidates, all_unmatched] = matching(...
                    prev_frame , current_frame , state_prev.C , new_candidate_threshold , klt_match_score );
    
    % Fill in the candidates features, based on which features you have
    % matched with previous candidates features
    state_curr.C = all_KLT_candidates( : , matched_logic_candidates );
    state_curr.Tau = state_prev.Tau( : , : , matched_logic_candidates );
    state_curr.F = state_prev.F( : , matched_logic_candidates );
%     state_curr.first_observation_frame = state_prev.first_observation_frame( matched_index_database );
    
    % Check if current candidates are okay to use as land_mark
    is_valid_landmark = check_candidates(state_curr, dataset.K, angel_threshold);
    disp(['num landmarks:',num2str(sum(is_valid_landmark))])
    
    % remove later
    matched_candidates = all_KLT_candidates( : , matched_logic_candidates );
    
    % Triangulate new valid landmarks 
    [new_landmarks] = triangulate_new_landmarks(state_curr, is_valid_landmark, dataset.K);
    
    % Remove new triangulated landmarks from candidates and add them to landmarks and keypoints 
    state_curr.P = [state_curr.P state_curr.C(: , is_valid_landmark)];
    new_keypoints = state_curr.C(: , is_valid_landmark); % remove later
    state_curr.X = [state_curr.X new_landmarks];
    state_curr.C = state_curr.C( : , ~is_valid_landmark );
    state_curr.Tau = state_curr.Tau( : , : , ~is_valid_landmark );
    state_curr.F = state_curr.F( : , ~is_valid_landmark );
%     state_curr.first_observation_frame = state_curr.first_observation_frame( ~is_valid_landmark );

%     

    % Add new candidates to state
    state_curr.C = [state_curr.C , all_candidates];
    state_curr.F = [state_curr.F , all_candidates];
%     state_curr.first_observation_frame = [state_curr.first_observation_frame, i*ones(size(new_candidates))];
    state_curr.Tau(:,:,end+1:end+size(all_candidates,2)) = repmat(state_curr.Pose,1,1,size(all_candidates,2));
        
    figure(17)
    imshow(current_frame); 3/180*pi
    hold on;
    scatter(state_curr.P(1,:),state_curr.P(2,:),'o','LineWidth',5);  
    scatter(state_prev.C(1,:),state_prev.C(2,:),'.','LineWidth',5);    
    scatter(matched_candidates(1,:),matched_candidates(2,:),'+','LineWidth',5);
    scatter(new_keypoints(1,:),new_keypoints(2,:),'*','LineWidth',5)
    %define the lines connecting old keypoints to new keypoints
    [x_track,y_track] = connecting_lines(...
                state_prev.C(:,matched_logic_candidates), matched_candidates);
    plot(x_track,y_track,'r','LineWidth',2);
    hold off;
    pause(0.2)
    legend({'current keypoints','old candidates','new candidates','new keypoints','tracks'})
    title('Adding new features');

    % Makes sure that plots refresh.    
    pause(0.5);
    state_prev = state_curr;
end
