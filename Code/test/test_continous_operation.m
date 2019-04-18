clc;
clear all;
close all;

dataset = Dataset('kitti'); % options: 'kitti', 'malaga', 'parking'

%% Bootstrap


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
    [W_landmarks] = linear_triangulation(matched_keypoints_boot_img1(: , : ),...
        matched_keypoints_boot_img2( : , : ) , M_1 , M_2 );

    % Remove landmark that are not reasonable
    % e.g. behind camera or to far away
    is_reasonable_landmark = sanity_check_landmarks(T_C_W,W_landmarks);
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
        display('***************************************');
    else
        found_next_keyframe = false;
        candidate_index = candidate_index + 1;
    end
    
end

% Plot matches and generated point cloud
plotMatchesAndPointCloud(rot_mat2homo_trans(eye(3),zeros(3,1)),invert_homo_trans(T_C_W),W_landmarks(:,W_landmarks(3,:)<200),...
    keyframe_1,keyframe_candidate,matched_keypoints_boot_img1(:,inliers),matched_keypoints_boot_img2(:,inliers));
% Conclusion: RANSAC does a good job in removing outliers from the matches


% state_prev.P - Previously matched keypoints
% state_prev.X - Landmarks corresponding to matched keypoints
% state_prev.C - Candidate keypoints
% state_prev.F - Image coordinates of first occurence of candidate keypoint
% state_prev.Tau - Pose at first occurence of candidate keypoint 
% T in R^{3x4xN_T}
% extend T -> T(:,:,end+1:end+n_t) = repmat(Pose,1,1,n_t);
% state_prev.I - Image
% state_prev.Pose - Pose as homogeneous transformation in R^{3x4} = [R|t]
% omitting the last line.
state_prev.P = matched_keypoints_boot_img2( : , inliers & is_reasonable_landmark );
state_prev.X = W_landmarks;
state_prev.C = [];%[ keypoints_boot_img2( ~matched_logic_boot_img2 ) , matched_keypoints_boot_img2( ~inliers ) ];
state_prev.F = [];%state_prev.C;
state_prev.Tau = [];%repmat(M);  % Tau should be of size:  3x4xsize(state_prev.C,2)
state_prev.I = keyframe_candidate;
state_prev.Pose = invert_homo_trans(T_C_W);

%% Continuous operation

matching_threshold = 5;


range = (candidate_index+1):dataset.last_frame;
for i = range

    fprintf('\n\nProcessing frame %d\n=====================\n', i);
   
    % Plot previous landmarks and pose
    figure
    hold on
    axis equal
    rotate3d on;
    grid;
    plot3(state_prev.X(1,:), state_prev.X(2,:), state_prev.X(3,:), 'o');
    R = state_prev.Pose(1:3,1:3);
    t = state_prev.Pose(1:3,4);
    plotCoordinateFrame( R , t , 0.8);
    text(t(1)-0.1,t(2)-0.1,t(3)-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
    
    % Get new frame
    state_curr.I = get_frame(dataset,i);
    
    % Get features from current frame
    [features_query] = some_keypoint_finding_algorithm(state_curr.I); % not used at the moment
    
    % Match features from current frame against keypoints from prev state
    [matched_index_database, matched_index_query,...
    matched_logic_database, matched_logic_query, all_KLT_tracked_points] =...
        matching(state_prev.I,state_curr.I,state_prev.P,features_query,matching_threshold);
   
    % Fill keypoints and landmarks of current state
    state_curr.P = all_KLT_tracked_points( : , matched_index_database );
    state_curr.X = state_prev.X( : , matched_index_database );
    
    % Localization with P3P & RANSAC to detect inlier, followed by DLT on
    % inliers
    T_WCcurr = localization(state_curr.P, state_curr.X, dataset.K);
    state_curr.Pose = T_WCcurr;
   
%     plotLocalization(state_prev.Pose,invert_homo_trans(T_WCcurr),state_prev.X , state_curr.X)
    % Plot previous landmarks and pose
    
    plot3(state_curr.X(1,:), state_curr.X(2,:), state_curr.X(3,:), 'ro');
    R = state_curr.Pose(1:3,1:3);
    t = state_curr.Pose(1:3,4);
    plotCoordinateFrame( R , t , 0.8);
    text(t(1)-0.1, t(2)-0.1, t(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');
    axis equal
    rotate3d on;
    
    % TODO later : add BUNDLE ADJUSTMENT here
    state_prev = state_curr;
    pause(2); 
end
 %%
    % matching against candidate feature of previous state
    unmatched_features_query = features_query( ~matched_logic_query );
    [matched_index_database, matched_index_query,...
    matched_logic_database, matched_logic_query] =...
        matching(state_prev.I,state_curr.I,state_prev.C,unmatched_features_query,matched_threshold);
    
    % Fill in the candidates features, based on which features you have
    % matched with previous candidates features
    state_curr.C = unmatched_features_query( matched_index_query );
    state_curr.Tau = state_prev.Tau( matched_index_database );
    state_curr.F = state_prev.F( matched_index_database );
    
    % Check if current candidates are okay to use as land_mark
    is_valid_landmark = check_candidates(state_curr);
    
    % Triangulate new valid landmarks 
    [new_keypoints, new_landmarks] = triangulate_new_landmarks(state_curr, is_valid_landmark, dataset.K);
    
    % Remove new triangulated landmarks from candidates and add them 
    state_curr.C = state_curr.C( ~is_valid_landmark );
    state_curr.Tau = state_curr.Tau( ~is_valid_landmark );
    state_curr.F = state_curr.F( ~is_valid_landmark );
    state_curr.P = [state_curr.P new_key_points];
    state_curr.X = [state_curr.X new_landmarks];
    
    % Add new candidates to state
    new_candidates = unmatched_features_query( ~matched_logic_query );
    state_curr.C = [state_curr.C , new_candidates];
    state_curr.F = [state_curr.F , new_candidates];
    state_curr.Tau = [state_curr.Tau , repmat(state_curr.Pose)];
    
    
    state_prev = state_curr;
    % Makes sure that plots refresh.    
    pause(0.01);
% end
