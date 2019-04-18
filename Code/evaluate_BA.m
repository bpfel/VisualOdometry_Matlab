% Take a fix bootstrap frame and let it fixed. Take pose from groundtruth
% (scale problem is resolved) and triangulate pose. remove landmarks with a
% reprojection error bigger than 1 pixel. then run multiple times the with
% and without BA

clc;
clear all;
close all;

global dataset;
dataset = Dataset('fpv3'); % options: 'kitti', 'malaga', 'parking', 'bike_straight', 'fpv1', 'fpv2', 'fpv3'

plot_bootstrap = true;
plot_continous_operation = false;
plot_prod = true;

BA_enabled = true;
BA_window_size = dataset.options.bundle_adjustment.window_size;
BA_periode = dataset.options.bundle_adjustment.periode;
bootstrap_frame = 3;


%% Bootstrap
bootstrap_frame = 3;
candidate_index = bootstrap_frame;
first_frame_index = dataset.first_frame;

%1. Get frames
[first_frame] = get_frame(dataset ,first_frame_index);
% Find keypoints in first image
[keypoints_first_frame] = localized_harris_maxima_supression(...
    first_frame,dataset.options.localized_harris{:},ceil( dataset.options.matching{4} /2));

[candidate_frame] = get_frame(dataset , bootstrap_frame);

%Track keypoints across frames
[matched_logic_keyframe, all_KLT_keypoints, all_candidates] = matching(...
    first_frame,candidate_frame,keypoints_first_frame,dataset.options.matching{:});

% extract the matched keypoints with help of the logic vectors
keypoints_first_frame = keypoints_first_frame(: , matched_logic_keyframe);
matched_keypoints_first_frame = keypoints_first_frame;
tracked_keypoints_from_first_frame = all_KLT_keypoints(: , matched_logic_keyframe);

% plotMatches(keyframe_1,keyframe_candidate,matched_keypoints_first_frame,matched_keypoints_candidate_frame)

%Get relative pose
%3x3 matrix, 1xo matrix (o<=m)
T_C_W = invert_homo_trans( dataset.ground_truth(:,:,bootstrap_frame+1) ); % frame index starts at zero

%Projection matrices for images 1 and 2
M_1 = dataset.K * rot_mat2homo_trans(eye(3),zeros(3,1));   % set first camera frame as world frame
M_2 = dataset.K * T_C_W;

% Triangulate landmarks in inertial frame, also outliers
[W_landmarks] = linear_triangulation( matched_keypoints_first_frame(:,:),tracked_keypoints_from_first_frame(:,:) , M_1 , M_2);
% Check reprojection error
C_landmarks = T_C_W*[W_landmarks;ones(1,size(W_landmarks,2))];
projected_points = project_points(C_landmarks,dataset.K);
reprojection_error = vecnorm(projected_points - tracked_keypoints_from_first_frame,2,1);

inliers = reprojection_error < 1;

% Remove landmark that are not reasonable
% e.g. behind camera or to far away or outliers
is_reasonable_landmark = sanity_check_landmarks( T_C_W , W_landmarks , dataset.options.triangulation.sanity_check_factor );
W_landmarks = W_landmarks( : , is_reasonable_landmark & inliers );


% Decide if we should take candiate as an keyframe
average_depth = mean( W_landmarks(3,:) );       % Compute average depth of triangulated landmarks
keyframe_distance = norm(T_C_W(:,4));                % Compute distance between keyframe and candidate keyframe
ratio = keyframe_distance / average_depth;

disp('***************** BOOTSTRAPPING **********************');
display(['Found new keyframe with index ', num2str(candidate_index) , newline, ...
    'Average depth = ', num2str(average_depth), '  ratio = ', num2str(ratio)]);
display('******************************************************');




% Plot matches and generated point cloud
if plot_bootstrap
    plotMatchesAndPointCloud(rot_mat2homo_trans(eye(3),zeros(3,1)),invert_homo_trans(T_C_W),W_landmarks(:,W_landmarks(3,:)<200),...
        first_frame,candidate_frame,matched_keypoints_first_frame(:,inliers),tracked_keypoints_from_first_frame(:,inliers), average_depth, ratio);
    % Conclusion: RANSAC does a good job in removing outliers from the matches
end

% **************************** CONVENTION *********************************
% state_prev.P - Previously matched keypoints
% state_prev.X - Landmarks corresponding to matched keypoints
% state_prev.C - Candidate keypoints
% state_prev.F - Image coordinates of first occurence of candidate keypoint
% state_prev.Tau - Pose at first occurence of candidate keypoint
% Tau in R^{3x4xN_T}
% extend Tau -> Tau(:,:,end+1:end+n_t) = repmat(Pose,1,1,n_t);
% state_prev.frame - number of treated image frame
% state_prev.Pose - Pose as homogeneous transformation in R^{3x4} = [R|t]
% omitting the last line.
% state_prev_first_observation_frame - frame in which the landmark was
% observed for the first time
% *************************************************************************
state_prev.P = tracked_keypoints_from_first_frame( : , inliers & is_reasonable_landmark );
state_prev.X = W_landmarks;
state_prev.C = [ all_candidates , tracked_keypoints_from_first_frame(:,~inliers) ];
state_prev.F = state_prev.C;
state_prev.Pose = invert_homo_trans(T_C_W);
state_prev.Tau = repmat( state_prev.Pose , 1 , 1 , size(state_prev.C,2) );  % 3x4xsize(state_prev.C,2)
state_prev.frame = candidate_index;
state_prev.first_observation_frame = state_prev.frame*ones(1,size(state_prev.C,2));
state_prev.plt = [];
first_state = state_prev;

%% Continuous operation

%Initialize iteration, make robust for repetition without bootstrapping

% if starting from the beginning:
state_prev = first_state;
state_hist = first_state;
plt.frame_first_index = first_frame_index;
plt.K = dataset.K;

range = (candidate_index+1):dataset.last_frame;

% % if starting from an intermediate point
% start_frame = 1890;
% load('state_hist_malaga_BA_until1850.mat');
% state_hist=state_hist(1,1:start_frame);
% state_prev=state_hist(1,end);
% plt = state_prev.plt;
% range = (start_frame+3):dataset.last_frame;

BA_counter = 2;
no_keypoint_counter = 0;

times = [];
for i = range
    % tic;
    
    frame = i;
    fprintf('\n\nProcessing frame %d\n=====================\n', frame);

    % Get new framenumber
    state_curr.frame = frame;
    plt.frame = frame;

    curr_frame = get_frame(dataset, state_curr.frame);
    prev_frame = get_frame(dataset, state_prev.frame);

    % Match features from current frame against keypoints AND candidates from prev state
    [matched_logic_points, all_KLT_points, new_candidates] = matching(...
        prev_frame , curr_frame , [state_prev.P state_prev.C] , dataset.options.matching{:} );
    % Extract only keypoints
    all_KLT_keypoints = all_KLT_points( : , 1:size(state_prev.P,2) );
    matched_logic_keypoints = matched_logic_points( : , 1:size(state_prev.P,2) );
    % Extract only candidates
    all_KLT_candidates = all_KLT_points( : , size(state_prev.P,2)+1:end );
    matched_logic_candidates = matched_logic_points( : , size(state_prev.P,2)+1:end );
    plt.new_candidates = new_candidates;

    % Fill keypoints and landmarks of current state
    state_curr.P = all_KLT_keypoints( : , matched_logic_keypoints );
    state_curr.X = state_prev.X( : , matched_logic_keypoints );

    plt.lost_keypoints = state_prev.P( : , ~matched_logic_keypoints );
    plt.old_matched_keypoints = state_prev.P( : , matched_logic_keypoints );
    plt.new_matched_keypoints = all_KLT_keypoints( : , matched_logic_keypoints );

    % Sanity check: Do we still have valid keypoints?
    if isempty(state_curr.P)
        warning(['you have lost all zour keypoints at frame ',num2str(frame)])
        T_WCcurr = state_prev.Pose; % If not take last pose as current pose
        inlier_mask_localization = [];
        plt.num_best_iterations = 0;
        no_keypoint_counter = no_keypoint_counter + 1;
        if no_keypoint_counter > 5
           error('stop it, tune your pipeline once more!!!') 
        end
    else
        % Localization with P3P & RANSAC to detect inlier, followed by DLT on
        % inliers and pose refinement minimizing the reprojection error
        [T_WCcurr, inlier_mask_localization, plt.num_best_iterations] = localization(state_curr.P, state_curr.X, dataset.K, dataset.options.localization{:});
        state_curr.Pose = T_WCcurr; % Set current pose
    end

    plt.pose = T_WCcurr;
    plt.pose_prev = state_prev.Pose;
    plt.landmark_inlier = state_curr.X(:,inlier_mask_localization);
    plt.landmark_outlier = state_curr.X(:,~inlier_mask_localization);
    plt.num_inlier = sum(inlier_mask_localization);
    plt.num_total = numel(inlier_mask_localization);

    % Removal of outlier landmarks
    state_curr.X = state_curr.X(:,inlier_mask_localization);
    state_curr.P = state_curr.P(:,inlier_mask_localization);

    % Check if remaining landmarks are in front of the camera and not too
    % far away EDIT: what was the problem with being to far away?
    valid_landmarks = sanity_check_landmarks(invert_homo_trans(state_curr.Pose) , state_curr.X , dataset.options.triangulation.sanity_check_factor );
    disp(['The number of valid landmarks is: ' num2str(sum(valid_landmarks))]);
    disp(['The number of invalid landmarks is: ' num2str(size(state_curr.X,2)-sum(valid_landmarks))]);
    % Remove points that do not pass sanity check
    state_curr.X = state_curr.X(:,valid_landmarks);
    state_curr.P = state_curr.P(:,valid_landmarks);
    % Remove too close keypoints if there are a lot of keypoints
    if size(state_curr.P,2)>500
        is_keeper = sparsify_keypoints(state_curr.P);
        state_curr.X = state_curr.X(:,is_keeper);
        state_curr.P = state_curr.P(:,is_keeper);
    end

    % Fill in the candidates features, based on which features you have
    % matched with previous candidates features
    state_curr.C = all_KLT_candidates( : , matched_logic_candidates );
    state_curr.Tau = state_prev.Tau( : , : , matched_logic_candidates );
    state_curr.F = state_prev.F( : , matched_logic_candidates );
    state_curr.first_observation_frame = state_prev.first_observation_frame( matched_logic_candidates );

    plt.lost_candidates = state_prev.C(:,~matched_logic_candidates);
    plt.old_matched_candidates = state_prev.C(:,matched_logic_candidates);
    plt.new_matched_candidates = all_KLT_candidates(:,matched_logic_candidates);
    plt.F = state_curr.F;

    % Check if current candidates are okay to use as land_mark
    [is_valid_landmark, is_wrong_triangulated, triangulated_candidates] = check_candidates(state_curr, dataset.K, dataset.options.triangulation.angle_treshold, dataset.options.triangulation.age_treshold,dataset.options.triangulation.pixel_treshold);
    is_reasonable_landmark = sanity_check_landmarks( invert_homo_trans(state_curr.Pose) , triangulated_candidates , dataset.options.triangulation.sanity_check_factor );
    new_found_landmarks = triangulated_candidates( : , is_valid_landmark&is_reasonable_landmark );
    new_found_keypoints = state_curr.C(: , is_valid_landmark&is_reasonable_landmark);

    plt.triangulated_candidates = triangulated_candidates;
    plt.landmarks_new = new_found_landmarks;
    plt.new_found_keypoints = new_found_keypoints;
    plt.tau_of_new_landmarks = state_curr.Tau(: , : , is_valid_landmark&is_reasonable_landmark);
    plt.F_of_new_landmarks = state_curr.F(: , is_valid_landmark&is_reasonable_landmark);
    plt.first_observation_frame = state_curr.first_observation_frame(: , is_valid_landmark&is_reasonable_landmark);
    plt.is_valid_landmark = is_valid_landmark;
    plt.is_reasonable_landmark = is_reasonable_landmark;
    disp(['Number of added landmarks: ',num2str(sum(is_valid_landmark&is_reasonable_landmark))]);

    % Remove new triangulated landmarks from candidates and add them to landmarks and keypoints
    % Remove also wrongly triangulated points
    state_curr.P = [state_curr.P new_found_keypoints];
    state_curr.X = [state_curr.X new_found_landmarks];
    state_curr.C = state_curr.C( : , ~(is_valid_landmark&is_reasonable_landmark) & ~is_wrong_triangulated );
    state_curr.Tau = state_curr.Tau( : , : , ~(is_valid_landmark&is_reasonable_landmark) & ~is_wrong_triangulated );
    state_curr.F = state_curr.F( : , ~(is_valid_landmark&is_reasonable_landmark) & ~is_wrong_triangulated );
    state_curr.first_observation_frame = state_curr.first_observation_frame( ~(is_valid_landmark&is_reasonable_landmark) & ~is_wrong_triangulated );

    % Add new candidates to state
    state_curr.C = [state_curr.C , new_candidates];
    state_curr.F = [state_curr.F , new_candidates];
    state_curr.first_observation_frame = [state_curr.first_observation_frame, state_curr.frame*ones(1,size(new_candidates,2))];
    state_curr.Tau(:,:,end+1:end+size(new_candidates,2)) = repmat(state_curr.Pose,1,1,size(new_candidates,2));

    % Do debugging plots
    if plot_continous_operation
       plot_debug(plt);
    end
    % Save the current plot in the current state, for adding it to the
    % state history lateron.
    state_curr.plt = plt;

    % Makes sure that plots refresh.
    pause(0.01);

    state_prev = state_curr;
    state_hist = [state_hist, state_curr];
    if BA_enabled & mod(BA_counter,BA_periode)==0 & BA_counter >= BA_window_size
        state_hist(1,end-BA_window_size+1:end) =...
            bundle_adjustment(state_hist(1,end-BA_window_size+1:end),...
            dataset.K, dataset.options.bundle_adjustment.max_iteration);
        state_prev = state_hist(1,end);
    end
    BA_counter = BA_counter + 1;

    if plot_prod
       plot_reproduction_live(state_hist, false, false);
    end
    
    %times = [times toc]; %Use to record iteration times, also enable tic at the beginning of the loop
    
end


%%
data = cell(1,10);
data{1,1} = load('evaluation_BA/state_hist_parking_1.mat');
data{1,2} = load('evaluation_BA/state_hist_parking_2.mat');
data{1,3} = load('evaluation_BA/state_hist_parking_3.mat');
data{1,4} = load('evaluation_BA/state_hist_parking_4.mat');
data{1,5} = load('evaluation_BA/state_hist_parking_5.mat');
data{1,6} = load('evaluation_BA/state_hist_parking_BA_1.mat');
data{1,7} = load('evaluation_BA/state_hist_parking_BA_2.mat');
data{1,8} = load('evaluation_BA/state_hist_parking_BA_3.mat');
data{1,9} = load('evaluation_BA/state_hist_parking_BA_4.mat');
data{1,10} = load('evaluation_BA/state_hist_parking_BA_5.mat');

error_without_BA = zeros(1,5);
error_with_BA = zeros(1,5);

for j = 1:5

    state_hist = data{1,j}.state_hist;
    ground_truth = dataset.ground_truth(:,:,bootstrap_frame+1:end);
    n = size(state_hist,2);
    vo_poses = zeros(3,4,n);
    for i = 1:n 
       vo_poses(:,:,i) =  state_hist(1,i).Pose;
    end
    translation_error = 0;
    scale_factor = 1;
    for i= 1:n-1
        C1_vo = vo_poses(:,:,i);
        C2_vo = vo_poses(:,:,i+1);
        C1_gt = ground_truth(:,:,i);
        C2_gt = ground_truth(:,:,i+1);

        T_C2C1_vo = invert_homo_trans(C2_vo)*[C1_vo;0 0 0 1];
        T_C2C1_gt = invert_homo_trans(C2_gt)*[C1_gt;0 0 0 1];

        trans_vo = T_C2C1_vo(:,4) * scale_factor;
        trans_gt = T_C2C1_gt(:,4);

        translation_error = translation_error + norm(trans_vo - trans_gt,'fro');

        scale_factor = norm(T_C2C1_gt(:,4))/norm(T_C2C1_vo(:,4));
    end

    error_without_BA(1,j) = translation_error;

end

for j = 6:10

    state_hist = data{1,j}.state_hist;
    ground_truth = dataset.ground_truth(:,:,bootstrap_frame+1:end);
    n = size(state_hist,2);
    vo_poses = zeros(3,4,n);
    for i = 1:n 
       vo_poses(:,:,i) =  state_hist(1,i).Pose;
    end
    translation_error = 0;
    scale_factor = 1;
    for i= 1:n-1
        C1_vo = vo_poses(:,:,i);
        C2_vo = vo_poses(:,:,i+1);
        C1_gt = ground_truth(:,:,i);
        C2_gt = ground_truth(:,:,i+1);

        T_C2C1_vo = invert_homo_trans(C2_vo)*[C1_vo;0 0 0 1];
        T_C2C1_gt = invert_homo_trans(C2_gt)*[C1_gt;0 0 0 1];

        trans_vo = T_C2C1_vo(:,4) * scale_factor;
        trans_gt = T_C2C1_gt(:,4);

        translation_error = translation_error + norm(trans_vo - trans_gt,'fro');

        scale_factor = norm(T_C2C1_gt(:,4))/norm(T_C2C1_vo(:,4));
    end

    error_with_BA(1,j-5) = translation_error;

end