function [] = plot_debug(plt)
%
%   This function helps to plot multiple informations(keypoints, landmarks,
%   ransac iterations) all in one figure
%
%   INPUT:
%           plt - struct that contains all information
%

global dataset

% load images
img_origin = get_frame(dataset,single(plt.frame_first_index));
img_curr = get_frame(dataset,plt.frame);
img_prev = get_frame(dataset,plt.frame-1);
img_prev_prev = get_frame(dataset,plt.frame-1);
    
fig10 = figure(10);
fig10.Visible = true;
% make it fullscreen
fig10.Units = 'normalized';
fig10.OuterPosition = [0 0 1 1];

    
h1 = subplot(4,2,[1 3]); % Plot previous frame with lost candidates/keyframes
    imshow(img_prev);
    hold on;
    scatter(plt.lost_keypoints(1,:),plt.lost_keypoints(2,:),'ro');    
    scatter(plt.lost_candidates(1,:),plt.lost_candidates(2,:),'bx');
    legend({'lost keypoints','lost candidates'},'Location','ne');
    title('lost features in old frame');
    hold off;
    
h2 = subplot(4,2,[2 4]); % Plot current frame with matched candidates/keyframes and new candidates/keypoints
    imshow(img_curr);
    hold on;
    scatter(plt.new_matched_keypoints(1,:),plt.new_matched_keypoints(2,:),'ro');    
    scatter(plt.new_matched_candidates(1,:),plt.new_matched_candidates(2,:),'bx');
    scatter(plt.new_found_keypoints(1,:),plt.new_found_keypoints(2,:),'gx');
    %define the lines connecting old keypoints to new keypoints
    [x_track_keypoints,y_track_keypoints] = connecting_lines(...
        plt.old_matched_keypoints, plt.new_matched_keypoints);
    plot(x_track_keypoints,y_track_keypoints,'r','LineWidth',2);
    [x_track_candidates,y_track_candidates] = connecting_lines(...
        plt.old_matched_candidates, plt.new_matched_candidates);
    plot(x_track_candidates,y_track_candidates,'b','LineWidth',1);
    legend({'matched keypoints','matched candidates','new found keypoint'},'Location','ne');
    title(['matched feature in current frame: ' num2str(plt.frame)]);
    hold off;
    
h3 = subplot(4,2,[5 7]); % Plot pointcloud from above with coordinate frame 
    scatter3(plt.landmark_inlier(1, :), plt.landmark_inlier(2, :), plt.landmark_inlier(3, :), 5,'g');
    hold on;
    scatter3(plt.landmark_outlier(1,:),plt.landmark_outlier(2,:), plt.landmark_outlier(3,:),5,'r');
    scatter3(plt.landmarks_new(1,:),plt.landmarks_new(2,:), plt.landmarks_new(3,:),5,'b');
    plotCoordinateFrame(plt.pose(:,1:3), plt.pose(:,4), 4);
    line([plt.pose_prev(1,4) plt.pose(1,4)],[plt.pose_prev(2,4) plt.pose(2,4)],[plt.pose_prev(3,4) plt.pose(3,4)])
    hold off;
    set(gcf, 'GraphicsSmoothing', 'on');
    view(0,0);
    axis equal;
    axis vis3d;
    axis([plt.pose(1,4)-20 plt.pose(1,4)+20 plt.pose(2,4)-20 plt.pose(2,4)+20 plt.pose(3,4)-1 plt.pose(3,4)+30]); % TODO: change this dependend of pose
    legend({'localisation inlier','localisation outlier','new triangulation'},'Location','se')
    title(['PointCloud and Pose: localization inlier: ', num2str(plt.num_inlier),' / ', num2str(plt.num_total),' ; RANSAC succ iter: ', num2str(plt.num_best_iterations)]);

h4 = subplot(4,2,[6 8]); % 3D triangulation plot

%     cla(h4);
    
%     plot_3D_camera(plt);
    plot_candidates(plt);

fig10.Visible = true;


end
