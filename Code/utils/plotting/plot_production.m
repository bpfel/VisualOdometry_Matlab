function [] = plot_production(state_hist)

global dataset
num_plotted_frames = 200;
num_states = size(state_hist,2);
state_curr = state_hist(1,end);
img_curr = get_frame(dataset,state_curr.frame);

% get all position from the last ... states
position_track = zeros(3,num_states);
landmark_track = [];
for i = 1:num_states
  position_track(:,i) = state_hist(1,i).Pose(1:3,4);
  landmark_track = [landmark_track , state_hist(1,i).X(:,:)];
end
landmark_track = unique(landmark_track','rows','stable')';

fig11 = figure(11);
fig11.Visible = true;
% make it fullscreen
fig11.Units = 'normalized';
fig11.OuterPosition = [0 0 1 1];

% plot current frame and all candidates and keypoints
l1 = subplot(2,2,[1]); % Plot current frame with matched candidates/keyframes and new candidates/keypoints
    imshow(img_curr);
    hold on;
    scatter(state_curr.P(1,:),state_curr.P(2,:),'ro');    
    scatter(state_curr.C(1,:),state_curr.C(2,:),'bx');
%     %define the lines connecting old keypoints to new keypoints
%     [x_track_keypoints,y_track_keypoints] = connecting_lines(...
%         plt.old_matched_keypoints, plt.new_matched_keypoints);
%     plot(x_track_keypoints,y_track_keypoints,'r','LineWidth',2);
%     [x_track_candidates,y_track_candidates] = connecting_lines(...
%         plt.old_matched_candidates, plt.new_matched_candidates);
%     plot(x_track_candidates,y_track_candidates,'b','LineWidth',1);
    legend({'all keypoints','all candidates'},'Location','ne');
    title(['matched feature in current frame: ' num2str(state_curr.frame)]);
    hold off;
% plot landmark and the position track
l2 = subplot(2,2,[2 4]);
    index = max([1, size(position_track,2)-num_plotted_frames]);
    scatter3(state_curr.X(1, :), state_curr.X(2, :), state_curr.X(3, :), 5,'g');
    hold on;
    plotCoordinateFrame(state_curr.Pose(:,1:3), state_curr.Pose(:,4), 4);
    line(position_track(1,index:end),position_track(2,index:end),position_track(3,index:end),'LineWidth',1)
    hold off;
    set(gcf, 'GraphicsSmoothing', 'on');
    view(0,0);
    axis equal;
    axis vis3d;
    axis([state_curr.Pose(1,4)-20 state_curr.Pose(1,4)+20 state_curr.Pose(2,4)-20 state_curr.Pose(2,4)+20 state_curr.Pose(3,4)-20 state_curr.Pose(3,4)+30]); % TODO: change this dependend of pose
    title(['landmarks and Pose']);

% plot all the landmarks and the whole path
l3 = subplot(2,2,[3]);
    scatter3(landmark_track(1, :), landmark_track(2, :), landmark_track(3, :), 5,'g');
    hold on;
    plotCoordinateFrame(state_curr.Pose(:,1:3), state_curr.Pose(:,4), 4);
    line(position_track(1,:),position_track(2,:),position_track(3,:),'LineWidth',2)
    hold off;
    set(gcf, 'GraphicsSmoothing', 'on');
    view(0,0);
    axis equal;
    axis([-Inf Inf -Inf Inf -Inf Inf])
    title(['all landmarks and path']);
    
    
fig11.Visible = true;
end