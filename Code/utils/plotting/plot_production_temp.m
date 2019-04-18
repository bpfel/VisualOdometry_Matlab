function [] = plot_production_temp(state_hist)

global dataset
num_plotted_frames = size(state_hist,2);
state_curr = state_hist(1,end);

% get all position from the last ... states
position_track = cell(1,num_plotted_frames);
landmark_track = [];

for i = 0:num_plotted_frames-1
   if size(state_hist,2)-i < 1   % handle the start, where we dont have enough state to plot  
      position_track{1,end-i} = state_hist(1,1).Pose(1:3,4);  
   else
      position_track{1,end-i} = state_hist(1,end-i).Pose(1:3,4);
      landmark_track = [landmark_track , state_hist(1,end-i).X(:,:)];
   end
end
position_track = cell2mat(position_track);
landmark_track = unique(landmark_track','rows','stable')';

% plot all the landmarks and the whole path
l1 = subplot(2,2,[1 3]);
    scatter3(landmark_track(1, :), landmark_track(2, :), landmark_track(3, :), 5,'g');
    hold on;
    plotCoordinateFrame(state_curr.Pose(:,1:3), state_curr.Pose(:,4), 4);
    line(position_track(1,:),position_track(2,:),position_track(3,:),'LineWidth',1)
    hold off;
    set(gcf, 'GraphicsSmoothing', 'on');
    view(0,0);
    axis equal;
    axis([-Inf Inf -Inf Inf -Inf Inf])
    title(['all landmarks and path']);

% plot landmark and the position track
l2 = subplot(2,2,[2,4]);
    scatter3(state_curr.X(1, :), state_curr.X(2, :), state_curr.X(3, :), 5,'g');
    hold on;
    plotCoordinateFrame(state_curr.Pose(:,1:3), state_curr.Pose(:,4), 4);
    line(position_track(1,:),position_track(2,:),position_track(3,:),'LineWidth',1)
    hold off;
    set(gcf, 'GraphicsSmoothing', 'on');
    view(0,0);
    axis equal;
    axis vis3d;
    axis([state_curr.Pose(1,4)-20 state_curr.Pose(1,4)+20 state_curr.Pose(2,4)-20 state_curr.Pose(2,4)+20 state_curr.Pose(3,4)-20 state_curr.Pose(3,4)+30]); % TODO: change this dependend of pose
    title(['current landmarks and Pose']);

end