function [] = plot_state_history_map(state_hist)

    % n - number of states
    n = size(state_hist,2);
    landmarks = cell(1,n); % stores all landmarks for each state 
    poses = cell(1,n);    
    
    % extract all landmarks and states
    for i = 1:n
        state = state_hist(1,i);
        landmarks{1,i} = state.X;
        poses{1,i} = state.Pose;
    end
      
    % build unique landmark 
    landmark = cell2mat(landmarks);
    [landmark_unique] = unique(landmark','rows','stable');
    landmark_unique = landmark_unique';

    figure;
    scatter3(landmark_unique(1, :), landmark_unique(2, :), landmark_unique(3, :), 5,'g');    
    hold on;
    for i = 1:n
        plotCoordinateFrame(poses{1,i}(:,1:3), poses{1,i}(:,4), 2);
    end
    hold off;

    axis equal;
    axis([-10 10 -20 20 -1 30]);
    view(0,0);
    axis vis3d;
end