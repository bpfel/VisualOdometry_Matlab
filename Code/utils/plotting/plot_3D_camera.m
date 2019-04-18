function [] = plot_3D_camera(plt)
    global dataset

    % load images
    img_origin = get_frame(dataset,single(plt.frame_first_index));
    img_curr = get_frame(dataset,plt.frame);
    img_prev = get_frame(dataset,plt.frame-1);
    img_prev_prev = get_frame(dataset,plt.frame-1);

    distance = 0.5;
    % plot images in space, the other will be added later   
    plot_frame(img_origin, [0;0;0;], eye(3),'y+',distance,plt.K); % set frame in 3D room
    hold on
    plot_frame(img_curr, plt.pose(1:3,4), plt.pose(1:3,1:3),'r+',distance,plt.K);

    frames = []; % save all frames where a landmark was first observed
    
    for j = 1:size(plt.landmarks_new,2)
        % ray from positions to landmarks
        init_ray = plt.landmarks_new(:,j)-plt.tau_of_new_landmarks(:,4,j);
        curr_ray = plt.landmarks_new(:,j)-plt.pose(:,4);
        quiver3(plt.pose(1,4),plt.pose(2,4), plt.pose(3,4), curr_ray(1), curr_ray(2), curr_ray(3), 'r');
        quiver3(plt.tau_of_new_landmarks(1,4,j), plt.tau_of_new_landmarks(2,4,j), plt.tau_of_new_landmarks(3,4,j), init_ray(1), init_ray(2), init_ray(3), 'g');
        % plot landmarks
        scatter3(plt.landmarks_new(1,j), plt.landmarks_new(2,j), plt.landmarks_new(3,j), 25, 'bo', 'LineWidth', 3)
        % plot keypoints of landmarks on the frames
        plot_landmark(size(img_curr),plt.pose(1:3,4), plt.pose(1:3,1:3), plt.new_found_keypoints(:,j),'ro',distance,plt.K);
        plot_landmark(size(img_origin),plt.tau_of_new_landmarks(1:3,4,j), plt.tau_of_new_landmarks(1:3,1:3,j), plt.F_of_new_landmarks(:,j),'go',distance,plt.K);
        % gather all frames
        frames = [frames, plt.first_observation_frame(j)];
        % remove dublicates
        frames = unique(frames.','rows').';
    end
    % plot those frames
    for k=1:size(frames,2)
        img = get_frame(dataset,frames(k));
        plot_frame(img, plt.tau_of_new_landmarks(1:3,4,k), plt.tau_of_new_landmarks(1:3,1:3,k),'g+',distance,plt.K)
    end
    % set view (from top, y-axis)
    axis vis3d
    view(0,-60)
    axis equal
    hold off
    title("red for current pose, yellow for first pose")
end