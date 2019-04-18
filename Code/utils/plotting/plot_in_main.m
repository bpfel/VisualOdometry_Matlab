figure(77);
    hold on
    axis equal
    
    orig_img = get_frame(dataset,candidate_index);
    curr_img = get_frame(dataset,state_curr.frame);
    plot_frame(orig_img, [0;0;0;], eye(3),'y+')
    plot_frame(curr_img, state_curr.Pose(1:3,4), state_curr.Pose(1:3,1:3),'r+')
    frames = [NaN];
    title("red for current pose, green for first pose")
    
    for i = 1:size(new_landmarks,2)
        %         title(['First sight  in frame: ', num2str(state_curr.first_observation_frame(i)), ', current frame is: ', num2str(state_curr.frame), '. Angle: ' , num2str(angles(i)) , ' Threshold: ' , num2str(ang_thr)]);
        
        init_ray = new_landmarks(:,i)-state_curr.Tau(:,4,i);
        curr_ray = new_landmarks(:,i)-state_curr.Pose(:,4);
        
        scatter3(new_landmarks(1,i), new_landmarks(2,i), new_landmarks(3,i), 15, 'b+', 'LineWidth', 3)
        
        plot_landmark(size(curr_img), state_curr.Pose(1:3,4), state_curr.Pose(1:3,1:3), state_curr.C(:,i),'ro')
        plot_landmark(size(orig_img), state_curr.Tau(1:3,4,i), state_curr.Tau(1:3,1:3,i), state_curr.F(:,i),'go')
        
        quiver3(state_curr.Pose(1,4), state_curr.Pose(2,4), state_curr.Pose(3,4), curr_ray(1), curr_ray(2), curr_ray(3), 'r');
        quiver3(state_curr.Tau(1,4,i), state_curr.Tau(2,4,i), state_curr.Tau(3,4,i), init_ray(1), init_ray(2), init_ray(3), 'g');
        
        
        if frames(end) ~= state_curr.first_observation_frame(i)
            frames = [frames, state_curr.first_observation_frame(i)];
        end
    end
    
    for i=2:size(frames+1,2)
        first_img = get_frame(dataset,frames(i));
        plot_frame(first_img, state_curr.Tau(1:3,4,i), state_curr.Tau(1:3,1:3,i),'g+')
    end
    
    view(state_curr.Pose(1:3,1:3)*[0;1;0])
    
    hold off