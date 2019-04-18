function [] = plot_candidates(plt)

    global dataset;

%     disp(['Number of candiates: ',num2str(size(plt.new_matched_candidates,2)),', number of valid ones: ',num2str(size(plt.landmarks_new))])
    subplot(4,2,6)
    imshow(get_frame(dataset, plt.frame))
    hold on
    scatter(plt.new_matched_candidates(1,:),plt.new_matched_candidates(2,:),'rx');    
    scatter(plt.F(1,:),plt.F(2,:),'gx');
    %define the lines connecting old keypoints to new keypoints
    % Not valid -> blue
    [x_track_keypoints,y_track_keypoints] = connecting_lines(...
        plt.new_matched_candidates(:,~plt.is_valid_landmark), plt.F(:,~plt.is_valid_landmark));
    plot(x_track_keypoints,y_track_keypoints,'b','LineWidth',2);
    % Reasonable and valid -> green
    [x_track_keypoints,y_track_keypoints] = connecting_lines(...
        plt.new_matched_candidates(:,plt.is_valid_landmark & plt.is_reasonable_landmark), plt.F(:,plt.is_valid_landmark & plt.is_reasonable_landmark));
    plot(x_track_keypoints,y_track_keypoints,'g','LineWidth',2);
    % Valid but not reasonable -> red
    [x_track_keypoints,y_track_keypoints] = connecting_lines(...
        plt.new_matched_candidates(:,plt.is_valid_landmark & ~plt.is_reasonable_landmark), plt.F(:,plt.is_valid_landmark & ~plt.is_reasonable_landmark));
    plot(x_track_keypoints,y_track_keypoints,'r','LineWidth',2);
%     legend('Candidates current frame','Candidates first occurance','Valid & Reasonable','Valid & not Reasonable','Not Valid')
    hold off
    legend({'tracked candidate','first observation','not valid tracks','reasonable and valid tracks','valid but not reasonable tracks'},'Location','ne')

    title('tracked candidates');
    
    
    subplot(4,2,8)
    imshow(get_frame(dataset, plt.frame))
    hold on
    landmarks_keypoints_camera = invert_homo_trans(plt.pose)*[plt.triangulated_candidates;ones(1,size(plt.triangulated_candidates,2))];
    landmarks_keypoints_camera = landmarks_keypoints_camera(1:3,:);
    projected_points = project_points(landmarks_keypoints_camera,plt.K);
    scatter(plt.new_matched_candidates(1,:),plt.new_matched_candidates(2,:),'rx');    
    scatter(projected_points(1,:),projected_points(2,:),'gx');
    %define the lines connecting old keypoints to new keypoints
    % Not valid -> blue
    [x_track_keypoints,y_track_keypoints] = connecting_lines(...
        plt.new_matched_candidates(:,~plt.is_valid_landmark), projected_points(:,~plt.is_valid_landmark));
    plot(x_track_keypoints,y_track_keypoints,'b','LineWidth',2);
    % Reasonable and valid -> green
    [x_track_keypoints,y_track_keypoints] = connecting_lines(...
        plt.new_matched_candidates(:,plt.is_valid_landmark & plt.is_reasonable_landmark), projected_points(:,plt.is_valid_landmark & plt.is_reasonable_landmark));
    plot(x_track_keypoints,y_track_keypoints,'g','LineWidth',2);
    % Valid but not reasonable -> red
    [x_track_keypoints,y_track_keypoints] = connecting_lines(...
        plt.new_matched_candidates(:,plt.is_valid_landmark & ~plt.is_reasonable_landmark), projected_points(:,plt.is_valid_landmark & ~plt.is_reasonable_landmark));
    plot(x_track_keypoints,y_track_keypoints,'r','LineWidth',2);
%     legend('Candidates current frame','Candidates first occurance','Valid & Reasonable','Valid & not Reasonable','Not Valid')
    hold off
    legend({'tracked candidates','backprojected triangulations','not valid tracks','reasonable and valid tracks','valid but not reasonable tracks'},'Location','se')
    title('reprojected candidates landmark')

%     disp(['Triangulated Landmarks median: ',num2str(median(plt.triangulated_candidates(3,:)))]);    
%     disp(plt.triangulated_candidates(:,plt.is_valid_landmark));   
    

end