function [] = plotMatchesAndPointCloud(T_W_C1, T_W_C2, W_landmarks, img1,...
    img2, matched_features_img1, matched_features_img2,average_depth, ratio)
    
    % INPUT Homogenious Transformation
    R_W_C1 = T_W_C1(1:3,1:3);
    t_W_C1 = T_W_C1(1:3,4);
    R_W_C2 = T_W_C2(1:3,1:3);
    t_W_C2 = T_W_C2(1:3,4);
    
    figure();
    if(nargin>3)
        subplot(2,2,1);
    end
    
    % P is a [4xN] matrix containing the triangulated point cloud (in
    % homogeneous coordinates), given by the function linearTriangulation
    plot3(W_landmarks(1,:), W_landmarks(2,:), W_landmarks(3,:), 'o');

    % Display camera pose

    plotCoordinateFrame( R_W_C1 , t_W_C1 , 6.0);
    text(t_W_C1(1)-0.3,t_W_C1(1)-0.3,t_W_C1(1)-0.3,'Cam 1','fontsize',10,'color','k','FontWeight','bold');

    plotCoordinateFrame(R_W_C2,t_W_C2, 6.0);
    text(t_W_C2(1)-0.1, t_W_C2(2)-0.1, t_W_C2(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');

    axis equal
    rotate3d on;
    grid;
    
    if(nargin>3)
        % Display matched points
        subplot(2,2,2);
        imshow(img1);
        hold on;
        scatter(matched_features_img1(1,:),matched_features_img1(2,:)); 
        
        %Display 2D projection
        subplot(2,2,3);
        scatter(W_landmarks(1,:),W_landmarks(3,:))
        hold on;
        plot([min(W_landmarks(1,:)),max(W_landmarks(1,:))],average_depth*[1 1]);
        plotCoordinateFrame(R_W_C2,t_W_C2, 6.0);
        text(t_W_C2(1)-0.1, t_W_C2(2)-0.1, t_W_C2(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');
        legend({'Ratio: ' num2str(ratio)});
        axis equal

        subplot(2,2,4);
        imshow(img2);
        hold on;
        scatter(matched_features_img1(1,:),matched_features_img1(2,:),'o');    
        scatter(matched_features_img2(1,:),matched_features_img2(2,:),'x');
        %define the lines connecting old keypoints to new keypoints
        [x_track,y_track] = connecting_lines(...
            matched_features_img1, matched_features_img2);
        plot(x_track,y_track,'r','LineWidth',2);
        legend({'Features_img1','Features_img2','tracks'})
    end

end

