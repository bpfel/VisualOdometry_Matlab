% 2018/12/06 somml
function plot_frame(img, pose_pos, pose_rot, pose_format,distance , K)
        % distance: how far the image plane is in front of the camera in
        % meters

        [height,width] = size(img);
        top_left = [1;1];
        top_right = [width;1];
        bottom_left = [1;height];
        bottom_right = [width;height];
        image_corners = ([top_left , top_right , bottom_left , bottom_right]);
        image_corners = [image_corners; ones(1,size(image_corners,2))];
        % Get normalized corners in camera frame
        for i = 1:4
           image_corners(:,i) = K\image_corners(:,i)*distance;
           image_corners(:,i) = pose_pos + pose_rot * image_corners(:,i);
        end
        
        % edges of image plane
        corner1 = image_corners(:,1);
        corner2 = image_corners(:,2);
        corner3 = image_corners(:,3);
        corner4 = image_corners(:,4);   
        
        x_img = [corner1(1), corner2(1); corner3(1), corner4(1)];
        y_img = [corner1(2), corner2(2); corner3(2), corner4(2)];
        z_img = [corner1(3), corner2(3); corner3(3), corner4(3)];   
        % span surface and color with image
        surf(x_img,y_img,z_img,...
                'CData',img,'FaceColor','texturemap','EdgeColor','none');
        % set colormap for gray image
        colormap(gray(256))
        hold on
        % plot pose in center of image
        scatter3(pose_pos(1), pose_pos(2), pose_pos(3), 55, pose_format, 'LineWidth', 3)
        % plot camera triangle
        line([pose_pos(1), corner1(1)],[pose_pos(2), corner1(2)],[pose_pos(3), corner1(3)],'Color','black','LineWidth', 1);
        line([pose_pos(1), corner2(1)],[pose_pos(2), corner2(2)],[pose_pos(3), corner2(3)],'Color','black','LineWidth', 1);
        line([pose_pos(1), corner3(1)],[pose_pos(2), corner3(2)],[pose_pos(3), corner3(3)],'Color','black','LineWidth', 1);
        line([pose_pos(1), corner4(1)],[pose_pos(2), corner4(2)],[pose_pos(3), corner4(3)],'Color','black','LineWidth', 1);

end

