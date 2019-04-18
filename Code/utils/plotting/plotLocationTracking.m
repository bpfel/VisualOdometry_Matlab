function [] = plotLocationTracking(frame, img, inlier_mask, Pose, W_matches, p_matches, num_iterations) 
%plots: 
% 1. img and outlier and inliers
% 2. 3d pointcloud and pose
% 3. number of inliers vs landmarks
% 4. number of iterations for ransac to converge

figure(37);

p_inlier = p_matches(:,inlier_mask);
p_outlier = p_matches(:,~inlier_mask);
W_inlier = W_matches(:,inlier_mask);
W_outlier = W_matches(:,~inlier_mask);

num_inlier = nnz(inlier_mask);
num_matches = size(inlier_mask,2);

% 1. img and outlier and inliers
subplot(2, 2, 1);
imshow(img);
hold on;
if(num_inlier < num_matches)
    plot(p_outlier(1,:),p_outlier(2,:), 'rx', 'Linewidth', 2);
end
if (num_inlier > 0)
    plot(p_inlier(1,:),p_inlier(2,:), 'gx', 'Linewidth', 2);
end
hold off
title('Outlier and inlier features')


% 2. 3d pointcloud and pose
subplot(2, 2, 2);
scatter3(W_matches(1, :), W_matches(2, :), W_matches(3, :), 5);
hold on;
if(num_inlier < num_matches)
    scatter3(W_outlier(1,:),W_outlier(2,:), W_outlier(3,:),5,'r');
end
if (num_inlier > 0)
    scatter3(W_inlier(1,:),W_inlier(2,:), W_inlier(3,:),5,'g');
end

plotCoordinateFrame(Pose(:,1:3), Pose(:,4), 4);
hold off

set(gcf, 'GraphicsSmoothing', 'on');
view(0,0);
axis equal;
axis vis3d;
axis([-8 8 -10 5 -1 20]);
title('3D-PointCloud and Pose')


% 3. number of inliers vs landmarks
subplot(2, 2, 3);
scatter(frame,num_inlier,5,'r')
hold on
scatter(frame,num_matches,5,'b')
title('Number of inliers and matches vs. frame number')


% 4. number of iterations needed for ransac
subplot(2,2,4)
scatter(frame,num_iterations,5,'b')
hold on
title('Number of iteration at which RANSAC found the best solution')



end
