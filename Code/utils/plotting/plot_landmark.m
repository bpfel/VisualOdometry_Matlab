% 2018/12/06 somml
function plot_landmark(img_size, pose_pos, pose_rot, keypoint, point_format, distance , K)
    C_keypoint_in_space = K \ [ keypoint ; ones( 1 , size(keypoint,2) ) ] * distance;
    W_keypoint_in_space = [pose_rot,pose_pos] * [ C_keypoint_in_space ; ones( 1 , size(C_keypoint_in_space,2) )];
    % plot keypoints on frame in 3D space
    scatter3(W_keypoint_in_space(1), W_keypoint_in_space(2), W_keypoint_in_space(3), 25, point_format, 'LineWidth', 3)

end

