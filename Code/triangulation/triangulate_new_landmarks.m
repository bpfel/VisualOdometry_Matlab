function [new_landmarks] = triangulate_new_landmarks(state_curr,is_valid_landmark,K)
%TRIANGULATE_NEW_LANDMARKS calculates the coordinates of new landmarks with the help of the current pose,
%coordinates of the landmark, pose at first observation and pose at that time.
%   INPUT:
%   state_curr - struct of current state
%   is_valid_landmark - boolean vector to extract coordinates of landmarks form keypoint coordinates
%   OUTPUT:
%   new_keypoints: updated list of keypoints, add coordinates of landmarks
%   new_landmarks: updated list of keypoints, add newly triangulated landmarks
    
    keypoints_curr = state_curr.C( : , is_valid_landmark);
    k = size(keypoints_curr,2);
    keypoints_first_obs  = state_curr.F( : , is_valid_landmark);
    pose_curr = invert_homo_trans( state_curr.Pose );
    pose_first_obs = state_curr.Tau( : , : , is_valid_landmark);
    
    new_landmarks = zeros(3,k);
    
    for i=1:k

        
        new_landmarks(:,i) = linear_triangulation(keypoints_curr(:,i),...
                          keypoints_first_obs(:,i), K * pose_curr, K * invert_homo_trans( pose_first_obs(:,:,i) ) );

    end

    
end