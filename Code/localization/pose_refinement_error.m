function reprojectionError = pose_refinement_error(x , W_landmarks, keypoints, K)
    
    Pose = twist2HomogMatrix(x);
    % projection matrix
    M = K*invert_homo_trans(Pose(1:3,:));
    
    % reproject points
    img_coords = M * [W_landmarks;ones(1,size(W_landmarks,2))];
    % dehomogize image coordinates
    reprojections = img_coords(1:2,:)./img_coords(3,:);

    reprojectionError = keypoints(:) - reprojections(:);

end