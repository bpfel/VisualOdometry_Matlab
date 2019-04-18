function p = project_points(P,K)
% Projects points from 3D onto the image plane 
%
% P: [3xN] vector containing the 3D positions in the camera frame
% K: [3x3] camera intrinsics
%
% p: [2xN] projected points on the camera plane
    
    vec = K * P;
    p = vec(1:2,:)./vec(3,:);
    
end
