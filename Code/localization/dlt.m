function M = dlt(p, P, K)
% Estimates the pose of a camera using a set of 2D-3D correspondences and a
% given camera matrix using Direct Linear Transform
%
% p: [2xN] vector containing the undistorted coordinates of the 2D points
% P: [3xN] vector containing the 3D point positions
% K: [3x3] camera intrinsics
%
% M: [3x4] projection matrix under the form M=[R|t] where R is a rotation
%    matrix. M encodes the transformation that maps points from the world
%    frame to the camera frame

    N = size(p,2);

    %convert p to normalized coordinates
    p_undist = (inv(K)*[p; ones(1,N)]);
    
    %fill measurement matrix Q
    Q = zeros(2*N,12);
    for i=1:N
       Q((2*i-1):(2*i),:) =  kron([1 0 -p_undist(1,i); 0 1 -p_undist(2,i)],[P(:,i)' 1]);
    end

    % Solve for Q*M=0
    [~,~,V] = svd(Q);
    M_temp = reshape(V(:,end),[4,3])';
    M_temp = M_temp * sign(det(M_temp(:,1:3)));

    %Extract R and T from M_temp
    R_temp = M_temp(:,1:3);
    [U,~,V] = svd(R_temp);
    R_tilde = U*V';
    alpha = norm(R_tilde,'fro')/norm(R_temp,'fro');

    M = [R_tilde M_temp(:,4)*alpha];
end