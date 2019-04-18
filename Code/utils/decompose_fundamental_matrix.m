function [rotation_matrix, translation_vector] =...
    decompose_fundamental_matrix(F,K,matched_keypoints_img0,matched_keypoints_img1)
%RELATIVE_POSE Estimate the relative pose of two frames based on the
%fundamental matrix and the camera intrinsics, it also disambiguate the
%four possible solution, checking which has the most triangulated points in
%front
%
%   INPUT
%       F - fundamental_matrix - 3x3 matrix
%       K - camera_intrinsics  - 3x3 matrix
%       matched_keypoints_img0 - 2xN matrix
%       matched_keypoints_img1 - 2xN matrix
%   OUTPUT
%       rotation_matrix    - 3x3 - world to camera
%       translation_vector - 3x1 - world to camera
%

    % Essential matrix - mono VO -> K is always the same
    E = K'*F*K;

    [U,~,V] = svd(E);

    % Translation
    u3 = U(:,3);

    % Rotations
    W = [0 -1 0; 1 0 0; 0 0 1];
    Rots(:,:,1) = U*W*V.';
    Rots(:,:,2) = U*W.'*V.';

    if det(Rots(:,:,1))<0
        Rots(:,:,1)=-Rots(:,:,1);
    end

    if det(Rots(:,:,2))<0
        Rots(:,:,2)=-Rots(:,:,2);
    end

    if norm(u3) ~= 0
        u3 = u3/norm(u3);
    end

    M0 = K * eye(3,4); % Projection matrix of camera 1

    total_points_in_front_best = 0;
    for iRot = 1:2
        R_C1_C0_test = Rots(:,:,iRot);

        for iSignT = 1:2
            T_C1_C0_test = u3 * (-1)^iSignT;

            M1 = K * [R_C1_C0_test, T_C1_C0_test];
            P_C0 = linear_triangulation(matched_keypoints_img0,matched_keypoints_img1,M0,M1);
            
            P_C0_homo = [P_C0; ones(1,size(P_C0,2))];

            % project in both cameras
            P_C1 = [R_C1_C0_test T_C1_C0_test] * P_C0_homo;

            num_points_in_front0 = sum(P_C0(3,:) > 0);
            num_points_in_front1 = sum(P_C1(3,:) > 0);
            total_points_in_front = num_points_in_front0 + num_points_in_front1;

            if (total_points_in_front > total_points_in_front_best)
                % Keep the rotation that gives the highest number of points
                % in front of both cameras
                rotation_matrix = R_C1_C0_test;
                translation_vector = T_C1_C0_test;
                total_points_in_front_best = total_points_in_front;
            end
        end
    end

end

