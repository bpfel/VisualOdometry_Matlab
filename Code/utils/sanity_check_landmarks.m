function is_reasonable_landmark = sanity_check_landmarks(T_C_W, W_landmarks,sanity_check_factor)
% This function checks if the landmarks are reasonable:
% In front of the camera and not to far away
%
% INPUT
%       T_C_W - 3x4 - homogenous transformation from world frame to camera
%       W_landmarks - 3xN - position of N landmark in world frame

C_landmarks = T_C_W * [ W_landmarks ; ones( 1 , size(W_landmarks,2) ) ];

average_depth = median( C_landmarks(3,:) );

is_reasonable_landmark = (C_landmarks(3,:) > 0) & (C_landmarks(3,:) < average_depth*sanity_check_factor);


end