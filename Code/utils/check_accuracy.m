function [R_error, t_error] = check_accuracy(first_frame_index, second_frame_index, T_C2C1)
% CHECK_ACCURACY Compare relative transformation between two frames
%   first_frame - index of first frame
%   second_frame - index of second frame
%   T_C2C1 - 3x4 matrix = [R|t], transformation matrix from second frame to
%   first
%OUTPUT Error of rotation and translation

R_error = zeros(3,3);
t_error = zeros(3,1);
global dataset;
first_frame_index = first_frame_index + 1 - dataset.first_frame;
second_frame_index = second_frame_index + 1 - dataset.first_frame;

if ~dataset.has_ground_truth
    disp('Dataset does not have ground truth')
else
    T_WC1_gt = dataset.ground_truth(:,:,first_frame_index);
    T_WC2_gt = dataset.ground_truth(:,:,second_frame_index);
    T_C2C1_gt = invert_homo_trans(T_WC2_gt)*[T_WC1_gt;0 0 0 1];
      
    R_error = T_C2C1_gt(:,1:3) - T_C2C1(:,1:3);
    R_error_norm = norm(R_error,'fro');
    
    t_error = (T_C2C1_gt(:,4)/norm(T_C2C1_gt(:,4)))...
        - (T_C2C1(:,4)/norm(T_C2C1(:,4)));
    t_error_norm = norm(t_error);
    
%     disp(['Error on rotation: ',num2str(R_error_norm),...
%         ', error on translation: ',num2str(t_error_norm)]);     
end

end