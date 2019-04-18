function hidden_state = runBA(hidden_state, first_pose, second_pose, observations, K,max_iter)
% Update the hidden state, encoded as explained in the problem statement,
% with 20 bundle adjustment iterations.
with_plot = false;
with_pattern = true;

if with_pattern % provide the nonlinear optimization with information on
    % which parts of the jacobian matrix (calculated for gradient descent)
    % will be nonzero to accelerate the algorithm.
    num_frames = observations(1);
    num_observations = (numel(observations)-2-num_frames)/3;
    % Factor 2, one error for each x and y direction.
    num_error_terms = 2 * num_observations;
    % Each error term will depend on one pose (6 entries), but not the first one and one landmark
    % position (3 entries), so 9 nonzero entries per error term:
    pattern = spalloc(num_error_terms, numel(hidden_state)-2*6, ...
        num_error_terms * 9);
    
    % Fill pattern for each frame individually:
    observation_i = 3;  % iterator into serialized observations
    error_i = 1;  % iterating frames, need another iterator for the error
    for frame_i = 1:num_frames
        num_keypoints_in_frame = observations(observation_i);
        % All errors of a frame are affected by its pose.
        if frame_i > 2 %add not first two poses as optimization variable
            pattern(error_i:error_i+2*num_keypoints_in_frame-1, ...
                 (frame_i-3)*6+1:(frame_i-2)*6) = 1;
        end
        
        % Each error is then also affected by the corresponding landmark.
        landmark_indices = observations(...
            observation_i+2*num_keypoints_in_frame+1:...
            observation_i+3*num_keypoints_in_frame);
        for kp_i = 1:numel(landmark_indices)
            pattern(error_i+(kp_i-1)*2:error_i+kp_i*2-1,...
                1+(num_frames-2)*6+(landmark_indices(kp_i)-1)*3:...
                (num_frames-2)*6+landmark_indices(kp_i)*3) = 1;
        end
        
        observation_i = observation_i + 1 + 3*num_keypoints_in_frame;
        error_i = error_i + 2 * num_keypoints_in_frame;
    end
    
    if with_plot
        figure(4);
        spy(pattern);
    end
end

% Also here, using an external error function for clean code.
error_terms = @(x) baError(x, first_pose,second_pose, observations, K);
options = optimoptions(@lsqnonlin, 'Display', 'iter', ...
    'MaxIter', max_iter);
if with_pattern
    options.JacobPattern = pattern;
    options.UseParallel = false;
end
hidden_state_wo_first = lsqnonlin(error_terms, hidden_state(13:end,1), [], [], options);
hidden_state = [HomogMatrix2twist(first_pose);HomogMatrix2twist(second_pose);hidden_state_wo_first];
end




 