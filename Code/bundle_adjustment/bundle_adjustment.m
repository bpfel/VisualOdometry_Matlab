function adjusted_state_history = bundle_adjustment(state_history, K, max_iter )
% This function takes a history of states and put them into the same
% formulation of exercise 9(bundle ajustment). then runs lsqnonlin to
% reduce the total reprojection error. the same landmarks in different
% states should have the same coordinates, otherwise it does not work. The
% sliding window has to be therefore dense or smartly select only
% keyframes.
%
%   INPUT:
%           -state_history - 1xn - state structs
%           
%   OUTPUT:
%           -state_history - 1xn - state structs- adjusted pose and
%                                                 landmarks
%
    adjusted_state_history = state_history;
    
    % n - number of states
    n = size(state_history,2);
    ind_state2queue = cell(1,n); % stores for each state where it is in the queue
    num_keypoints = cell(1,n); % stores number of keypoints for each state 
    keypoints = cell(1,n); % stores keypoints for each state 
    landmarks = cell(1,n); % stores all landmarks for each state 
    poses = cell(1,n); % stores all poses for each state
    poses_adjusted = cell(1,n);
    start_index = 0;
    
    
    % extract all twists and concatenate all landmarks/keypoints and build
    % index
    for i = 1:n
        state = state_history(1,i);
        num_keypoints{1,i} = size(state.P,2);
        keypoints{1,i} = double(state.P);
        landmarks{1,i} = state.X;
        poses{1,i} = state.Pose;
        ind_state2queue{1,i} = (start_index+1):(start_index+num_keypoints{1,i}) ;
        start_index = start_index + num_keypoints{1,i};
    end
    
    % build unique landmark queue and its indices
    queue_landmark = cell2mat(landmarks);
    [queue_landmark_unique,index_to_unique, index_from_unique] = unique(queue_landmark','rows','stable');
    queue_landmark_unique = queue_landmark_unique';
    index_to_unique = index_to_unique';
    index_from_unique = index_from_unique';
    m = size(queue_landmark_unique,2);
    queue_indices_unique = 1:m;
    queue_indices = queue_indices_unique(index_from_unique); % find which landmark index fits to which keypoints
    
    observations = [n ; m];
    hidden_state = zeros(6*n + 3*m,1);
    hidden_state(n*6+1:end,1) = queue_landmark_unique(:);
    
    % fill in the information in observations and state
    for i = 1:n
       % fill in twist
       hidden_state((i-1)*6+1:i*6,1) = HomogMatrix2twist(poses{1,i});
       
       O_i = [num_keypoints{1,i} ; keypoints{1,i}(:) ; queue_indices( ind_state2queue{1,i} )' ]; % ?x1 matrix
       observations = [observations;O_i];       
    end
        
    % runBA
    hidden_state_adjusted = runBA(hidden_state , poses{1,1},poses{1,2} , observations,K,max_iter);
    
    queue_landmarks_adjusted_unique = reshape( hidden_state_adjusted(n*6+1:end) , 3 , m);
    queue_landmarks_adjusted = queue_landmarks_adjusted_unique( : , index_from_unique );
    
    % put adjusted hidden_state back into state history
    for i = 1:n
        state_adjusted = state_history(1,i);
        
        % update Pose and landmarks adn TAU
        HomogMatrix = twist2HomogMatrix( hidden_state_adjusted( (i-1)*6+1 : i*6 , 1 ) );
        state_adjusted.Pose = HomogMatrix(1:3,1:4);
        state_adjusted.X = queue_landmarks_adjusted( : , ind_state2queue{1,i} );
        poses_adjusted{1,i} = HomogMatrix(1:3,1:4);
        adjusted_state_history(1,i) = state_adjusted;
    end    
    
    % Replace all adjusted poses in the newest state
    Tau = state_history(1,end).Tau;
    Tau_adjusted = Tau;
    n_tau = size(Tau,3);
    for j = 1:n_tau
       for i = 1:n
        if isequal(Tau(:,:,j),poses{1,i}) 
            Tau_adjusted(:,:,j) = poses_adjusted{1,i};
        end
       end
    end
    state_history(1,end).Tau = Tau_adjusted;
    
end