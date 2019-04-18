function is_keeper = sparsify_keypoints(keypoints)
% remove 2D points that are to close too each other

    treshold = 10;
    n = size(keypoints,2);

    for i = 1:n
        if keypoints(1,i) ~= -1
            D = pdist2(keypoints',keypoints(:,i)','euclidean')';
            is_to_close = D < treshold;
            is_to_close(i) = 0;

            keypoints(:,is_to_close) = -treshold;
        end
    end
    is_keeper = keypoints(1,:) >= 0;
end 