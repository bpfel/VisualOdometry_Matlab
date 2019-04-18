function [keypoints] = localized_harris(img,rows,cols,quality)
%LOCALIZED_HARRIS Find keypoints in image by dividing it into multiple
%rectangular regions of interest
%   INPUT
%   rows    -   number of rectangular regions in vertical direction
%   cols    -   number of rectangular regions in horizontal direction
%   quality -   percentage of included keypoints per rectangle
%   OUTPUT
%   keypoints - 2xk matrix


%% Division into multiple regions of interest

[M,N] = size(img);
% take floor, lost pixels are neglected
column_width = floor(N/cols);
row_width = floor(M/rows);

% Define regions of interest
col_indices = 1:column_width:N;
row_indices = 1:row_width:M;

%Check out region by region
imshow(img); hold on; % plotting
keypoint_collection = {};
for ic = 1:cols
    for ir = 1:rows
        harris_struct = detectHarrisFeatures(img,'ROI',[col_indices(ic) row_indices(ir) column_width row_width]);
        % selection of best points
        % calculation of the best metric
        max_score = max(harris_struct.Metric);
        % finding of the indices of the 1-quality% best points
        selected_indices = harris_struct.Metric > max_score*quality;
        % Select the Locations of the chosen keypoints
        keypoints = harris_struct.Location(selected_indices,:).';
        
        %1+(ic-1)*(ir-1)+(ir-1): index mapping to have a vector instead of
        %a matrix of keypoint clouds
        keypoint_collection{1+(ic-1)*(ir-1)+(ir-1)} = keypoints;
        scatter(keypoints(1,:),keypoints(2,:)); %plotting
        hold on;
    end
end
end

