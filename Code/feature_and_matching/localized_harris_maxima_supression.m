function [keypoint_collection] = localized_harris_maxima_supression(img,rows,cols,radius,npoints,minQuality,FilterSize,border_treshold)
%LOCALIZED_HARRIS Find keypoints in image by dividing it into multiple
%rectangular regions of interest
%   INPUT
%   rows    -   number of rectangular regions in vertical direction
%   cols    -   number of rectangular regions in horizontal direction
%   radius  -   percentage of included keypoints per rectangle
%   npoints -   number of points per region
%   OUTPUT
%   keypoints - 2xk matrix

%plotting
histogram_equaliztion = true;
plotting = false;
warn = false;
%% Division into multiple regions of interest
[M,N] = size(img);
% take floor, lost pixels are neglected
column_width = floor(N/cols);
row_width = floor(M/rows);

% Define regions of interest
col_indices = 1:column_width:N;
row_indices = 1:row_width:M;

if histogram_equaliztion
   img = histeq(img); 
end

%Check out region by region
if plotting
    imshow(img); hold on; % plotting
end

keypoint_collection = [];
for ic = 1:cols
    for ir = 1:rows
        harris_struct = detectHarrisFeatures(img,'ROI',...
            [col_indices(ic) row_indices(ir) column_width row_width],...
            'MinQuality',minQuality,'FilterSize',FilterSize);
        % maxima supression
        keypoints = [];
        flag = false;
        for ip = 1:npoints
            %find maximum only in unselected and undeleted points
            [~,ind_max] = max(harris_struct.Metric);
            %include the newly found maximum if not to close to image
            %border
            candidate = harris_struct.Location(ind_max,:)';
            if isempty( candidate(1,:) )
                break;
            end
            if candidate(1) > border_treshold & ...
               candidate(1) < N-border_treshold & ...
               candidate(2) > border_treshold & ...
               candidate(2) < M-border_treshold
           
                keypoints = [keypoints candidate];
            end
            %find close points
            distances = pdist2(harris_struct.Location(ind_max,:)...
                ,harris_struct.Location);
            ind_suppressed = any(distances<radius,1).';
            %check whether deleting more points depletes the reservoir
            if (numel(harris_struct.Metric) - sum(ind_suppressed)) < npoints && ~flag
                if warn
                    warning('Radius too large or not enough points available!');
                end
                flag = true;
            end
            %exclude the newly suppressed points
            harris_struct(ind_suppressed) = [];
            
        end
        
        %1+(ic-1)*(ir-1)+(ir-1): index mapping to have a vector instead of
        %a matrix of keypoint clouds
        keypoint_collection = [keypoint_collection keypoints];
        
        if plotting
            scatter(keypoints(1,:),keypoints(2,:)); %plotting
            hold on;
        end
    end
end
end


