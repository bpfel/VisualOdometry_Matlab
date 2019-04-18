function [validity_KLT_points, all_KLT_points, all_candidates]=...
    matching(img_database,img_query,keypoints_database,maxBidirectionalError,threshold,score_limit,block_size)
%MATCHING tracks the given keypoints from the database image to the query
%image using KLT and generates new candidates based on a HARRIS search in
%the query image, which then go through a closeness selection based on a
%given threshold
%   INPUT:
%   img_database, img_query - MxN images
%   keypoints_database      - 2xk1 matrix
%   threshold               - scalar
%   OUTPUT:
%   validity_KLT_points     - bolean vector k1x1
%   all_KLT_points          - vector 2xk1
%   all candidates          - vector 2xk2 where k2 is the number of
%   candidates

%include dataset
global dataset;
histogram_equalization = true;
if histogram_equalization
   img_database = histeq(img_database); 
   img_query = histeq(img_query); 
end

%Create a KLT tracker 
%Why recreate every time? -> The KLT algorithm only establishes 1-1
%correspondences between 2 frames and does not improve when running in the
%same instance over multiple frames. At least if matlab implemented the
%standard algorithm (the one I found - may that be standard or not)
%http://web.yonsei.ac.kr/jksuhr/articles/Kanade-Lucas-Tomasi%20Tracker.pdf
tracker = vision.PointTracker('MaxBidirectionalError',maxBidirectionalError,'BlockSize',[block_size,block_size]);
%Initialize the tracker with the database keypoints and the database image
initialize(tracker,keypoints_database',img_database);
%Find the corresponding features in the query_image and return the query
%keypoints, their validity and their score
[all_KLT_points,validity_KLT_points,score] = tracker(img_query);
validity_KLT_points = (validity_KLT_points&(score>score_limit))';
all_KLT_points = all_KLT_points';

%% Finding keypoint candidates

%Generate possible new keypoints
keypoint_candidates = localized_harris_maxima_supression...
    (img_query,dataset.options.localized_harris{:},ceil(dataset.options.matching{4}/2));

%Calculate distances to existing valid keypoints
distances = pdist2(all_KLT_points(:,validity_KLT_points)',keypoint_candidates');

%Find indices of candidates close to existing keypoints
%This is done by thresholding the distance matrix and then checking
%whether the candidates were close to any other existing keypoint
indices_noncandidates = any(distances<threshold,1);

all_candidates = keypoint_candidates(:,~indices_noncandidates);

%% Plotting
% figure
% imshow(img_query);
% hold on;
% scatter(all_KLT_points(1,validity_KLT_points),all_KLT_points(2,validity_KLT_points));
% scatter(all_candidates(1,:),all_candidates(2,:),'gx');
% scatter(all_KLT_points(1,~validity_KLT_points),all_KLT_points(2,~validity_KLT_points),'ro');
% scatter(keypoint_candidates(1,indices_noncandidates),...
%     keypoint_candidates(2,indices_noncandidates),'rx');
% 
% legend({['Tracked KLT points, number: ' num2str(size(all_KLT_points(1,validity_KLT_points),2))...
%     ' required score: ' num2str(score_limit)]...
%     ,['Valid candidates, threshold: ', num2str(threshold), ' number: '...
%     num2str(size(all_candidates(1,:),2))],...
%     ['Untracked KLT points, number:', num2str(size(all_KLT_points(1,~validity_KLT_points),2))]...
%     ,['Invalid Keypoints, number: ',...
%     num2str(size(keypoint_candidates(1,indices_noncandidates),2))]});

end

