function [] = plotKeypoints(img,keypoints)
%PLOTKEYPOINTS Plot an image and display keypoints
%   img MxN matrix
%   keypoints 2xK matrix
imshow(img);
hold on
scatter(keypoints(1,:),keypoints(2,:));
end

