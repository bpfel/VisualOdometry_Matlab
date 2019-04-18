function [keypoints] =...
    some_keypoint_finding_algorithm(img);
%Find keypoints in the image
%   keypoints - 2xn matrix

HarrisStruct = detectHarrisFeatures(img);
keypoints = HarrisStruct.Location';


end
