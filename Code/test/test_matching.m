clc;
clear all;
close all;

dataset = Dataset('kitti'); % options: 'kitti', 'malaga', 'parking'

%%

first_frame_index = dataset.first_frame;

%1. Get frames
[keyframe_1] = get_frame(dataset ,first_frame_index);
[keyframe_2] = get_frame(dataset ,first_frame_index+1);

[keypoints_keyframe_1] = some_keypoint_finding_algorithm(keyframe_1);

threshold = 2;

%Track keypoints across frames
[validity_KLT_points, all_KLT_points, all_candidates]=...
    matching(keyframe_1,keyframe_2,keypoints_keyframe_1,threshold,0.997);
