clear all;
close all;
clc;
% creates dataset instance
dataset = Dataset('kitti'); % valid arguments are:'kitti', 'malaga', 'parking'
% set bootstrap distance, distance 1 means that it uses first image of
% dataset and the consecutive one;
dataset = set_bootstrap_distance(dataset,2); %if you use this, then only directly after constructor call
% get bootstrap image: the first one and the "first+distance" one
[img1,img2] = get_bootstrap_frame(dataset);

figure
subplot(2,1,1)
imshow(img1)
subplot(2,1,2)
imshow(img2)

% get a frame with a certain index, here 10
img10 = get_frame(dataset, 10);
figure
imshow(img10)

% this methods can be used to get always the next frame (without explicit
% indexing) , starting from the image after the bootstrap image.
% returns is_last_frame + true when handing back the last frame
[dataset, img, is_last_frame] = get_next_frame(dataset);
figure
imshow(img)
counter = dataset.next_frame_index;

while ~is_last_frame
    fprintf('\n\nProcessing frame %d\n=====================\n', counter);
    
    [dataset, image, is_last_frame] = get_next_frame(dataset);

    counter = dataset.next_frame_index;
    prev_image = image;
end
