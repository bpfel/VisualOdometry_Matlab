# Contributors
--------------

* [GianAndrea Müller](https://github.com/bpfel) - muellegi@student.ethz.ch

* [Kamil Ritz](https://github.com/kamilritz) - kritz@student.ethz.ch

* [Luca Somm]() - somml@student.ethz.ch

* [Christian Sprecher](https://github.com/christiansprecher) - cspreche@student.ethz.ch

# Code of Conduct

1. Feature-oriented commits and descriptive commit messages.

2. Work on your own branch, merge with master when everything complies.

# Setup

**Navigate to the `Code` folder and run the script `setup.m`**. This add the the code folder, subfolders and the Datasets to your Matlab path.
**Make sure that your current Matlab Folder is always the `Code` folder.** Otherwise you get errors with the paths!

# Dataset Class

This class handles the whole interface with the dataset in the Dataset folder. You can use the class like this:

```matlab
% creates dataset instance
dataset = Dataset('kitti'); % valid arguments are:'kitti', 'malaga', 'parking'
% set bootstrap distance, distance 1 means that it uses first image of
% dataset and the consecutive one;
dataset = set_bootstrap_distance(dataset,2); %if you use this, then only directly after constructor call
% get bootstrap image: the first one and the "first+distance" one
[img1,img2] = get_bootstrap_frame(dataset);
% get a frame with a certain index, here 10
img = get_frame(dataset, 10);
% this methods can be used to get always the next frame (without explicit
% indexing) , starting from the image after the bootstrap image.
% returns is_last_frame + true when handing back the last frame
[dataset, img, is_last_frame] = get_next_frame(dataset);

% important class members
dataset.first_frame             % index of first frame
dataset.last_frame              % index of last frame
dataset.bootstrap_frames        % 1x2 indices of bootstrap frames
dataset.BOOTSTRAP_FRAME_DISTANCE% bootstrap distance
dataset.has_ground_truth        % logic value
dataset.ground_truth            % contains ground_truth array
dataset.K                       % camera intrinsic
dataset.next_frame_index        % index of next frame, only used with
                                % function: get_next_frame()
```

# Naming
```
First letter is the frame
W -> world frame
C -> camera frame
p -> camera coordinates

C_Landmarks = Coordinates of Landmarks in camera frame

R_WC = Rotation matrix from camera to world frame
t_WC = Translation vector from camera to world frame
T_WC = [R_WC | t_WC]


Points: Coordinate frame + _ + describing name (unmatched_feature , etc.) + _ + (norm, homo, (without))
Examples:
p_features_matched = (u,v)
p_features_matched_homo  = (u,v,1)
p_features_matched_norm
C_features_matched = (x,y,z)
C_features_matched_homo = (x,y,z,1)

Arrays of points are in the form (2 x N) resp. (3 x N) or (4 x N)

States: State ( + _curr or _prev if needed)
Includes members P, X, C, F, Tau, I

```
