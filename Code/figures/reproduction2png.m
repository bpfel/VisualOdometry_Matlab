global dataset

dataset = Dataset('kitti');
load('state_hist_kitti_BA.mat');
plot_reproduction(state_hist, false, false, true);

dataset = Dataset('malaga');
load('state_hist_malaga_BA.mat');
plot_reproduction(state_hist, false, false, true);

dataset = Dataset('parking');
load('state_hist_parking_BA.mat');
plot_reproduction(state_hist, false, false, true);

dataset = Dataset('bike_straight');
load('state_hist_bike_straight_BA.mat');
plot_reproduction(state_hist, false, false, true);

dataset = Dataset('fpv1');
load('state_hist_fpv1_BA.mat');
plot_reproduction(state_hist, false, true, true);

dataset = Dataset('fpv2');
load('state_hist_fpv2_BA.mat');
plot_reproduction(state_hist, false, true, true);

dataset = Dataset('fpv3');
load('state_hist_fpv3_BA.mat');
plot_reproduction(state_hist, false, true, true);