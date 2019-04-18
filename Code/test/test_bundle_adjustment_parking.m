clc;
clear all;
close all;


global dataset
dataset = Dataset('parking');
load('state_hist_parking.mat');
%%
state_hist = state_hist_start;
figure(1)
plot_production_temp(state_hist);
title('original')
state_hist_adjusted = bundle_adjustment(state_hist,dataset.K)

figure(2)
plot_production_temp(state_hist_adjusted);
title('adjusted')

%%
diff = norm( state_hist(1,1).Pose - state_hist_adjusted(1,1).Pose ,'fro')