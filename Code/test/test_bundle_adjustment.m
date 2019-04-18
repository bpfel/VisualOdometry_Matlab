close all; clear all; clc;
dataset = Dataset('malaga');
load('test_state_hist.mat');
plot_state_history_map(state_hist);
pause(0.2);
state_hist_adjusted =  bundle_adjustment(state_hist, dataset.K);
plot_state_history_map(state_hist_adjusted);
