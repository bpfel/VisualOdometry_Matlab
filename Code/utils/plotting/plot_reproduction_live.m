function [] = plot_reproduction_live(state_hist, enable_3d, savefile)

if nargin<2
  enable_3d = false;
end

if nargin<3
  savefile = false;
end

global dataset
if savefile && size(state_hist,2) == 2
    mkdir('figures')
    mkdir(['figures/',dataset.name])
end

num_last_states = 20;
num_states = size(state_hist,2);

first_frame = state_hist(1,1).frame;

fig34 = figure(34);
fig34.Visible = true;
% make it fullscreen
fig34.Units = 'normalized';
fig34.OuterPosition = [0 0 1 1];

last_states = [max(1,num_states-num_last_states + 1):num_states];
last_states_ext = [];
if num_last_states <= num_states
    last_states_ext = last_states;
else
    last_states_ext = [0 last_states];
end

n_last_states = min(num_states,num_last_states);
state_curr = state_hist(num_states);

poses = zeros(4,num_states + 1);
number_landmarks = zeros(3,n_last_states);

% Loop over all states
for i = 1:num_states
    poses(1,i+1) = state_hist(i).Pose(1,4); %x
    poses(2,i+1) = state_hist(i).Pose(3,4); %z
    poses(3,i+1) = state_hist(i).Pose(2,4); %y
    poses(4,i+1) = state_hist(i).frame;
end
    
for i = 1:n_last_states
   number_landmarks(1,i) = state_hist(1,last_states(i)).frame;
   number_landmarks(2,i) = size(state_hist(1,last_states(i)).P,2);
   number_landmarks(3,i) = size(state_hist(1,last_states(i)).C,2);
end

%%%%%%%%% First plot: Image of current frame with keypoints and candidates
p1 = subplot(2,5,[1:3]); 
imshow(get_frame(dataset,state_curr.frame));
hold on;
scatter(state_curr.P(1,:),state_curr.P(2,:),'ro');    
scatter(state_curr.C(1,:),state_curr.C(2,:),'bx');
legend({'all keypoints','all candidates'},'Location','ne');
title(['Matched feature in current frame: ' num2str(state_curr.frame)]);
hold off;
    
    %%%%%%%%%%% Second plot: Number of landmarks of last frames
p2 = subplot(2,5,6); 

plot(number_landmarks(1,:), number_landmarks(2,:),'r')
hold on
plot(number_landmarks(1,:), number_landmarks(3,:),'b')
hold off
xlim([max(first_frame,state_curr.frame-19)-0.00001 state_curr.frame+0.00001])
legend({'keypoints','candidates'},'Location','se');
title(['Number of keypoints and candidates in last frames']);    

    %%%%%%%%%% Third plot: Full trajectory
p3 = subplot(2,5,7);

plot(poses(1,1:num_states+1),poses(2,1:num_states+1),'b-x')
axis equal 
title(['Full Trajectory'])
    
%%%%%%%%% Fourth Plot: Trajectory of last frames and landmarks
p4 = subplot(2,5,[4 5 9 10]);

if enable_3d
    plot3(poses(1,last_states_ext+1),poses(2,last_states_ext+1),-poses(3,last_states_ext+1),'b-x','MarkerSize',3)
else
    plot(poses(1,last_states_ext+1),poses(2,last_states_ext+1),'b-x','MarkerSize',3)
end
hold on
axis equal 
title(['Local Trajectory and Landmarks'])

if dataset.has_ground_truth

    %Add groundtrouth for last frames
    first_frame_index = [];
    first_pose = [];
       last_frame_index = state_curr.frame + 1 - dataset.first_frame;
    if num_last_states <= num_states
        first_frame_index = state_hist(last_states(1)).frame + 1 - dataset.first_frame;
        first_pose = state_hist(num_states-num_last_states + 1).Pose;
    else
        first_frame_index = 1;
        first_pose = [eye(3), zeros(3,1)];
    end

    T_WC_gt = dataset.ground_truth(:,:,first_frame_index:last_frame_index);

    poses_gt = zeros(3,last_frame_index-first_frame_index + 1);

        %Make sure,the groundtruth poses got the same length
        %Assume, that the distance of the first frame to last frame of
        %'num_last_states' is the same
    norm_tra = norm(poses(1:3,last_states_ext(1)+1) - poses(1:3,num_states+1));
    T_WC_gt_last = invert_homo_trans(T_WC_gt(:,:,1))*[T_WC_gt(:,:,end);0 0 0 1];
    norm_gt = norm(T_WC_gt_last([1 2 3],4));

        %Transform groundtruth, such that it aligns with the coordinate
        %frame of the first pose of the 'num_last_states' poses
    for j = 1:size(poses_gt,2)
        T_WC_gt1 = invert_homo_trans(T_WC_gt(:,:,1))*[T_WC_gt(:,:,j);0 0 0 1];

        T_WC_gt1(:,4) = T_WC_gt1(:,4) * norm_tra / norm_gt;
        T_WC_gt2 = first_pose * [T_WC_gt1; [ 0 0 0 1]];

        poses_gt(1,j) = T_WC_gt2(1,4);
        poses_gt(2,j) = T_WC_gt2(3,4);
        poses_gt(3,j) = T_WC_gt2(2,4);

    end

    if enable_3d
        plot3(poses_gt(1,:),poses_gt(2,:),-poses_gt(3,:),'k-x','MarkerSize',3)
        scatter3(state_curr.X(1,:),state_curr.X(3,:),-state_curr.X(2,:),20, 'filled','r')
        trans_mat = [1 0 0 0; 0 0 1 0; 0 -1 0 0];
        pose3d = trans_mat * [state_curr.Pose; 0 0 0 1];
        plotCoordinateFrame(pose3d(:,1:3), pose3d(:,4), 4);
    else
        plot(poses_gt(1,:),poses_gt(2,:),'k-x','MarkerSize',3)
        scatter(state_curr.X(1,:),state_curr.X(3,:),20, 'filled','r')
    end
    legend('Trajectory', 'Groundtruth', 'Landmarks')
else
    if enable_3d
        scatter3(state_curr.X(1,:),state_curr.X(3,:),-state_curr.X(2,:),20, 'filled','r')
        trans_mat = [1 0 0 0; 0 0 1 0; 0 -1 0 0];
        pose3d = trans_mat * [state_curr.Pose; 0 0 0 1];
        plotCoordinateFrame(pose3d(:,1:3), pose3d(:,4), 4);
    else
        scatter(state_curr.X(1,:),state_curr.X(3,:),20, 'filled','r')
    end
    legend('Trajectory','Landmarks')
end
hold off

    %%%%%%%%% Fifth Plot: Height
p5 = subplot(2,5,8);

plot(poses(4,1:num_states+1),-poses(3,1:num_states+1),'b')
xlim([max(first_frame,state_curr.frame-19)-0.00001 state_curr.frame+0.00001]) 
title(['Height of last frames'])
    
pause(0.01);
if savefile
    name = strcat('figures/', dataset.name, '/', dataset.name, '_', num2str(state_curr.frame,'%04.f'));
    saveas(fig34, name, 'png')
end    
    
end

