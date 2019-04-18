% This is a class that handles the interaction with the different datasets
% Author: Kamil Ritz ( kritz@ethz.ch )
classdef Dataset
    properties
        BOOTSTRAP_FRAME_DISTANCE = 2;    % distance between bootstrapes frames
        % 1 means that they are consecutive, 2 is
        % recommended by the TAs
        next_frame_index % index of the next image that will be returned by getNextFrame()
        has_ground_truth
        last_frame
        first_frame
        ground_truth
        bootstrap_frames
        path
        name
        K            % camera intrinsics
        cameraParams
        image_struct % not the image itself, only stores information about them
        options
        v
    end
    methods
        function obj = Dataset(name)  %constructor
            obj.name = name;
            
            %load options
            all_options = jsondecode(fileread('options.json'));
            
            if isequal(name,'kitti')
                obj.path = '../Datasets/kitti';
                obj.first_frame = 0;
                obj.has_ground_truth = true;
                ground_truth = load([obj.path '/poses/00.txt']);
                obj.ground_truth = permute(reshape(ground_truth',4,3,size(ground_truth,1)),[2 1 3]);
                obj.last_frame = 4540;
                obj.K = [7.188560000000e+02 0 6.071928000000e+02
                    0 7.188560000000e+02 1.852157000000e+02
                    0 0 1];
                obj.image_struct = [];
                obj.options = all_options.kitti;
                obj = obj.process_options();
            elseif isequal(name,'malaga')
                obj.path = '../Datasets/malaga-urban-dataset-extract-07';
                obj.first_frame = 1;
                obj.has_ground_truth = false;
                images = dir([obj.path ...
                    '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
                obj.image_struct = images(3:2:end);
                obj.last_frame = length(obj.image_struct);
                obj.K = [621.18428 0 404.0076
                    0 621.18428 309.05989
                    0 0 1];
                
                obj.options = all_options.malaga;
                obj = obj.process_options();
                
            elseif isequal(name,'parking')
                obj.path = '../Datasets/parking';
                obj.first_frame = 0;
                obj.has_ground_truth = true;
                obj.last_frame = 598;
                obj.K = load([obj.path '/K.txt']);
                ground_truth = load([obj.path '/poses.txt']);
                obj.ground_truth = permute(reshape(ground_truth',4,3,size(ground_truth,1)),[2 1 3]);
                obj.image_struct = [];
                
                obj.options = all_options.parking;
                obj = obj.process_options();
                
            elseif isequal(name,'own_bike')
                obj.path = '../Datasets/self_generated/';
                obj.v = VideoReader([obj.path,'bikeride_zurich.mp4']);
                obj.first_frame = 1;
                obj.has_ground_truth = false;
                obj.last_frame = floor(obj.v.Duration*obj.v.FrameRate);
                calibs = load([obj.path '/camera_calibration/calibrationSession.mat']);
                obj.K = calibs.calibrationSession.CameraParameters.IntrinsicMatrix';
                obj.cameraParams = calibs.calibrationSession.CameraParameters;
                obj.options = all_options.own_bike;
                obj = obj.process_options();
                
            elseif isequal(name,'own_ethz')
                obj.path = '../Datasets/self_generated/';
                obj.v = VideoReader([obj.path,'walk_ethz.mp4']);
                obj.first_frame = 1;
                obj.has_ground_truth = false;
                obj.last_frame = floor(obj.v.Duration*obj.v.FrameRate);
                calibs = load([obj.path '/camera_calibration/calibrationSession.mat']);
                obj.K = calibs.calibrationSession.CameraParameters.IntrinsicMatrix';
                obj.cameraParams = calibs.calibrationSession.CameraParameters;
                obj.options = all_options.own_ethz;
                obj = obj.process_options();
                
            elseif isequal(name,'stairs_rpg')
                obj.path = '../Datasets/self_generated/';
                obj.v = VideoReader([obj.path,'stairs_rpg.mp4']);
                obj.first_frame = 1;
                obj.has_ground_truth = false;
                obj.last_frame = floor(obj.v.Duration*obj.v.FrameRate);
                calibs = load([obj.path '/camera_calibration/calibrationSession.mat']);
                obj.K = calibs.calibrationSession.CameraParameters.IntrinsicMatrix';
                obj.cameraParams = calibs.calibrationSession.CameraParameters;
                obj.options = all_options.own_ethz;
                obj = obj.process_options();
                
            elseif isequal(name,'entrance_rpg')
                obj.path = '../Datasets/self_generated/';
                obj.v = VideoReader([obj.path,'entrance_rpg.mp4']);
                obj.first_frame = 1;
                obj.has_ground_truth = false;
                obj.last_frame = floor(obj.v.Duration*obj.v.FrameRate);
                calibs = load([obj.path '/camera_calibration/calibrationSession.mat']);
                obj.K = calibs.calibrationSession.CameraParameters.IntrinsicMatrix';
                obj.cameraParams = calibs.calibrationSession.CameraParameters;
                obj.options = all_options.own_ethz;
                obj = obj.process_options();
                
            elseif isequal(name,'bike_straight')
                obj.path = '../Datasets/self_generated/';
                obj.v = VideoReader([obj.path,'bike_straight.mp4']);
                obj.first_frame = 1;
                obj.has_ground_truth = false;
                obj.last_frame = floor(obj.v.Duration*obj.v.FrameRate);
                calibs = load([obj.path '/GOPRO_1440_Linear/GOPRO_1440_linear_12_14.mat']);
                obj.K = calibs.GOPRO_1980_rec1440_linear.IntrinsicMatrix';
                obj.cameraParams = calibs.GOPRO_1980_rec1440_linear;
                obj.options = all_options.bike_straight;
                obj = obj.process_options();            
            elseif isequal(name,'fpv1')
                obj.path = '../Datasets/self_generated/';
                obj.v = VideoReader([obj.path,'fpv1.mp4']);
                obj.first_frame = 1;
                obj.has_ground_truth = false;
                obj.last_frame = floor(obj.v.Duration*obj.v.FrameRate);
                calibs = load([obj.path '/GOPRO_1440_Linear/GOPRO_1440_linear_12_14.mat']);
                K = calibs.GOPRO_1980_rec1440_linear.IntrinsicMatrix';
                K(1:2,:) = K(1:2,:)/2;
                obj.K = K;
                obj.cameraParams = calibs.GOPRO_1980_rec1440_linear;
                obj.options = all_options.fpv1;
                obj = obj.process_options();                  
             elseif isequal(name,'fpv2')
                obj.path = '../Datasets/self_generated/';
                obj.v = VideoReader([obj.path,'fpv2.mp4']);
                obj.first_frame = 1;
                obj.has_ground_truth = false;
                obj.last_frame = floor(obj.v.Duration*obj.v.FrameRate);
                calibs = load([obj.path '/GOPRO_1440_Linear/GOPRO_1440_linear_12_14.mat']);
                K = calibs.GOPRO_1980_rec1440_linear.IntrinsicMatrix';
                K(1:2,:) = K(1:2,:)/2;
                obj.K = K;
                obj.cameraParams = calibs.GOPRO_1980_rec1440_linear;
                obj.options = all_options.fpv2;
                obj = obj.process_options();    
            elseif isequal(name,'fpv3')
                obj.path = '../Datasets/self_generated/';
                obj.v = VideoReader([obj.path,'fpv3.mp4']);
                obj.first_frame = 1;
                obj.has_ground_truth = false;
                obj.last_frame = floor(obj.v.Duration*obj.v.FrameRate);
                calibs = load([obj.path '/GOPRO_1440_Linear/GOPRO_1440_linear_12_14.mat']);
                K = calibs.GOPRO_1980_rec1440_linear.IntrinsicMatrix';
                K(1:2,:) = K(1:2,:)/2;
                obj.K = K;
                obj.cameraParams = calibs.GOPRO_1980_rec1440_linear;
                obj.options = all_options.fpv3;
                obj = obj.process_options();   
            else
                error(['No Dataset with this name found'...
                    ' Possible names: kitti, malaga, parking']);
            end
            
            obj.bootstrap_frames = [obj.first_frame  obj.first_frame+obj.BOOTSTRAP_FRAME_DISTANCE];
            obj.next_frame_index = obj.bootstrap_frames(end) + 1;
            
        end
        
        function [obj,img,is_last_frame] = get_next_frame(obj)
            if( obj.next_frame_index <= obj.last_frame )
                
                if isequal(obj.name, 'kitti')
                    img = imread([obj.path '/00/image_0/' ...
                        sprintf('%06d.png',obj.next_frame_index)]);
                    
                elseif isequal(obj.name, 'malaga')
                    img = rgb2gray(imread([obj.path ...
                        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
                        obj.image_struct(obj.next_frame_index).name]));
                    
                elseif isequal(obj.name, 'parking')
                    img = rgb2gray(imread([obj.path ...
                        sprintf('/images/img_%05d.png',obj.next_frame_index)]));
                    
                elseif isequal(obj.name, 'own_bike') | isequal(obj.name, 'own_ethz') | isequal(obj.name, 'stairs_rpg') | isequal(obj.name, 'entrance_rpg')
                    img = obj.get_frame(obj.next_frame_index);
                    
                else
                    error('Something is gone wrong')
                end
                
                % Check if it is the last frame
                if( obj.next_frame_index == obj.last_frame )
                    is_last_frame = true;
                    obj.next_frame_index = NaN;
                else
                    is_last_frame = false;
                    obj.next_frame_index = obj.next_frame_index + 1;
                end
            else
                error('You want to get an frame after the last frame');
            end
        end
        
        function [img1, img2] = get_bootstrap_frame(obj)
            if isequal(obj.name, 'kitti')
                img1 = imread([obj.path '/00/image_0/' ...
                    sprintf('%06d.png',obj.bootstrap_frames(1))]);
                img2 = imread([obj.path '/00/image_0/' ...
                    sprintf('%06d.png',obj.bootstrap_frames(2))]);
            elseif isequal(obj.name, 'malaga')
                img1 = rgb2gray(imread([obj.path ...
                    '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
                    obj.image_struct(obj.bootstrap_frames(1)).name]));
                img2 = rgb2gray(imread([obj.path ...
                    '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
                    obj.image_struct(obj.bootstrap_frames(2)).name]));
            elseif isequal(obj.name, 'parking')
                img1 = rgb2gray(imread([obj.path ...
                    sprintf('/images/img_%05d.png',obj.bootstrap_frames(1))]));
                img2 = rgb2gray(imread([obj.path ...
                    sprintf('/images/img_%05d.png',obj.bootstrap_frames(2))]));
            elseif isequal(obj.name, 'own_bike') | isequal(obj.name, 'own_ethz') | isequal(obj.name, 'bike_straight')| isequal(obj.name, 'stairs_rpg') | isequal(obj.name, 'entrance_rpg')
                img1 = obj.get_frame(obj.bootstrap_frames(1));
                img2 = obj.get_frame(obj.bootstrap_frames(2));
            else
                error('Something is gone wrong')
            end
        end
        
        function img = get_frame(obj, frame_index)
            if( frame_index <= obj.last_frame )
                
                if isequal(obj.name, 'kitti')
                    img = imread([obj.path '/00/image_0/' ...
                        sprintf('%06d.png',frame_index)]);
                elseif isequal(obj.name, 'malaga')
                    img = rgb2gray(imread([obj.path ...
                        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
                        obj.image_struct(frame_index).name]));
                elseif isequal(obj.name, 'parking')
                    img = rgb2gray(imread([obj.path ...
                        sprintf('/images/img_%05d.png',frame_index)]));
                elseif isequal(obj.name, 'own_bike') | isequal(obj.name, 'own_ethz')
                    img = imcrop(undistortImage(rgb2gray(read(obj.v,frame_index)),obj.cameraParams),[0 14 1920 1049]);
                elseif isequal(obj.name, 'bike_straight') 
                    img = (undistortImage((rgb2gray(read(obj.v,frame_index))),obj.cameraParams));     
                elseif isequal(obj.name, 'fpv1')
                    img_temp = imcrop(rgb2gray(read(obj.v,frame_index)),[0 180 1920 1079]);
                    img = imresize( undistortImage(img_temp,obj.cameraParams) , 0.5 );     
                elseif isequal(obj.name, 'fpv2')
                    img_temp = imcrop(rgb2gray(read(obj.v,frame_index)),[0 180 1920 1079]);
                    img = imresize( undistortImage(img_temp,obj.cameraParams) , 0.5 );     
                elseif isequal(obj.name, 'fpv3')
                    img_temp = imcrop(rgb2gray(read(obj.v,frame_index)),[0 180 1920 1079]);
                    img = imresize( undistortImage(img_temp,obj.cameraParams) , 0.5 );     
                else
                    error('Something is gone wrong')
                end
            else
                error('You want to get an frame after the last frame');
            end
        end
        
        function [obj] = set_bootstrap_distance(obj, distance)
            if distance < obj.last_frame - obj.first_frame
                obj.BOOTSTRAP_FRAME_DISTANCE =  distance;
                obj.bootstrap_frames = [obj.first_frame  obj.first_frame+obj.BOOTSTRAP_FRAME_DISTANCE];
                obj.next_frame_index = obj.bootstrap_frames(end) + 1;
            else
                error('bootstrap distance is too big');
            end
        end
        
        function obj = process_options(obj)
            obj.options.localized_harris = struct2cell(obj.options.localized_harris);
            obj.options.matching = struct2cell(obj.options.matching);
            obj.options.localization = struct2cell(obj.options.localization);
        end
    end
    
end