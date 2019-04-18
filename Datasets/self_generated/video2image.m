function [] = video2image(input_path,output_path)
 vid=VideoReader(input_path);
 numFrames = round(vid.Duration*vid.FrameRate);
 n=numFrames;
 for i = 1:10:n
 frames = read(vid,i);
 imwrite(frames,[output_path int2str(i), '.jpg']);
 end


end