% do not use this :D

content = dir;
num_names = 1;
for i=3:size(content,1)
    if strcmp(content(i).name(end-3:end), '.mp4')
        names{num_names} = content(i).name(1:end-4);
        num_names = num_names+1;
    end
end

for i=1:size(names,2)
    mkdir([names{i},'_frames']);
    
    v = VideoReader([names{i},'.mp4']);
    
    num_frames = round(v.Duration*v.FrameRate);
    
    for j=1:num_frames
        frame = read(v,j);
        g_frame = rgb2gray(frame);
        imwrite(g_frame,[[names{i},'_frames/'],num2str(j),'.png'])
        disp(['Frame: ',num2str(j),' of ',num2str(num_frames),' processed!']);
    end
end
