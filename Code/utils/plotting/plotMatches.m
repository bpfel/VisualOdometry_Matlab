function [] = plotMatches(img1,img2, matched_features_img1, matched_features_img2)

    figure;
    subplot(2,1,1);
    imshow(img1);
    hold on;
    scatter(matched_features_img1(1,:),matched_features_img1(2,:));    
    
    subplot(2,1,2);
    imshow(img2);
    hold on;
    scatter(matched_features_img1(1,:),matched_features_img1(2,:),'o');    
    scatter(matched_features_img2(1,:),matched_features_img2(2,:),'x');
    %define the lines connecting old keypoints to new keypoints
    [x_track,y_track] = connecting_lines(...
        matched_features_img1, matched_features_img2);
    plot(x_track,y_track,'r','LineWidth',2);
    legend({'Features_img1','Features_img2','tracks'})
    
end