function [] = plotLocalization(T_W_C1, T_W_C2, W_landmarks1, W_landmarks2)    

    % INPUT Homogenious Transformation
    R_W_C1 = T_W_C1(1:3,1:3);
    t_W_C1 = T_W_C1(1:3,4);
    R_W_C2 = T_W_C2(1:3,1:3);
    t_W_C2 = T_W_C2(1:3,4);
 
    figure();
    
    % P is a [4xN] matrix containing the triangulated point cloud (in
    % homogeneous coordinates), given by the function linearTriangulation
    plot3(W_landmarks1(1,:), W_landmarks1(2,:), W_landmarks1(3,:), 'o');
    plot3(W_landmarks2(1,:), W_landmarks2(2,:), W_landmarks2(3,:), 'ro');

    % Display camera pose

    plotCoordinateFrame( R_W_C1 , t_W_C1 , 0.8);
    text(t_W_C1(1)-0.1,t_W_C1(1)-0.1,t_W_C1(1)-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');

    plotCoordinateFrame(R_W_C2,t_W_C2, 0.8);
    text(t_W_C2(1)-0.1, t_W_C2(2)-0.1, t_W_C2(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');

    
    axis equal
    rotate3d on;
    grid;
    
end