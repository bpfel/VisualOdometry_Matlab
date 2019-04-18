function [x,y] = connecting_lines(start_points,end_points)
%CONNECTING_LINES Reshapes two point vectors such that lines between the
%two can be plotted directly with plot(x,y)
%   Vectors have to be handed as 2xN!

x = [start_points(1,:); end_points(1,:)];
y = [start_points(2,:); end_points(2,:)];
end

