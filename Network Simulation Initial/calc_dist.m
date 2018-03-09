function [dist] = calc_dist(rob, time)
%CALC_DIST calculates the distance between two objects
%   Given two objects odometry this function will return the relative
%   distances the robots are apart

% r1.X = rob1.X;
% r2.X = rob2.X;
% 
% r1.y = rob1.Y;
% r2.y = rob2.Y;
% 
% dist.x = sqrt((r1.x.^2 - r2.x.^2));
% dist.y = sqrt((r1.y.^2 - r2.y.^2));

r = rob;
n = length(rob);
t = length(time);
for i =1:n
    for j=2:n
        for time=1:t
            x_dist(i,j,t) = norm(r(i).X(t) -r(j).X(t));
            y_dist(i,j,t) = norm(r(i).Y(t) -r(j).Y(t)); 
        end
        dist(i,j,:) = norm(x_dist(i,j,:) + y_dist(i,j,:));
    end
end

end

