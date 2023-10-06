function plot3Dpoints(pts, lineStyle, varargin)
    if nargin > 2
        color = varargin(1);
        color = color{1};
    else
        color = [0 0.4470 0.7410];
    end

    plot3(pts.x,pts.y,pts.z,lineStyle,'Color',color,'MarkerSize',5,...
          'MarkerFaceColor',color)
    
    for i = 1:numel(pts.x)
        text(pts.x(i)+0.1,pts.y(i)+0.1,pts.z(i)+0.1, num2str(i))
    end

    grid on
    axis equal
    xlabel('x'); ylabel('y'); zlabel('z');
end