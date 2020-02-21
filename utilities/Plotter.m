% Class to keep everything about the plot organized
classdef Plotter < matlab.mixin.Copyable
    properties
        lines
        path
        sampler
    end
    
    methods
        function self = Plotter(N)
            self.lines = zeros(N + 2, 1);
        end
    
        function self = plot_edges(self, q1, q2, i)
            if size(q1,1) == 1
                if i ~= -1
                    self.lines(i) = plot([q1(1); q2(1)],[q1(2); q2(2)], ...
                        'LineWidth', 1, 'Color', 'b');
                else
                    q1 = [q1; q1];
                    q2 = [q2; q2];
                end
            end
            if size(q1,1) > 1
                self.path = zeros(size(q1, 1), 1);
                for j = 1:size(q1,1)
                    self.path(j) = plot([q1(j,1); q2(j,1)],[q1(j,2); q2(j,2)], ...
                        'LineWidth', 3, 'Color', 'g');
                end
            end
        end
        
        function self = ellipse(self, ra, rb, ang, x0, y0, C)
            % number of points to draw ellipse
            Nb = 300;
            
            % get number of ellipses
            if length(ra)~=length(x0)
                maxk=length(ra)*length(x0);
            else
                maxk=length(ra);
            end
            
            % draw ellipses
            for k=1:maxk
                if length(x0)==1
                    xpos=x0;
                    ypos=y0;
                    radm=ra(k);
                    radn=rb(k);
                    if length(ang)==1
                        an=ang;
                    else
                        an=ang(k);
                    end
                elseif length(ra)==1
                    xpos=x0(k);
                    ypos=y0(k);
                    radm=ra;
                    radn=rb;
                    an=ang;
                elseif length(x0)==length(ra)
                    xpos=x0(k);
                    ypos=y0(k);
                    radm=ra(k);
                    radn=rb(k);
                    an=ang(k);
                else
                    an=ang(fix((k-1)/size(x0,1))+1);
                    xpos=x0(rem(k-1,size(x0,1))+1);
                    ypos=y0(rem(k-1,size(y0,1))+1);
                end
                co=cos(an);
                si=sin(an);
                the=linspace(0,2*pi,Nb(rem(k-1,size(Nb,1))+1,:)+1);
                self.sampler = line(radm*cos(the)*co-si*radn*sin(the)+xpos,radm*cos(the)*si+co*radn*sin(the)+ypos);
                set(self.sampler,'color', 'c', 'Linewidth', 3);
            end
        end
        
        function self = delete_edges(self, i)
            %delete(self.lines(i));
            set(self.lines(i), 'visible', 'off');
        end
        
        function self = clear_path(self)
            delete(self.path);
        end
        
        function self = clear_sampler(self)
            delete(self.sampler);
        end
    end
       
    methods (Static)
        function setup(map)
            figure(1); clf; hold on;
            set(gca,'xtick',[])
            set(gca,'visible','off')
            for i = 1:size(map.obstacles, 1)
                rectangle('Position', map.obstacles(i,:), ...
                    'FaceColor','r','EdgeColor','r');
            end
            if isfield(map, 'goal_r')
                rectangle('Position', [map.goal(1) - map.goal_r, ...
                    map.goal(2) - map.goal_r,  2 * map.goal_r, ...
                    2 * map.goal_r], 'Curvature', [1 1], 'FaceColor', ...
                    [.7 .7 .7], 'EdgeColor', [.7 .7 .7]);
            end
            plot(map.start(1), map.start(2), ...
                'ko', 'MarkerSize',10, 'MarkerFaceColor','k');
            plot(map.goal(1), map.goal(2), ...
                'ko', 'MarkerSize',10, 'MarkerFaceColor','k');
            axis(map.bounds)
        end
        
        function collisions(x)
            for i = 1 : size(x, 1)
                plot(x(i, 1), x(i, 2), 'rx', 'MarkerFaceColor','r');
            end
        end
        
        function refresh()
            drawnow;
        end
    end
end