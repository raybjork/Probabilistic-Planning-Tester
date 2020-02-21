% This script is for generating report ready graphics

clear all; close all;

% Add that folder plus all subfolders to the path.
folder = fileparts(which(mfilename)); 
addpath(genpath(folder));

warning('off')

set(groot,'defaulttextinterpreter','latex');
set(groot, 'DefaultLegendInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');  

colors = {'k', 'b', 'g', 'r'};

num_algs = 4;
n = 10;

xmax = [2000, 3000, 1300, 2000];

for m = 2:5
    if ~exist(['map' num2str(m) '2.mat'])
        continue
    end
    load(['map' num2str(m) '2.mat']);
    mins = Inf(1, num_algs);
       
    for a = 1:num_algs
        for i = 1:n
            mins(a) = min(length(T{a, i}), mins(a));
        end

        for i = 1:10
            temp1 = T{a, i};
            temp2 = J{a, i};
            temp3 = K{a, i};
            ts_i(:,i)=temp1(1:mins(a));
            js_i(:,i)=temp2(1:mins(a));
            ks_i(:,i)=temp3(1:mins(a))-temp3(1); 
        end
        
        figure(2*(m - 1) - 1); hold on;
            xlim([0 xmax(m-1)]);
            shadedErrorBar(ks_i(:,1),js_i',{@mean,@std}, 'lineprops', {colors{a},'LineWidth',5,'LineStyle',':'}); 
            l=legend('$RRT^*$', 'Informed $RRT^*$', '$RRT^*$-Smart', 'Local $RRT^*$');
            %xlim([0 2000])
            set(gca,'fontsize', 14);
            set(gca,'TickLabelInterpreter','latex')
            title('Trajectory Cost of RRT* Variants')
            xlabel('Sampled Nodes')
            ylabel('Trajectory Cost')
        hold off;
        
        figure(2*(m - 1)); hold on;
            xlim([0 xmax(m-1)]);
            shadedErrorBar(ks_i(:,1),ts_i',{@mean,@std}, 'lineprops', {colors{a},'LineWidth',5,'LineStyle',':'}); 
            l=legend('$RRT^*$', 'Informed $RRT^*$', '$RRT^*$-Smart', 'Local $RRT^*$');
            set(l, 'Location', 'northwest');
            title('Computational Efficiency of $RRT*$ Variants')
            set(gca,'fontsize', 14);
            set(gca,'TickLabelInterpreter','latex')
            xlabel('Sampled Nodes')
            ylabel('Program Runtime(s)') 
        hold off;
            
        clear ts_i js_i ks_i
    end
end

for a = 1:8
    figure(a);
    saveas(gca, ['fig' num2str(a)], 'png');
end