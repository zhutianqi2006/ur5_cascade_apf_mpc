h_fig = figure('Name', 'sim_plot_distance_all');
plot(i_sequence, min_distance-0.05,'LineWidth',1.5,'LineStyle','-');
hold on;
% plot constrain
down_constrain=0.1*ones(length(i_sequence));
plot(i_sequence,down_constrain(1,:),'LineWidth',1.5,'LineStyle','--','Color',[0,0,0]);
hold on;
grid on;
set(gca,'YTick',0:0.6/6:0.6);
legend({'APF-Double-MPC'},'FontSize',9,'Interpreter','latex','NumColumns',1);
set(gca,'XLim',[0 20],'FontSize',7);
set(gca,'YLim',[0 0.60],'FontSize',7);
xlabel('Time(s)','FontSize',9,'Interpreter','latex');
ylabel('$\Lambda$(m)','FontSize',9,'Interpreter','latex');
x_range = 20;
y_range = 0.61;
% aspect ratio
daspect([x_range*1 y_range*2.5 1])
hold on;
saveas(h_fig, h_fig.Name, 'fig') 
saveas(h_fig, h_fig.Name, 'svg')      