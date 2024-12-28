h_fig = figure('Name', 'sim_plot_xyz_error_tra');
plot(i_sequence, ex_sequence,'LineWidth',2.2,'LineStyle','-');
hold on;
plot(i_sequence, ey_sequence,'LineWidth',2.2,'LineStyle','-');
hold on;
plot(i_sequence, ez_sequence,'LineWidth',2.2,'LineStyle','-');
hold on;
grid on;
set(gca,'XLim',[0 20],'FontSize',14);
set(gca,'YLim',[-0.601 0.601],'FontSize',14);
legend({'$e_x$','$e_y$','$e_z$'},'FontSize',18,'NumColumns',1,'Interpreter','latex');
xlabel('Time(s)','FontSize',18,'Interpreter','latex');
ylabel('Error(m)','FontSize',18,'Interpreter','latex');
x_range = 20;
y_range = 1.2;
% aspect ratio
daspect([x_range*3 y_range*5 1])
hold on;
saveas(h_fig, h_fig.Name, 'fig')
saveas(h_fig, h_fig.Name, 'svg')