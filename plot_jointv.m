h_fig = figure('Name', 'sim_plot_jointv_tra');
plot(i_sequence,joint1v_sequence,'LineWidth',2.2,'LineStyle','-');
hold on
plot(i_sequence,joint2v_sequence,'LineWidth',2.2,'LineStyle','-');
hold on
plot(i_sequence,joint3v_sequence,'LineWidth',2.2,'LineStyle','-');
hold on
plot(i_sequence,joint4v_sequence,'LineWidth',2.2,'LineStyle','-.');
hold on
plot(i_sequence,joint5v_sequence,'LineWidth',2.2,'LineStyle','-.');
hold on
plot(i_sequence,joint6v_sequence,'LineWidth',2.2,'LineStyle','-.');
hold on
% plot constrain
down_constrain=-0.6*ones(length(i_sequence));
up_constrain=0.6*ones(length(i_sequence));
plot(i_sequence,down_constrain(1,:),'LineWidth',2.2,'LineStyle','--','Color',[0,0,0]);
hold on;
plot(i_sequence,up_constrain(1,:),'LineWidth',2.2,'LineStyle','--','Color',[0,0,0]);
grid on;
set(gca,'YTick',-0.8:1.6/8:0.8);
legend({'$u_1$','$u_2$','$u_3$','$u_4$','$u_5$','$u_6$'},'FontSize',18,'Interpreter','latex','NumColumns',2);
set(gca,'XLim',[0 22],'FontSize',14);
set(gca,'YLim',[-0.8 0.8],'FontSize',14)
xlabel('Time(s)','FontSize',18,'Interpreter','latex');
ylabel('Velocity(rad/s)','FontSize',18,'Interpreter','latex');
x_range = 20;
y_range = 1.6;
% aspect ratio
daspect([x_range*3 y_range*5 1])
hold on;
saveas(h_fig, h_fig.Name, 'fig')
saveas(h_fig, h_fig.Name, 'svg')