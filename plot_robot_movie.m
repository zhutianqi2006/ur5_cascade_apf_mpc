% video setting
myVideo=VideoWriter('movie.avi');
myVideo.FrameRate=25;
open(myVideo);
joint_pos = [0.05;-1.57;1.57;-1.57;-1.57;0];
aMovObs = collisionSphere(0.10);
bMovObs = collisionSphere(0.10);
aMovObs.Pose = trvec2tform([0.3 0.1 0.3]);
bMovObs.Pose = trvec2tform([0.3 0.4 0.3]);
aobs_stop_flag = 0;
bobs_stop_flag = 0;
matlab_UR5 = loadrobot("universalUR5",DataFormat="column");
for i=1:500
    show(matlab_UR5,joint_pos);
    hold on;
    show(bMovObs);
    if(aobs_stop_flag == 0)
        aMovObs.Pose = trvec2tform([0.3+0.3*sin(i*0.04) 0.1 0.4]);
    end
    if(bobs_stop_flag == 0)
        bMovObs.Pose = trvec2tform([0.3+0.4*sin(i*0.08) 0.4 0.4]);
    end
    if((aobs_stop_flag == 1)||(i>325))
        aMovObs.Pose = trvec2tform([0.3+0.3*sin(325*0.04)+(i-325)*0.03 0.1 0.4]);
        aobs_stop_flag = 1;
        if(aMovObs.Pose(1,4)>0.55)
             aMovObs.Pose = trvec2tform([0.55 0.1 0.4]);
        end
    end
    if((bobs_stop_flag == 1)||(i>325))
        bMovObs.Pose = trvec2tform([0.3+0.4*sin(325*0.08)+(i-325)*0.03 0.4 0.4]);
        bobs_stop_flag = 1;
        if(bMovObs.Pose(1,4)>0.65)
             bMovObs.Pose = trvec2tform([0.65 0.4 0.4]);
        end
    end
    joint_pos = [joint1p_sequence(i);joint2p_sequence(i);joint3p_sequence(i);...
                 joint4p_sequence(i);joint5p_sequence(i);joint6p_sequence(i);];
    
    %view(180,40)
    set(gca,'XLim',[-1 1],'FontSize',14);
    set(gca,'YLim',[-0.3 0.6],'FontSize',14);
    set(gca,'ZLim',[0 1],'FontSize',14);
    set(gcf,'WindowState','fullscreen');
    drawnow;
    currFrame = getframe(gcf);
    writeVideo(myVideo,currFrame);
    hold off;
end
close(myVideo);