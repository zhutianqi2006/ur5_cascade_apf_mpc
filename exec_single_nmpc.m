%% Init ref ts obs joint pos robot
ref = [0.2 0.5 0.4 0.7068  0.0003  0.7074 0.0003 0.0 0.0 0.0 0.0 0.0 0.0];%last six elements is joint velocity
Ts = 0.04;
aobs_stop_flag = 0;
bobs_stop_flag = 0;
%Init object size and pose
aMovObs = collisionSphere(0.10);
aMovObs.Pose = trvec2tform([0.3 0.1 0.3]);
bMovObs = collisionSphere(0.10);
bMovObs.Pose = trvec2tform([0.3 0.3 0.3]);
%Init robot adn set arm joint position
joint_pos = [0.05;-1.57;1.57;-1.57;-1.57;0];
matlab_UR5 = loadrobot("universalUR5",DataFormat="column");
%% Get translation matrix of end-effecotr and other collision check points
tform = getTransform(matlab_UR5, joint_pos,'ee_link');
p1 = getTransform(matlab_UR5, joint_pos,'forearm_link');
p2 = getTransform(matlab_UR5, joint_pos,'wrist_1_link');
ee_quat = rotm2quat(tform(1:3,1:3));
ee_xyz = tform(1:3,4);
robotJacobianTemp = geometricJacobian(matlab_UR5,joint_pos,"ee_link");
robotdetJacobianTemp = det(robotJacobianTemp);
joint_pos_q1_add = joint_pos +[0.001;0;0;0;0;0];
joint_pos_q2_add = joint_pos +[0;0.001;0;0;0;0];
joint_pos_q3_add = joint_pos +[0;0;0.001;0;0;0];
joint_pos_q4_add = joint_pos +[0;0;0;0.001;0;0];
joint_pos_q5_add = joint_pos +[0;0;0;0;0.001;0];
joint_pos_q6_add = joint_pos +[0;0;0;0;0;0.001];
% get the jacobian devriate
robotdet_q1_add = det(geometricJacobian(matlab_UR5,joint_pos_q1_add,"ee_link"));
robotdet_q2_add = det(geometricJacobian(matlab_UR5,joint_pos_q2_add,"ee_link"));
robotdet_q3_add = det(geometricJacobian(matlab_UR5,joint_pos_q3_add,"ee_link"));
robotdet_q4_add = det(geometricJacobian(matlab_UR5,joint_pos_q4_add,"ee_link"));
robotdet_q5_add = det(geometricJacobian(matlab_UR5,joint_pos_q5_add,"ee_link"));
robotdet_q6_add = det(geometricJacobian(matlab_UR5,joint_pos_q6_add,"ee_link"));
Jq1 = (robotdet_q1_add -robotdetJacobianTemp)/0.001;
Jq2 = (robotdet_q2_add -robotdetJacobianTemp)/0.001;
Jq3 = (robotdet_q3_add -robotdetJacobianTemp)/0.001;
Jq4 = (robotdet_q4_add -robotdetJacobianTemp)/0.001;
Jq5 = (robotdet_q5_add -robotdetJacobianTemp)/0.001;
Jq6 = (robotdet_q6_add -robotdetJacobianTemp)/0.001;
% get the jacobian with quat
tform_q1_add = getTransform(matlab_UR5, joint_pos_q1_add,'ee_link');
ee_quat_q1_add = rotm2quat(tform_q1_add(1:3,1:3));
tform_q2_add = getTransform(matlab_UR5, joint_pos_q2_add,'ee_link');
ee_quat_q2_add = rotm2quat(tform_q2_add(1:3,1:3));
tform_q3_add = getTransform(matlab_UR5, joint_pos_q3_add,'ee_link');
ee_quat_q3_add = rotm2quat(tform_q3_add(1:3,1:3));
tform_q4_add = getTransform(matlab_UR5, joint_pos_q4_add,'ee_link');
ee_quat_q4_add = rotm2quat(tform_q4_add(1:3,1:3));
tform_q5_add = getTransform(matlab_UR5, joint_pos_q5_add,'ee_link');
ee_quat_q5_add = rotm2quat(tform_q5_add(1:3,1:3));
tform_q6_add = getTransform(matlab_UR5, joint_pos_q6_add,'ee_link');
ee_quat_q6_add = rotm2quat(tform_q6_add(1:3,1:3));
J_ee(1:3,:) = robotJacobianTemp(4:6,:);
J_ee(4:7,1) = ((ee_quat_q1_add-ee_quat)/0.001)';
J_ee(4:7,2) = ((ee_quat_q2_add-ee_quat)/0.001)';
J_ee(4:7,3) = ((ee_quat_q3_add-ee_quat)/0.001)';
J_ee(4:7,4) = ((ee_quat_q4_add-ee_quat)/0.001)';
J_ee(4:7,5) = ((ee_quat_q5_add-ee_quat)/0.001)';
J_ee(4:7,6) = ((ee_quat_q6_add-ee_quat)/0.001)';
% get forearm_link jacobian and wrist_1_link
robotJacobianTemp_P1 = geometricJacobian(matlab_UR5,joint_pos,'forearm_link');
robotJacobianTemp_P2 = geometricJacobian(matlab_UR5,joint_pos,'wrist_1_link');
J_p1(1:3,:) = robotJacobianTemp_P1(4:6,:);
J_p2(1:3,:) = robotJacobianTemp_P2(4:6,:);
J_p1(4:6,:) = zeros(3,6);
J_p2(4:6,:) = zeros(3,6);
%% Init mpc states
current_states(1) = ee_xyz(1);
current_states(2) = ee_xyz(2);
current_states(3) = ee_xyz(3);
current_states(4) = ee_quat(1);
current_states(5) = ee_quat(2);
current_states(6) = ee_quat(3);
current_states(7) = ee_quat(4);
current_states(8) = joint_pos(1);
current_states(9) = joint_pos(2);
current_states(10) = joint_pos(3);
current_states(11) = joint_pos(4);
current_states(12) = joint_pos(5);
current_states(13) = joint_pos(6);
current_states(14) = p1(1,4);
current_states(15) = p1(2,4);
current_states(16) = p1(3,4);
current_states(17) = p2(1,4);
current_states(18) = p2(2,4);
current_states(19) = p2(3,4);
current_states(20) = aMovObs.Pose(1,4);
current_states(21) = aMovObs.Pose(2,4);
current_states(22) = aMovObs.Pose(3,4);
current_states(23) = 0;
current_states(24) = 0;
current_states(25) = 0;
current_states(26) = bMovObs.Pose(1,4);
current_states(27) = bMovObs.Pose(2,4);
current_states(28) = bMovObs.Pose(3,4);
current_states(29) = 0;
current_states(30) = 0;
current_states(31) = 0;
current_states(32) = robotdetJacobianTemp;
%Run the acado mexfile
out = low_nmpc_RUN(0.0,ref,current_states, ...
    J_ee(1,1),J_ee(1,2),J_ee(1,3),J_ee(1,4),J_ee(1,5),J_ee(1,6), ...
    J_ee(2,1),J_ee(2,2),J_ee(2,3),J_ee(2,4),J_ee(2,5),J_ee(2,6), ...
    J_ee(3,1),J_ee(3,2),J_ee(3,3),J_ee(3,4),J_ee(3,5),J_ee(3,6), ...
    J_ee(4,1),J_ee(4,2),J_ee(4,3),J_ee(4,4),J_ee(4,5),J_ee(4,6), ...
    J_ee(5,1),J_ee(5,2),J_ee(5,3),J_ee(5,4),J_ee(5,5),J_ee(5,6), ...
    J_ee(6,1),J_ee(6,2),J_ee(6,3),J_ee(6,4),J_ee(6,5),J_ee(6,6), ...
    J_ee(7,1),J_ee(7,2),J_ee(7,3),J_ee(7,4),J_ee(7,5),J_ee(7,6), ...
    J_p1(1,1),J_p1(1,2),J_p1(1,3),J_p1(1,4),J_p1(1,5),J_p1(1,6), ...
    J_p1(2,1),J_p1(2,2),J_p1(2,3),J_p1(2,4),J_p1(2,5),J_p1(2,6), ...
    J_p1(3,1),J_p1(3,2),J_p1(3,3),J_p1(3,4),J_p1(3,5),J_p1(3,6), ...
    J_p2(1,1),J_p2(1,2),J_p2(1,3),J_p2(1,4),J_p2(1,5),J_p2(1,6), ...
    J_p2(2,1),J_p2(2,2),J_p2(2,3),J_p2(2,4),J_p2(2,5),J_p2(2,6), ...
    J_p2(3,1),J_p2(3,2),J_p2(3,3),J_p2(3,4),J_p2(3,5),J_p2(3,6), ...
    Jq1,Jq2,Jq3,Jq4,Jq5,Jq6);
%plot sequence
total_distance = [];
min_distance = [];
joint1v_sequence = [];
joint2v_sequence = [];
joint3v_sequence = [];
joint4v_sequence = [];
joint5v_sequence = [];
joint6v_sequence = [];
joint1p_sequence = [];
joint2p_sequence = [];
joint3p_sequence = [];
joint4p_sequence = [];
joint5p_sequence = [];
joint6p_sequence = [];
ex_sequence = [];
ey_sequence = [];
ez_sequence = [];
r_q1_sequence =[];
r_q2_sequence =[];
r_q3_sequence =[];
r_q4_sequence =[];
i_sequence =[];
last_aMovObs.Pose = trvec2tform([0.3 0.1 0.4]);
last_bMovObs.Pose = trvec2tform([0.3 0.4 0.4]);
%% run
for i=1:550
    %Change joint position in sampling time
    joint_pos = [joint_pos(1)+Ts*out.U(1);joint_pos(2)+Ts*out.U(2);joint_pos(3)+Ts*out.U(3);joint_pos(4)+Ts*out.U(4);joint_pos(5)+Ts*out.U(5);joint_pos(6)+Ts*out.U(6)];
    tform = getTransform(matlab_UR5, joint_pos,'ee_link');
    p1 = getTransform(matlab_UR5, joint_pos,'forearm_link');
    p2 = getTransform(matlab_UR5, joint_pos,'wrist_1_link');
    ee_quat = rotm2quat(tform(1:3,1:3));
    ee_xyz = tform(1:3,4);
    %Update the obstacle
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
    %Update jacobian
    robotJacobianTemp = geometricJacobian(matlab_UR5,joint_pos,"ee_link");
    robotdetJacobianTemp = det(robotJacobianTemp);
    joint_pos_q1_add = joint_pos +[0.001;0;0;0;0;0];
    joint_pos_q2_add = joint_pos +[0;0.001;0;0;0;0];
    joint_pos_q3_add = joint_pos +[0;0;0.001;0;0;0];
    joint_pos_q4_add = joint_pos +[0;0;0;0.001;0;0];
    joint_pos_q5_add = joint_pos +[0;0;0;0;0.001;0];
    joint_pos_q6_add = joint_pos +[0;0;0;0;0;0.001];
    % get the jacobian devriate
    robotdet_q1_add = det(geometricJacobian(matlab_UR5,joint_pos_q1_add,"ee_link"));
    robotdet_q2_add = det(geometricJacobian(matlab_UR5,joint_pos_q2_add,"ee_link"));
    robotdet_q3_add = det(geometricJacobian(matlab_UR5,joint_pos_q3_add,"ee_link"));
    robotdet_q4_add = det(geometricJacobian(matlab_UR5,joint_pos_q4_add,"ee_link"));
    robotdet_q5_add = det(geometricJacobian(matlab_UR5,joint_pos_q5_add,"ee_link"));
    robotdet_q6_add = det(geometricJacobian(matlab_UR5,joint_pos_q6_add,"ee_link"));
    Jq1 = (robotdet_q1_add -robotdetJacobianTemp)/0.001;
    Jq2 = (robotdet_q2_add -robotdetJacobianTemp)/0.001;
    Jq3 = (robotdet_q3_add -robotdetJacobianTemp)/0.001;
    Jq4 = (robotdet_q4_add -robotdetJacobianTemp)/0.001;
    Jq5 = (robotdet_q5_add -robotdetJacobianTemp)/0.001;
    Jq6 = (robotdet_q6_add -robotdetJacobianTemp)/0.001;
    % get the jacobian with quat
    tform_q1_add = getTransform(matlab_UR5, joint_pos_q1_add,'ee_link');
    ee_quat_q1_add = rotm2quat(tform_q1_add(1:3,1:3));
    tform_q2_add = getTransform(matlab_UR5, joint_pos_q2_add,'ee_link');
    ee_quat_q2_add = rotm2quat(tform_q2_add(1:3,1:3));
    tform_q3_add = getTransform(matlab_UR5, joint_pos_q3_add,'ee_link');
    ee_quat_q3_add = rotm2quat(tform_q3_add(1:3,1:3));
    tform_q4_add = getTransform(matlab_UR5, joint_pos_q4_add,'ee_link');
    ee_quat_q4_add = rotm2quat(tform_q4_add(1:3,1:3));
    tform_q5_add = getTransform(matlab_UR5, joint_pos_q5_add,'ee_link');
    ee_quat_q5_add = rotm2quat(tform_q5_add(1:3,1:3));
    tform_q6_add = getTransform(matlab_UR5, joint_pos_q6_add,'ee_link');
    ee_quat_q6_add = rotm2quat(tform_q6_add(1:3,1:3));
    J_ee(1:3,:) = robotJacobianTemp(4:6,:);
    J_ee(4:7,1) = ((ee_quat_q1_add-ee_quat)/0.001)';
    J_ee(4:7,2) = ((ee_quat_q2_add-ee_quat)/0.001)';
    J_ee(4:7,3) = ((ee_quat_q3_add-ee_quat)/0.001)';
    J_ee(4:7,4) = ((ee_quat_q4_add-ee_quat)/0.001)';
    J_ee(4:7,5) = ((ee_quat_q5_add-ee_quat)/0.001)';
    J_ee(4:7,6) = ((ee_quat_q6_add-ee_quat)/0.001)';
    % get forearm_link jacobian and wrist_1_link
    robotJacobianTemp_P1 = geometricJacobian(matlab_UR5,joint_pos,'forearm_link');
    robotJacobianTemp_P2 = geometricJacobian(matlab_UR5,joint_pos,'wrist_1_link');
    J_p1(1:3,:) = robotJacobianTemp_P1(4:6,:);
    J_p2(1:3,:) = robotJacobianTemp_P2(4:6,:);
    J_p1(4:6,:) = zeros(3,6);
    J_p2(4:6,:) = zeros(3,6);
    %% Init mpc states
    current_states(1) = ee_xyz(1);
    current_states(2) = ee_xyz(2);
    current_states(3) = ee_xyz(3);
    current_states(4) = ee_quat(1);
    current_states(5) = ee_quat(2);
    current_states(6) = ee_quat(3);
    current_states(7) = ee_quat(4);
    current_states(8) = joint_pos(1);
    current_states(9) = joint_pos(2);
    current_states(10) = joint_pos(3);
    current_states(11) = joint_pos(4);
    current_states(12) = joint_pos(5);
    current_states(13) = joint_pos(6);
    current_states(14) = p1(1,4);
    current_states(15) = p1(2,4);
    current_states(16) = p1(3,4);
    current_states(17) = p2(1,4);
    current_states(18) = p2(2,4);
    current_states(19) = p2(3,4);
    current_states(20) = aMovObs.Pose(1,4);
    current_states(21) = aMovObs.Pose(2,4);
    current_states(22) = aMovObs.Pose(3,4);
    current_states(23) = (aMovObs.Pose(1,4)-last_aMovObs.Pose(1,4))/0.04;
    current_states(24) = (aMovObs.Pose(2,4)-last_aMovObs.Pose(2,4))/0.04;
    current_states(25) = (aMovObs.Pose(3,4)-last_aMovObs.Pose(3,4))/0.04;
    current_states(26) = bMovObs.Pose(1,4);
    current_states(27) = bMovObs.Pose(2,4);
    current_states(28) = bMovObs.Pose(3,4);
    current_states(29) = (bMovObs.Pose(1,4)-last_bMovObs.Pose(1,4))/0.04;
    current_states(30) = (bMovObs.Pose(2,4)-last_bMovObs.Pose(2,4))/0.04;
    current_states(31) = (bMovObs.Pose(3,4)-last_bMovObs.Pose(3,4))/0.04;
    current_states(32) = robotdetJacobianTemp;
%Run the acado mexfile
    out = low_nmpc_RUN(0.0,ref,current_states, ...
    J_ee(1,1),J_ee(1,2),J_ee(1,3),J_ee(1,4),J_ee(1,5),J_ee(1,6), ...
    J_ee(2,1),J_ee(2,2),J_ee(2,3),J_ee(2,4),J_ee(2,5),J_ee(2,6), ...
    J_ee(3,1),J_ee(3,2),J_ee(3,3),J_ee(3,4),J_ee(3,5),J_ee(3,6), ...
    J_ee(4,1),J_ee(4,2),J_ee(4,3),J_ee(4,4),J_ee(4,5),J_ee(4,6), ...
    J_ee(5,1),J_ee(5,2),J_ee(5,3),J_ee(5,4),J_ee(5,5),J_ee(5,6), ...
    J_ee(6,1),J_ee(6,2),J_ee(6,3),J_ee(6,4),J_ee(6,5),J_ee(6,6), ...
    J_ee(7,1),J_ee(7,2),J_ee(7,3),J_ee(7,4),J_ee(7,5),J_ee(7,6), ...
    J_p1(1,1),J_p1(1,2),J_p1(1,3),J_p1(1,4),J_p1(1,5),J_p1(1,6), ...
    J_p1(2,1),J_p1(2,2),J_p1(2,3),J_p1(2,4),J_p1(2,5),J_p1(2,6), ...
    J_p1(3,1),J_p1(3,2),J_p1(3,3),J_p1(3,4),J_p1(3,5),J_p1(3,6), ...
    J_p2(1,1),J_p2(1,2),J_p2(1,3),J_p2(1,4),J_p2(1,5),J_p2(1,6), ...
    J_p2(2,1),J_p2(2,2),J_p2(2,3),J_p2(2,4),J_p2(2,5),J_p2(2,6), ...
    J_p2(3,1),J_p2(3,2),J_p2(3,3),J_p2(3,4),J_p2(3,5),J_p2(3,6), ...
    Jq1,Jq2,Jq3,Jq4,Jq5,Jq6);
    last_aMovObs.Pose = aMovObs.Pose;
    last_bMovObs.Pose = bMovObs.Pose;
    %plot sequence
     total_distance(i) = power(power(ee_xyz(1)-aMovObs.Pose(1,4),2)+power(ee_xyz(2)-aMovObs.Pose(2,4),2)+power(ee_xyz(3)-aMovObs.Pose(3,4),2),1/2)+...
                        power(power(p1(1,4)-aMovObs.Pose(1,4),2)+power(p1(2,4)-aMovObs.Pose(2,4),2)+power(p1(3,4)-aMovObs.Pose(3,4),2),1/2)+...
                        power(power(p2(1,4)-aMovObs.Pose(1,4),2)+power(p2(2,4)-aMovObs.Pose(2,4),2)+power(p2(3,4)-aMovObs.Pose(3,4),2),1/2)+...
                        power(power(ee_xyz(1)-bMovObs.Pose(1,4),2)+power(ee_xyz(2)-bMovObs.Pose(2,4),2)+power(ee_xyz(3)-bMovObs.Pose(3,4),2),1/2)+...
                        power(power(p1(1,4)-bMovObs.Pose(1,4),2)+power(p1(2,4)-bMovObs.Pose(2,4),2)+power(p1(3,4)-bMovObs.Pose(3,4),2),1/2)+...
                        power(power(p2(1,4)-bMovObs.Pose(1,4),2)+power(p2(2,4)-bMovObs.Pose(2,4),2)+power(p2(3,4)-bMovObs.Pose(3,4),2),1/2);
     min_distance(i) = min([ ...
         power(power(ee_xyz(1)-aMovObs.Pose(1,4),2)+power(ee_xyz(2)-aMovObs.Pose(2,4),2)+power(ee_xyz(3)-aMovObs.Pose(3,4),2),1/2),...
         power(power(ee_xyz(1)-bMovObs.Pose(1,4),2)+power(ee_xyz(2)-bMovObs.Pose(2,4),2)+power(ee_xyz(3)-bMovObs.Pose(3,4),2),1/2),...
         power(power(p1(1,4)-aMovObs.Pose(1,4),2)+power(p1(2,4)-aMovObs.Pose(2,4),2)+power(p1(3,4)-aMovObs.Pose(3,4),2),1/2),...
         power(power(p2(1,4)-aMovObs.Pose(1,4),2)+power(p2(2,4)-aMovObs.Pose(2,4),2)+power(p2(3,4)-aMovObs.Pose(3,4),2),1/2),...
         power(power(p1(1,4)-bMovObs.Pose(1,4),2)+power(p1(2,4)-bMovObs.Pose(2,4),2)+power(p1(3,4)-bMovObs.Pose(3,4),2),1/2),...
         power(power(p2(1,4)-bMovObs.Pose(1,4),2)+power(p2(2,4)-bMovObs.Pose(2,4),2)+power(p2(3,4)-bMovObs.Pose(3,4),2),1/2),...
         ]);
  joint1v_sequence(i) = out.U(1);
  joint2v_sequence(i) = out.U(2);
  joint3v_sequence(i) = out.U(3);
  joint4v_sequence(i) = out.U(4);
  joint5v_sequence(i) = out.U(5);
  joint6v_sequence(i) = out.U(6);
  joint1p_sequence(i) = joint_pos(1);
  joint2p_sequence(i) = joint_pos(2);
  joint3p_sequence(i) = joint_pos(3);
  joint4p_sequence(i) = joint_pos(4);
  joint5p_sequence(i) = joint_pos(5);
  joint6p_sequence(i) = joint_pos(6);
  r_q1_sequence(i) = ee_quat(1)-ref(4);
  r_q2_sequence(i) = ee_quat(2)-ref(5);
  r_q3_sequence(i) = ee_quat(3)-ref(6);
  r_q4_sequence(i) = ee_quat(4)-ref(7);
  ex_sequence(i) = ref(1)-current_states(1);
  ey_sequence(i) = ref(2)-current_states(2);
  ez_sequence(i) = ref(3)-current_states(3);
  i_sequence(i) = Ts*i;
end



