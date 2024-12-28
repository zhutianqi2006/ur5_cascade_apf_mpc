clear;
% ACADO PROBLEM CONSTRUCT
BEGIN_ACADO;
acadoSet('problemname', 'high_apf_nmpc'); %mex file name
% states
DifferentialState ee_px ee_py ee_pz ee_r1 ee_r2 ee_r3 ee_r4...
    joint1 joint2 joint3 joint4 joint5 joint6...
    p1_px p1_py p1_pz p2_px p2_py p2_pz...
    object1x object1y object1z object1x_v object1y_v object1z_v...
    object2x object2y object2z object2x_v object2y_v object2z_v;
% input
Control joint1_v joint2_v joint3_v joint4_v joint5_v joint6_v;
AlgebraicState total_energy;
% time vary parameter:starttime = 0,input,current state,jacobian of ee,jacobian of other joint, obstacles position; 
startime = acado.MexInput;
input_refernce = acado.MexInputVector;
current_state = acado.MexInputVector;
Jee11 = acado.MexInput;
Jee12 = acado.MexInput;
Jee13 = acado.MexInput;
Jee14 = acado.MexInput;
Jee15 = acado.MexInput;
Jee16 = acado.MexInput;
Jee21 = acado.MexInput;
Jee22 = acado.MexInput;
Jee23 = acado.MexInput;
Jee24 = acado.MexInput;
Jee25 = acado.MexInput;
Jee26 = acado.MexInput;
Jee31 = acado.MexInput;
Jee32 = acado.MexInput;
Jee33 = acado.MexInput;
Jee34 = acado.MexInput;
Jee35 = acado.MexInput;
Jee36 = acado.MexInput;
Jee41 = acado.MexInput;
Jee42 = acado.MexInput;
Jee43 = acado.MexInput;
Jee44 = acado.MexInput;
Jee45 = acado.MexInput;
Jee46 = acado.MexInput;
Jee51 = acado.MexInput;
Jee52 = acado.MexInput;
Jee53 = acado.MexInput;
Jee54 = acado.MexInput;
Jee55 = acado.MexInput;
Jee56 = acado.MexInput;
Jee61 = acado.MexInput;
Jee62 = acado.MexInput;
Jee63 = acado.MexInput;
Jee64 = acado.MexInput;
Jee65 = acado.MexInput;
Jee66 = acado.MexInput;
Jee71 = acado.MexInput;
Jee72 = acado.MexInput;
Jee73 = acado.MexInput;
Jee74 = acado.MexInput;
Jee75 = acado.MexInput;
Jee76 = acado.MexInput;
Jp111 = acado.MexInput;
Jp112 = acado.MexInput;
Jp113 = acado.MexInput;
Jp114 = acado.MexInput;
Jp115 = acado.MexInput;
Jp116 = acado.MexInput;
Jp121 = acado.MexInput;
Jp122 = acado.MexInput;
Jp123 = acado.MexInput;
Jp124 = acado.MexInput;
Jp125 = acado.MexInput;
Jp126 = acado.MexInput;
Jp131 = acado.MexInput;
Jp132 = acado.MexInput;
Jp133 = acado.MexInput;
Jp134 = acado.MexInput;
Jp135 = acado.MexInput;
Jp136 = acado.MexInput;
Jp211 = acado.MexInput;
Jp212 = acado.MexInput;
Jp213 = acado.MexInput;
Jp214 = acado.MexInput;
Jp215 = acado.MexInput;
Jp216 = acado.MexInput;
Jp221 = acado.MexInput;
Jp222 = acado.MexInput;
Jp223 = acado.MexInput;
Jp224 = acado.MexInput;
Jp225 = acado.MexInput;
Jp226 = acado.MexInput;
Jp231 = acado.MexInput;
Jp232 = acado.MexInput;
Jp233 = acado.MexInput;
Jp234 = acado.MexInput;
Jp235 = acado.MexInput;
Jp236 = acado.MexInput;
error_gain = acado.MexInputMatrix;
% function of the problem
f = acado.DifferentialEquation();
f.add(dot(ee_px) == Jee11*joint1_v+ Jee12*joint2_v+Jee13*joint3_v + Jee14*joint4_v+Jee15*joint5_v+Jee16*joint6_v);
f.add(dot(ee_py) == Jee21*joint1_v+ Jee22*joint2_v+Jee23*joint3_v + Jee24*joint4_v+Jee25*joint5_v+Jee26*joint6_v);
f.add(dot(ee_pz) == Jee31*joint1_v+ Jee32*joint2_v+Jee33*joint3_v + Jee34*joint4_v+Jee35*joint5_v+Jee36*joint6_v);
f.add(dot(ee_r1) == Jee41*joint1_v+ Jee42*joint2_v+Jee43*joint3_v + Jee44*joint4_v+Jee45*joint5_v+Jee46*joint6_v);
f.add(dot(ee_r2) == Jee51*joint1_v+ Jee52*joint2_v+Jee53*joint3_v + Jee54*joint4_v+Jee55*joint5_v+Jee56*joint6_v);
f.add(dot(ee_r3) == Jee61*joint1_v+ Jee62*joint2_v+Jee63*joint3_v + Jee64*joint4_v+Jee65*joint5_v+Jee66*joint6_v);
f.add(dot(ee_r4) == Jee71*joint1_v+ Jee72*joint2_v+Jee73*joint3_v + Jee74*joint4_v+Jee75*joint5_v+Jee76*joint6_v);
f.add(dot(joint1) == joint1_v);
f.add(dot(joint2) == joint2_v);
f.add(dot(joint3) == joint3_v);
f.add(dot(joint4) == joint4_v);
f.add(dot(joint5) == joint5_v);
f.add(dot(joint6) == joint6_v);
f.add(dot(p1_px) == Jp111*joint1_v+ Jp112*joint2_v+Jp113*joint3_v + Jp114*joint4_v+Jp115*joint5_v+Jp116*joint6_v);
f.add(dot(p1_py) == Jp121*joint1_v+ Jp122*joint2_v+Jp123*joint3_v + Jp124*joint4_v+Jp125*joint5_v+Jp126*joint6_v);
f.add(dot(p1_pz) == Jp131*joint1_v+ Jp132*joint2_v+Jp133*joint3_v + Jp134*joint4_v+Jp135*joint5_v+Jp136*joint6_v);
f.add(dot(p2_px) == Jp211*joint1_v+ Jp212*joint2_v+Jp213*joint3_v + Jp214*joint4_v+Jp215*joint5_v+Jp216*joint6_v);
f.add(dot(p2_py) == Jp221*joint1_v+ Jp222*joint2_v+Jp223*joint3_v + Jp224*joint4_v+Jp225*joint5_v+Jp226*joint6_v);
f.add(dot(p2_pz) == Jp231*joint1_v+ Jp232*joint2_v+Jp233*joint3_v + Jp234*joint4_v+Jp235*joint5_v+Jp236*joint6_v);
f.add(dot(object1x) == object1x_v);
f.add(dot(object1y) == object1y_v);
f.add(dot(object1z) == object1z_v);
f.add(dot(object1x_v) == 0);
f.add(dot(object1y_v) == 0);
f.add(dot(object1z_v) == 0);
f.add(dot(object2x) == object2x_v);
f.add(dot(object2y) == object2y_v);
f.add(dot(object2z) == object2z_v);
f.add(dot(object2x_v) == 0);
f.add(dot(object2y_v) == 0);
f.add(dot(object2z_v) == 0);
f.add(...
0 == total_energy-...
(...
 (1/(power(power(ee_px-object1x,2)+power(ee_py-object1y,2)+power(ee_pz-object1z,2),1/2))) ...
+(1/(power(power(p1_px-object1x,2)+power(p1_py-object1y,2)+power(p1_pz-object1z,2),1/2))) ...
+(1/(power(power(p2_px-object1x,2)+power(p2_py-object1y,2)+power(p2_pz-object1z,2),1/2)))...
+(1/(power(power(ee_px-object2x,2)+power(ee_py-object2y,2)+power(ee_pz-object2z,2),1/2))) ...
+(1/(power(power(p1_px-object2x,2)+power(p1_py-object2y,2)+power(p1_pz-object2z,2),1/2))) ...
+(1/(power(power(p2_px-object2x,2)+power(p2_py-object2y,2)+power(p2_pz-object2z,2),1/2)))...
)...
);
% t_end is the predict time,setp num is the step length
t_start	= 0.0;
t_end	= 0.8;
step_num = 2;
ocp = acado.OCP(t_start, t_end, step_num);
% The weighting Variables
h={ee_px ee_py ee_pz ee_r1 ee_r2 ee_r3 ee_r4 total_energy};
% reference defined by mexinput
r= input_refernce;
% The weighting matrix
Q = error_gain;
ocp.minimizeLSQ(Q, h, r);
% The constrains problem
ocp.subjectTo( f );
ocp.subjectTo( -0.6 <= joint1_v <= 0.6 );
ocp.subjectTo( -0.6 <= joint2_v <= 0.6 );
ocp.subjectTo( -0.6 <= joint3_v <= 0.6 );
ocp.subjectTo( -0.6 <= joint4_v <= 0.6 );
ocp.subjectTo( -0.6 <= joint5_v <= 0.6 );
ocp.subjectTo( -0.6 <= joint6_v <= 0.6 );
% Solving method and final setup
algo = acado.RealTimeAlgorithm(ocp,(t_end-t_start)/step_num);
algo.set('MAX_NUM_ITERATIONS', 1 );
algo.set('HESSIAN_APPROXIMATION', 'GAUSS_NEWTON');
controller = acado.Controller(algo);
controller.init(startime, current_state);
controller.step(startime, current_state);
END_ACADO;