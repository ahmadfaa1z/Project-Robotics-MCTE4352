%% Setting of the lengths of each links
a1 = 30;
a2 = 30;
d1 = 30; % Height of the Robot
d3 = 3; % Length of prismatic joint 

%% L = Link([Theta d a alpha 0/1]) >> 0 for revolute and 1 for prismatic
% SCARA Robot
L1 = Link([0 d1 a1 0 0], 'standard');
L2 = Link([0 0 a2 pi 0], 'standard');
L3 = Link([0 d3 0 0 1], 'standard'); % Prismatic joint
L4 = Link([0 0 0 0 0], 'standard'); % End effector

%%
L3.qlim = [0 5]; % up/down of prismatic joint for link 3
Rob = SerialLink([L1 L2 L3 L4],'name','SCARA Robot');

%% Transformation points (x,y,z)
matrix = [
8	41 20;
12	45 20;
16	46 20;
21	47 20;
35	47 20;
36	46 20;
36	41 20;
35	39 20;
34	37 20;
33	36 20;
32	34 20;
31	32 20;
30	30 20;
29	29 20;
32	29 20;
32	28 20;
29	25 20;
27	24 20;
25	24 20;
24	22 20;
16	14 20;
14	13 20;
12	11 20;
10	10 20;
9	10 20;
8	11 20;
8	13 20;
12	17 20;
14	18 20;
17	20 20;
21	24 20;
13	24 20;
13	27 20;
14	28 20;
18	29 20;
24	29 20;
28	33 20;
28	34 20;
30	37 20;
31	39 20;
31	41 20;
19	41 20;
18	40 20;
15	40 20;
11	36 20;
9	35 20;
7	35 20;
6	36 20;
6	37 20;
7	39 20;
8	41 20;
];

%% Transformation matrix
q0 = [0 0 0 0];
P1 = transl(matrix(1,1),matrix(1,2),matrix(1,3));
P2 = transl(matrix(2,1),matrix(2,2),matrix(2,3));
P3 = transl(matrix(3,1),matrix(3,2),matrix(3,3));
P4 = transl(matrix(4,1),matrix(4,2),matrix(4,3));
P5 = transl(matrix(5,1),matrix(5,2),matrix(5,3));
P6 = transl(matrix(6,1),matrix(6,2),matrix(6,3));
P7 = transl(matrix(7,1),matrix(7,2),matrix(7,3));
P8 = transl(matrix(8,1),matrix(8,2),matrix(8,3));
P9 = transl(matrix(9,1),matrix(9,2),matrix(9,3));
P10 = transl(matrix(10,1),matrix(10,2),matrix(10,3));
P11 = transl(matrix(11,1),matrix(11,2),matrix(11,3));
P12 = transl(matrix(12,1),matrix(12,2),matrix(12,3));
P13 = transl(matrix(13,1),matrix(13,2),matrix(13,3));
P14 = transl(matrix(14,1),matrix(14,2),matrix(14,3));
P15 = transl(matrix(15,1),matrix(15,2),matrix(15,3));
P16 = transl(matrix(16,1),matrix(16,2),matrix(16,3));
P17 = transl(matrix(17,1),matrix(17,2),matrix(17,3));
P18 = transl(matrix(18,1),matrix(18,2),matrix(18,3));
P19 = transl(matrix(19,1),matrix(19,2),matrix(19,3));
P20 = transl(matrix(20,1),matrix(20,2),matrix(20,3));
P21 = transl(matrix(21,1),matrix(21,2),matrix(21,3));
P22 = transl(matrix(22,1),matrix(22,2),matrix(22,3));
P23 = transl(matrix(23,1),matrix(23,2),matrix(23,3));
P24 = transl(matrix(24,1),matrix(24,2),matrix(24,3));
P25 = transl(matrix(25,1),matrix(25,2),matrix(25,3));
P26 = transl(matrix(26,1),matrix(26,2),matrix(26,3));
P27 = transl(matrix(27,1),matrix(27,2),matrix(27,3));
P28 = transl(matrix(28,1),matrix(28,2),matrix(28,3));
P29 = transl(matrix(29,1),matrix(29,2),matrix(29,3));
P30 = transl(matrix(30,1),matrix(30,2),matrix(30,3));
P31 = transl(matrix(31,1),matrix(31,2),matrix(31,3));
P32 = transl(matrix(32,1),matrix(32,2),matrix(32,3));
P33 = transl(matrix(33,1),matrix(33,2),matrix(33,3));
P34 = transl(matrix(34,1),matrix(34,2),matrix(34,3));
P35 = transl(matrix(35,1),matrix(35,2),matrix(35,3));
P36 = transl(matrix(36,1),matrix(36,2),matrix(36,3));
P37 = transl(matrix(37,1),matrix(37,2),matrix(37,3));
P38 = transl(matrix(38,1),matrix(38,2),matrix(38,3));
P39 = transl(matrix(39,1),matrix(39,2),matrix(39,3));
P40 = transl(matrix(40,1),matrix(40,2),matrix(40,3));
P41 = transl(matrix(41,1),matrix(41,2),matrix(41,3));
P42 = transl(matrix(42,1),matrix(42,2),matrix(42,3));
P43 = transl(matrix(43,1),matrix(43,2),matrix(43,3));
P44 = transl(matrix(44,1),matrix(44,2),matrix(44,3));
P45 = transl(matrix(45,1),matrix(45,2),matrix(45,3));
P46 = transl(matrix(46,1),matrix(46,2),matrix(46,3));
P47 = transl(matrix(47,1),matrix(47,2),matrix(47,3));
P48 = transl(matrix(48,1),matrix(48,2),matrix(48,3));
P49 = transl(matrix(49,1),matrix(49,2),matrix(49,3));
P50 = transl(matrix(50,1),matrix(50,2),matrix(50,3));
P51 = transl(matrix(51,1),matrix(51,2),matrix(51,3));


%% To calculate inverse kinematics
q1 = Rob.ikine(P1,q0,[1,1,1,0,0,0]);
q2 = Rob.ikine(P2,q1,[1,1,1,0,0,0]);
q3 = Rob.ikine(P3,q2,[1,1,1,0,0,0]);
q4 = Rob.ikine(P4,q3,[1,1,1,0,0,0]);
q5 = Rob.ikine(P5,q4,[1,1,1,0,0,0]);
q6 = Rob.ikine(P6,q5,[1,1,1,0,0,0]);
q7 = Rob.ikine(P7,q6,[1,1,1,0,0,0]);
q8 = Rob.ikine(P8,q7,[1,1,1,0,0,0]);
q9 = Rob.ikine(P9,q8,[1,1,1,0,0,0]);
q10 = Rob.ikine(P10,q9,[1,1,1,0,0,0]);
q11 = Rob.ikine(P11,q10,[1,1,1,0,0,0]);
q12 = Rob.ikine(P12,q11,[1,1,1,0,0,0]);
q13 = Rob.ikine(P13,q12,[1,1,1,0,0,0]);
q14 = Rob.ikine(P14,q13,[1,1,1,0,0,0]);
q15 = Rob.ikine(P15,q14,[1,1,1,0,0,0]);
q16 = Rob.ikine(P16,q15,[1,1,1,0,0,0]);
q17 = Rob.ikine(P17,q16,[1,1,1,0,0,0]);
q18 = Rob.ikine(P18,q17,[1,1,1,0,0,0]);
q19 = Rob.ikine(P19,q18,[1,1,1,0,0,0]);
q20 = Rob.ikine(P20,q19,[1,1,1,0,0,0]);
q21 = Rob.ikine(P21,q20,[1,1,1,0,0,0]);
q22 = Rob.ikine(P22,q21,[1,1,1,0,0,0]);
q23 = Rob.ikine(P23,q22,[1,1,1,0,0,0]);
q24 = Rob.ikine(P24,q23,[1,1,1,0,0,0]);
q25 = Rob.ikine(P25,q24,[1,1,1,0,0,0]);
q26 = Rob.ikine(P26,q25,[1,1,1,0,0,0]);
q27 = Rob.ikine(P27,q26,[1,1,1,0,0,0]);
q28 = Rob.ikine(P28,q27,[1,1,1,0,0,0]);
q29 = Rob.ikine(P29,q28,[1,1,1,0,0,0]);
q30 = Rob.ikine(P30,q29,[1,1,1,0,0,0]);
q31 = Rob.ikine(P31,q30,[1,1,1,0,0,0]);
q32 = Rob.ikine(P32,q31,[1,1,1,0,0,0]);
q33 = Rob.ikine(P33,q32,[1,1,1,0,0,0]);
q34 = Rob.ikine(P34,q33,[1,1,1,0,0,0]);
q35 = Rob.ikine(P35,q34,[1,1,1,0,0,0]);
q36 = Rob.ikine(P36,q35,[1,1,1,0,0,0]);
q37 = Rob.ikine(P37,q36,[1,1,1,0,0,0]);
q38 = Rob.ikine(P38,q37,[1,1,1,0,0,0]);
q39 = Rob.ikine(P39,q38,[1,1,1,0,0,0]);
q40 = Rob.ikine(P40,q39,[1,1,1,0,0,0]);
q41 = Rob.ikine(P41,q40,[1,1,1,0,0,0]);
q42 = Rob.ikine(P42,q41,[1,1,1,0,0,0]);
q43 = Rob.ikine(P43,q42,[1,1,1,0,0,0]);
q44 = Rob.ikine(P44,q43,[1,1,1,0,0,0]);
q45 = Rob.ikine(P45,q44,[1,1,1,0,0,0]);
q46 = Rob.ikine(P46,q45,[1,1,1,0,0,0]);
q47 = Rob.ikine(P47,q46,[1,1,1,0,0,0]);
q48 = Rob.ikine(P48,q47,[1,1,1,0,0,0]);
q49 = Rob.ikine(P49,q48,[1,1,1,0,0,0]);
q50 = Rob.ikine(P50,q49,[1,1,1,0,0,0]);
q51 = Rob.ikine(P51,q50,[1,1,1,0,0,0]);

%% To calculate forward kinematics
%{
q1_P1 = Rob.fkine(q1);
q2_P2 = Rob.fkine(q2);
q3_P3 = Rob.fkine(q3);
q4_P4 = Rob.fkine(q4);
q5_P5 = Rob.fkine(q5);
q6_P6 = Rob.fkine(q6);
q7_P7 = Rob.fkine(q7);
q8_P8 = Rob.fkine(q8);
q9_P9 = Rob.fkine(q9);
q10_P10 = Rob.fkine(q10);
q11_P11 = Rob.fkine(q11);
q12_P12 = Rob.fkine(q12);
q13_P13 = Rob.fkine(q13);
q14_P14 = Rob.fkine(q14);
q15_P15 = Rob.fkine(q15);
q16_P16 = Rob.fkine(q16);
q17_P17 = Rob.fkine(q17);
q18_P18 = Rob.fkine(q18);
q19_P19 = Rob.fkine(q19);
q20_P20 = Rob.fkine(q20);
q21_P21 = Rob.fkine(q21);
q22_P22 = Rob.fkine(q22);
q23_P23 = Rob.fkine(q23);
q24_P24 = Rob.fkine(q24);
q25_P25 = Rob.fkine(q25);
q26_P26 = Rob.fkine(q26);
q27_P27 = Rob.fkine(q27);
q28_P28 = Rob.fkine(q28);
q29_P29 = Rob.fkine(q29);
q30_P30 = Rob.fkine(q30);
q31_P31 = Rob.fkine(q31);
q32_P32 = Rob.fkine(q32);
q33_P33 = Rob.fkine(q33);
q34_P34 = Rob.fkine(q34);
q35_P35 = Rob.fkine(q35);
q36_P36 = Rob.fkine(q36);
q37_P37 = Rob.fkine(q37);
q38_P38 = Rob.fkine(q38);
q39_P39 = Rob.fkine(q39);
q40_P40 = Rob.fkine(q40);
q41_P41 = Rob.fkine(q41);
q42_P42 = Rob.fkine(q42);
q43_P43 = Rob.fkine(q43);
q44_P44 = Rob.fkine(q44);
q45_P45 = Rob.fkine(q45);
q46_P46 = Rob.fkine(q46);
q47_P47 = Rob.fkine(q47);
q48_P48 = Rob.fkine(q48);
q49_P49 = Rob.fkine(q49);
q50_P50 = Rob.fkine(q50);
q51_P51 = Rob.fkine(q51);
%}
%% Trajectory path of the robot
trajectorypath = [8	41 0;
12	45 0;
16	46 0;
21	47 0;
35	47 0;
36	46 0;
36	41 0;
35	39 0;
34	37 0;
33	36 0;
32	34 0;
31	32 0;
30	30 0;
29	29 0;
32	29 0;
32	28 0;
29	25 0;
27	24 0;
25	24 0;
24	22 0;
16	14 0;
14	13 0;
12	11 0;
10	10 0;
9	10 0;
8	11 0;
8	13 0;
12	17 0;
14	18 0;
17	20 0;
21	24 0;
13	24 0;
13	27 0;
14	28 0;
18	29 0;
24	29 0;
28	33 0;
28	34 0;
30	37 0;
31	39 0;
31	41 0;
19	41 0;
18	40 0;
15	40 0;
11	36 0;
9	35 0;
7	35 0;
6	36 0;
6	37 0;
7	39 0;
8	41 0;
];
[nx,ny] = size(trajectorypath);

figure
hold on

for i = 1:nx-1
    v=[trajectorypath(i,:);trajectorypath(i+1,:)];
    plot3(v(:,1),v(:,2),v(:,3),'g');
    plot3(v(:,1),v(:,2),v(:,3),'g.')
end

% view settings of 3D plot
axis([-100 100 -100 100 -100 100]);
xlabel('X-Axis');
ylabel('Y-Axis');
zlabel('Z-Axis');

%view(0,90);
%view(3);

%% Path of the SCARA Robot
% Time vector:
t = (0: .05: 0.2)';

% transformation animation
% 1
line1 = jtraj(q0,q1,t);
Rob.plot(line1, 'workspace', [-40 100 -40 100 -40 100]);

% 2
line2 = jtraj(q1,q2,t);
Rob.plot(line2, 'workspace', [-40 100 -40 100 -40 100]);

% 3
line3 = jtraj(q2,q3,t);
Rob.plot(line3, 'workspace', [-40 100 -40 100 -40 100]);

% 4
line4 = jtraj(q3,q4,t);
Rob.plot(line4, 'workspace', [-40 100 -40 100 -40 100]);

% 5
line5 = jtraj(q4,q5,t);
Rob.plot(line5, 'workspace', [-40 100 -40 100 -40 100]);

% 6
line6 = jtraj(q5,q6,t);
Rob.plot(line6, 'workspace', [-40 100 -40 100 -40 100]);

% 7
line7 = jtraj(q6,q7,t);
Rob.plot(line7, 'workspace', [-40 100 -40 100 -40 100]);

% 8
line8 = jtraj(q7,q8,t);
Rob.plot(line8, 'workspace', [-40 100 -40 100 -40 100]);

% 9
line9 = jtraj(q8,q9,t);
Rob.plot(line9, 'workspace', [-40 100 -40 100 -40 100]);

% 10
line10 = jtraj(q9,q10,t);
Rob.plot(line10, 'workspace', [-40 100 -40 100 -40 100]);

% 11
line11 = jtraj(q10,q11,t);
Rob.plot(line11, 'workspace', [-40 100 -40 100 -40 100]);

% 12
line12 = jtraj(q11,q12,t);
Rob.plot(line12, 'workspace', [-40 100 -40 100 -40 100]);

% 13
line13 = jtraj(q12,q13,t);
Rob.plot(line13, 'workspace', [-40 100 -40 100 -40 100]);

% 14
line14 = jtraj(q13,q14,t);
Rob.plot(line14, 'workspace', [-40 100 -40 100 -40 100]);

% 15
line15 = jtraj(q14,q15,t);
Rob.plot(line15, 'workspace', [-40 100 -40 100 -40 100]);

% 16
line16 = jtraj(q15,q16,t);
Rob.plot(line16, 'workspace', [-40 100 -40 100 -40 100]);

% 17
line17 = jtraj(q16,q17,t);
Rob.plot(line17, 'workspace', [-40 100 -40 100 -40 100]);

% 18
line18 = jtraj(q17,q18,t);
Rob.plot(line18, 'workspace', [-40 100 -40 100 -40 100]);

% 19
line19 = jtraj(q18,q19,t);
Rob.plot(line19, 'workspace', [-40 100 -40 100 -40 100]);

% 20
line20 = jtraj(q19,q20,t);
Rob.plot(line20, 'workspace', [-40 100 -40 100 -40 100]);

% 21
line21 = jtraj(q20,q21,t);
Rob.plot(line21, 'workspace', [-40 100 -40 100 -40 100]);

% 22
line22 = jtraj(q21,q22,t);
Rob.plot(line22, 'workspace', [-40 100 -40 100 -40 100]);

% 23
line23 = jtraj(q22,q23,t);
Rob.plot(line23, 'workspace', [-40 100 -40 100 -40 100]);

% 24
line24 = jtraj(q23,q24,t);
Rob.plot(line24, 'workspace', [-40 100 -40 100 -40 100]);

% 25
line25 = jtraj(q24,q25,t);
Rob.plot(line25, 'workspace', [-40 100 -40 100 -40 100]);

% 26
line26 = jtraj(q25,q26,t);
Rob.plot(line26, 'workspace', [-40 100 -40 100 -40 100]);

% 27
line27 = jtraj(q26,q27,t);
Rob.plot(line27, 'workspace', [-40 100 -40 100 -40 100]);

% 28
line28 = jtraj(q27,q28,t);
Rob.plot(line28, 'workspace', [-40 100 -40 100 -40 100]);

% 29
line29 = jtraj(q28,q29,t);
Rob.plot(line29, 'workspace', [-40 100 -40 100 -40 100]);

% 30
line30 = jtraj(q29,q30,t);
Rob.plot(line30, 'workspace', [-40 100 -40 100 -40 100]);

% 31
line31 = jtraj(q30,q31,t);
Rob.plot(line31, 'workspace', [-40 100 -40 100 -40 100]);

% 32
line32 = jtraj(q31,q32,t);
Rob.plot(line32, 'workspace', [-40 100 -40 100 -40 100]);

% 33
line33 = jtraj(q32,q33,t);
Rob.plot(line33, 'workspace', [-40 100 -40 100 -40 100]);

% 34
line34 = jtraj(q33,q34,t);
Rob.plot(line34, 'workspace', [-40 100 -40 100 -40 100]);

% 35
line35 = jtraj(q34,q35,t);
Rob.plot(line35, 'workspace', [-40 100 -40 100 -40 100]);

% 36
line36 = jtraj(q35,q36,t);
Rob.plot(line36, 'workspace', [-40 100 -40 100 -40 100]);

% 37
line37 = jtraj(q36,q37,t);
Rob.plot(line37, 'workspace', [-40 100 -40 100 -40 100]);

% 38
line38 = jtraj(q37,q38,t);
Rob.plot(line38, 'workspace', [-40 100 -40 100 -40 100]);

% 39
line39 = jtraj(q38,q39,t);
Rob.plot(line39, 'workspace', [-40 100 -40 100 -40 100]);

% 40
line40 = jtraj(q39,q40,t);
Rob.plot(line40, 'workspace', [-40 100 -40 100 -40 100]);

% 41
line41 = jtraj(q40,q41,t);
Rob.plot(line41, 'workspace', [-40 100 -40 100 -40 100]);

% 42
line42 = jtraj(q41,q42,t);
Rob.plot(line42, 'workspace', [-40 100 -40 100 -40 100]);

% 43
line43 = jtraj(q42,q43,t);
Rob.plot(line43, 'workspace', [-40 100 -40 100 -40 100]);

% 44
line44 = jtraj(q43,q44,t);
Rob.plot(line44, 'workspace', [-40 100 -40 100 -40 100]);

% 45
line45 = jtraj(q44,q45,t);
Rob.plot(line45, 'workspace', [-40 100 -40 100 -40 100]);

% 46
line46 = jtraj(q45,q46,t);
Rob.plot(line46, 'workspace', [-40 100 -40 100 -40 100]);

% 47
line47 = jtraj(q46,q47,t);
Rob.plot(line47, 'workspace', [-40 100 -40 100 -40 100]);

% 48
line48 = jtraj(q47,q48,t);
Rob.plot(line48, 'workspace', [-40 100 -40 100 -40 100]);

% 49
line49 = jtraj(q48,q49,t);
Rob.plot(line49, 'workspace', [-40 100 -40 100 -40 100]);

% 50
line50 = jtraj(q49,q50,t);
Rob.plot(line50, 'workspace', [-40 100 -40 100 -40 100]);

% 51
line51 = jtraj(q50,q51,t);
Rob.plot(line51, 'workspace', [-40 100 -40 100 -40 100]);

