%% Trajectory path of the robot

trajectorypath = [
%Letter F    
8	41 0;
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
8	41 2;
%Letter a1
35	26	0;
41	32	0;
43	33	0;
44	34	0;
46	35	0;
49	36	0;
51	36	0;
52	35	0;
52	33	0;
51	33	0;
49	32	0;
47	31	0;
45	29	0;
43	28	0;
41	27	0;
38	24	0;
38	22	0;
39	21	0;
40	21	0;
48	29	0;
48	30	0;
50	32	0;
51	32	0;
52	31	0;
52	29	0;
48	25	0;
48	23	0;
49	22	0;
51	22	0;
53	24	0;
53	21	0;
50	18	0;
48	17	0;
46	17	0;
45	18	0;
45	22	0;
44	22	0;
40	18	0;
37	18	0;
35	22	0;
35	26	0;
35	26	2;
%Letter a2
56	26	0;
62	32	0;
64	33	0;
65	34	0;
67	35	0;
70	36	0;
72	36	0;
73	35	0;
73	33	0;
72	33	0;
70	32	0;
68	31	0;
66	29	0;
64	28	0;
62	27	0;
59	24	0;
59	22	0;
60	21	0;
61	21	0;
69	29	0;
69	30	0;
71	32	0;
72	32	0;
73	31	0;
73	29	0;
69	25	0;
69	23	0;
70	22	0;
72	22	0;
74	24	0;
74	21	0;
71	18	0;
69	17	0;
67	17	0;
66	18	0;
66	22	0;
65	22	0;
61	18	0;
58	18	0;
56	22	0;
56	26	0;
56	26	2;
% i dot
86	38	0;
88	40	0;
90	40	0;
91	39	0;
90	37	0;
89	36	0;
87	35	0;
86	36	0;
86	38	0;
86	38	2;
% continue Letter i
78	28	0;
84	34	0;
86	34	0;
87	33	0;
87	32	0;
86	30	0;
81	25	0;
81	22	0;
83	21	0;
85	22	0;
89	26	0;
88	24	0;
88	22	0;
84	18	0;
82	17	0;
81	17	0;
79	18	0;
78	19	0;
78	28	0;
78	28	2;
%Letter z
90	25	0;
92	27	0;
92	28	0;
91	29	0;
91	30	0;
92	32	0;
93	34	0;
96	35	0;
97	34	0;
96	33	0;
100	33	0;
101	34	0;
103	34	0;
103	32	0;
104	31	0;
104	30	0;
101	27	0;
99	26	0;
97	24	0;
97	23	0;
98	22	0;
100	22	0;
101	23	0;
103	24	0;
105	26	0;
105	23	0;
104	22	0;
102	21	0;
99	18	0;
99	17	0;
98	15	0;
98	14	0;
95	11	0;
95	10	0;
93	8	0;
91	7	0;
90	6	0;
87	6	0;
87	11	0;
93	17	0;
95	18	0;
96	19	0;
95	20	0;
93	21	0;
92	22	0;
92	23	0;
95	26	0;
97	27	0;
98	28	0;
98	29	0;
96	29	0;
95	28	0;
93	26	0;
90	23	0;
90	25	0;
90  25  2;
% continue Letter z
89	10	0;
94	15	0;
96	16	0;
96	14	0;
95	12	0;
91	9	0;
89	9	0;
89	10	0;
89	10	2;
];

[nx,ny] = size(trajectorypath);

figure
hold on

for i = 1:nx-1
    v=[trajectorypath(i,:);trajectorypath(i+1,:)];
    plot3(v(:,1),v(:,2),v(:,3),'g');
    plot3(v(:,1),v(:,2),v(:,3),'g.')
end

axis([-100 100 -100 100 -100 100]);
xlabel('X-Axis');
ylabel('Y-Axis');
zlabel('Z-Axis');

view(0,90);