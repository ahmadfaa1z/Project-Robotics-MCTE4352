%  a1 = input ('key in 1st link length');
%  a2 = input ('key in 2nd link length');
a1 = 20;
a2 = 20;
a3 = 20;
delay = 0.2;

%---
locationX = [
6
6
6
6
7
7
8
9
10
11
12
14
15
16
18
19
20
21
22
23
24
25
26
27
28
29
30
31
32
33
34
35
35
35
35
35
34
34
33
33
32
32
31
31
30
30
29
28
28
29
30
31
31
30
29
28
27
26
25
24
23
23
22
21
20
19
18
17
16
15
14
13
12
11
9
7
9
10
11
12
14
16
17
18
19
20
20
19
18
17
16
15
14
13
12
12
12
13
14
16
17
18
19
20
21
22
23
24
25
26
27
28
29
29
30
31
31
31
23
20
17
15
13
12
11
10
9
6

]';
%---

locationY = [
30
31
32
33
34
35
36
37
38
39
39
40
40
40
41
41
41
41
41
41
42
42
42
42
42
42
42
42
42
42
42
41
40
39
38
37
36
35
34
33
32
31
30
29
28
27
26
25
24
24
24
24
23
22
21
20
19
19
19
19
18
17
16
15
14
13
12
11
10
9
8
8
7
6
5
6
9
10
11
12
13
14
15
16
17
18
19
19
19
19
19
19
19
19
19
20
21
22
22
23
23
23
23
24
24
24
24
24
25
26
27
29
30
31
33
34
35
36
36
35
35
34
34
33
32
31
30
30
]';
%---

theta_1 = [-7.8396
1.1006
8.9211
9.1049
9.3875
9.556
9.4234
8.1819
7.8339
3.9644
-0.4244
-2.9252
-5.4481
-6.9842
-7.4011
-7.7178
-7.8396

    ]';
theta_2 = [55.4442
50.2596
49.1114
48.6136
47.7175
46.9487
45.5966
45.3033
41.6982
43.6997
46.8385
49.0139
51.6527
53.7309
54.4555
55.1273
55.4442

]';
theta_3 = [46.1164
40.0287
27.2262
27.2031
27.1391
27.0643
26.9202
27.009
32.4021
34.929
38.3886
40.6002
43.083
44.8465
45.4085
45.898
46.1164

]';
sz = size(locationX);
numOfSteps = sz(1,2);
f1 = figure;
figure(f1);

w = 1 ;
 for n = 1:numOfSteps
%      theta = [final(n,3),final(n,4)];
 if w == 1
        hold off   
        plot(locationX,locationY); %plot the path in a cartesian coordinate system
        hold on;
        grid on;
        %{
        aa(1) = 0; % origin of the robot arm
        bb(1) = 0; % origin of the robot arm
        aa(2) = a1 * cosd(theta_1(n) ); % 1st robot arm X position
        bb(2) = a1 * sind(theta_1(n) ); % 1st robot arm Y position
        aa(3) = aa(2) + a2 * cosd(theta_2(n) + theta_1(n)); % 2nd robot arm X position or tooltip X Position
        bb(3) = bb(2) + a2 * sind(theta_2(n) + theta_1(n) );% 2nd robot arm Y position or tooltip Y Position
        aa(4) = aa(3) + a3 * cosd(theta_3(n) + theta_2(n) + theta_1(n)); % 2nd robot arm X position or tooltip X Position
        bb(4) = bb(3) + a3 * sind(theta_3(n) + theta_2(n) + theta_1(n));% 2nd robot arm Y position or tooltip Y Position
        plot(aa, bb,'r','linewidth',3);
        axis([-20 20 -5 30]);
        pause(delay);
        %}
 end
 end

 function F = IKK (x)
X=16.3;
Y=16.02263;
L=10;

F =[X-((L*cos(x(1)+x(2)+x(3)))+(L*cos(x(1)+x(2)))+(L*cos(x(1))));
    Y-((L*sin(x(1)+x(2)+x(3)))+(L*sin(x(1)+x(2)))+(L*sin(x(1))))];

 end
 