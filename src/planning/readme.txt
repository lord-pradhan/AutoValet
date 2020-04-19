g++ -g planner.cpp utilfunction.cpp params.cpp dubins.c -I/usr/include/python2.7 -lpython2.7 -I/home/lord-pradhan/.local/lib/python2.7/site-packages/numpy/core/include

-- configs that work for motion prims
resolution=0.1;
num_angles=16;
Rmin=5.0;

forward_cost = 1;
backward_cost = 5;
turn_cost = 3;
short_segment_cost = 3;

ntraj=14
points = zeros(ntraj, 3);
costs = zeros(ntraj,1);

% use gen_position script to get x,y from Radius,resolution,initial angle,
% end angle. For example:
% [x y] = gen_position(10,0.2,0,22.5) = [19.1342, 3.8060] --> [19 4]

%  R   Xi  Yi
%  6  11   2
%  8  15   3
% 10  19   4
% 12  23   5
% 14  27   5

x0=10;

%R1
x1=10;
y1=4;

%R2
x2=23;
y2=5;

%R3
x3=27;
y3=5;

------------------- (All feasible) -----
resolution=0.1;
num_angles=16;
Rmin=2.0;

forward_cost = 1;
backward_cost = 5;
turn_cost = 3;
short_segment_cost = 3;

ntraj=14
points = zeros(ntraj, 3);
costs = zeros(ntraj,1);

% use gen_position script to get x,y from Radius,resolution,initial angle,
% end angle. For example:
% [x y] = gen_position(10,0.2,0,22.5) = [19.1342, 3.8060] --> [19 4]

%  R   Xi  Yi
%  6  11   2
%  8  15   3
% 10  19   4
% 12  23   5
% 14  27   5

x0=10;

%R1
x1=16;
y1=4;

%R2
x2=23;
y2=5;

%R3
x3=27;
y3=5;