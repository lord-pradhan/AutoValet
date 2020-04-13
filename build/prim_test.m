resolution=0.1;
num_angles=16;
Rmin = 4.0;

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

points( 1,:) = [ x0*resolution   0*resolution  0*2*pi/num_angles];
points( 2,:) = [-x0*resolution   0*resolution  0*2*pi/num_angles];
points( 3,:) = [ x1*resolution   y1*resolution  1*2*pi/num_angles];
points( 4,:) = [ x1*resolution  -y1*resolution -1*2*pi/num_angles];
points( 5,:) = [ x2*resolution   y2*resolution  1*2*pi/num_angles];
points( 6,:) = [ x2*resolution  -y2*resolution -1*2*pi/num_angles];
points( 7,:) = [ x3*resolution   y3*resolution  1*2*pi/num_angles];
points( 8,:) = [ x3*resolution  -y3*resolution -1*2*pi/num_angles];
points( 9,:) = [-x1*resolution  -y1*resolution  1*2*pi/num_angles];
points(10,:) = [-x1*resolution   y1*resolution -1*2*pi/num_angles];
points(11,:) = [-x2*resolution  -y2*resolution  1*2*pi/num_angles];
points(12,:) = [-x2*resolution   y2*resolution -1*2*pi/num_angles];
points(13,:) = [-x3*resolution  -y3*resolution  1*2*pi/num_angles];
points(14,:) = [-x3*resolution   y3*resolution -1*2*pi/num_angles];

% points( 1,:) = [ 10*resolution   0*resolution  0*2*pi/num_angles];
% points( 2,:) = [-10*resolution   0*resolution  0*2*pi/num_angles];
% points( 3,:) = [ 11*resolution   2*resolution  1*2*pi/num_angles];
% points( 4,:) = [ 11*resolution  -2*resolution -1*2*pi/num_angles];
% points( 5,:) = [ 15*resolution   3*resolution  1*2*pi/num_angles];
% points( 6,:) = [ 15*resolution  -3*resolution -1*2*pi/num_angles];
% points( 7,:) = [ 19*resolution   4*resolution  1*2*pi/num_angles];
% points( 8,:) = [ 19*resolution  -4*resolution -1*2*pi/num_angles];
% points( 9,:) = [-11*resolution  -2*resolution  1*2*pi/num_angles];
% points(10,:) = [-11*resolution   2*resolution -1*2*pi/num_angles];
% points(11,:) = [-15*resolution  -3*resolution  1*2*pi/num_angles];
% points(12,:) = [-15*resolution   3*resolution -1*2*pi/num_angles];
% points(13,:) = [-19*resolution  -4*resolution  1*2*pi/num_angles];
% points(14,:) = [-19*resolution   4*resolution -1*2*pi/num_angles];

costs( 1) = forward_cost;
costs( 2) = backward_cost;
costs( 3) = forward_cost+turn_cost; %R1
costs( 4) = forward_cost+turn_cost; %R1
costs( 5) = forward_cost+turn_cost; %R2
costs( 6) = forward_cost+turn_cost; %R2
costs( 7) = forward_cost+turn_cost; %R3
costs( 8) = forward_cost+turn_cost; %R3
costs( 9) = backward_cost+turn_cost; %R1
costs(10) = backward_cost+turn_cost; %R1
costs(11) = backward_cost+turn_cost; %R2
costs(12) = backward_cost+turn_cost; %R2
costs(13) = backward_cost+turn_cost; %R3
costs(14) = backward_cost+turn_cost; %R3

% points( 1,:) = [ 5.6             0             0*2*pi/num_angles];
% points( 2,:) = [ 2.8             0             0*2*pi/num_angles];
% points( 3,:) = [ 5               1.4           1*2*pi/num_angles];
% points( 4,:) = [ 5              -1.4          -1*2*pi/num_angles];
% points( 5,:) = [ 5               2.0           2*2*pi/num_angles];
% points( 6,:) = [ 5              -2.0          -2*2*pi/num_angles];
% points( 7,:) = [ 2.8             0.6           1*2*pi/num_angles];
% points( 8,:) = [ 2.8            -0.6          -1*2*pi/num_angles];
% points( 9,:) = [-5.6             0             0*2*pi/num_angles];
% points(10,:) = [-2.8             0             0*2*pi/num_angles];
% points(11,:) = [-5               1.4          -1*2*pi/num_angles];
% points(12,:) = [-5              -1.4           1*2*pi/num_angles];
% points(13,:) = [-5               2.0          -2*2*pi/num_angles];
% points(14,:) = [-5              -2.0           2*2*pi/num_angles];
% points(15,:) = [-2.8             0.6          -1*2*pi/num_angles];
% points(16,:) = [-2.8            -0.6           1*2*pi/num_angles];
% 
% 
% costs( 1) = forward_cost;
% costs( 2) = forward_cost+short_segment_cost;
% costs( 3) = forward_cost+turn_cost;
% costs( 4) = forward_cost+turn_cost;
% costs( 5) = forward_cost+turn_cost;
% costs( 6) = forward_cost+turn_cost;
% costs( 7) = forward_cost+turn_cost+short_segment_cost;
% costs( 8) = forward_cost+turn_cost+short_segment_cost;
% costs( 9) = backward_cost;
% costs(10) = backward_cost+short_segment_cost;
% costs(11) = backward_cost+turn_cost;
% costs(12) = backward_cost+turn_cost;
% costs(13) = backward_cost+turn_cost;
% costs(14) = backward_cost+turn_cost;
% costs(15) = backward_cost+turn_cost+short_segment_cost;
% costs(16) = backward_cost+turn_cost+short_segment_cost;



p=generate_primitives(resolution,num_angles,points,costs,Rmin);
save_primitives('prim_test.mprim',p);

