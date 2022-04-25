clc
clear

planner = Traj_Planner();

p0 = [0;0;0];
pf = [1;2;3];

v0 = [0;0;0];
vf = [0;0;0];

t = 0:10;

coords = planner.cubic_traj(t, v0, vf, p0, pf);


% plot3(coords(2,:), coords(3,:), coords(4,:))


plot(coords(1,:),coords(4,:))
