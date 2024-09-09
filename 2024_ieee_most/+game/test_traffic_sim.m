clear;clc;

veh_width = 2.5;
veh_length = 5.0;
lane_width = 4.0;

% ego start point
s0 = 0.0;
l0 = 0.0;
v0 = 20.0;

% target vehicle (potential cut in)
% the longer the vehicle, the likelihood for cut-in (makes sense) as
% follower role. As leader role, it is more aggressive.
% How to estimate role is a big question? Speed?
% For some of the parameter, it is possible that both role gives the same
% trajectories.
s1 = 15.0;
l1 = lane_width;

v1 = 20.0;


% road vehicle
% one on the other lane
s2 = 50.0;
l2 = lane_width;
v2 = 15.0;

% one on the ego lane

s3 = 80;
l3 = 0.0;
v3 = 15.0;
x_ini = [s0, v0, 0.0, s1, v1, l1, s2, v2, l2, s3, v3, l3]';

traffic = game.traffic_sim(x_ini);


pacc_solver = sim.PACC.SolvePACC();

traffic.reset(x_ini);
u = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8];
traffic.step(u);