clc;clear; close all;
Util.plot_setup();
%% system dynamics and reward
% matrix of all vehicle center, with respect to ego
veh_width = 2.5;
veh_length = 5.0;
lane_width = 4.0;


% reward terms related
veh_dim.veh_width = veh_width;
veh_dim.veh_length = veh_length;
veh_dim.veh_d = veh_length / 3;
veh_dim.lane_width = lane_width;
v_max = 30;


delta_a = 2;
game_param.deltaT = 1;
% ovm simulation, because the ovm parameter is chosen for dt = 0.1
% if too large it may not be stable.
game_param.sample_dt = 0.1; 
game_param.N = 5;
game_param.u1_lane_change_step = 2;
assert(game_param.N > game_param.u1_lane_change_step)
lateral_speed_step = lane_width / game_param.u1_lane_change_step;
game_param.role = "Leader";

% col = action dimension
% row = num of actions
game_param.u0_action_set = [0.0; delta_a; -delta_a];
game_param.u1_action_set = [0.0, 0.0;
                           delta_a, 0.0;  
                           -delta_a, 0.0; 
                           0.0, lateral_speed_step; 
                           0.0, -lateral_speed_step;];
% only consider acceleration before lane change.
% sub sections: longitudinal motion + consecutive lane change
game_param.u1_action_set_long = [0.0, 0.0;
                                delta_a * 2 / 3, 0.0;  
                                -delta_a * 2 / 3, 0.0;];
game_param.u1_action_lc_sub_long = [0.0, 0.0;
                                   delta_a, 0.0;  
                                   -delta_a, 0.0;];
game_param.u1_action_lc_sub_seq = [0.0; -lane_width / game_param.u1_lane_change_step] * ones(1, game_param.u1_lane_change_step);

% game_param.n_other_vehicles = 2;

% for sample
game_param.u1_action_before_lc = [0, 0.0;
                                  delta_a, 0.0;];

% state constraints
game_param.v_max = v_max;
game_param.l_max = lane_width;
game_param.l_min = 0.0;
game_param.lane_width = lane_width;

% noise level
game_param.noise_W = eye(12, 12) * 0.1;

% OVM parameters
ovm_params.alpha = 0.4;
ovm_params.beta = 0.5;
ovm_params.kappa = 0.6;
ovm_params.delay = 0.0;
ovm_params.vmax = v_max;
ovm_params.hst = 5;
ovm_params.hgo = ovm_params.hst + ovm_params.vmax / ovm_params.kappa;
ovm_params.l_veh = veh_length;
game_param.ovm_params = ovm_params;

game_param.u_max = 2.0; % this is a global cap, should be large enough
game_param.u_min = -4.0; % same as mpc game
%% reward
% need to tune, collision, liveness pos, liveness lateral (when lane
% chaange) and effort.
game_param.reward.w_collision = 400;
game_param.reward.w_live_pos = 1;
game_param.reward.w_live_lateral = 40; % motivates cut-in
game_param.reward.w_live_speed = 0.0;
game_param.reward.w_headway = 5.0;
game_param.reward.w_effort = 0.1;

game_param.reward.v_desired = v_max;
game_param.reward.T_desired = 1.0;
game_param.reward.veh_dim = veh_dim;
% For finite horizon it doesn't really matter, but choose value smaller
% than 1
% Need to have an incentive, if in the middle of the lane, finish the lane
% change is the goal.
game_param.reward.lambda = 0.9;
game_param.reward.lambda_sample = 0.99; % because sample step is much smaller
%% Initialize
obj = game.CutInGame(game_param);

%% Scenario 1 at the beginning of cut 1
% only involves 4 vehicles.
% ego start point
s0 = 0.0;
l0 = 0.0;
v0 = 20.0;

% target vehicle (potential cut in)
% the further ahead the likelihood for cut-in
% for n=5
% 35 leader folower same cutin action
% 30 25 20 leader will cut in follower will not
% 15 leader will cut-in after
% for n = 4
% 
s1 = 25;
l1 = lane_width;
v1 = 16.0;


% road vehicle
% one on the cut-in vehicle lane, originally in front of ego
s2 = s1 + 30;
l2 = lane_width;
v2 = 16.0;

% one on the ego lane
s3 = s0 + 100;
l3 = 0.0;
v3 = 16.0;


x_ini = [s0; v0; 0.0; s1; v1; l1; s2; v2; l2; s3; v3; l3];
% assert(check_x_ini(x_ini, delta_a, true))
% % quick test
% u0_sequence = obj.u0_tensor(:, :, 2);
% u1_sequence = obj.u1_tensor(:, :, 2);
% s_trace = obj.get_state_trace(x_ini, u0_sequence, u1_sequence);
% s_other_trace = obj.get_others_trace(x_ini);
% [r_bar, x_sequence] = obj.get_reward(x_ini, u0_sequence, u1_sequence);


% tic;
% [Rbar0, Rbar1, all_trajectories] = obj.calculate_full_Rbar_from_u_simplify(x_ini, obj.u0_tensor, obj.u1_tensor, obj.n_s);
[~, other_info] = obj.get_veh1_decision_from_u(x_ini);
[leader_sample, follower_sample] = obj.convert_result_to_sample(other_info);
obj.plot_sample(x_ini, leader_sample, "title", "leader");
obj.plot_sample(x_ini, follower_sample, "title", "follower");


% u0_sequence_u1_as_follower = other_info.u0_sequence_u1_as_follower;
% u1_sequence_u1_as_follower = other_info.u1_sequence_u1_as_follower;
% u0_sequence_u1_as_leader = other_info.u0_sequence_u1_as_leader;
% u1_sequence_u1_as_leader = other_info.u1_sequence_u1_as_leader;
% 
% toc;
% obj.plot_sequence(x_ini, ...
%                   u0_sequence_u1_as_follower, ...
%                   u1_sequence_u1_as_follower, ...
%                   "title", "Before lane change, Follower");
% 
% [~, x_all] = obj.plot_sequence(x_ini, ...
%                   u0_sequence_u1_as_leader, ...
%                   u1_sequence_u1_as_leader, ...
%                   "title", "Before lane change, Leader");

% samples = obj.sample_trajectories(x_ini);
% [Rbar0, Rbar1] = obj.calculate_full_Rbar_from_samples(x_ini, samples);
% 
% [u1, other_info] = get_veh1_decision_from_sample(obj, x_ini);
% [leader_sample, follower_sample] = obj.convert_result_to_sample(other_info);
% close all;
% obj.plot_sample(x_ini, leader_sample, "title", "leader");
% obj.plot_sample(x_ini, follower_sample, "title", "follower");
% %%
% x_ini = [s0; v0; 0.0; s1; v1; l1/2; s2; v2; l2; s3; v3; l3];
% obj.game_stage = "During";
% samples = obj.sample_trajectories(x_ini);
% [Rbar0, Rbar1] = obj.calculate_full_Rbar_from_samples(x_ini, samples);
% 
% [u1, other_info] = get_veh1_decision_from_sample(obj, x_ini);
% [leader_sample, follower_sample] = obj.convert_result_to_sample(other_info);
% 
% obj.plot_sample(x_ini, leader_sample, "title", "During leader");
% obj.plot_sample(x_ini, follower_sample, "title", "During follower");