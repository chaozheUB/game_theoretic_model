function [results, params] = sim_no_cut_in(x_ini, varargin)
defaults = {"ego_vehice_type", 1, ...
            "sigma", 0.6, ...
            "horizon", 5,...
            "consider_lc_abort", true, ...
            "total_time", 15.0,...
            "considered_delay", 0.6};
options = Util.SetOptions(defaults, varargin);

veh_width = 2.5;
veh_length = 5.0;
lane_width = 4.0;
v_max = 30;

sim_param.dt = 0.1;
sim_param.total_time = options.total_time;
sim_param.x_ini = x_ini;
sim_param.lane_width = lane_width;
sim_param.v_max = v_max;
n_veh = length(x_ini) / 3;

veh_dim.veh_width = veh_width;
veh_dim.veh_length = veh_length;
veh_dim.veh_d = veh_length / 3;
veh_dim.lane_width = lane_width;

% This OVM is used by both ego ovm and cut in game.
ovm_params.alpha = 0.4;
ovm_params.beta = 0.5;
ovm_params.kappa = 0.6;
% ovm_params.delay = ego_param.sigma;
ovm_params.vmax = v_max;
ovm_params.hst = 5;
ovm_params.hgo = ovm_params.hst + ovm_params.vmax / ovm_params.kappa;
ovm_params.l_veh = veh_length;

ego_ovm = @(s, v, s1, v1) game.OVM(s1 - s - ovm_params.l_veh, v, v1, ovm_params);

%% ego controller and simulation parameter set up
% [TODO] need a ego dynamic class
% 1 means ovm
% 2 means mpc
ego_vehice_type = options.ego_vehice_type;
ego_param.ego_vehice_type = ego_vehice_type;
ego_param.sigma = options.sigma; % for simplcity
ego_param.q = round(ego_param.sigma / sim_param.dt);

% needed for energy calcluation.
ego_param.ar = 1.47 * 0.1;
ego_param.cr = 2.75 * 1e-4;
ego_param.horizon = 5;
ego_param.u_max = 2.0; % this is a global cap, should be large enough
ego_param.u_min = -4.0; % same as mpc game
ego_param.v_max = v_max;
ego_param.epsilon = 0.97;
ego_param.solver = "gurobi";
ego_param.infeasibility_type = "individual";
ego_param.considered_delay = options.considered_delay;

% pretty nasty that PACC umin is positive
ego_param.q_considered = round(ego_param.considered_delay / sim_param.dt);
solver = sim.PACC.SolvePACC("dt", sim_param.dt, ...
                            "sigma", ego_param.considered_delay, ...
                            "horizon", ego_param.horizon, ...
                            "infeasibility_type", ego_param.infeasibility_type, ...
                            "umin", abs(ego_param.u_min), ...
                            "vmax", ego_param.v_max, ...
                            "solver", ego_param.solver);


%% Cut-in vehicle game setup
delta_a_lc = 2;
delta_a_s = delta_a_lc * 2 / 3; % make it slightly smaller to make straight and lane change motion distinguished.

% % role ("Leader", "Follower")
% does not matter because won't use it.
% game_param.role = options.cut_in_vehicle_role;

game_param.deltaT = 1;
% ovm simulation, because the ovm parameter is chosen for dt = 0.1
% if too large it may not be stable.

game_param.N = round(ego_param.horizon / game_param.deltaT);
game_param.u1_lane_change_step = 2;
assert(game_param.N > game_param.u1_lane_change_step);
lateral_speed_step = lane_width / game_param.u1_lane_change_step;
game_param.lateral_speed = lane_width / game_param.u1_lane_change_step / game_param.deltaT;
% col = action dimension
% row = num of actions
game_param.u0_action_set = [0.0; delta_a_lc; -delta_a_lc];
game_param.u1_action_set = [0.0, 0.0;
                           delta_a_lc, 0.0;  
                           -delta_a_lc, 0.0; 
                           0.0, lateral_speed_step; 
                           0.0, -lateral_speed_step;];
% only consider acceleration before lane change.
% sub sections: longitudinal motion + consecutive lane change
game_param.u1_action_set_long = [0.0, 0.0;
                                 delta_a_s, 0.0;  
                                -delta_a_s, 0.0;];
game_param.u1_action_lc_sub_long = [0.0, 0.0;
                                   delta_a_lc, 0.0;  
                                   -delta_a_lc, 0.0;];
game_param.u1_action_lc_sub_seq = [0.0; -lane_width / game_param.u1_lane_change_step] * ones(1, game_param.u1_lane_change_step);

% for lane change abort, only do keep speed or reduce speed
game_param.u1_action_abort_lc_sub_long = [0.0, 0.0;
                                         -delta_a_lc, 0.0;];
game_param.u1_action_abort_lc_sub_seq = game_param.u1_action_lc_sub_seq;

game_param.consider_lc_abort = options.consider_lc_abort;

% for trajectory sample
game_param.u1_action_before_lc = [0, 0.0;
                                  delta_a_lc, 0.0;];
game_param.sample_dt = sim_param.dt; % use the same frequence as simulation.


% state constraints
game_param.v_max = v_max;
% not this is at the lane center so at most one to the left.
game_param.l_max = lane_width;
game_param.l_min = 0.0;
game_param.lane_width = lane_width;

% noise level
game_param.noise_W = zeros(12);
% only add significal noise level to
game_param.noise_W(4, 4) = 0.002;
game_param.noise_W(5, 5) = 0.001;
game_param.noise_W(6, 6) = 0.0002;

game_param.u_max = ego_param.u_max; % this is a global cap, should be large enough
game_param.u_min = ego_param.u_min; % same as mpc

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

% game decision frequency 
game_param.execute_deltaT = 0.5; % [sec]
game_param.ovm_params = ovm_params;

assert(game_param.deltaT > game_param.execute_deltaT, "game_time_interval should be smaller than game base time delta T")

cut_game = game.CutInGame(game_param);

%% Traffic set up
assert(check_x_ini(x_ini, delta_a_s, true))
traffic = game.traffic_sim(x_ini, 'noise_W', game_param.noise_W, 'add_noise', 'True');

find_lead = @(s, l, s_others, l_others, v_others, id, v_ids) traffic.find_lead_generic(s, l, s_others, l_others, v_others, id, veh_width, veh_length, v_ids);

%% Set up simulation and results
%% misc constant for iteration

tsim = (0:sim_param.dt:sim_param.total_time)';
simlen = length(tsim);
x_all = zeros(length(x_ini), simlen);
x_all(:, 1) = x_ini;

u_all = zeros(n_veh * 2, simlen - 1);
ego_state = zeros(10, simlen);
cut_state = zeros(10, simlen);
% u_ego is special because it may have delay
u_ego_all = zeros(simlen - 1 + ego_param.q, 1);
u_ego_all_uncapped = zeros(simlen - 1 + ego_param.q, 1);
% u_cut_all = zeros(2, cut_game.param.full_sample_len, simlen);
u_cut_all = zeros(2, cut_game.param.N, simlen);
u_limit = zeros(2, simlen);


%% simulate
for kk = 1 : simlen - 1
    sim_time = tsim(kk);
    % get background vehicle action, they are "replayed" no interaction
    u_all_others = zeros(2 * (n_veh - 2) , 1);

    tic;
    %% get cut-in vehicle action
    % [TODO] having a simple model for it as it may use game-theoretic
    % decision making.
    % cut in at some time and after that also do OVM
    tic;
    cut_s = x_all(4, kk);
    cut_v = x_all(5, kk);
    cut_l = x_all(6, kk);

    % % game-theoretic 
    cut_s_others = [x_all(1, kk); x_all(7:3:end, kk)];
    cut_v_others = [x_all(2, kk); x_all(8:3:end, kk)];
    cut_l_others = [x_all(3, kk); x_all(9:3:end, kk)];

    [cut_lead_s, cut_lead_v, cut_lead_id] = find_lead(cut_s, cut_l, cut_s_others, cut_l_others,  cut_v_others, 2, [1, 3:n_veh]);

    
    u_cut_long = cut_game.car_following([cut_s, cut_v], [cut_lead_s, cut_lead_v]);
    u_cut_lateral = 0.0;
    u_cut = [u_cut_long; u_cut_lateral];
    
    cut_state(:, kk) = [sim_time; 
                        cut_s; cut_v; cut_l;
                        cut_lead_s;cut_lead_v; cut_lead_id; cut_lead_s - cut_s - ovm_params.l_veh; 
                        u_cut(1); u_cut(2)];
    time_cut = toc;

    %% get ego vehicle action
    % for simplicity use a simple OVM model
    s = x_all(1, kk);
    v = x_all(2, kk);
    l = x_all(3, kk);
    [umin, umax] = solver.get_control_limit(v);
    u_limit(:, kk) = [umin; umax];
    % find lead vehicle
    s_others = x_all(4:3:end, kk);
    v_others = x_all(5:3:end, kk);
    l_others = x_all(6:3:end, kk);

    [lead_s, lead_v, lead_id] = find_lead(s, l, s_others, l_others, v_others, 1, 2:n_veh);
    switch ego_vehice_type
        case 1
            % use OVM to do car following, no lane change
            u_ego_all(kk + ego_param.q) = ego_ovm(s, v, lead_s, lead_v);
            u_ego_all(kk + ego_param.q) = max(umin, min(u_ego_all(kk + ego_param.q), umax));
            if u_ego_all_uncapped(kk + ego_param.q) ~= u_ego_all(kk + ego_param.q)
                fprintf("ego control is capped. \n");
            end    
        case 2
            % use standard MPC controller that does not account for cut-in
            % TODO decouple prediction and compute command
            % for now only predict using the current state (no history)
            [s1pred, ~] = solver.get_prediction(lead_s, lead_v);
            if lead_id ~= 2
                fprintf("no cut in yet. \n");
            else
                fprintf("veh 1 already cut-in.\n");
            end
            % mpc may not consider delay
            uplan = solver.compute_cmd(s, v, s1pred, u_ego_all(kk + ego_param.q - ego_param.q_considered:kk + ego_param.q - 1));            
            u_ego_all(kk + ego_param.q) = uplan(1 + ego_param.q);
        otherwise
            fprintf("Fault.\n");
    end

    u_ego_all_uncapped(kk + ego_param.q) = u_ego_all(kk + ego_param.q);

    % regardless of controller, deal with delay
    if kk <= ego_param.q
        % may be the following? 
        u_execute = u_ego_all(kk) + (ego_param.ar + ego_param.cr * x_all(2, 1).^2);
        % this was what's there before in the TRC paper implementation.
        % Basically no compensation
        % u_compensated = u_ego_all(kk);
    else
        % because at this control can only be for compensation at at
        % old states.
        u_execute = u_ego_all(kk) + (ego_param.ar + ego_param.cr * x_all(2, kk - ego_param.q).^2);
    end
    
    u_ego = [u_execute; 0.0];
    ego_state(:, kk) = [sim_time; 
                    s; v; l;
                    lead_s; lead_v; lead_id; lead_s - s - ovm_params.l_veh;
                    u_ego(1); u_ego(2)];  
    time_ego = toc;
    

    %% run traffic sim one step (potentiall contains noise)
    
    u_all(:, kk) = [u_ego; u_cut; u_all_others];
    traffic.step(u_all(:, kk));
    x_all(:, kk + 1) = traffic.x;

    fprintf('step %i, ego time %.4f, cut veh time %.4f \n', kk, time_ego, time_cut);
end

%% calculate energy (both ego and cut-in
if ego_param.q == 0
    a_ego = [u_ego_all;u_ego_all(end)];
else
    a_ego = u_ego_all(1: simlen);
end
v_ego = x_all(2, :);
w_tmp = max(a_ego(:) + ego_param.ar + ego_param.cr * v_ego(:).^2, 0) .* v_ego(:);
w_ego = trapz(tsim, w_tmp);
w_ego = w_ego * 0.001;

a_cut = [u_all(3, :), u_all(3, end)];
v_cut = x_all(5, :);
w_tmp = max(a_cut(:)).*v_cut(:);
w_cut = trapz(tsim, w_tmp);
w_cut = w_cut * 0.001;
fprintf("total energy ego %.4f cut-in veh %.4f\n", w_ego, w_cut);
%% collect results

results.w_ego = w_ego;
results.w_cut = w_cut;
results.tsim = tsim;

results.x_all = x_all;
results.u_all = u_all;
results.ego_state = ego_state;
results.cut_state = cut_state;
% u_ego is special because it may have delay
results.u_ego_all = u_ego_all;
results.u_ego_all_uncapped = u_ego_all_uncapped;
results.u_cut_all = u_cut_all;
results.u_limit = u_limit;

% also save the two function handles, but may not be reused should the function class changed.
results.solver = solver;
results.cut_game = cut_game;
%% Also collect parameters
params.sim_param = sim_param;
params.game_param = game_param;
params.ego_param = ego_param;
end
