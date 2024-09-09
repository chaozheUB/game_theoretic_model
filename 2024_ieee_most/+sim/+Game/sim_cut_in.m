function [results, params] = sim_cut_in(x_ini, varargin)
%% TODO May be also make this a class?
%% one can gradually add the parameters that want to vary
% the following ones are the main ones
defaults = {"ego_vehice_type", 1, ...
            "sigma", 0.6, ...
            "cut_in_vehicle_role", "Leader", ...
            "horizon", 5,...
            "consider_lc_abort", true, ...
            "total_time", 15.0};
options = Util.SetOptions(defaults, varargin);
%% traffic scenario simulation param

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
sim_param.ovm_params = ovm_params;

ego_ovm = @(s, v, s1, v1) game.OVM(s1 - s - ovm_params.l_veh, v, v1, ovm_params);

%% ego controller and simulation parameter set up
% [TODO] need a ego dynamic class
% 1 means ovm
% 2 means mpc
% 3 means mpc + consider cut-in but know the role
% 4 means mpc + consider cut-in probablitity
ego_vehice_type = options.ego_vehice_type;
ego_param.ego_vehice_type = ego_vehice_type;
ego_param.sigma = options.sigma; % for simplcity
ego_param.q = round(ego_param.sigma / sim_param.dt);

% needed for energy calcluation.
ego_param.ar = 1.47 * 0.1;
ego_param.cr = 2.75 * 1e-4;
ego_param.horizon = options.horizon;
ego_param.u_max = 2.0; % this is a global cap, should be large enough
ego_param.u_min = -4.0; % same as mpc game
ego_param.v_max = v_max;
ego_param.epsilon = 0.97;
ego_param.solver = "gurobi";
ego_param.infeasibility_type = "individual";

% pretty nasty that PACC umin is positive
solver = sim.PACC.SolvePACC("dt", sim_param.dt, ...
                            "sigma", ego_param.sigma, ...
                            "horizon", ego_param.horizon, ...
                            "infeasibility_type", ego_param.infeasibility_type, ...
                            "umin", abs(ego_param.u_min), ...
                            "vmax", ego_param.v_max, ...
                            "solver", ego_param.solver);

%% Cut-in vehicle game setup
delta_a_lc = 2;
delta_a_s = delta_a_lc * 2 / 3; % make it slightly smaller to make straight and lane change motion distinguished.

% role ("Leader", "Follower")
game_param.role = options.cut_in_vehicle_role;

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

% Initialize

roles_estimate_priors = [0.5; 0.5];

%% Traffic set up
assert(check_x_ini(x_ini, delta_a_s, true))
traffic = game.traffic_sim(x_ini, 'noise_W', game_param.noise_W, 'add_noise', 'True');

find_lead = @(s, l, s_others, l_others, v_others, id, v_ids) traffic.find_lead_generic(s, l, s_others, l_others, v_others, id, veh_width, veh_length, v_ids);

%% Set up simulation and results
%% misc constant for iteration
game_iter = 1;
game_frequency = round(game_param.execute_deltaT / sim_param.dt);

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
cut_game_stage_all = cell(1, simlen);

roles_estimate_priors_all = zeros(2, simlen);
roles_estimate_priors_all(:, 1) = roles_estimate_priors;
x1_predict_leader = @(t) NaN;
x1_predict_follower = @(t) NaN;


num_cut_game_output = length(1:game_frequency:simlen);
% actual size may be smaller
cut_game_output_flag = zeros(3, num_cut_game_output);
cut_game_output_all = cell(6, num_cut_game_output);

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

    % estimate role
    if cut_game.game_stage ~= "After" && ~any(isnan(x1_predict_leader(sim_time))) && ~any(isnan(x1_predict_follower(sim_time)))

        x_1_actual = x_all(4:6, kk);
        x1_errors = [x1_predict_leader(sim_time) - x_1_actual, ...
                     x1_predict_follower(sim_time) - x_1_actual];

        [role_post, ~] = cut_game.estimate_role(roles_estimate_priors_all(:, kk - 1), x1_errors);
        roles_estimate_priors_all(:, kk) = role_post;
        fprintf("Role estimation Leader %.2f, Follower %.2f. \n", role_post(1), role_post(2));
    else
        if kk > 1
            roles_estimate_priors_all(:, kk) = roles_estimate_priors_all(:, kk - 1);
        end
    end
    [cut_lead_s, cut_lead_v, cut_lead_id] = find_lead(cut_s, cut_l, cut_s_others, cut_l_others,  cut_v_others, 2, [1, 3:n_veh]);
    cut_game_stage_all{kk} = cut_game.game_stage;
    if cut_game.game_stage == "After"
        % the cut_game car following will have proper cap
        u_cut_long = cut_game.car_following([cut_s, cut_v], [cut_lead_s, cut_lead_v]);
        u_cut_lateral = max(-game_param.lateral_speed, min(- 20 * cut_l, 0.0));
        u_cut = [u_cut_long; u_cut_lateral];
    else      
        if mod(kk - 1, game_frequency) == 0
            cut_game_output_all{5, game_iter} = cut_game.game_stage;

            % [U, other_info] = cut_game.get_veh1_decision_from_sample(x_all(:, kk));
            % t_pred = sim_time + [cut_game.param.full_sample_time, cut_game.param.deltaT * cut_game.param.N + 1];

            [U, other_info] = cut_game.get_veh1_decision_from_u(x_all(:, kk));
            
            t_pred = sim_time + cut_game.param.deltaT * (0:cut_game.param.N + 1);
            cut_game_output_flag(1, game_iter) = sim_time;
            cut_game_output_flag(2, game_iter) = kk;

            cut_game_output_all{1, game_iter} = U;
            cut_game_output_all{2, game_iter} = other_info;

            s_predict_u1_as_follower = other_info.x1_sequence_u1_as_follower;
            s_predict_u1_as_leader = other_info.x1_sequence_u1_as_leader;
            cut_game_output_all{3, game_iter} = s_predict_u1_as_follower;
            cut_game_output_all{4, game_iter} = s_predict_u1_as_leader;
            
            if ~isequal(other_info.u1_sequence_u1_as_leader, other_info.u1_sequence_u1_as_follower)
                fprintf("leader follower sequence are different. \n");
                cut_game_output_flag(3, game_iter) = 1;
            end
            % also need to extend till next game_time_interval
            
            
            s_ext = [s_predict_u1_as_follower(1, end) + s_predict_u1_as_follower(2) * cut_game.param.deltaT;
                        s_predict_u1_as_follower(2:3, end)];
            s_cut_in_pred_as_follower = [t_pred;
                                            [x_all(4:6, kk), s_predict_u1_as_follower(1:3, :), s_ext]];

            s_ext = [s_predict_u1_as_leader(1, end) + s_predict_u1_as_leader(2) * cut_game.param.deltaT;
                        s_predict_u1_as_leader(2:3, end)];

            s_cut_in_pred_as_leader = [t_pred;
                                        [x_all(4:6, kk), s_predict_u1_as_leader(1:3, :), s_ext]];

            x1_predict_follower = @(t) [interp1(t_pred, s_cut_in_pred_as_follower(2, :), t);
                                        interp1(t_pred, s_cut_in_pred_as_follower(3, :), t);
                                        interp1(t_pred, s_cut_in_pred_as_follower(4, :), t)];
            
            x1_predict_leader = @(t) [interp1(t_pred, s_cut_in_pred_as_leader(2, :), t);
                                        interp1(t_pred, s_cut_in_pred_as_leader(3, :), t);
                                        interp1(t_pred, s_cut_in_pred_as_leader(4, :), t)];

            game_iter = game_iter + 1;

        end
        u_cut_sequence = U;
        % the following is OK ONLY when game interval is smaller than
        % game parameter deltaT
        u_cut = u_cut_sequence(:, 1);
        u_cut_all(:, :, kk) = u_cut_sequence;
        % put outside the game frequency because this could happen in
        % the middle.
    end
    cut_game.determine_stage(x_all(:, kk), u_cut);
    fprintf("Cut-in vehicle game stage %s. \n", cut_game.game_stage);    
    
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
            uplan = solver.compute_cmd(s, v, s1pred, u_ego_all(kk:kk + ego_param.q - 1));
            u_ego_all(kk + ego_param.q) = uplan(1 + ego_param.q);
        case 3
            % ideal chase, role is known.
            % mpc that considers a cut-in vehicle as game theoretic model
            % and know the leader role for certain
            [s1pred, v1pred] = solver.get_prediction(lead_s, lead_v);

            if cut_game.role == "Leader"
                s_cut_in_pred = s_cut_in_pred_as_leader;
            else
                s_cut_in_pred = s_cut_in_pred_as_follower;
            end
            % shift the first row because cut_game prediction may not run
            % at every step.
            s_cut_in_pred(1, :) =  s_cut_in_pred(1, :) - sim_time;
            if lead_id ~= 2
                [s1pred_update, ~] = solver.get_prediction_with_cut_in(cut_s, cut_v, s_cut_in_pred, s1pred, v1pred);
            else
                fprintf("veh 1 already cut-in will fall back to traditional prediction.\n");
                s1pred_update = s1pred;
            end

            uplan = solver.compute_cmd(s, v, s1pred_update, u_ego_all(kk:kk + ego_param.q - 1));
            u_ego_all(kk + ego_param.q) = uplan(1 + ego_param.q);
        case 4
            % based on role estimation
            [s1pred, v1pred] = solver.get_prediction(lead_s, lead_v);
            % first calculate by ignoring
            uplan_ignore = solver.compute_cmd(s, v, s1pred, u_ego_all(kk:kk + ego_param.q - 1));
            splan_ignore = solver.last_result{2};

            role_prop = roles_estimate_priors_all(:, kk);
            if role_prop(1) >= ego_param.epsilon
                % estimate to be leader
                s_cut_in_pred = s_cut_in_pred_as_leader;
                s_cut_in_pred(1, :) =  s_cut_in_pred(1, :) - sim_time;
            else
                if role_prop(2) >= ego_param.epsilon
                    % estimate to be follower
                    s_cut_in_pred = s_cut_in_pred_as_follower;
                    s_cut_in_pred(1, :) =  s_cut_in_pred(1, :) - sim_time;
                else
                    % both have posibilities
                    s_cut_in_pred = cat(3, s_cut_in_pred_as_leader, s_cut_in_pred_as_follower);
                    s_cut_in_pred(1, :, 1) =  s_cut_in_pred(1, :, 1) - sim_time;
                    s_cut_in_pred(1, :, 2) =  s_cut_in_pred(1, :, 2) - sim_time;
                end
            end
            % shift the first row because cut_game prediction may not run
            % at every step.
            
            if cut_game.game_stage ~= "After"
                [s1pred_update, ~, ignored] = solver.get_prediction_with_cut_in(cut_s, cut_v, s_cut_in_pred, splan_ignore, s1pred, v1pred);
            else
                fprintf("cut-in game finished will fall back to traditional prediction.\n");
                % and not need to recalculate
                ignored = true;
                s1pred_update = s1pred;
            end
            if ~ignored
                uplan = solver.compute_cmd_with_prob(s, v, {s1pred_update, role_prop}, u_ego_all(kk:kk + ego_param.q - 1));
            else
                fprintf("Predicted that cut-in will happen from the back, ignore the cut-in vehicle")
                uplan = uplan_ignore;
            end
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

% cap to proper length
cut_game_output_flag = cut_game_output_flag(:, 1:game_iter - 1);
cut_game_output_all = cut_game_output_all(:, 1:game_iter - 1);


%% calculate energy (both ego and cut-in)
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
results.cut_game_stage_all = cut_game_stage_all;
results.roles_estimate_priors_all = roles_estimate_priors_all;

% actual size may be smaller
results.cut_game_output_flag = cut_game_output_flag;
results.cut_game_output_all = cut_game_output_all;

% also save the two function handles, but may not be reused should the function class changed.
results.solver = solver;
results.cut_game = cut_game;
%% Also collect parameters
params.sim_param = sim_param;
params.game_param = game_param;
params.ego_param = ego_param;
end
