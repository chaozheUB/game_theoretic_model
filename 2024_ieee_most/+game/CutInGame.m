classdef CutInGame < handle
    % This is a cut-in game interactions
    % It involves 4 vehicles
    % veh 0 is the ego vehicle, stay in lane
    % veh 1 is the cut-in vehicle, can change lane to the ego lane
    % veh 2 is the slow vehicle travel in from of veh 1
    % veh 3 is the slow vehicle travel in from of veh 0
    % veh 2 and 3 are traveling with constant speed, mimicing the slow
    % traffic
    % veh 0 and veh 1 are making decisions using the game theoretic
    % framework.
    % handle class is better 
    % https://www.mathworks.com/help/matlab/matlab_oop/comparing-handle-and-value-classes.html
    properties
        param;  
        % role is considered as a state
        role; % enumerate, "Leader" or "Follower"
        u0_tensor;
        u1_tensor;
        % or compute lane change and straight one seperately
        % in this case straight ones only select from two actions.
        u1_tensor_before_lc; % sample based trajectory
        u1_tensor_lc;
        u1_tensor_abort_lc;
        u1_tensor_s;
        n_s;
        n_lc;
        % bruteforth
        u0_tensor_all;
        u1_tensor_all;
        % number of actions that involves lane change
        n_s_all;
        game_stage; % enumerate, lane_change_not_started, lane_change_started

    end
    methods
        function obj = CutInGame(param)
            % initialize parameters
            obj.param = param;

            % this ratio is used for reward calculation and remain the same as long as horizon does not change.
            ratio = zeros(obj.param.N, 1);

            for i = 1: obj.param.N
                ratio(i) = obj.param.reward.lambda ^ (i - 1);
            end
            
            obj.param.reward.ratio_vector = ratio;

            obj.param.full_sample_time = 0:obj.param.sample_dt:obj.param.deltaT * obj.param.N;
            obj.param.sample_steps = obj.param.deltaT / obj.param.sample_dt;
            obj.param.full_sample_len = round(obj.param.N * obj.param.sample_steps);
            assert(obj.param.full_sample_len == length(obj.param.full_sample_time) - 1);

            ratio_sample = zeros(obj.param.full_sample_len, 1);

            for i = 1: obj.param.full_sample_len
                ratio_sample(i) = (obj.param.reward.lambda_sample) ^ (i - 1) ;
            end
            obj.param.reward.ratio_sample_vector = ratio_sample;
            if isfield(obj.param, "role")
                obj.assign_role(obj.param.role);
            else
                fprintf("No role given, default will be Follower.\n")
                obj.assign_role("Follower");
            end
            
            obj.game_stage = "Before";
            obj.initialize_action_sequence();
            fprintf("Cut game is initialized.\n");
        end
        function assign_role(obj, new_role)
            if new_role == "Leader"
                fprintf("This vehicle assumes Leader role.\n");
            else 
                if new_role == "Follower"
                    fprintf("This vehicle assumes Follower role.\n");
                else
                    fprintf("wrong role, either Leader or Follower, defualt to Follower role.\n");
                    new_role = "Follower";
                end
            end
            obj.role = new_role;
        end
        function determine_stage(obj, x, u1)
            % should be called at every step in the simulation
            l1 = x(6);
            if obj.game_stage == "Before" && u1(2) < 0
                % lane change started
                obj.game_stage = "During";
            else
                % [CRH] hard coded to avoid noise stage
                if obj.game_stage == "During" && l1 <= obj.param.lane_width / 4
                    % consider finish and end the cut-in game
                    % reached the target lane
                    obj.game_stage = "After";
                else
                    if obj.game_stage == "During" && l1 >= obj.param.l_max - sqrt(obj.param.noise_W(6, 6))
                    % go back to original lane, allow traveling in the lane decision.
                        obj.game_stage = "Before";
                    end
                end
            end
        end
        function initialize_action_sequence(obj)
            % For discrete action based.
            % the action sequence is indepedent of states.
            % prefered way should be generate just a subsects
            if isfield(obj.param, "u0_action_set")
                fprintf("Initialize u0 actions sets with all possible combinations.\n");
                obj.u0_tensor = obj.generate_all_sequences(obj.param.u0_action_set, obj.param.N, "u0");
            end
                
            fprintf("Initialize with actions sets using subspace search.\n");
            % for lane change generate it with sub speace
            sub_horizon = obj.param.N - obj.param.u1_lane_change_step;
            
            % note, this sub long action could be different from the u1_action_s, e.g., different delta_a
            % u1_tensor_sub = obj.generate_all_sequences(obj.param.u1_action_set_long, sub_horizon, "u1 sub");
            u1_tensor_sub = obj.generate_all_sequences(obj.param.u1_action_lc_sub_long, sub_horizon, "u1 lc long sub");
            obj.u1_tensor_lc = zeros(size(u1_tensor_sub, 1), obj.param.N, size(u1_tensor_sub, 1) * (sub_horizon + 1));
            for j = 1:size(u1_tensor_sub, 3)
                seq = u1_tensor_sub(:, :, j);
                seq_ext = zeros(2, obj.param.N, sub_horizon + 1);
                for i = 1:sub_horizon + 1
                    seq_before = seq(:, 1:i-1);
                    seq_after = seq(:, i:end);
                    seq_ext(:, :, i) = [seq_before, obj.param.u1_action_lc_sub_seq, seq_after];
                end
                obj.u1_tensor_lc(:, :, ((j-1) * (sub_horizon + 1) + (1:sub_horizon + 1))) = seq_ext;
            end

            if isfield(obj.param, "u1_action_abort_lc_sub_long")
                u1_tensor_sub = obj.generate_all_sequences(obj.param.u1_action_abort_lc_sub_long, sub_horizon, "u1 lc abort sub");
                obj.u1_tensor_abort_lc = zeros(size(u1_tensor_sub, 1), obj.param.N, size(u1_tensor_sub, 1) * (sub_horizon + 1));
                for j = 1:size(u1_tensor_sub, 3)
                    seq = u1_tensor_sub(:, :, j);
                    seq_ext = zeros(2, obj.param.N, sub_horizon + 1);
                    for i = 1:sub_horizon + 1
                        seq_before = seq(:, 1:i-1);
                        seq_after = seq(:, i:end);
                        % remark need to flip the sigh of lc sub seq
                        % because it is changing back.
                        seq_ext(:, :, i) = [seq_before, -obj.param.u1_action_lc_sub_seq, seq_after];
                    end
                    obj.u1_tensor_abort_lc(:, :, ((j-1) * (sub_horizon + 1) + (1:sub_horizon + 1))) = seq_ext;
                end
            else
                obj.u1_tensor_abort_lc = [];
            end
            % for straight generate also with sub space, save
            % seperately because for these actions, one do not need to
            % search with 
            obj.u1_tensor_s = obj.generate_all_sequences(obj.param.u1_action_set_long, obj.param.N, "u1 long only");
            % put togather but not really want to use it, will do it
            % seperately
            obj.u1_tensor = cat(3, obj.u1_tensor_s, obj.u1_tensor_lc, obj.u1_tensor_abort_lc);
            obj.n_s = size(obj.u1_tensor_s, 3);
            obj.n_lc = size(obj.u1_tensor_lc, 3);
            fprintf("%s is generated based on sub action set: straight only %.d, lane changes %d, and lane change abort %.d.\n", ...
                "u1", size(obj.u1_tensor_s, 3), size(obj.u1_tensor_lc, 3), size(obj.u1_tensor_abort_lc, 3));

            % % generate all combinations
            % % prefer not to use, but still generate for validation purpose
            if isfield(obj.param, "u1_action_set")
                fprintf("Also initialize with actions sets using brute force search.\n");
                % need to order u1_tensor_all such that some involves lane
                % changes, other do not.
                % for some initial conditions, the non-lane change ones
                % involves interaction, so should keep going.
                obj.u1_tensor_all = obj.generate_all_sequences(obj.param.u1_action_set, obj.param.N, "u1");
                [u1_tensor_all_sorted, ns] = obj.sort_u1_tensor(obj.u1_tensor_all);
                obj.u1_tensor_all = u1_tensor_all_sorted;
                obj.n_s_all = ns;
                obj.u0_tensor_all = obj.u0_tensor;
            end

            % For sample based, this is only to sample the steps before the lane
            % changes, during and after lane change, it will be by OVM
            % sub_horizon = obj.param.N - obj.param.u1_lane_change_step;
            % it is possible that we only do one step lane change step
            n_action = size(obj.param.u1_action_before_lc, 1);
            obj.u1_tensor_before_lc = cell(1, obj.param.N - 1);
            max_u1_lc_seqs = 0;
            max_u1_lc_seqs_check = 0;
            for i = 1:obj.param.N - 1
                obj.u1_tensor_before_lc{i} = obj.generate_all_sequences(obj.param.u1_action_before_lc, ...
                                             i, sprintf("u1_before_lc at %d ", i + 1));
                assert(isequal(size(obj.u1_tensor_before_lc{i}), [2, i, n_action ^ i]));
                max_u1_lc_seqs = max_u1_lc_seqs + n_action ^ (i);
                max_u1_lc_seqs_check = max_u1_lc_seqs_check + size(obj.u1_tensor_before_lc{i}, 3);
            end
            assert(max_u1_lc_seqs_check == max_u1_lc_seqs)
            fprintf("Sample based trajectory: total num of lane change actions are %d. \n", max_u1_lc_seqs);
        end
        function samples = sample_trajectories(obj, x_ini)
            % sample trajectories from the current initial condition
            % instead of using discrete action space.
            % the sample 
            sample_dt = obj.param.sample_dt;
            sample_steps = obj.param.sample_steps;
            full_sample_len =obj.param.full_sample_len;
            samples = NaN;

            % A baseline trajectory, that every one assume
            x_veh0 = x_ini(1:3);
            % for veh2 and 3 they are assumed to be constant always.
            x_veh2 = x_ini(7:9);
            x_veh3 = x_ini(10:12);

            veh0_state_trace_straight = zeros(3, full_sample_len);
            veh0_u_trace_straight = zeros(1, full_sample_len);
            veh2_state_sequence = zeros(3, full_sample_len);
            veh3_state_sequence = zeros(3, full_sample_len);
            
            for i = 1:full_sample_len

                % veh 0 car following with ovm
                u_veh0 = obj.car_following(x_veh0, x_veh3);
                x_veh0 = obj.vehicle_dynamics(x_veh0, [u_veh0; 0.0], sample_dt);

                
                % veh 2 veh 3 maintain constant speed
                x_veh2 = obj.vehicle_dynamics(x_veh2, [0.0; 0.0], sample_dt);                        
                x_veh3 = obj.vehicle_dynamics(x_veh3, [0.0; 0.0], sample_dt);
                
                veh0_state_trace_straight(:, i) = x_veh0;
                veh0_u_trace_straight(:, i) = u_veh0;
                
                veh2_state_sequence(:, i) = x_veh2;
                veh3_state_sequence(:, i) = x_veh3;
            end

            switch obj.game_stage
                case "Before"
                    tic;
                    % most intensed case, consider all possible future
                    num_of_seqences = 0; % straight
                    max_step_before_lc = obj.param.N - obj.param.u1_lane_change_step;
                    for i = 0: max_step_before_lc
                        if i == 0
                            u_before_lc_all = zeros(2, 0, 1);
                        else
                            u_before_lc_all = obj.u1_tensor_before_lc{i};
                        end
                        num_of_seqences = num_of_seqences +  size(u_before_lc_all, 3);
                    end
                    fprintf("total of samples %d for veh0 (ego) and %d for veh1 (cut-in) \n.", num_of_seqences + 1, 2 * num_of_seqences + 1);
                    
                    veh0_state_sequences = zeros(3, full_sample_len, num_of_seqences + 1);
                    veh0_u_sequences = zeros(1, full_sample_len, num_of_seqences + 1);

                    veh1_state_sequences = zeros(3, full_sample_len, 2 * num_of_seqences + 1);
                    veh1_u_sequences = zeros(2, full_sample_len, 2 * num_of_seqences + 1);

                    % veh 1 is in the original lane
                    % option 1 could go straight according to ovm with respect to
                    % lead, and x2 x3 trace will not change.

                    % veh 0 follows its current lead using ovm, from
                    % baseline reference
                    % or act as if it ignores the cut in motion

                    x_veh1 = x_ini(4:6);
                    x_veh1_full = zeros(3, full_sample_len);
                    u_veh1_full = zeros(2, full_sample_len);
                    x_veh2 = x_ini(7:9);
                    for i = 1:full_sample_len
                        % veh 1 car following with ovm w.r.t veh2 in its
                        % lane.
                        u_veh1 = obj.car_following(x_veh1, x_veh2);
                        x_veh1 = obj.vehicle_dynamics(x_veh1, [u_veh1; 0.0], sample_dt);

                        x_veh1_full(:, i) = x_veh1;
                        u_veh1_full(:, i) = [u_veh1; 0.0];
                        x_veh2 = veh2_state_sequence(:, i);

                    end

                    obj.n_s = 1;
                    veh0_state_sequences(:, :, 1) = veh0_state_trace_straight;
                    veh1_state_sequences(:, :, 1) = x_veh1_full;
                    veh0_u_sequences(:, :, 1) = veh0_u_trace_straight;
                    veh1_u_sequences(:, :, 1) = u_veh1_full;

                    % quick check
                    % x_full = [x_ini(:), [veh0_state_trace_straight;x_veh1_full; veh2_state_sequence; veh3_state_sequence]];
                    % u_full = [veh0_u_trace_straight; u_veh1_full];
                    % u_veh1_full(:, 1:sample_steps:end)
                    % obj.plot_trajectory(obj.param.full_sample_time, x_full, u_full);
                    % obj.plot_one_topview(obj.param.full_sample_time, x_full);


                    % option 2
                    % veh1 can cut-in in front of veh0
                    % with all possibilities action discretization before
                    % or after

                    iter_of_sequences = 2;
                    for i = 0: max_step_before_lc
                        if i == 0
                            u_before_lc_all = zeros(2, 0, 1);
                        else
                            u_before_lc_all = obj.u1_tensor_before_lc{i};
                        end
                        num_u_before_lc_all = size(u_before_lc_all, 3);
                        
                        
                        for j = 1:num_u_before_lc_all
                            valid_sample = true;
                            % intialize the trajectory
                            x_veh0_full = zeros(3, full_sample_len);
                            u_veh0_full = zeros(1, full_sample_len);
    
                            x_veh1 = x_ini(4:6);
                            x_veh1_full = zeros(3, full_sample_len);
                            u_veh1_full = zeros(2, full_sample_len);
                            
                            % first go with u sequence before lane change
                            % get veh1 u before lane change
                            u1_before_lc = u_before_lc_all(:, :, j);
                            % expand to full
                            u1_before_lc_full_sim = kron(u1_before_lc, ones(1, sample_steps));
                            step_before_lc = size(u1_before_lc_full_sim, 2);
                            u_veh1_full(:, 1:step_before_lc) = u1_before_lc_full_sim;
                            for k = 1:step_before_lc
                                
                                x_veh1 = obj.vehicle_dynamics(x_veh1, u1_before_lc_full_sim(:, k), sample_dt);
                                x_veh1_full(:, k) = x_veh1;
                            end
                            x_veh0 = x_ini(1:3);
                            if step_before_lc > 0
                                % for veh0 no need to recalculate
                                u_veh0_full(:, 1:step_before_lc) = veh0_u_trace_straight(:, 1:step_before_lc);
                                x_veh0_full(:, 1:step_before_lc) = veh0_state_trace_straight(:, 1:step_before_lc);
                                x_veh0 = x_veh0_full(:, step_before_lc);
                            end
                            
                            % second do lane change
                            % this is ``Before" stage so always go for obj.param.u1_lane_change_step
                            lane_change_seq = [0; -obj.param.lane_width / obj.param.u1_lane_change_step] * ones(1, obj.param.u1_lane_change_step);
                            u1_lc_full_sim = kron(lane_change_seq, ones(1, sample_steps));
                            step_lc = size(u1_lc_full_sim, 2);
                            u_veh1_full(:, step_before_lc + 1:step_before_lc + step_lc) = u1_lc_full_sim;
                            for k = 1:step_lc
                                % for simplicity start to react to veh1 eariler
                                u_veh0 = obj.car_following(x_veh0, x_veh1);
                                x_veh0 = obj.vehicle_dynamics(x_veh0, [u_veh0; 0.0], sample_dt);
                                u_veh0_full(:, k + step_before_lc) = u_veh0;
                                x_veh0_full(:, k + step_before_lc) = x_veh0;

                                % vehicle 1 takes lane change
                                x_veh1 = obj.vehicle_dynamics(x_veh1, u1_lc_full_sim(:, k), sample_dt);
                                x_veh1_full(:, k + step_before_lc) = x_veh1;

                            end
                            % third do ovm with respect to the supposed
                            % lead Note here we ignore whether they collide
                            start_post_lc = step_before_lc + step_lc + 1; 
                            x_veh2 = veh2_state_sequence(:, step_before_lc + step_lc);
                            for k = start_post_lc:full_sample_len
                                % veh0 continue to follow veh1
                                u_veh0 = obj.car_following(x_veh0, x_veh1);
                                x_veh0 = obj.vehicle_dynamics(x_veh0, [u_veh0; 0.0], sample_dt);
                                u_veh0_full(:, k) = u_veh0;
                                x_veh0_full(:, k) = x_veh0;
                                % veh1 starts to follow veh 2 (remark, need
                                % to response to previous states.
                                u_veh1 = obj.car_following(x_veh1, x_veh2);
                                x_veh1 = obj.vehicle_dynamics(x_veh1, [u_veh1; 0.0], sample_dt);
                                u_veh1_full(:, k) = [u_veh1; 0.0];
                                x_veh1_full(:, k) = x_veh1;

                                % update veh2 from already generated list
                                x_veh2 = veh2_state_sequence(:, k);

                            end

                            if valid_sample
                                assert(iter_of_sequences <= num_of_seqences + 1);
                                veh0_state_sequences(:, :, iter_of_sequences) = x_veh0_full;
                                veh1_state_sequences(:, :, iter_of_sequences) = x_veh1_full;
                                veh0_u_sequences(:, :, iter_of_sequences) = u_veh0_full;
                                veh1_u_sequences(:, :, iter_of_sequences) = u_veh1_full;
    
                                % quick check plot
                                % x_full = [x_ini(:), [x_veh0_full;x_veh1_full; veh2_state_sequence; veh3_state_sequence]];
                                % u_full = [u_veh0_full; u_veh1_full];
                                % u_veh1_full(:, 1:sample_steps:end)
                                % obj.plot_trajectory(obj.param.full_sample_time, x_full, u_full);
                                % obj.plot_one_topview(obj.param.full_sample_time, x_full);
                                iter_of_sequences = iter_of_sequences + 1;
                            end
                        end
                    end
                    
                    assert(iter_of_sequences == num_of_seqences + 2);

                    % option 3
                    % veh1 can cut-in in behind veh0
                    % it needs to keep or reduce speed while veh0 maintain
                    % speed
                    for i = 0: max_step_before_lc
                        if i == 0
                            u_before_lc_all = zeros(2, 0, 1);
                        else
                            u_before_lc_all = obj.u1_tensor_before_lc{i};
                        end
                        num_u_before_lc_all = size(u_before_lc_all, 3);
                        
                        
                        for j = 1:num_u_before_lc_all
                            valid_sample = true;
                            % intialize the trajectory
                            x_veh1 = x_ini(4:6);
                            x_veh1_full = zeros(3, full_sample_len);
                            u_veh1_full = zeros(2, full_sample_len);
                            
                            % first go with u sequence before lane change
                            % get veh1 u before lane change
                            % only decelerate
                            u1_before_lc = - u_before_lc_all(:, :, j);
                            % expand to full
                            u1_before_lc_full_sim = kron(u1_before_lc, ones(1, sample_steps));
                            step_before_lc = size(u1_before_lc_full_sim, 2);
                            u_veh1_full(:, 1:step_before_lc) = u1_before_lc_full_sim;
                            for k = 1:step_before_lc
                                x_veh1 = obj.vehicle_dynamics(x_veh1, u1_before_lc_full_sim(:, k), sample_dt);
                                x_veh1_full(:, k) = x_veh1;
                            end
                            
                            % second do lane change
                            % this is ``Before" stage so always go for obj.param.u1_lane_change_step
                            lane_change_seq = [0; -obj.param.lane_width / obj.param.u1_lane_change_step] * ones(1, obj.param.u1_lane_change_step);
                            u1_lc_full_sim = kron(lane_change_seq, ones(1, sample_steps));
                            step_lc = size(u1_lc_full_sim, 2);
                            u_veh1_full(:, step_before_lc + 1:step_before_lc + step_lc) = u1_lc_full_sim;
                            for k = 1:step_lc
                                % vehicle 1 takes lane change
                                x_veh1 = obj.vehicle_dynamics(x_veh1, u1_lc_full_sim(:, k), sample_dt);
                                x_veh1_full(:, k + step_before_lc) = x_veh1;
                            end
                            % third do ovm with respect to the v0
                            % lead Note here we ignore whether they collide
                            start_post_lc = step_before_lc + step_lc + 1; 
                            x_veh0 = veh0_state_trace_straight(:, step_before_lc + step_lc);
                            for k = start_post_lc:full_sample_len
                                % veh1 starts to follow veh 0 (remark, need
                                % to response to previous states.
                                u_veh1 = obj.car_following(x_veh1, x_veh0);
                                x_veh1 = obj.vehicle_dynamics(x_veh1, [u_veh1; 0.0], sample_dt);
                                u_veh1_full(:, k) = [u_veh1; 0.0];
                                x_veh1_full(:, k) = x_veh1;

                                % update veh2 from already generated list
                                x_veh0 = veh0_state_trace_straight(:, k);

                            end

                            if valid_sample
                                assert(iter_of_sequences <= 2* num_of_seqences + 1);
                                veh1_state_sequences(:, :, iter_of_sequences) = x_veh1_full;
                                veh1_u_sequences(:, :, iter_of_sequences) = u_veh1_full;
    
                                % quick check plot
                                % x_full = [x_ini(:), [veh0_state_trace_straight;x_veh1_full; veh2_state_sequence; veh3_state_sequence]];
                                % u_full = [veh0_u_trace_straight; u_veh1_full];
                                % u_veh1_full(:, 1:sample_steps:end)
                                % obj.plot_trajectory(obj.param.full_sample_time, x_full, u_full);
                                % obj.plot_one_topview(obj.param.full_sample_time, x_full);
                                iter_of_sequences = iter_of_sequences + 1;
                            end
                        end
                    end
                    assert(iter_of_sequences == 2* num_of_seqences + 2);
                    time_used = toc;
                    samples = struct();
                    samples.veh0_state_sequences = veh0_state_sequences;
                    samples.veh1_state_sequences = veh1_state_sequences;
                    samples.veh0_u_sequences = veh0_u_sequences;
                    samples.veh1_u_sequences = veh1_u_sequences;
                    samples.veh2_state_sequence = veh2_state_sequence;
                    samples.veh3_state_sequence = veh3_state_sequence;
                    fprintf("Currently in `Before` lane change stage, get %d samples. Takes %.4f secs.\n", iter_of_sequences, time_used);

                case "During"
                    tic;
                    % may be during the lane change, the incentive was to
                    % finish the lane change. (update the reward) but still
                    % allows it to speed up and then change
                    % but it will not consider straight motions any more.

                    l1 = x_ini(6);
                    assert(l1 < obj.param.lane_width);
                    % based on l1 determine how many lane change sequence
                    % is needed
                    lateral_speed = obj.param.lane_width / obj.param.u1_lane_change_step;
                     % step needed to finish the lane change
                    step_left_to_cut_in = ceil(abs(l1 / lateral_speed / sample_dt));                   

                    num_of_seqences = 0; % straight
                    % the major differences between "Before and During"
                    max_step_before_lc = obj.param.N - ceil(abs(l1 / lateral_speed));
                    for i = 0: max_step_before_lc
                        if i == 0
                            u_before_lc_all = zeros(2, 0, 1);
                        else
                            u_before_lc_all = obj.u1_tensor_before_lc{i};
                        end
                        num_of_seqences = num_of_seqences +  size(u_before_lc_all, 3);
                    end

                    fprintf("total of %d samples for veh0 (ego) and %d for veh1 (cut-in) \n.", num_of_seqences + 1, 2 * num_of_seqences + 1);
                    % The ego still has the options to ignore cut-in
                    % consider the "worse case", it accelerate but not at
                    % the hardest but with some goal in mind
                    % store it at the end
                    veh0_state_sequences = zeros(3, full_sample_len, num_of_seqences + 1);
                    veh0_u_sequences = zeros(1, full_sample_len, num_of_seqences + 1);

                    veh0_state_sequences(:, :, 1) = veh0_state_trace_straight; 
                    veh0_u_sequences(:, :, 1) = veh0_u_trace_straight; 

                    
                    % with an abort sequence
                    veh1_state_sequences = zeros(3, full_sample_len, 2 * num_of_seqences + 1);
                    veh1_u_sequences = zeros(2, full_sample_len, 2 * num_of_seqences + 1);
                    


                    step_left_to_abort = ceil(abs((obj.param.lane_width - l1) / lateral_speed / sample_dt));
                    % The cut-in vehicle has the opiton to abort by lane
                    % first lane change back and start to follow old lead
                    x_veh1 = x_ini(4:6);
                    x_veh2 = x_ini(7:9);
                    u_veh1 = zeros(2, 1);

                    x_veh1_full = zeros(3, full_sample_len);
                    u_veh1_full = zeros(2, full_sample_len);                    
                    
                    for i = 1:full_sample_len
                        u_veh1(2) = 0.0;
                        if i <= step_left_to_abort
                            % go back to original lane
                            u_veh1(2) = lateral_speed;
                        end
                        % meanwhile start to react to veh2
                        u_veh1(1) = obj.car_following(x_veh1, x_veh2);
                        x_veh1 = obj.vehicle_dynamics(x_veh1, u_veh1, sample_dt);

                        x_veh1_full(:, i) = x_veh1;
                        u_veh1_full(:, i) = u_veh1;
                        x_veh2 = veh2_state_sequence(:, i);
                    end
                    % store abort trajectory at the end.
                    veh1_state_sequences(:, :, 1) = x_veh1_full; 
                    veh1_u_sequences(:, :, 1) = u_veh1_full; 
                    
                    % quick check
                    % x_full = [x_ini(:), [veh0_state_trace_straight;x_veh1_full; veh2_state_sequence; veh3_state_sequence]];
                    % u_full = [veh0_u_trace_straight; u_veh1_full];
                    % u_veh1_full(:, 1:sample_steps:end)
                    % obj.plot_trajectory(obj.full_sample_time, x_full, u_full);
                    % obj.plot_one_topview(obj.param.full_sample_time, x_full);

                    % option 2 continue cut-in in front of veh0
                    iter_of_sequences = 2;
                    for i = 0: max_step_before_lc
                        if i == 0
                            u_before_lc_all = zeros(2, 0, 1);
                        else
                            u_before_lc_all = obj.u1_tensor_before_lc{i};
                        end
                        num_u_before_lc_all = size(u_before_lc_all, 3);
                        
                        for j = 1:num_u_before_lc_all
                            valid_sample = true;
                            % intialize the trajectory
                            x_veh0_full = zeros(3, full_sample_len);
                            u_veh0_full = zeros(1, full_sample_len);
    
                            x_veh1 = x_ini(4:6);
                            x_veh1_full = zeros(3, full_sample_len);
                            u_veh1_full = zeros(2, full_sample_len);
                            
                            % first step go with u sequence before lane change
                            % get veh1 u before lane change
                            u1_before_lc = u_before_lc_all(:, :, j);
                            % expand to full
                            u1_before_lc_full_sim = kron(u1_before_lc, ones(1, sample_steps));
                            step_before_lc = size(u1_before_lc_full_sim, 2);
                            u_veh1_full(:, 1:step_before_lc) = u1_before_lc_full_sim;
                            for k = 1:step_before_lc
                                
                                x_veh1 = obj.vehicle_dynamics(x_veh1, u1_before_lc_full_sim(:, k), sample_dt);
                                x_veh1_full(:, k) = x_veh1;
                            end
                            x_veh0 = x_ini(1:3);
                            if step_before_lc > 0
                                % for veh0 no need to recalculate
                                u_veh0_full(:, 1:step_before_lc) = veh0_u_trace_straight(:, 1:step_before_lc);
                                x_veh0_full(:, 1:step_before_lc) = veh0_state_trace_straight(:, 1:step_before_lc);
                                x_veh0 = x_veh0_full(:, step_before_lc);
                            end
                            
                            % second step do lane change
                            % this is ``Before" stage so always go for obj.param.u1_lane_change_step
                            u1_lc_full_sim = [0; -lateral_speed] * ones(1, step_left_to_cut_in);

                            
                            u_veh1_full(:, step_before_lc + 1:step_before_lc + step_left_to_cut_in) = u1_lc_full_sim;
                            for k = 1:step_left_to_cut_in
                                % for simplicity start to react to veh1 eariler
                                u_veh0 = obj.car_following(x_veh0, x_veh1);
                                x_veh0 = obj.vehicle_dynamics(x_veh0, [u_veh0; 0.0], sample_dt);
                                u_veh0_full(:, k + step_before_lc) = u_veh0;
                                x_veh0_full(:, k + step_before_lc) = x_veh0;

                                % vehicle 1 takes lane change
                                x_veh1 = obj.vehicle_dynamics(x_veh1, u1_lc_full_sim(:, k), sample_dt);
                                x_veh1_full(:, k + step_before_lc) = x_veh1;

                            end
                            % third step do ovm with respect to the supposed
                            % lead Note here we ignore whether they collide
                            start_post_lc = step_before_lc + step_left_to_cut_in + 1; 
                            x_veh2 = veh2_state_sequence(:, step_before_lc + step_left_to_cut_in);
                            for k = start_post_lc:full_sample_len
                                % veh0 continue to follow veh1
                                u_veh0 = obj.car_following(x_veh0, x_veh1);
                                x_veh0 = obj.vehicle_dynamics(x_veh0, [u_veh0; 0.0], sample_dt);
                                u_veh0_full(:, k) = u_veh0;
                                x_veh0_full(:, k) = x_veh0;
                                % veh1 starts to follow veh 2 (remark, need
                                % to response to previous states.
                                u_veh1 = obj.car_following(x_veh1, x_veh2);
                                x_veh1 = obj.vehicle_dynamics(x_veh1, [u_veh1; 0.0], sample_dt);
                                u_veh1_full(:, k) = [u_veh1; 0.0];
                                x_veh1_full(:, k) = x_veh1;

                                % update veh2 from already generated list
                                x_veh2 = veh2_state_sequence(:, k);

                            end

                            if valid_sample
                                assert(iter_of_sequences <= num_of_seqences + 1);
                                veh0_state_sequences(:, :, iter_of_sequences) = x_veh0_full;
                                veh1_state_sequences(:, :, iter_of_sequences) = x_veh1_full;
                                veh0_u_sequences(:, :, iter_of_sequences) = u_veh0_full;
                                veh1_u_sequences(:, :, iter_of_sequences) = u_veh1_full;
    
                                % quick check plot
                                % x_full = [x_ini(:), [x_veh0_full;x_veh1_full; veh2_state_sequence; veh3_state_sequence]];
                                % u_full = [u_veh0_full; u_veh1_full];
                                % u_veh1_full(:, 1:sample_steps:end)
                                % obj.plot_trajectory(obj.full_sample_time, x_full, u_full);
                                % obj.plot_one_topview(obj.param.full_sample_time, x_full);
                                iter_of_sequences = iter_of_sequences + 1;
                            end
                        end
                    end
                    
                    assert(iter_of_sequences == num_of_seqences + 2);

                    % option 3 continue cut-in in behind veh0
                    
                    for i = 0: max_step_before_lc
                        if i == 0
                            u_before_lc_all = zeros(2, 0, 1);
                        else
                            u_before_lc_all = obj.u1_tensor_before_lc{i};
                        end
                        num_u_before_lc_all = size(u_before_lc_all, 3);
                        
                        for j = 1:num_u_before_lc_all
                            valid_sample = true;
                            % intialize the trajectory
                            x_veh1 = x_ini(4:6);
                            x_veh1_full = zeros(3, full_sample_len);
                            u_veh1_full = zeros(2, full_sample_len);
                            
                            % first go with u sequence before lane change
                            % get veh1 u before lane change
                            % ONLY slow down
                            u1_before_lc = - u_before_lc_all(:, :, j);
                            % expand to full
                            u1_before_lc_full_sim = kron(u1_before_lc, ones(1, sample_steps));
                            step_before_lc = size(u1_before_lc_full_sim, 2);
                            u_veh1_full(:, 1:step_before_lc) = u1_before_lc_full_sim;
                            for k = 1:step_before_lc
                                
                                x_veh1 = obj.vehicle_dynamics(x_veh1, u1_before_lc_full_sim(:, k), sample_dt);
                                x_veh1_full(:, k) = x_veh1;
                            end
                            
                            % second do lane change
                            % this is ``Before" stage so always go for obj.param.u1_lane_change_step
                            u1_lc_full_sim = [0; -lateral_speed] * ones(1, step_left_to_cut_in);

                            
                            u_veh1_full(:, step_before_lc + 1:step_before_lc + step_left_to_cut_in) = u1_lc_full_sim;
                            for k = 1:step_left_to_cut_in
                                % vehicle 1 takes lane change
                                x_veh1 = obj.vehicle_dynamics(x_veh1, u1_lc_full_sim(:, k), sample_dt);
                                x_veh1_full(:, k + step_before_lc) = x_veh1;

                            end
                            % third do ovm with respect to the supposed
                            % lead Note here we ignore whether they collide
                            start_post_lc = step_before_lc + step_left_to_cut_in + 1; 
                            x_veh0 = veh0_state_trace_straight(:, step_before_lc + step_left_to_cut_in);
                            for k = start_post_lc:full_sample_len
                                % veh1 starts to follow veh 0 (remark, need
                                % to response to previous states.
                                u_veh1 = obj.car_following(x_veh1, x_veh0);
                                x_veh1 = obj.vehicle_dynamics(x_veh0, [u_veh1; 0.0], sample_dt);
                                u_veh1_full(:, k) = [u_veh1; 0.0];
                                x_veh1_full(:, k) = x_veh1;

                                % update veh2 from already generated list
                                x_veh0 = veh0_state_trace_straight(:, k);

                            end

                            if valid_sample
                                assert(iter_of_sequences <= 2 * num_of_seqences + 1);
                                veh1_state_sequences(:, :, iter_of_sequences) = x_veh1_full;
                                veh1_u_sequences(:, :, iter_of_sequences) = u_veh1_full;
    
                                % quick check plot
                                % x_full = [x_ini(:), [veh0_state_trace_straight;x_veh1_full; veh2_state_sequence; veh3_state_sequence]];
                                % u_full = [veh0_u_trace_straight; u_veh1_full];
                                % u_veh1_full(:, 1:sample_steps:end)
                                % obj.plot_trajectory(obj.full_sample_time, x_full, u_full);
                                % obj.plot_one_topview(obj.param.full_sample_time, x_full);
                                iter_of_sequences = iter_of_sequences + 1;
                            end
                        end
                    end
                    assert(iter_of_sequences == 2* num_of_seqences + 2);
                    
                    
                    time_used = toc;
                    samples = struct();
                    samples.veh0_state_sequences = veh0_state_sequences;
                    samples.veh1_state_sequences = veh1_state_sequences;
                    samples.veh0_u_sequences = veh0_u_sequences;
                    samples.veh1_u_sequences = veh1_u_sequences;
                    samples.veh2_state_sequence = veh2_state_sequence;
                    samples.veh3_state_sequence = veh3_state_sequence;
                    fprintf("Currently in 'During' lane change stage, get %d samples. Takes %.4f secs.\n", iter_of_sequences, time_used);


                case "After"
                    % simpliest case
                    % both follows ovm (do we still need to simulate?
                    % no need for sampling.

                otherwise
                    fprintf("Error. should not be here.\n");

            end
        end
        function xnext = vehicle_dynamics(obj, x, u, dt)
            xnext = zeros(3, 1);
            % longitudinal position (capped at v_max * deltaT)
            xnext(1) = min(x(1) + x(2) * dt + u(1) / 2 * dt^2, ...
                        x(1) + obj.param.v_max * dt);
            % longitunidal speed (capped between v_max and 0)
            xnext(2) = max(0.0, min(x(2) + u(1) * dt, obj.param.v_max));
            % lateral position (capped at l_max and l_min)
            xnext(3) = x(3) + u(2) * dt;
            xnext(3) = max(obj.param.l_min, min(xnext(3), obj.param.l_max));

        end
        function u = car_following(obj, x, x_lead)
            s = x(1);
            v = x(2);
            s1 = x_lead(1);
            v1 = x_lead(2);
            % gives back capped ovm responses.
            u = obj.cap_u(game.OVM(s1 - s - obj.param.ovm_params.l_veh, v, v1, obj.param.ovm_params));
        end
        function u_capped = cap_u(obj, u)
            u_capped = max(obj.param.u_min, min(u, obj.param.u_max));
        end
        function u_tensor = generate_all_sequences(obj, action_set, N, name)
            % col num = action dimension
            % row num = num of actions
            num_of_actions = size(action_set, 1);
            action_dimension = size(action_set, 2);
            u_candidate = obj.get_seq(num_of_actions, N);
            num_of_sequences = length(u_candidate);
            u_tensor = zeros(action_dimension, N, num_of_sequences);
            for i = 1:num_of_sequences
                u_tensor(:, :, i) = action_set(u_candidate(:, i), :)';
            end
            fprintf("%s is generated from %d actions of dim %.d, for horizon %d, get %d sequneces. \n", ...
                name, num_of_actions, action_dimension, N, num_of_sequences);
        end
        function [u1_tensor_sorted, n_s] = sort_u1_tensor(obj, u1_tensor)
            % split the decision space.
            % devide the actions to going straight (s) vs doing a lane
            % change/cut-in (lc)
            s_idx = find(all(u1_tensor(2,:,:) == zeros(1, obj.param.N)));
            n_s = length(s_idx);
            u1_tensor_sub_s = u1_tensor(:,:,s_idx);
            u1_tensor_sub_lc = u1_tensor(:, :, setdiff(1:size(u1_tensor, 3), s_idx));
            % quick sanity check
            for i = 1:size(u1_tensor_sub_s, 3)
                assert(all(u1_tensor_sub_s(2, :, i) == zeros(1, obj.param.N)))
            end
            for i = 1:size(u1_tensor_sub_lc, 3)
                assert(any(u1_tensor_sub_lc(2, :, i) ~= zeros(1, obj.param.N)))
            end
            u1_tensor_sorted = cat(3, u1_tensor_sub_s, u1_tensor_sub_lc);
        end
        function [r_bar, x_sequence] = get_reward_from_control(obj, x_ini, u0_sequence, u1_sequence)
            % NOT FOR FULL ITERATION, ONLY SAMML U0 U1 SAMPLE
            % calculate reward for ego and other vehicle given u0 u1
            % sequence. Remark: the reward would be different for each
            % vehicle
            s_trace = obj.get_state_trace_from_u(x_ini, u0_sequence, u1_sequence);
            s_other_trace = [];
            x_sequence = [s_trace; s_other_trace];
            % control consequence is sparse, only 0.1.
            r_bar_0 = obj.cal_reward(x_ini(1:3), s_trace(1:3, :), [s_trace(4:6, :); s_other_trace], u0_sequence, obj.param.reward.ratio_vector);
            r_bar_1 = obj.cal_reward(x_ini(4:6), s_trace(4:6, :), [s_trace(1:3, :); s_other_trace], u1_sequence, obj.param.reward.ratio_vector);
            r_bar = [r_bar_0, r_bar_1];
        end
        function s_trace = get_state_trace_from_u(obj, x_ini, u0_sequence, u1_sequence)
             % NOT FOR FULL ITERATION, ONLY SAMML U0 U1 SAMPLE
            % other than the 0 and 1, all the other vehicles maintain their
            % current speed.
            % out put will be 5 * N with out initial points
            x0 = x_ini(1:3);
            x1 = x_ini(4:6);

            s0 = zeros(3, obj.param.N);
            s1 = zeros(3, obj.param.N);

            % This part is shared by the system dynamcis too.
            for i = 1:obj.param.N
                u0 = u0_sequence(:, i);
                u1 = u1_sequence(:, i);
                x0(1) = min(x0(1) + x0(2) * obj.param.deltaT + u0 / 2 * obj.param.deltaT^2, x0(1) + obj.param.v_max * obj.param.deltaT);
                x0(2) = min(x0(2) + u0 * obj.param.deltaT, obj.param.v_max);
                
                % x0(3) should stay at zero becuase no lane change is
                % assumed.
                x1(1) = min(x1(1) + x1(2) * obj.param.deltaT + u1(1) / 2 * obj.param.deltaT^2, ...
                            x1(1) + obj.param.v_max * obj.param.deltaT);

                x1(2) = min(x1(2) + u1(1) * obj.param.deltaT, obj.param.v_max);
                x1(3) = x1(3) + u1(2) * obj.param.deltaT;
                x1(3) = max(obj.param.l_min, min(x1(3), obj.param.l_max));

                s0(:, i) = x0;
                s1(:, i) = x1;
            end
            s_trace = [s0; s1];
        end
        function r_bar = get_reward(obj, x_ini, sample, ratio_vector)
            % calculate reward for a trajectory samples
            % the sample should contains the following five fields
            veh0_state_sequence = sample.veh0_state_sequence;
            veh1_state_sequence = sample.veh1_state_sequence;
            veh0_u_sequence = sample.veh0_u_sequence;
            veh1_u_sequence = sample.veh1_u_sequence;
            veh_other_state_sequence = sample.veh_other_state_sequence;
            r_bar_0 = obj.cal_reward(x_ini(1:3), veh0_state_sequence, [veh1_state_sequence; veh_other_state_sequence], veh0_u_sequence, ratio_vector);
            r_bar_1 = obj.cal_reward(x_ini(4:6), veh1_state_sequence, [veh0_state_sequence; veh_other_state_sequence], veh1_u_sequence, ratio_vector);
            r_bar = [r_bar_0, r_bar_1];
        end
        % reward and helper functions
        function r_bar = cal_reward(obj, ego_ini, ego_trace, other_trace, ego_control_trace, ratio_vector)
            % x_trace, u0_sequence and u1_sequence should be of same length
            % and x_trace does not include initial conditions
            % (TODO) have a unit test.
            s = ego_trace(1, :);
            v = ego_trace(2, :);
            l = ego_trace(3, :);
            s_0 = ego_ini(1);

            s_others = other_trace(1:3:end,:);
            l_others = other_trace(3:3:end,:);

            % s and s_others are both of N columns, but different roll
            % any will give give a row of N columns
            dim = obj.param.reward.veh_dim;
            reward = obj.param.reward;
            %% Part 1 no collision, if collide gives -1
            % [TODO] potential bug: does not account for collision between steps
            % it may be ok if the steps are fin enough.
            collision =  - any((s - s_others < dim.veh_length) & (s- s_others > -dim.veh_length) & (l - l_others < dim.veh_width) & (l- l_others > -dim.veh_width));
            
            %% Part 2 liveness, travel further while achieve cut-in
            liveness_lateral = - abs(l);
            liveness_speed = - abs(reward.v_desired - v) / reward.v_desired;
            % if obj.game_stage == "During"
                % the sampled trajectories could only be finish lane change or abort.
                % assert(l(end) < 1e-6 || l(end) - dim.lane_width > -1e-6)
                % if l(end) < 1e-6
                %     % cut-in trajectory, encourage early finish
                %     liveness_lateral = - (l > 1e-6);
                % end

            % end
            % and travel further, the large the better.
            % but do we really want to encourage progress too?
            liveness_pos = s - s_0;
            

            %% Part 3 want to stay a safe distance.
            % and stay reasonable gap with lead vehicle.
            % find lead, closest one in lane and in the front (> veh_length, otherwise
            % will collide
            delta_s = s_others - s - dim.veh_length;
            % [TODO] bug or potential issue which of the following consideration make sense?
            
            % better to be consist in the simulation. 
            % in_lane_at_the_front = (abs(l - l_others) < dim.lane_width) & (delta_s > dim.veh_length);
            % for those that are really close at dim_veh_length, it would
            % not consider as collision but should consider as too close.
            in_lane_at_the_front = (abs(l - l_others) <= dim.veh_width) & (delta_s >= 0);
            % [TODO] bug, need to find the minimum one
            % luckily for this case it is fine because we only panelize
            % if max(sum(in_lane_at_the_front)) > 1
            %     fprintf("Warning more than one lead found.\n");
            % end
            % if any of the delta s stay too close to the front, give -1
            headway = -any((delta_s < v * reward.T_desired) & in_lane_at_the_front); 
            
            
            %% Part 5 least effort
            % want to use the least effort, thus the closer to zero the
            % better.
            if size(ego_control_trace, 1) > 1                
                u_norm = -vecnorm(ego_control_trace);
            else
                u_norm = -abs(ego_control_trace);
            end

            % calculate total reward for each step
            step_term = [collision; 
                         liveness_pos; 
                         liveness_lateral; 
                         liveness_speed; 
                         headway; 
                         u_norm];
            weight = [reward.w_collision, reward.w_live_pos, reward.w_live_lateral, ...
                      reward.w_live_speed, reward.w_headway, reward.w_effort];
            step_reward = weight * step_term;
            r_bar = step_reward * ratio_vector;
        end
        function [x0_tensor, x1_tensor, other_sequence] = get_state_trace(obj, x_ini, u0_tensor, u1_tensor)
            % other vehicle, only need to get once
            num_of_veh = length(x_ini(7:3:end));
            x = x_ini(7:end);
            other_sequence = zeros(num_of_veh * 3, obj.param.N);
            
            for i = 1:obj.param.N
                for j = 1:num_of_veh
                    idx = (j - 1) * 3 + 1:j * 3;
                    x(idx) = obj.vehicle_dynamics(x(idx), [0.0; 0.0], obj.param.deltaT);
                end
                other_sequence(:, i) = x;
            end
             
            n_seq_u0 = size(u0_tensor, 3);
            n_seq_u1 = size(u1_tensor, 3);

            % because there is no interaction the two can calculate
            % seperately
            x0_tensor = zeros(3, obj.param.N, n_seq_u0);
            
            for k = 1:n_seq_u0
                x0 = x_ini(1:3);
                for i = 1:obj.param.N
                    x0 = obj.vehicle_dynamics(x0, [u0_tensor(1, i, k); 0.0], obj.param.deltaT);
                    x0_tensor(:, i, k) = x0;
                end
            end
            x1_tensor = zeros(3, obj.param.N, n_seq_u1);
            
            for k = 1:n_seq_u1
                x1 = x_ini(4:6);
                for i = 1:obj.param.N
                    x1 = obj.vehicle_dynamics(x1, u1_tensor(:, i, k), obj.param.deltaT);
                    x1_tensor(:, i, k) = x1;
                end
            end
        end

        function [Rbar0, Rbar1, all_trjectories] = calculate_full_Rbar_from_u(obj, x_ini, u0_tensor, u1_tensor)
            % this may be simplified because the state trace does not have
            % interactions
            tic;
            n_seq_u0 = size(u0_tensor, 3);
            n_seq_u1 = size(u1_tensor, 3);
            fprintf("R matrix calculation for u0 u1 of size %d %d may take some time...\n", n_seq_u0, n_seq_u1);
            [x0_tensor, x1_tensor, other_sequence] = obj.get_state_trace(x_ini, u0_tensor, u1_tensor);
            all_trjectories = {x0_tensor, x1_tensor, other_sequence};
            sample.veh_other_state_sequence = other_sequence;
            % each role corresponds to one u1 sequence, 
            % each column coresponds to one u0 sequence
            Rbar1 = zeros(n_seq_u1, n_seq_u0);
            Rbar0 = zeros(n_seq_u1, n_seq_u0);
            for i = 1:n_seq_u1
               u1_sequence = u1_tensor(:, :, i);
               x1_sequence = x1_tensor(:, :, i);
               for j = 1:n_seq_u0
                   u0_sequence = u0_tensor(:, :, j);
                   x0_sequence = x0_tensor(:, :, j);
                   sample.veh0_state_sequence = x0_sequence;
                   sample.veh0_u_sequence = u0_sequence;
                   sample.veh1_state_sequence = x1_sequence;
                   sample.veh1_u_sequence = u1_sequence;
                   r_bar = obj.get_reward(x_ini, sample, obj.param.reward.ratio_vector);
                   Rbar0(i, j) = r_bar(1); 
                   Rbar1(i, j) = r_bar(2);
               end
            end
            time_used = toc;
            fprintf("R matrix calculation takes %.4f for u0 u1 of size %d %d.\n", time_used, n_seq_u0, n_seq_u1);
        end
        function [Rbar0, Rbar1, all_trjectories] = calculate_full_Rbar_from_u_simplify(obj, x_ini, u0_tensor, u1_tensor, n_s)
            fprintf("R matrix calcuation for u0 u1 simplified, assume u1_tensor is sorted with first %d no lane changes. \n", n_s);
            % note for now assume l1 can only be between l_max and l_min
            l1 = x_ini(6);

            if obj.game_stage == "Before" || (obj.game_stage == "During" && l1 > obj.param.reward.veh_dim.veh_width)
                % when the two interacting vehicle are not "in the same
                % lane", then the straight motion between them does not
                % affect each other, can simplify
                fprintf("veh 0 and veh 1 are not in the same lane, so no interaction when they both traveling straight. R matrix calculation can simplify.\n");
                flag = 1;
            else
                if obj.game_stage == "After"
                    flag = 2;
                    fprintf("veh 1 finished lane change, R may be simplified by consider only interaction between straight action. \n");
                else
                    flag = 3;
                    fprintf("During lane change and, veh 0 and veh 1 start to get into the same lane, R matrix cannot be simplified. \n");
                end
            end
            tic;
            % common part the size is the same
            n_seq_u0 = size(u0_tensor, 3);
            n_seq_u1 = size(u1_tensor, 3);
            fprintf("R matrix calculation for u0 u1 of size %d %d may take some time... \n", n_seq_u0, n_seq_u1);
            [x0_tensor, x1_tensor, other_sequence] = obj.get_state_trace(x_ini, u0_tensor, u1_tensor);
            all_trjectories = {x0_tensor, x1_tensor, other_sequence};
            sample.veh_other_state_sequence = other_sequence;
            % each role corresponds to one u1 sequence, 
            % each column coresponds to one u0 sequence
            Rbar1 = zeros(n_seq_u1, n_seq_u0);
            Rbar0 = zeros(n_seq_u1, n_seq_u0);
            switch flag
                case 1
                    % the straight part is not interactive
                    % should not matter
                    u0_sequence_nominal = u0_tensor(:, :, 1);
                    x0_sequence_nominal = x0_tensor(:, :, 1);
                    sample.veh0_state_sequence = x0_sequence_nominal;
                    sample.veh0_u_sequence = u0_sequence_nominal;
                    for i = 1:n_s
                        u1_sequence = u1_tensor(:, :, i);
                        x1_sequence = x1_tensor(:, :, i);
                        sample.veh1_state_sequence = x1_sequence;
                        sample.veh1_u_sequence = u1_sequence;
                        r_bar = obj.get_reward(x_ini, sample, obj.param.reward.ratio_vector);
                        Rbar1(i, :) = r_bar(2);
                    end
                    u1_sequence_nominal = u1_tensor(:, :, 1);
                    x1_sequence_nominal = x1_tensor(:, :, 1);
                    sample.veh1_state_sequence = x1_sequence_nominal;
                    sample.veh1_u_sequence = u1_sequence_nominal;
                    for j = 1:n_seq_u0
                        u0_sequence = u0_tensor(:, :, j);
                        x0_sequence = x0_tensor(:, :, j);
                        sample.veh0_state_sequence = x0_sequence;
                        sample.veh0_u_sequence = u0_sequence;
                        r_bar = obj.get_reward(x_ini, sample, obj.param.reward.ratio_vector);
                        Rbar0(:, j) = r_bar(1); 
                    end
                    % for the lane change part, need to consider
                    for i = n_s + 1:n_seq_u1
                       u1_sequence = u1_tensor(:, :, i);
                       x1_sequence = x1_tensor(:, :, i);
                       sample.veh1_state_sequence = x1_sequence;
                       sample.veh1_u_sequence = u1_sequence;
                       for j = 1:n_seq_u0
                           u0_sequence = u0_tensor(:, :, j);
                           x0_sequence = x0_tensor(:, :, j);
                           sample.veh0_state_sequence = x0_sequence;
                           sample.veh0_u_sequence = u0_sequence;
                           r_bar = obj.get_reward(x_ini, sample, obj.param.reward.ratio_vector);
                           Rbar0(i, j) = r_bar(1); 
                           Rbar1(i, j) = r_bar(2);
                       end
                    end
                case 2
                    % only calculate the straight part, and the rest left
                    % as zero
                     for i = 1:n_s
                       u1_sequence = u1_tensor(:, :, i);
                       x1_sequence = x1_tensor(:, :, i);
                       sample.veh1_state_sequence = x1_sequence;
                       sample.veh1_u_sequence = u1_sequence;
                       for j = 1:n_seq_u0
                           u0_sequence = u0_tensor(:, :, j);
                           x0_sequence = x0_tensor(:, :, j);
                           sample.veh0_state_sequence = x0_sequence;
                           sample.veh0_u_sequence = u0_sequence;
                           r_bar = obj.get_reward(x_ini, sample, obj.param.reward.ratio_vector);
                           Rbar0(i, j) = r_bar(1); 
                           Rbar1(i, j) = r_bar(2);
                       end
                     end
                     Rbar0 = Rbar0(1:n_s, :);
                     Rbar1 = Rbar1(1:n_s, :);
                otherwise
                    % full interactions
                    for i = 1:n_seq_u1
                       u1_sequence = u1_tensor(:, :, i);
                       x1_sequence = x1_tensor(:, :, i);
                       sample.veh1_state_sequence = x1_sequence;
                       sample.veh1_u_sequence = u1_sequence;
                       for j = 1:n_seq_u0
                           u0_sequence = u0_tensor(:, :, j);
                           x0_sequence = x0_tensor(:, :, j);
                           sample.veh0_state_sequence = x0_sequence;
                           sample.veh0_u_sequence = u0_sequence;
                           r_bar = obj.get_reward(x_ini, sample, obj.param.reward.ratio_vector);
                           Rbar0(i, j) = r_bar(1); 
                           Rbar1(i, j) = r_bar(2);
                       end
                    end
            end

            time_used = toc;
            fprintf("R matrix calculation takes %.4f for u0 u1 of size %d %d. \n", time_used, n_seq_u0, n_seq_u1);
        end
        function [Rbar0, Rbar1] = calculate_full_Rbar_from_samples(obj, x_ini, samples)
            tic;
            veh0_state_sequences = samples.veh0_state_sequences;
            veh1_state_sequences = samples.veh1_state_sequences;
            veh0_u_sequences = samples.veh0_u_sequences;
            veh1_u_sequences = samples.veh1_u_sequences;
            % should be the same
            veh_other_state_sequence = [samples.veh2_state_sequence;
                                         samples.veh3_state_sequence;];

            sample.veh_other_state_sequence = veh_other_state_sequence;

            assert(size(veh0_state_sequences, 3) == size(veh0_u_sequences, 3));
            assert(size(veh1_state_sequences, 3) == size(veh1_u_sequences, 3));

            n_seq_u0 = size(veh0_state_sequences, 3);
            n_seq_u1 = size(veh1_state_sequences, 3);
            fprintf("R matrix calculation for u0 u1 of size %d %d may take some time...\n", n_seq_u0, n_seq_u1);
            % each role corresponds to one u1 sequence, 
            % each column coresponds to one u0 sequence
            Rbar1 = zeros(n_seq_u1, n_seq_u0);
            Rbar0 = zeros(n_seq_u1, n_seq_u0);
            for i = 1:n_seq_u1
                veh1_state_sequence = veh1_state_sequences(:, :, i);
                veh1_u_sequence = veh1_u_sequences(:, :, i);
                for j = 1:n_seq_u0
                    veh0_state_sequence = veh0_state_sequences(:, :, j);
                    veh0_u_sequence = veh0_u_sequences(:, :, j);
                    sample.veh0_state_sequence = veh0_state_sequence;
                    sample.veh1_state_sequence = veh1_state_sequence;
                    sample.veh0_u_sequence = veh0_u_sequence;
                    sample.veh1_u_sequence = veh1_u_sequence;

                    r_bar = obj.get_reward(x_ini, sample, obj.param.reward.ratio_sample_vector);
                    % close all; obj.plot_sample(x_ini, sample);
                    Rbar0(i, j) = r_bar(1); 
                    Rbar1(i, j) = r_bar(2);
                end
            end
            time_used = toc;
            fprintf("R matrix calculation takes %.4f for u0 u1 of size %d %d.\n", time_used, n_seq_u0, n_seq_u1);

        end
        function [U, Q] = get_follower_sequences_idx(~, Rbar0, Rbar1)
            % a full brute forth search for U(index) and Q          
            % each role corresponds to one u1 sequence, 
            % each column coresponds to one u0 sequence

            % vehicle 1 (potential cut-in vehicle) is the follower
            % veh 1 select the worse case for each control sequence
            Q1_min = min(Rbar1, [], 2);
            Q1_max_min = max(Q1_min);
            U1_optimal_idx = find(Q1_min == Q1_max_min);
            if size(U1_optimal_idx, 1) > 1
               fprintf("U1 optimal set is not unique.\n");
            end
            % U1_optimal_set = u1_tensor(:, :, U1_optimal_idx);

            % u1_follower_sequence = U1_optimal_set;
            Q1_follower = Q1_min;

            % base on veh 1's section, veh 0 select the best out of all
            % U0_options

            % for leader first it gets the optimal sequences by U1
            % takes the maximum among each roles
            if size(U1_optimal_idx, 1) > 1
                Q0_max = max(Rbar0(U1_optimal_idx, :));
            else
                Q0_max = Rbar0(U1_optimal_idx, :);
            end

            U0_optimal_idx = find(Q0_max == max(Q0_max));
            if size(U0_optimal_idx, 1) > 1
                fprintf("U0 optimal set is not unique.\n");
            end
            Q0_leader = Q0_max(:);

            % summarize the Q and U mappting for potential probability
            % analysis, veh0
            
            U = {U0_optimal_idx, U1_optimal_idx};
            % should both be role vectors.
            Q = {Q0_leader, Q1_follower};
        end
        function [U, Q] = get_leader_sequences_idx(~, Rbar0, Rbar1)
            % a full brute forth search for U and Q
            % Note to give flexibility, 
            % 1. decouple Rbar0 and Rbar1 calculation
            % 2. u0 and u1 tensors are provided rather than uses from obj
            % presaved value
            % each role corresponds to one u1 sequence, 
            % each column coresponds to one u0 sequence

            % vehicle 1 (potential cut-in vehicle) is the leader 
            % veh 0 will consider the worse cases and find max min solution
            Q0_min = min(Rbar0, [], 1);
            Q0_max_min = max(Q0_min);
            U0_optimal_idx = find(Q0_min == Q0_max_min);
            if size(U0_optimal_idx, 1) > 1
               fprintf("U0 optimal set is not unique.\n");
            end
            Q0_follower = Q0_min(:);

            % base on veh 0's section, veh 1 select the best out of all
            % U1_options

            % for leader first it gets the optimal sequences by U1
            % takes the maximum among each roles
            if size(U0_optimal_idx, 1) > 1
                Q1_max = max(Rbar1(:, U0_optimal_idx), [], 2);
            else
                Q1_max = Rbar1(:, U0_optimal_idx);
            end
            U1_optimal_idx = find(Q1_max == max(Q1_max));
            if size(U1_optimal_idx, 1) > 1
                fprintf("U1 optimal set is not unique.\n");
            end

            Q1_leader = Q1_max(:);
            
            % summarize the Q and U mappting for potential probability
            % analysis, veh0
            
            U = {U0_optimal_idx, U1_optimal_idx};
            Q = {Q0_follower, Q1_leader};
        end
        function [u1, other_info] = get_veh1_decision_from_u(obj, x_ini)
            % [Rbar0, Rbar1, all_trajectories] = obj.calculate_full_Rbar_from_u(x_ini, obj.u0_tensor, obj.u1_tensor);
            if obj.game_stage~= "During" || ~obj.param.consider_lc_abort
                u1_tensor_used = obj.u1_tensor(:, :, 1:obj.n_s + obj.n_lc);
            else
                u1_tensor_used = obj.u1_tensor;
            end
            [Rbar0, Rbar1, all_trajectories] = obj.calculate_full_Rbar_from_u_simplify(x_ini, obj.u0_tensor, u1_tensor_used, obj.n_s);
            x0_tensor = all_trajectories{1};
            x1_tensor = all_trajectories{2};
            veh_other_state_sequence = all_trajectories{3};

            [U, Q_u1_as_follower] = obj.get_follower_sequences_idx(Rbar0, Rbar1);
            u0_sequence_u1_as_follower = obj.u0_tensor(:,:,U{1});
            u1_sequence_u1_as_follower = obj.u1_tensor(:,:,U{2});
            x0_sequence_u1_as_follower = x0_tensor(:,:,U{1});
            x1_sequence_u1_as_follower = x1_tensor(:,:,U{2});

            [U, Q_u1_as_leader] = obj.get_leader_sequences_idx(Rbar0, Rbar1);
            u0_sequence_u1_as_leader = obj.u0_tensor(:,:,U{1});
            u1_sequence_u1_as_leader = obj.u1_tensor(:,:,U{2});
            x0_sequence_u1_as_leader = x0_tensor(:,:,U{1});
            x1_sequence_u1_as_leader = x1_tensor(:,:,U{2});
            if obj.role == "Leader"
                if size(u1_sequence_u1_as_leader, 3) > 1
                    fprintf("Warning, u1 as leader is not unique, take the first one. \n");
                end
                u1 = u1_sequence_u1_as_leader(:, :, 1);
            else
                if obj.role == "Follower"
                    if size(u1_sequence_u1_as_follower, 3) > 1
                        fprintf("Warning, u1 as leader is not unique, take the first one. \n");
                    end
                    u1 = u1_sequence_u1_as_follower(:, :, 1);
                else
                    error("Should not be here.\n")
                end

            end
            other_info.u0_sequence_u1_as_follower = u0_sequence_u1_as_follower;
            other_info.u1_sequence_u1_as_follower = u1_sequence_u1_as_follower;
            other_info.u0_sequence_u1_as_leader = u0_sequence_u1_as_leader;
            other_info.u1_sequence_u1_as_leader = u1_sequence_u1_as_leader;
            other_info.x0_sequence_u1_as_follower = x0_sequence_u1_as_follower;
            other_info.x1_sequence_u1_as_follower = x1_sequence_u1_as_follower;
            other_info.x0_sequence_u1_as_leader = x0_sequence_u1_as_leader;
            other_info.x1_sequence_u1_as_leader = x1_sequence_u1_as_leader;
            other_info.Q_u1_as_follower = Q_u1_as_follower;
            other_info.Q_u1_as_leader = Q_u1_as_leader;
            other_info.Rbar0 = Rbar0;
            other_info.Rbar1 = Rbar1;
            other_info.veh_other_state_sequence = veh_other_state_sequence;
        end
        function [u1, other_info] = get_veh1_decision_from_sample(obj, x_ini)
            samples = obj.sample_trajectories(x_ini);
            veh0_state_sequences = samples.veh0_state_sequences;
            veh1_state_sequences = samples.veh1_state_sequences;
            veh0_u_sequences = samples.veh0_u_sequences;
            veh1_u_sequences = samples.veh1_u_sequences;
            veh_other_state_sequence = [samples.veh2_state_sequence;
                                        samples.veh3_state_sequence];
            [Rbar0, Rbar1] = obj.calculate_full_Rbar_from_samples(x_ini, samples);
            [U, Q_u1_as_follower] = obj.get_follower_sequences_idx(Rbar0, Rbar1);

            u0_sequence_u1_as_follower = veh0_u_sequences(:,:,U{1});
            u1_sequence_u1_as_follower = veh1_u_sequences(:,:,U{2});
            x0_sequence_u1_as_follower = veh0_state_sequences(:,:,U{1});
            x1_sequence_u1_as_follower = veh1_state_sequences(:,:,U{2});

            [U, Q_u1_as_leader] = obj.get_leader_sequences_idx(Rbar0, Rbar1);
            u0_sequence_u1_as_leader = veh0_u_sequences(:,:,U{1});
            u1_sequence_u1_as_leader = veh1_u_sequences(:,:,U{2});
            x0_sequence_u1_as_leader = veh0_state_sequences(:,:,U{1});
            x1_sequence_u1_as_leader = veh1_state_sequences(:,:,U{2});
            if obj.role == "Leader"
                if size(u1_sequence_u1_as_leader, 3) > 1
                    fprintf("Warning, u1 as leader is not unique, take the first one. \n");
                end
                u1 = u1_sequence_u1_as_leader(:, :, 1);
            else
                if obj.role == "Follower"
                    if size(u1_sequence_u1_as_follower, 3) > 1
                        fprintf("Warning, u1 as leader is not unique, take the first one. \n");
                    end
                    u1 = u1_sequence_u1_as_follower(:, :, 1);
                else
                    error("Should not be here.\n")
                end

            end
            other_info.u0_sequence_u1_as_follower = u0_sequence_u1_as_follower;
            other_info.u1_sequence_u1_as_follower = u1_sequence_u1_as_follower;
            other_info.u0_sequence_u1_as_leader = u0_sequence_u1_as_leader;
            other_info.u1_sequence_u1_as_leader = u1_sequence_u1_as_leader;
            other_info.x0_sequence_u1_as_follower = x0_sequence_u1_as_follower;
            other_info.x1_sequence_u1_as_follower = x1_sequence_u1_as_follower;
            other_info.x0_sequence_u1_as_leader = x0_sequence_u1_as_leader;
            other_info.x1_sequence_u1_as_leader = x1_sequence_u1_as_leader;
            other_info.Q_u1_as_follower = Q_u1_as_follower;
            other_info.Q_u1_as_leader = Q_u1_as_leader;
            other_info.Rbar0 = Rbar0;
            other_info.Rbar1 = Rbar1;
            other_info.veh_other_state_sequence = veh_other_state_sequence;
        end

        function [role_post, probs] = estimate_role(obj, role_priors, x1_errors)
            sigma = obj.param.noise_W(4:6, 4:6);
            n = size(x1_errors, 2);
            assert(all(size(role_priors) == [n, 1]))
            probs = zeros(1, n);
            for i = 1:n
                probs(i) = mvnpdf(x1_errors(:, i), zeros(3, 1), sigma);
            end
            role_post = (probs'.*role_priors) / (probs * role_priors);
        end
        %% Misc    
        % get all possible sequence
        function seq = get_seq(~, n, N)
            % very heavy, do not use if not necessary
            % seq will be of size N * (n^N) array where each columns
            % corresponds to one possible action sequence
            seq = game.gen_seqence(n, N);
        end
        function sample = convert_to_sample(~, x_full, u_full)
            sample.veh0_state_sequence = x_full(1:3, :);
            sample.veh1_state_sequence = x_full(4:6, :);
            sample.veh0_u_sequence = u_full(1,:);
            sample.veh1_u_sequence = u_full(2:3,:);
            sample.veh_other_state_sequence = x_full(7:end,:);
        end
        function figs = plot_one_frame(obj, x_ini, other_info, varargin)
            [leader_sample, follower_sample] = obj.convert_result_to_sample(other_info);
            if obj.role == "Leader"
                figs = obj.plot_sample(x_ini, leader_sample, varargin{:});
            else
                figs = obj.plot_sample(x_ini, follower_sample, varargin{:});
            end
        end
        function [leader_sample, follower_sample] = convert_result_to_sample(~, other_info)
            leader_sample.veh0_state_sequence = other_info.x0_sequence_u1_as_leader;
            leader_sample.veh1_state_sequence = other_info.x1_sequence_u1_as_leader;
            leader_sample.veh0_u_sequence = other_info.u0_sequence_u1_as_leader;
            leader_sample.veh1_u_sequence = other_info.u1_sequence_u1_as_leader;
            leader_sample.veh_other_state_sequence = other_info.veh_other_state_sequence;
            
            follower_sample.veh0_state_sequence = other_info.x0_sequence_u1_as_follower;
            follower_sample.veh1_state_sequence = other_info.x1_sequence_u1_as_follower;
            follower_sample.veh0_u_sequence = other_info.u0_sequence_u1_as_follower;
            follower_sample.veh1_u_sequence = other_info.u1_sequence_u1_as_follower;
            follower_sample.veh_other_state_sequence = other_info.veh_other_state_sequence;
        end
        function figs = plot_sample(obj, x_ini, sample, varargin)
            veh0_state_sequence = sample.veh0_state_sequence;
            veh1_state_sequence = sample.veh1_state_sequence;
            veh0_u_sequence = sample.veh0_u_sequence;
            veh1_u_sequence = sample.veh1_u_sequence;
            veh_other_state_sequence = sample.veh_other_state_sequence;
            r_bar = obj.get_reward(x_ini, sample, obj.param.reward.ratio_vector);
            fprintf("Reward veh 0 (ego) %.4f, veh1 (cut-in) %.4f. \n", r_bar(1), r_bar(2));
            x_all = [x_ini, [veh0_state_sequence; veh1_state_sequence; veh_other_state_sequence]];
            u_all = [veh0_u_sequence; veh1_u_sequence];
            if size(u_all, 2) > obj.param.N
                u_all(:, 1:obj.param.sample_steps:end)
                fig1 = obj.plot_trajectory(obj.param.full_sample_time, x_all, u_all, varargin{:});
                fig2 = obj.plot_one_topview(obj.param.full_sample_time, x_all, varargin{:});
                figs = [fig1, fig2];
            else
                u_all
                time = 0:obj.param.deltaT:obj.param.N * obj.param.deltaT;
                fig1 = obj.plot_trajectory(time, x_all, u_all, varargin{:});
                varargin{end + 1} = "frame_rate";
                varargin{end + 1} = 1;
                fig2 = obj.plot_one_topview(time, x_all, varargin{:});
                figs = [fig1, fig2];
            end
        end
        function fig = plot_one_topview(obj, tsim, x_all, varargin)
            defaults = {"title", [], "frame_rate", obj.param.deltaT / obj.param.sample_dt};
            options = Util.SetOptions(defaults, varargin);
            veh_dim = obj.param.reward.veh_dim;
            % determine size
            simlen = size(x_all, 2);
            s_range = x_all(1:3:end, :);
            s_range = s_range(:);
            l_range = x_all(3:3:end, :);
            l_range = l_range(:);
            x_min = floor((min(s_range) - veh_dim.veh_length) / 10) * 10;
            x_max = ceil((max(s_range) + veh_dim.veh_length * 10) / 10) * 10;
            y_min = floor((min(l_range) - veh_dim.veh_width) / 3) * 3;
            y_max = ceil((max(l_range) + veh_dim.veh_width) / 3) * 3;
            
            fig = figure();
            if ~isempty(options.title)
                set(fig, "Name", options.title)
            end
            
            set(gcf, "unit", "inches");
            ps = get(gcf, "Position");
            width = 15;
            height = 9;
            set(gcf, "Position", [ps(1) / 10, ps(2) / 10, width, height])
            

            frame = (1:options.frame_rate:simlen);
            lane_width = veh_dim.lane_width;
            for i = 1:length(frame)
                ax = subplot(length(frame), 1, i);hold on;
                plot([x_min, x_max], [- lane_width / 2, - lane_width / 2], "c--")
                plot([x_min, x_max], [lane_width / 2, lane_width / 2], "c--")
                plot([x_min, x_max], [lane_width * 3 / 2, lane_width * 3 / 2], "c--")
                xlim([x_min, x_max]);
                ylim([y_min, y_max]);
                game.plot_vehs(x_all(:, frame(i)), veh_dim, sprintf("time %.1f ", tsim(frame(i))), ax);
                ylabel("lateral [m]");
            end
            xlabel("longitudinal [m]");
        end

        function fig = plot_trajectory(~, t, x_all, u_all, varargin)
            fig = figure();
            defaults = {"title", [], "colors", {"b-", "r-.", "g--", "k:"}};
            options = Util.SetOptions(defaults, varargin);
            if ~isempty(options.title)
                set(fig, "Name", options.title)
            end
            set(gcf, "unit", "inches");
            ps = get(gcf, "Position");
            width = 6;
            height = 8;
            set(gcf, "Position", [ps(1) / 10, ps(2) / 10, width, height])
            ax = zeros(5, 1);
            ax(1) = subplot(5,1,1); hold on; box on;
            ylabel("s [m]");
            ax(2) = subplot(5,1,2); hold on; box on;
            ylabel("v [m]");
            ax(3) = subplot(5,1,3); hold on; box on;
            ylabel("l [m]")
            ax(4) = subplot(5,1,4); hold on; box on;
            ylabel("a [m/s^2]")
            ax(5) = subplot(5,1,5); hold on; box on;
            ylabel("v_l [m/s]")
            xlabel("time [s]")
            linkaxes(ax, "x")
            for i = 1:4
                subplot(5,1,1);
                plot(t, x_all(1 + (i-1)*3, :), options.colors{i})
                subplot(5,1,2);
                plot(t, x_all(2 + (i-1)*3, :), options.colors{i})
                subplot(5,1,3);
                plot(t, x_all(3 + (i-1)*3, :), options.colors{i})
            end
            if ~isempty(u_all)
                subplot(5,1,4)
                plot(t(1:end-1), u_all(1, :), options.colors{1})
                plot(t(1:end-1), u_all(2, :), options.colors{2})
                subplot(5,1,5)
                plot(t(1:end-1), u_all(3, :), options.colors{2})
            end

        end
    end 
end
