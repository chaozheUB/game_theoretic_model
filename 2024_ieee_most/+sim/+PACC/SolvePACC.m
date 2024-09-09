classdef SolvePACC < handle
    % handle class recovers some of the python class property
    % SIMPACC Converted from sim_pacc function
    % Need to have a class so that it can simulate just one step.
    % handle class is better 
    % https://www.mathworks.com/help/matlab/matlab_oop/comparing-handle-and-value-classes.html
    % make some of the properties not changable.
    % https://www.mathworks.com/help/matlab/matlab_oop/mutable-and-immutable-properties.html
    properties (SetAccess = protected)
        param
        % save results
        last_result
        last_pred
        uhist
    end
    methods
        function obj = SolvePACC(varargin)
            %SIMPACC Construct an instance of this class
            %   Detailed explanation goes here
            obj = obj.setParam(varargin{:});
        end
        
        function obj = setParam(obj, varargin)
            % mpc configs
            mpc_config = {'dt', 0.1,... % time step
                          'horizon', 10.0,... % prediction horizon
                          'ARoption', false,... % data driven prediction
                          'past_time_horizon', 5.0,... % used for ARoption
                          'pred_only', false,...
                          'sigma', 0.6, ... % control delay
                          'infeasibility_type', 'uniform'}; % uniform means one z, individual means z of # of steps
            mpc_problem_config = {"l_veh", 5.0, "Tmin", 0.67, "T", 1.67, "dmin", 3, "dst", 5, ...
                                  "lane_width", 4.0, ...
                                   "qg", 1, "qa", 960, "w_penalty", 1e6,...
                                   "umin", 8.5, "vmax", 35, "m1", 0.285, "m2", -0.121, "b1", 2, "b2", 4.83};
            mpc_problem_chance_constratins = {"epsilon", 0.01};
            solver_config = {"solver", "gurobi"}; % could use qp too.
            defaults = cat(2, [mpc_config, mpc_problem_config, mpc_problem_chance_constratins, solver_config]);
            p = Util.SetOptions(defaults, varargin);

            if p.horizon > 16
                fprintf("Reduce horizon to 16.\n");
                p.horizon = 16.0;
            end

            % derived parameter, not set
            p.N = round(p.horizon / p.dt) + 1;
            p.q = round(p.sigma / p.dt);
            
            obj.param = p;
            %% Load chance constraint vector
            % [CRH Note] Do we have code how this is generated?
            % I think this need to change if N changes right?
            w = load(fullfile('params', 'chance_constraint.mat'), 'd_cc_full_vec');
            obj.param.d_cc_full_vec = w.d_cc_full_vec(1: obj.param.N);
            
        end
        function [s1pred, v1pred] = get_prediction(obj, lead_pos_history, lead_vel_history)
            tpred = obj.param.dt * (0: obj.param.N-1)';
            s1 = lead_pos_history(end);
            v1 = lead_vel_history(end);
            % Estimate n_h and predict s1.
            if obj.param.ARoption
                % [CRH] this does not work yet
                [s1pred, v1pred] = sim.PACC.s1pred_AR(lead_pos_history, lead_vel_history, tpred, ...
                    'past_time_horizon', obj.param.past_time_horizon, 'future_time_horizon', obj.param.horizon);
            else
                [s1pred, v1pred] = sim.PACC.s1pred_constv(s1, v1, tpred);
            end
            obj.last_pred = {s1pred, v1pred};
        end

        function [s1pred_update, v1pred_update, ignored] = get_prediction_with_cut_in(obj, cut_s, cut_v, cut_in_estimate_traj, splan_ignore,s1pred, v1pred)
            num_future = size(cut_in_estimate_traj, 3);
            % s1pred is columns
            future_length = length(s1pred);
            s1pred_update = zeros(future_length, 1, num_future);
            v1pred_update = zeros(future_length, 1, num_future);
            fprintf("%d future predictions. \n", num_future);
            ignored = true;
            % l_veh = obj.param.l_veh;
            for idx_cut = 1:num_future
                s1pred_update(:, :, idx_cut) = s1pred; 
                v1pred_update(:, :, idx_cut) = v1pred;
                tpred = obj.param.dt * (0: obj.param.N-1)';
                est_time = cut_in_estimate_traj(1, :, idx_cut);
                
                est_l = cut_in_estimate_traj(4, :, idx_cut);
                
                cut_in_l = interp1(est_time, est_l, tpred);
                % REMARK, this is different from finding lead, this is when
                % vehicle crosses lane boundary
                % this means that veh may chose to react to a lead vehicle
                % even before it crosses the boundary line.
                idx = find(abs(cut_in_l) <= obj.param.lane_width / 2);
                if isempty(idx) > 0
                    return;
                end
                fprintf("cutin happens, for future %d incorperate cutin in prediction.\n", idx_cut);
                est_s = cut_in_estimate_traj(2, :, idx_cut);
                cut_in_s = interp1(est_time, est_s, tpred);
                % if the cut-in part is behind the ignored planing trajectory, we can
                % ignore this trajectory
                % strictly speaking should ad l_veh, but it can be too
                % conservative
                if all(cut_in_s(idx) <= splan_ignore(idx))
                    fprintf("cutin happens, for future %d but from the behind, skip.\n", idx_cut);
                    continue
                end
                ignored = false;
                % 
                % 
                est_v = cut_in_estimate_traj(3, :, idx_cut);
                % slighly more accurate
                cut_in_v = interp1(est_time, est_v, tpred);
                cut_in_s = cut_s + cumtrapz(tpred, cut_in_v);
                
                s1pred_update(idx, : , idx_cut) = cut_in_s(idx);
                v1pred_update(idx, : , idx_cut) = cut_in_v(idx);
            end
        end
        function [umin, umax] = get_control_limit(obj, v)
            m1 = obj.param.m1;
            m2 = obj.param.m2;
            b1 = obj.param.b1;
            b2 = obj.param.b2;
            umin = -obj.param.umin;
            umax = min(m1 * v + b1, m2 * v + b2);
        end

        function uplan = compute_cmd(obj, s0, v0, s1pred, u_history)
            % [TODO] make it a better way.
            N = obj.param.N;
            q = obj.param.q;
            dt = obj.param.dt;
            qg = obj.param.qg;
            qa = obj.param.qa;
            l_veh = obj.param.l_veh;
            
            Tmin = obj.param.Tmin;
            T = obj.param.T;
            dmin = obj.param.dmin;
            dst = obj.param.dst;
            umin = obj.param.umin;
            vmax = obj.param.vmax;
            m1 = obj.param.m1;
            m2 = obj.param.m2;
            b1 = obj.param.b1;
            b2 = obj.param.b2;
            w_penalty = obj.param.w_penalty;
            d_cc_full_vec = obj.param.d_cc_full_vec;
            
            assert(length(u_history) == q);

            % These are optimization parameters
            tic;
            s = sdpvar(N, 1);  % position of CAV
            v = sdpvar(N, 1);  % velocity of CAV
            u = sdpvar(N - 1 + q, 1); % Acceleration of CAV
            obj_main = qg * sum((s1pred - s - l_veh - dst - T * v).^2) ...
                       + qa * (sum(u(1: N-1).^2) + sum(u(1+q: N-1+q).^2));
            
            % constraint
            cons_main = [s(2: end) == s(1: end-1) + dt * v(1: end-1) + 0.5 * dt * dt * u(1: N-1), ...
                         v(2: end) == v(1: end-1) + dt * u(1: N-1), ...
                         0 <= v, v <= vmax, ...
                         - umin <= u, u(1: N-1) <= m1 * v(1: end-1) + b1, ...
                         u(1:N-1) <= m2 * v(1: end-1) + b2, ...
                         u(1+q: N-1+q) <= m1 * v(1: end-1) + b1, ...
                         u(1+q: N-1+q) <= m2 * v(1: end-1) + b2, ...
                         s(1) == s0, v(1) == v0];

            if obj.param.infeasibility_type == "uniform"
                % original version
                % problem with this one is when infeasiable solution is
                % inevitable (say vehicle cut-in), then the optimal solution
                % would be one that ignores the safety constraints.
                z = sdpvar(1);
                obj_feasibility = w_penalty * z;

                cons_feasibility = [s1pred - s - l_veh - Tmin * v - dmin - d_cc_full_vec >= -z * ones(N,1), ...
                                    z >= 0];
            else
                % down side, will make problem bigger and longer to solve
                z = sdpvar(N, 1);
                obj_feasibility = w_penalty * sum(z);

                cons_feasibility = [s1pred - s - l_veh - Tmin * v - dmin - d_cc_full_vec >= -z, ...
                                    z >= 0];
            end
            objective = obj_main + obj_feasibility;
            cons = [cons_main, cons_feasibility];
            for ii = 1: q
                cons = [cons, u(ii) == u_history(ii)];
            end
            % solve the optimization problem
            % solve the optimization problem
            switch obj.param.solver
                case "qp"
                    options = sdpsettings('solver', 'quadprog');
                otherwise
                    % default guribi
                    options = sdpsettings('solver', 'gurobi', 'verbose', 0);
            end

            % options = sdpsettings('solver', 'quadprog');
            % options = sdpsettings('solver', 'ipopt', 'ipopt.max_iter', 50000, 'ipopt.max_cpu_time', 10000);
            optimize(cons, objective, options);
            time_used = toc;
            fprintf("Optimzation time used %.4f \n", time_used);
            if max(value(z)) > 1e-4
                fprintf("infeasibility is inevitable with z %.4f. \n", max(value(z)));
            end
            uplan = value(u);
            % for mpc, only uplan(1 + q) should be taken
            % and if q = 0, this means that only the first one is taken
            obj.last_result = {value(u), value(s), value(v), value(z)};
            clear("u","s","v","z", "cons", "objective");
        end

        function uplan = compute_cmd_with_prob(obj, s0, v0, s1pred_with_prob, u_history)
            % [TODO] make it a better way.
            N = obj.param.N;
            q = obj.param.q;
            dt = obj.param.dt;
            qg = obj.param.qg;
            qa = obj.param.qa;
            l_veh = obj.param.l_veh;
            
            Tmin = obj.param.Tmin;
            T = obj.param.T;
            dmin = obj.param.dmin;
            dst = obj.param.dst;
            umin = obj.param.umin;
            vmax = obj.param.vmax;
            m1 = obj.param.m1;
            m2 = obj.param.m2;
            b1 = obj.param.b1;
            b2 = obj.param.b2;
            w_penalty = obj.param.w_penalty;
            d_cc_full_vec = obj.param.d_cc_full_vec;
            
            assert(length(u_history) == q);



            % These are optimization parameters
            tic;
            s = sdpvar(N, 1);  % position of CAV
            v = sdpvar(N, 1);  % velocity of CAV
            u = sdpvar(N - 1 + q, 1); % Acceleration of CAV

            % s1pred_with_prob can be a cell or a table
            s1preds = s1pred_with_prob{1};
            probs = s1pred_with_prob{2};
            num_futures = size(s1preds, 3);
            obj_dist = 0;
            if num_futures == 1
                % only one future
                % revert back to old cases.
                % the probablity will be ignored.
                s1pred = s1preds(:, :, 1);
                obj_dist = sum((s1pred - s - l_veh - dst - T * v).^2);
            else
                s1pred = s1preds(:, :, 1);
                for i = 1:num_futures
                    obj_dist = obj_dist + sum((s1preds(:, :, i) - s - l_veh - dst - T * v).^2) * probs(i);
                    s1pred = min(s1pred, s1preds(:, :, i));
                end
            end
            % quickly plot the future
            % figure()
            % hold on;
            % colors = {'b', 'r--','g-.', 'c:'};
            % for i = 1:num_futures
            %     plot(s1preds(:, :, i), colors{i});
            % end
            % plot(s1pred, 'k:')

            obj_main = qg * obj_dist ...
                       + qa * (sum(u(1: N-1).^2) + sum(u(1+q: N-1+q).^2));
            
            % constraint
            cons_main = [s(2: end) == s(1: end-1) + dt * v(1: end-1) + 0.5 * dt * dt * u(1: N-1), ...
                         v(2: end) == v(1: end-1) + dt * u(1: N-1), ...
                         0 <= v, v <= vmax, ...
                         - umin <= u, u(1: N-1) <= m1 * v(1: end-1) + b1, ...
                         u(1:N-1) <= m2 * v(1: end-1) + b2, ...
                         u(1+q: N-1+q) <= m1 * v(1: end-1) + b1, ...
                         u(1+q: N-1+q) <= m2 * v(1: end-1) + b2, ...
                         s(1) == s0, v(1) == v0];

            if obj.param.infeasibility_type == "uniform"
                % original version
                % problem with this one is when infeasiable solution is
                % inevitable (say vehicle cut-in), then the optimal solution
                % would be one that ignores the safety constraints.
                z = sdpvar(1);
                obj_feasibility = w_penalty * z;
                % how to do this as an convex
                cons_feasibility = [s1pred - s - l_veh - Tmin * v - dmin - d_cc_full_vec >= -z * ones(N,1), ...
                                    z >= 0];
            else
                % down side, will make problem bigger and longer to solve
                z = sdpvar(N, 1);
                obj_feasibility = w_penalty * sum(z);

                cons_feasibility = [s1pred - s - l_veh - Tmin * v - dmin - d_cc_full_vec >= -z, ...
                                    z >= 0];
            end
            objective = obj_main + obj_feasibility;
            cons = [cons_main, cons_feasibility];
            for ii = 1: q
                cons = [cons, u(ii) == u_history(ii)];
            end
            % solve the optimization problem
            switch obj.param.solver
                case "qp"
                    options = sdpsettings('solver', 'quadprog');
                otherwise
                    % default guribi
                    options = sdpsettings('solver', 'gurobi', 'verbose', 0);
            end
            % options = sdpsettings('solver', 'ipopt', 'ipopt.max_iter', 50000, 'ipopt.max_cpu_time', 10000);
            optimize(cons, objective, options);
            time_used = toc;
            fprintf("Optimzation time used %.4f \n", time_used);
            uplan = value(u);
            % for mpc, only uplan(1 + q) should be taken
            % and if q = 0, this means that only the first one is taken
            obj.last_result = {value(u), value(s), value(v), value(z)};
            clear("u","s","v","z", "cons", "objective");
        end
    end
end

