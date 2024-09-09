classdef traffic_sim < handle
    %CUT-IN_TRAFFIC Summary of this class goes here
    %   Detailed explanation goes here
    % [CRH] need to figure out if it is easier to give one step or whole
    % trajectory.
    properties (SetAccess = private)
        % for ego vehicle only
        % not to be change or seen for now
        ar = 1.47 * 0.1;
        cr = 2.75 * 1e-4;
    end
    properties (SetAccess = protected)
        % may be accessed but not changed.
        dt
        n_veh
        time
        x
        noise_W
        add_noise
    end
    methods
        function obj = traffic_sim(x_ini, varargin)
            %CUT-IN_TRAFFIC Construct an instance of this class
            %   Detailed explanation goes here
            obj.set_param(varargin{:})
            obj.reset(x_ini);
        end
        function set_param(obj, varargin)
            % private parameters
            % add more optional parameters that may be set this way.
            defaults = {'dt', 0.1, 'noise_W', zeros(12), 'add_noise', false};
            p = Util.SetOptions(defaults, varargin);
            obj.dt = p.dt;
            obj.noise_W = p.noise_W;
            obj.add_noise = p.add_noise;
            v = eig(obj.noise_W);
            % for simplicity, consider only diagonal covariance (no
            % dependency between each other.
            % obj.noise_W = diag(obj.noise_W);
            if max(v) <= 1e-6
                fprintf("Noise covariance matrix too small, traffic sim class will not add noise.\n");
                obj.add_noise = false;
            end
        end
        function reset(obj, x_ini)
            obj.time = 0.0;
            obj.x = x_ini;
            obj.n_veh = round(length(x_ini) / 3);
        end
        function step(obj, u)
            % generic step, treat all other vehicles the same
            % each vehicle has three states
            x_ego = obj.step_ego(obj.x(1:3), u(1:2));
            x_other = obj.step_other_traffic(obj.x(4:end), u(3:end));

            if obj.add_noise
                % chol(A) factorizes symmetric positive definite matrix A into an upper triangular R that satisfies A = R'*R
                % R = chol(obj.noise_W);
                % noise = R' * randn(12, 1);
                % or noise = mvnrnd(mu, obj.noise_W, 8) 
                noise = mvnrnd(zeros(12, 1), obj.noise_W, 1);
                obj.x = [x_ego; x_other] + noise(:);
                % cap the lateral of veh 1 to not be too crazy
                obj.x(6) = max(obj.x(6), -sqrt(obj.noise_W(6, 6)));
            else
                obj.x = [x_ego; x_other];
            end
            
            obj.time = obj.time + obj.dt;
        end
        function x = step_ego(obj, x, u_ego)
            % apply ego dynamics
            % the only difference is that there are some physical force to
            % be overcome
            % note that ego here does not know about delay
            a_ego =  u_ego(1) - (obj.ar + obj.cr * x(2)^2);
            x(1) = x(1) + x(2) * obj.dt + a_ego / 2 * obj.dt^2;
            x(2) = x(2) + a_ego * obj.dt;
            x(3) = x(3) + u_ego(2)* obj.dt;
            % (TODO) should I add limits here too?
        end
        function x = step_other_traffic(obj, x, u)
            % apply all other vehicle dynamics
            % each should have 3 states s, v, l
            % the control should of 2 * (n_veh - 1)
            % each columns corresponds to u
            % because of cut-game, the cut vehicle may be left out
            for i_veh = 1:obj.n_veh - 1
                i_start = (i_veh - 1) * 3;
                i_u_start = (i_veh - 1) * 2;
                x(i_start + 1) = x(i_start + 1) + x(i_start + 2) * obj.dt + u(i_u_start + 1) / 2 * obj.dt^2;
                x(i_start + 2) = x(i_start + 2) + u(i_u_start + 1) * obj.dt;
                x(i_start + 3) = x(i_start + 3) + u(i_u_start + 2) * obj.dt;
                % (TODO) Should I add limits here too?
            end
        end
        function [lead_s, lead_v, lead_id] = find_lead_generic(~, s, l, s_others, l_others, v_others, id, veh_width, veh_length, v_ids)
            delta_s = s_others - s;
            % [TODO] keep an eye on this logic, be consistent with game class logic
            in_lane_at_the_front = (abs(l - l_others) <= veh_width) & (delta_s > veh_length);
            if sum(in_lane_at_the_front) == 0
                fprintf("There is no lead for veh %d. \n", id);
                lead_s = inf;
                lead_v = inf;
                lead_id = NaN;
            else
                if sum(in_lane_at_the_front) == 1
                    idx = find(in_lane_at_the_front);
                    lead_id = v_ids(idx);
                    fprintf("Unique lead for veh %d, veh %d. \n", id, lead_id);
                    lead_s = s_others(idx);
                    lead_v = v_others(idx);
                else
                    idx = find(in_lane_at_the_front & (delta_s == min(delta_s(in_lane_at_the_front))));
                    lead_id = v_ids(idx);
                    fprintf("Found lead for veh %d with closest distance, veh %d. \n", id, lead_id);
                    lead_s = s_others(idx);
                    lead_v = v_others(idx);
                end
            end
        end
    end
end

