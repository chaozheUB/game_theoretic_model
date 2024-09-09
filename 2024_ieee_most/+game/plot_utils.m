classdef plot_utils < handle
    %PLOT_UTILS Summary of this class goes here
    %   Detailed explanation goes here
    properties
        % in inches
        default_height = 8;
        default_width = 6;
        veh_dim;
        results;
    end
    
    methods
        function obj = plot_utils()
            % not expect to change
            veh_width = 2.5;
            veh_length = 5.0;
            lane_width = 4.0;
            
            veh_dim.veh_width = veh_width;
            veh_dim.veh_length = veh_length;
            veh_dim.veh_d = veh_length / 3;
            veh_dim.lane_width = lane_width;
            obj.veh_dim = veh_dim;
            obj.results = struct();
        end
        function load_results(obj, results)
            obj.results = results;
        end
        function energy_dict = get_energy(~, run_dict)
            states_keys = keys(run_dict);
            energy_dict = dictionary();
            for k = 1:run_dict.numEntries
                res = load(run_dict(states_keys(k)), "results");
                res = res.results;
                energy_dict(states_keys(k)) = {[res.w_ego, res.w_cut]};
            end
        end
        function [energy_dict, states, veh1_states, figs] = compare_plot(obj, run_dict)
            % input will be a dictionary with key being the display name and value
            % being the path to the file.
            y_names_map_ego = dictionary("v [m/s]", 3, ... 
                                     "h [m]", 8, ...
                                     " lead v [m/s]", 6,...
                                     "lead id", 7, ...
                                     "l [m]", 4, ...
                                     "long u [m/s^2]", 9, ...
                                     "lateral v [m/s]", 10);
            y_names_map_cut = dictionary("v [m/s]", 3, ... 
                                     "h [m]", 8, ...
                                     " lead v [m/s]", 6,...
                                     "lead id", 7, ...
                                     "l [m]", 4, ...
                                     "long u [m/s^2]", 9, ...
                                     "lateral v [m/s]", 10);
            states_keys = keys(run_dict);
            energy_dict = dictionary();
            states = dictionary();
            veh1_states = dictionary();
            all_results = cell(1, run_dict.numEntries);
            for k = 1:run_dict.numEntries
                res = load(run_dict(states_keys(k)), "results");
                res = res.results;
                if isfield(res, "roles_estimate_priors_all")
                    res.ego_state = [res.ego_state; res.roles_estimate_priors_all(1,:)];
                    y_names_map_ego("prob leader") = 11;
                end
                if isfield(res, "cut_game_stage_all")
                    cut_game_stage_map = dictionary("Before", 2, "During", 1, "After", 0);
                    cut_game_stage_all = zeros(1, length(res.cut_game_stage_all));
                    for i = 1:length(res.cut_game_stage_all)
                        if ~isempty(res.cut_game_stage_all{i})
                            cut_game_stage_all(i) = cut_game_stage_map(res.cut_game_stage_all{i});
                        else
                            cut_game_stage_all(i) = -1;
                        end
                    end
                    res.cut_state = [res.cut_state; cut_game_stage_all];
                    y_names_map_cut("game stage") = 11;
                end
                all_results{k} = res;
                states(states_keys(k)) = {res.ego_state(:, 1:end-1)};
                veh1_states(states_keys(k)) = {res.cut_state(:, 1:end-1)};
                % to add different size values, make it a cell.
                energy_dict(states_keys(k)) = {[res.w_ego, res.w_cut]};
            end

            figs = zeros(1, 2 + run_dict.numEntries);
            for k = 1:run_dict.numEntries
                res = all_results{k};
                fig = obj.plot_one_topview(res.x_all, res.tsim, "title", states_keys(k));
                figs(k) = fig;
            end
            fig = obj.plot_trajectories(states, "title", "ego", "y_names_map", y_names_map_ego);
            figs(end-1) = fig;
            fig = obj.plot_trajectories(veh1_states, "title", "veh 1 (potential cut-in vehicle)",...
                                  "y_names_map", y_names_map_cut);
            figs(end) = fig;
        end
        function fig = plot_trajectories(~, states, varargin)
            y_names_map_default = dictionary("v [m/s]", 3, ... 
                                     "h [m]", 8, ...
                                     " lead v [m/s]", 6,...
                                     "lead id", 7, ...
                                     "l [m]", 4, ...
                                     "long u [m/s^2]", 9, ...
                                     "lateral v [m/s]", 10);
            defaults = {'title', [], "y_names_map", y_names_map_default};
            options = Util.SetOptions(defaults, varargin);
            % states is a cell and name is the ideal name
            colors = {'b', 'r--', 'g-.', 'k:', 'c-.', 'm--', 'y-'};
            n = length(states.keys());
            veh_names = keys(states);
            veh_states = values(states);
            
            fig = figure();
            if ~isempty(options.title)
                set(fig, "Name", options.title)
            end
            set(gcf, "unit", "inches");
            ps = get(gcf, "Position");
            width = 11;
            height = 13;
            set(gcf, "Position", [ps(1)/10, ps(2)/10, width, height])
            % to do figure to paper size
            % set(gcf, 'PaperUnits', 'inches');
            % x_width=7.25 ;y_width=9.125;
            % set(gcf, 'PaperPosition', [0 0 x_width y_width]); %
            
            y_names_map = options.y_names_map;
            % if states
            n_panels = length(y_names_map.keys());
            y_labels = keys(y_names_map);
            row_idx = values(y_names_map);
            ax = zeros(1, n_panels);
            for i = 1:n_panels
                ax(i) = subplot(n_panels, 1, i);hold on; box on; grid on;
                ylabel(y_labels(i));
            end
            linkaxes(ax, "x");
            xlabel("t [s]");
            x_lim = [0, 0];
            for i = 1:n_panels
                subplot(n_panels, 1, i);
                for i_veh = 1:n
                    ss = veh_states(i_veh);
                    if row_idx(i) <= size(ss{1}, 1)
                        fprintf("%d, %d, %d \n", i, i_veh, row_idx(i));
                        plot(ss{1}(1, :), ss{1}(row_idx(i), :), colors{i_veh});
                        x_lim = [min(x_lim(1), min(ss{1}(1, :))), max(x_lim(1), max(ss{1}(1, :)))];
                        legend(veh_names);
                    end
                end
            end
            xlim(x_lim);
        end
        function fig = plot_one_topview(obj, s_all, tsim, varargin)
            defaults = {"title", [], "frame_rate", 20};
            options = Util.SetOptions(defaults, varargin);
            % determine size
            simlen = size(s_all, 2);
            s_range = s_all(1:3:end, :);
            s_range = s_range(:);
            l_range = s_all(3:3:end, :);
            l_range = l_range(:);
            x_min = floor((min(s_range) - obj.veh_dim.veh_length) / 10) * 10;
            x_max = ceil((max(s_range) + obj.veh_dim.veh_length * 10) / 10) * 10;
            y_min = floor((min(l_range) - obj.veh_dim.veh_width) / 3) * 3;
            y_max = ceil((max(l_range) + obj.veh_dim.veh_width) / 3) * 3;
            
            fig = figure();
            if ~isempty(options.title)
                set(fig, "Name", options.title)
            end
            
            set(gcf, "unit", "inches");
            ps = get(gcf, "Position");
            width = 15;
            height = 9;
            set(gcf, "Position", [ps(1)/10, ps(2)/10, width, height])
            

            frame = (1:options.frame_rate:simlen);
            lane_width = obj.veh_dim.lane_width;
            for i = 1:length(frame)
                ax = subplot(length(frame), 1, i);hold on;
                plot([x_min, x_max], [- lane_width / 2, - lane_width / 2], "c--")
                plot([x_min, x_max], [lane_width / 2, lane_width / 2], "c--")
                plot([x_min, x_max], [lane_width * 3 / 2, lane_width * 3 / 2], "c--")
                xlim([x_min, x_max]);
                ylim([y_min, y_max]);
                game.plot_vehs(s_all(:, frame(i)), obj.veh_dim, sprintf("time %.1f ", tsim(frame(i))), ax);
                ylabel("lateral [m]");
            end
            xlabel("longitudinal [m]");
        end
        function check_frame(obj, time_of_interest)
            if isempty(fieldnames(obj.results))
                fprintf("No result is loaded. Use 'load_result' to load. \n");
                return
            end
            cut_game_output_flag = obj.results.cut_game_output_flag;
            cut_game_output_all = obj.results.cut_game_output_all;
            x_all = obj.results.x_all;
            % tsim = obj.results.tsim;
            % [~, sim_idx] = min(abs(tsim - time_of_interest));
            % cut_game_output_all{5, game_idx};
            [~, game_idx] = min(abs(cut_game_output_flag(1, :) - time_of_interest));
            x_frame = x_all(:, cut_game_output_flag(2, game_idx));
            other_info_frame = cut_game_output_all{2, game_idx};
            cut_game = obj.results.cut_game;
            % it uses plot_one_frame from the CutInClass, it is better put
            % there fore now as it uses its internal variables.
            cut_game.plot_one_frame(x_frame, other_info_frame, "title", sprintf("Role %s Sim Time %.1f", cut_game.role, time_of_interest));
        end
    end
end

