clear;clc;close all;
Util.plot_setup();
plotter = game.plot_utils();

%% Where to save the results
stk = dbstack; filepath = which(stk(1).file);
save_root = fileparts(filepath);
results_folder_root = fullfile(save_root, "reproduce_results");

sigma = 0.6;

% 0 for cut-from behind case. 
% 30 for normal cut-in case, rune 10 times
% remark: you may not get the exact same results as those in the paper, but
% quantatively, it should be the same

base_name = "normal_cut_in";
run_num = 10;
s1 = 30.0;

% base_name = "cut_in_from_behind";
% run_num = 1;
% s1 = 0.0;




%% Traffic Scenario Initial condition
% made up a cut-in case, involve in only four cars, two slowly driving car static, does
% not change lane 
% ego start point
s0 = 0.0; 
l0 = 0.0;
v0 = 20.0;

% target vehicle (potential cut in)
% the further ahead the likelihood for cut-in
% for N=5 (lane changes takes 2 steps)
% 35 leader folower same cutin action
% 30 25 20 leader will cut in follower will not
% 15 leader will cut-in after
% for N=4
% 20 follower will not cut in leader will
% s0 = 30.0;

l1 = 4.0; % lane_width
v1 = 16.0;


% road vehicle
% one on the cut-in vehicle lane, originally in front of ego
s2 = s1 + 30;
l2 = l1;
v2 = 16.0;

% one on the ego lane
s3 = s0 + 100;
l3 = l0;
v3 = 16.0;

x_ini = [s0, v0, 0.0, s1, v1, l1, s2, v2, l2, s3, v3, l3]';

% check game feasibility
% For discrete action only, make sure that collision on straight would not happen

% 1 means ovm
% 2 means mpc
% 3 means mpc + consider cut-in but know the role
% 4 means mpc + consider cut-in probablitity

cut_in_vehicle_role_all = ["Leader", "Follower"];
ego_vehice_type_all = [2];

for iter = 1:length(cut_in_vehicle_role_all)
    cut_in_vehicle_role = cut_in_vehicle_role_all(iter);
    for jter = 1:length(ego_vehice_type_all)
        ego_vehice_type = ego_vehice_type_all(jter);
        for run = 1:run_num
            % do this clear all step because there seems to be some issue with gurobi which
            % increase optimization time over time.
            save("temp_iteration_variables_do_not_touch.mat");
            clear all;
            load("temp_iteration_variables_do_not_touch.mat");
            [results, params] = sim.Game.sim_cut_in(x_ini, "ego_vehice_type", ego_vehice_type, ...
                                                    "cut_in_vehicle_role", cut_in_vehicle_role, ...
                                                    "sigma", sigma);
            %% save workspace
            if run_num == 1
                save_result_name = sprintf("%s_%d_%s", base_name, ego_vehice_type, cut_in_vehicle_role);
                results_folder = results_folder_root;
            else
                results_folder = fullfile(results_folder_root, sprintf("%s_%d_%s", base_name, ego_vehice_type, cut_in_vehicle_role));
                save_result_name = sprintf("run%d", run);
            end
            if ~exist(results_folder, 'dir')
                mkdir(results_folder);
            end
            save(fullfile(results_folder, save_result_name + ".mat"), "results", "params");
            %% show time trajectory
            if run_num == 1
                close all;
                states = dictionary(["veh1", "veh2"], {results.ego_state(:, 1:end-1), results.cut_state(:, 1:end-1)});
                plotter.plot_trajectories(states);
                plotter.plot_one_topview(results.x_all, results.tsim);
                
                figure;
                ax = zeros(2,1);
                ax(1) = subplot(2,1,1); hold on;
                plot(results.tsim, results.roles_estimate_priors_all(1,:))
                ax(2) = subplot(2,1,2); hold on;
                plot(results.cut_game_output_flag(1, :), results.cut_game_output_flag(3, :))
                linkaxes(ax, "x")
            end
        end
    end
end
delete("temp_iteration_variables_do_not_touch.mat");
