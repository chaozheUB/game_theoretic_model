%
clear; close all;clc;
get_full_name = @(f) fullfile("cutin_results", f + ".mat");
%% get summary for more than one 
% base_name = "cut_in_from_behind_no_delay";
base_name = "normal_cut_in";
% base_name = "cut_in_from_behind";

role = "Follower";
total_run = 10;
type = {"1", "2", "4"};
summary_name = get_full_name(fullfile(base_name + "_" + role + "_energy_summary"));
if exist(summary_name, "file")
    disp(summary_name);
    fprintf(" already exisits. Loaded.\n")
    load(summary_name);
else
    energy_ego_sum = zeros(total_run, 3);
    energy_cut_sum = zeros(total_run, 3);
    
    for run = 1:total_run
        for j = 1:length(type)
            res = load(get_full_name(fullfile(base_name + "_" + type{j} + "_" + role, "run" + run)));
            energy_ego_sum(run, j) = res.results.w_ego;
            energy_cut_sum(run, j) = res.results.w_cut;
        end
    end
    save(summary_name, "energy_cut_sum", "energy_ego_sum");
end


%%
base_name = "no_cut_in";
sigma_all = [0.0, 0.6, 0.6];
sigma_considered_all = [0.0, 0.0, 0.6];
type = {"1", "2"};

summary_name = get_full_name(fullfile(base_name + "_delay_energy_summary"));
if exist(summary_name, "file")
    disp(summary_name);
    fprintf(" already exisits. Loaded.\n")
    load(summary_name);
else
    energy_ego_sum = zeros(length(sigma_all), 4);
    
    for i = 1:length(sigma_all)
        sigma = sigma_all(i);
        sigma_considered = sigma_considered_all(i);
        for j = 1:length(type)
            res = load(get_full_name(fullfile(sprintf("%s_%s_%.1f_%.1f", base_name, type{j}, sigma, sigma_considered))));
            energy_ego_sum(i, 1) = res.params.ego_param.sigma;
            energy_ego_sum(i, 2) = res.params.ego_param.considered_delay;
            energy_ego_sum(i, 2 + j) = res.results.w_ego;
        end
    end
    save(summary_name, "energy_ego_sum");
end