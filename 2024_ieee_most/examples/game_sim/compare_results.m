%
clear; close all;clc;
Util.plot_setup();
% this is mainly for visualization, for paper, use the standalone name
plotter = game.plot_utils();
get_full_name = @(f) fullfile("cutin_results", f + ".mat");
% get_full_name = @(f) fullfile("temp_results", f + ".mat");
full_save_path = @(pic_name) fullfile("cutin_results", 'figures', pic_name + ".eps");
%% show time trajectory
run_dict = dictionary();
% name_base = "cut_in_from_behind_no_delay";
% name_base = "normal_cut_in_new_test";
name_base = "cut_in_from_behind";

role = "Leader";
run = "";
%%
% run_dict("ovm") = get_full_name(name_base + "_1_5_Follower");
% run_dict("mpc") = get_full_name(name_base + "_2_5_Follower");
% run_dict("mpc consider cut-in") = get_full_name(name_base + "_4_5_Follower");
% % 
% [energy_dict, states, veh1_states, figs] = plotter.compare_plot(run_dict);
% energy_dict

if run ~= ""
    run_dict("ovm") = get_full_name(fullfile(name_base + "_1_" + role, "run" + run));
    run_dict("mpc") = get_full_name(fullfile(name_base + "_2_" + role, "run" + run));
    run_dict("mpc consider cut-in") = get_full_name(fullfile(name_base + "_4_" + role, "run" + run));
else

    run_dict("ovm") = get_full_name(fullfile(name_base + "_1_" + role));
    run_dict("mpc") = get_full_name(fullfile(name_base + "_2_" + role));
    run_dict("mpc consider cut-in") = get_full_name(fullfile(name_base + "_4_" + role));
end
% run_dict("ovm leader") = get_full_name(name_base + "_1_5_" + "Leader");
% run_dict("ovm Follower") = get_full_name(name_base + "_1_5_" + "Follower");
% 
[energy_dict, states, veh1_states, figs] = plotter.compare_plot(run_dict);
energy_dict

% energy_dict = plotter.get_energy(run_dict)
%%
% num_keys =  length(run_dict.keys());
% keys = run_dict.keys();
% for i = 1:num_keys
%     Util.exportfig_rgb(figs(i), ...
%                   full_save_path(name_base + '_' + role + '_' + keys(i)),...
%                   'width',10, 'height',6, ...
%                   'fontmode','fixed', 'fontsize',10, 'color', 'rgb');
% end
% Util.exportfig_rgb(figs(end), ...
%               full_save_path(name_base + '_' + role + '_veh1'),...
%               'width',6, 'height',10, ...
%               'fontmode','fixed', 'fontsize',10, 'color', 'rgb');
% Util.exportfig_rgb(figs(end - 1), ...
%               full_save_path(name_base + '_' + role + '_veh0'),...
%               'width',6, 'height',10, ...
%               'fontmode','fixed', 'fontsize',10, 'color', 'rgb');

%% quick check in case one want check the intermidate decision
% all_results = load(fullfile(results_folder, save_result_name + ".mat"));
% plotter.load_results(all_results.results)
% plotter.check_frame(0.0)
