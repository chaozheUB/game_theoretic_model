%
clear; close all;clc;
Util.plot_setup();
% save as svg for Inkscape editing
full_save_path = @(video_name) fullfile("most_conf_videos", 'results', video_name);
load_run = @(f)  get_w_ego_cum(load(fullfile("cutin_results", f + ".mat")));
% make_video = @(title, title_show) generate_video(load_run(fullfile(title, "run1")), title, full_save_path(title + "_video"), title_show);
%% no cut in
make_video = @(title, title_show) generate_video(load_run(title), title, full_save_path(title + "_video"), title_show);
make_video("cut_in_from_behind_4_Follower", "Cut in vehicle cut-in from behind, Eco Driving accounts for cut-in");
make_video("no_cut_in_1_0.6_0.6", "No cut in, Baseline OVM");
make_video("no_cut_in_2_0.6_0.6", "No cut in, Eco Driving");
%% No cut in
res = cell(3, 3);
res{1, 1} = load_run("no_cut_in_1_0.6_0.6");
res{1, 2} = "No cut in, Baseline OVM";
res{2, 1} = load_run("no_cut_in_2_0.6_0.6");
res{2, 2} = "No cut in, Eco Driving";
res{3, 1} = load_run("cut_in_from_behind_4_Follower");
res{3, 2} = "Cut in vehicle cut-in from behind, Eco Driving accounts for cut-in";

generate_multiple_videos(res, full_save_path("no_cut_in_with_energy"));
%%
make_video = @(title, title_show) generate_video(load_run(fullfile(title, "run1")), title, full_save_path(title + "_video"), title_show);
%
make_video("normal_cut_in_1_Leader", "Cut in vehicle holds leader role, Baseline OVM");
make_video("normal_cut_in_2_Leader", "Cut in vehicle holds leader role, Eco Driving");
make_video("normal_cut_in_4_Leader", "Cut in vehicle holds leader role, Eco Driving accounts for cut-in");
% 
% %%
make_video("normal_cut_in_1_Follower", "Cut in vehicle holds follower role, Baseline OVM");
make_video("normal_cut_in_2_Follower", "Cut in vehicle holds follower role, Eco Driving");
make_video("normal_cut_in_4_Follower", "Cut in vehicle holds follower role, Eco Driving accounts for cut-in");

%%
res = cell(3, 3);
res{1, 1} = load_run(fullfile("normal_cut_in_1_Leader", "run1"));
res{1, 2} = "Cut in vehicle holds leader role, Baseline OVM";
res{2, 1} = load_run(fullfile("normal_cut_in_2_Leader", "run1"));
res{2, 2} = "Cut in vehicle holds leader role, Eco Driving";
res{3, 1} = load_run(fullfile("normal_cut_in_4_Leader", "run1"));
res{3, 2} = "Cut in vehicle holds leader role, Eco Driving accounts for cut-in";

generate_multiple_videos(res, full_save_path("normal_cut_in_leader_with_energy"));
%%
close all;
res = cell(3, 3);
res{1, 1} = load_run(fullfile("normal_cut_in_1_Follower", "run1"));
res{1, 2} = "Cut in vehicle holds follower role, Baseline OVM";
res{2, 1} = load_run(fullfile("normal_cut_in_2_Follower", "run1"));
res{2, 2} = "Cut in vehicle holds follower role, Eco Driving";
res{3, 1} = load_run(fullfile("normal_cut_in_4_Follower", "run1"));
res{3, 2} = "Cut in vehicle holds follower role, Eco Driving accounts for cut-in";

generate_multiple_videos(res, full_save_path("normal_cut_in_follower_with_energy"));

% %%
% make_video("normal_cut_in_4_Leader", "Cut in vehicle holds leader role, Eco Driving accounts for cut-in");
% make_video("normal_cut_in_4_Follower", "Cut in vehicle holds follower role, Eco Driving accounts for cut-in");
