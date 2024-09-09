%
clear; close all;clc;
Util.plot_setup();
stk = dbstack; filepath = which(stk(1).file);
save_root = fileparts(filepath);
% save as svg for Inkscape editing
full_save_path = @(pic_name) fullfile(save_root, "most_conf_figs", 'results', pic_name + ".svg");
load_run = @(f) get_w_ego_cum(load(fullfile("cutin_results", f + ".mat")));
override = 0; % 1 will override the saved figure 
%%
colors = {"b-", "r--", "g:", "k:"};
v_idx = 3;
h_idx = 8;
lead_v_idx = 6;
long_u_idx = 9;
l_idx = 4;
prob_idx = 1;

%% no cut_in compare with delay
res = cell(3, 1);
res{1} = load_run("no_cut_in_1_0.6_0.6");
res{2} = load_run("no_cut_in_2_0.6_0.6");
% res{3} = load_run("cut_in_from_behind_4_Leader");
res{3} = load_run("cut_in_from_behind_4_Follower");

close all;
fig = figure();
ax = zeros(3, 1);
for i = 1:length(ax)
    ax(i) = subplot(length(ax),1,i);hold on; box on; grid on;
end
linkaxes(ax, "x");
for i = 1:length(res)
    subplot(3,1,1);
    plot(res{i}.results.tsim(1:end-1), res{i}.results.ego_state(v_idx, 1:end-1), colors{i});
    subplot(3,1,2);
    plot(res{i}.results.tsim(1:end-1), res{i}.results.ego_state(h_idx, 1:end-1), colors{i});
    subplot(3,1,3);
    plot(res{i}.results.tsim(1:end-1), res{i}.results.ego_state(long_u_idx, 1:end-1), colors{i});
end
xlim([0, res{i}.results.tsim(end)])
subplot(3,1,1);
% plot(res{i}.results.tsim(1:end-1), res{i}.results.ego_state(lead_v_idx, 1:end-1), colors{end});
plot([1, 2], [19, 19], colors{1})
plot([5.5, 6.5], [19, 19], colors{2})
plot([1, 2], [17, 17], colors{3})
% plot([5.5, 6.5], [17, 17], colors{4})
ylim([15.9, 24]);
set(gcf, "unit", "inches");
ps = get(gcf, "Position");
width = 8;
height = 6;
set(gcf, "Position", [ps(1)/3, ps(2)/3, width, height])
check_before_save(fig, full_save_path('no_cut_in_compare_with_delay_raw'), override);


%% Cut in in the front as leader
res = cell(3, 1);
res{1} = load_run(fullfile("normal_cut_in_1_Leader", "run1"));
res{2} = load_run(fullfile("normal_cut_in_2_Leader", "run1"));
res{3} = load_run(fullfile("normal_cut_in_4_Leader", "run1"));

close all;
fig = figure();
ax = zeros(6, 1);
for i = 1:length(ax)
    ax(i) = subplot(length(ax),1,i);hold on; box on; grid on;
end
linkaxes(ax, "x");

for i = 1:length(res)
    subplot(6,1,1);
    plot(res{i}.results.tsim(1:end-1), res{i}.results.ego_state(v_idx, 1:end-1), colors{i});
    subplot(6,1,2);
    plot(res{i}.results.tsim(1:end-1), res{i}.results.ego_state(h_idx, 1:end-1), colors{i});
    subplot(6,1,3);
    plot(res{i}.results.tsim(1:end-1), res{i}.results.ego_state(long_u_idx, 1:end-1), colors{i});
    subplot(6,1,4);
    plot(res{i}.results.tsim(1:end-1), res{i}.results.cut_state(l_idx, 1:end-1), colors{i});
    subplot(6,1,5);
    plot(res{i}.results.tsim(1:end-1), res{i}.results.cut_state(v_idx, 1:end-1), colors{i});
    subplot(6,1,6);
    plot(res{i}.results.tsim(1:end-1), res{i}.results.roles_estimate_priors_all(prob_idx, 1:end-1), colors{i});
end

xlim([0, res{i}.results.tsim(end)]);
subplot(6,1,1);
ylim([13, 24]);
subplot(6,1,5);
ylim([13, 24]);

subplot(6,1,6);
ylim([-0.05, 1.05]);
plot([1, 2], [0.7, 0.7], colors{1})
plot([7, 8], [0.7, 0.7], colors{2})
plot([1, 2], [0.3, 0.3], colors{3})
% plot([7, 8], [0.3, 0.3], colors{4})

set(gcf, "unit", "inches");
ps = get(gcf, "Position");
width = 8;
height = 10;
set(gcf, "Position", [ps(1)/3, ps(2)/3, width, height])
% saveas(fig, full_save_path('normal_cut_in_as_Leader_raw'));

figure; hold on;grid on; box on;
set(gcf, "unit", "inches");
ps = get(gcf, "Position");
width = 13;
height = 1.5;
set(gcf, "Position", [ps(1)/3, ps(2)/3, width, height])
for i = 1:length(res)
    plot(res{i}.results.tsim, res{i}.results.w_ego_cum, colors{i});
end
%% Cut in in the front as follower
res = cell(3, 1);
res{1} = load_run(fullfile("normal_cut_in_1_Follower", "run1"));
res{2} = load_run(fullfile("normal_cut_in_2_Follower", "run1"));
res{3} = load_run(fullfile("normal_cut_in_4_Follower", "run1"));

close all;
fig = figure();
ax = zeros(6, 1);
for i = 1:length(ax)
    ax(i) = subplot(length(ax),1,i);hold on; box on; grid on;
end
linkaxes(ax, "x");

for i = 1:length(res)
    subplot(6,1,1);
    plot(res{i}.results.tsim(1:end-1), res{i}.results.ego_state(v_idx, 1:end-1), colors{i});
    subplot(6,1,2);
    plot(res{i}.results.tsim(1:end-1), res{i}.results.ego_state(h_idx, 1:end-1), colors{i});
    subplot(6,1,3);
    plot(res{i}.results.tsim(1:end-1), res{i}.results.ego_state(long_u_idx, 1:end-1), colors{i});
    subplot(6,1,4);
    plot(res{i}.results.tsim(1:end-1), res{i}.results.cut_state(l_idx, 1:end-1), colors{i});
    subplot(6,1,5);
    plot(res{i}.results.tsim(1:end-1), res{i}.results.cut_state(v_idx, 1:end-1), colors{i});
    subplot(6,1,6);
    plot(res{i}.results.tsim(1:end-1), res{i}.results.roles_estimate_priors_all(prob_idx, 1:end-1), colors{i});
end

xlim([0, res{i}.results.tsim(end)]);
subplot(6,1,1);
ylim([13, 24]);
subplot(6,1,5);
ylim([13, 24]);
subplot(6,1,6);
ylim([-0.05, 1.05]);
plot([1, 2], [0.7, 0.7], colors{1})
plot([7, 8], [0.7, 0.7], colors{2})
plot([1, 2], [0.3, 0.3], colors{3})
% plot([7, 8], [0.3, 0.3], colors{4})

set(gcf, "unit", "inches");
ps = get(gcf, "Position");
width = 8;
height = 10;
set(gcf, "Position", [ps(1)/3, ps(2)/3, width, height])
check_before_save(fig, full_save_path('normal_cut_in_as_Follower_raw'), override);
figure; hold on;grid on; box on;
for i = 1:length(res)
    plot(res{i}.results.tsim, res{i}.results.w_ego_cum, colors{i});
end
