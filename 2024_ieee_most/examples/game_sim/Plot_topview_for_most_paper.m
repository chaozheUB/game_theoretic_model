%
clear; close all;clc;
Util.plot_setup();
stk = dbstack; filepath = which(stk(1).file);
save_root = fileparts(filepath);
% save as svg for Inkscape editing
full_save_path = @(pic_name) fullfile(save_root, "most_conf_figs", 'results', pic_name + ".svg");
load_run = @(f) load(fullfile("cutin_results", f + ".mat"));
override = 0; % 1 will override the saved figure 
%% cut_in_From_behind
title = "cut_in_from_behind_4_Follower";
ego_pic = imread(fullfile("most_conf_figs", "pics ", "blue_car.png"));
cut_pic = imread(fullfile("most_conf_figs", "pics ", "red_car.png"));
other_pic = imread(fullfile("most_conf_figs", "pics ", "grey_car.png"));
% frame_rate = 30;
frame = [1, 11, 21, 31, 41];
res = load_run(title);
s_all = res.results.x_all;
veh_dim = res.params.game_param.reward.veh_dim;

simlen = size(s_all, 2);
x_min = -10;
x_max = 200;
y_min = -6.5;
y_max = 6.5;


fig = figure();
if ~isempty(title)
    set(fig, "Name", title)
end

set(gcf, "unit", "inches");
ps = get(gcf, "Position");
width = 13;
height = 7;
set(gcf, "Position", [ps(1)/10, ps(2)/10, width, height])


lane_width = veh_dim.lane_width;
put_veh = @(pic, pos) image(pic,'xdata',[pos(1),  pos(1) + veh_dim.veh_length], ...
                          'ydata',[pos(3) - veh_dim.veh_width / 2,  pos(3) + veh_dim.veh_width / 2]);
for i = 1:length(frame)
    ax = subplot(length(frame), 1, i);hold on;
    %box on; grid on;
    plot([x_min, x_max], [- lane_width / 2, - lane_width / 2], "k")
    plot([x_min, x_max], [lane_width / 2, lane_width / 2], "k--")
    plot([x_min, x_max], [lane_width * 3 / 2, lane_width * 3 / 2], "k")
    xlim([x_min, x_max]);
    ylim([y_min, y_max]);
    ego_pos = s_all(1:3, frame(i));
    put_veh(ego_pic, s_all(1:3, frame(i)));
    put_veh(cut_pic, s_all(4:6, frame(i)));
    put_veh(other_pic, s_all(7:9, frame(i)));
    put_veh(other_pic, s_all(10:12, frame(i)));
    for j = 1:5
        put_veh(other_pic, s_all(7:9, frame(i)) + [j* 30, 0, 0]' );
        put_veh(other_pic, s_all(10:12, frame(i))+ [j* 30, 0, 0]');
    end
    axis equal;
end

check_before_save(fig, full_save_path(title + "_top_view_raw"), override);

%% cut_in leader
title = "normal_cut_in_4_Leader";

% frame_rate = 30;
frame = [1, 11, 21, 31, 41];
res = load_run( fullfile(title, "run1"));
s_all = res.results.x_all;
veh_dim = res.params.game_param.reward.veh_dim;

simlen = size(s_all, 2);
x_min = -10;
x_max = 200;
y_min = -6.5;
y_max = 6.5;


fig = figure();
if ~isempty(title)
    set(fig, "Name", title)
end

set(gcf, "unit", "inches");
ps = get(gcf, "Position");
width = 13;
height = 7;
set(gcf, "Position", [ps(1)/10, ps(2)/10, width, height])


lane_width = veh_dim.lane_width;
put_veh = @(pic, pos) image(pic,'xdata',[pos(1),  pos(1) + veh_dim.veh_length], ...
                          'ydata',[pos(3) - veh_dim.veh_width / 2,  pos(3) + veh_dim.veh_width / 2]);
for i = 1:length(frame)
    ax = subplot(length(frame), 1, i);hold on;box on; grid on;
    plot([x_min, x_max], [- lane_width / 2, - lane_width / 2], "k")
    plot([x_min, x_max], [lane_width / 2, lane_width / 2], "k--")
    plot([x_min, x_max], [lane_width * 3 / 2, lane_width * 3 / 2], "k")
    xlim([x_min, x_max]);
    ylim([y_min, y_max]);
    ego_pos = s_all(1:3, frame(i));
    put_veh(ego_pic, s_all(1:3, frame(i)));
    put_veh(cut_pic, s_all(4:6, frame(i)));
    put_veh(other_pic, s_all(7:9, frame(i)));
    put_veh(other_pic, s_all(10:12, frame(i)));
    for j = 1:5
        put_veh(other_pic, s_all(7:9, frame(i)) + [j* 30, 0, 0]' );
        put_veh(other_pic, s_all(10:12, frame(i))+ [j* 30, 0, 0]');
    end
    axis equal;
end
check_before_save(fig, full_save_path(title + "_top_view_raw"), override);

%% cut_in follower
title = "normal_cut_in_4_Follower";

% frame_rate = 30;
frame = [1, 11, 21, 31, 41];
res = load_run( fullfile(title, "run1"));
s_all = res.results.x_all;
veh_dim = res.params.game_param.reward.veh_dim;

simlen = size(s_all, 2);
x_min = -10;
x_max = 200;
y_min = -6.5;
y_max = 6.5;


fig = figure();
if ~isempty(title)
    set(fig, "Name", title)
end

set(gcf, "unit", "inches");
ps = get(gcf, "Position");
width = 13;
height = 7;
set(gcf, "Position", [ps(1)/10, ps(2)/10, width, height])


lane_width = veh_dim.lane_width;
put_veh = @(pic, pos) image(pic,'xdata',[pos(1),  pos(1) + veh_dim.veh_length], ...
                          'ydata',[pos(3) - veh_dim.veh_width / 2,  pos(3) + veh_dim.veh_width / 2]);
for i = 1:length(frame)
    ax = subplot(length(frame), 1, i);hold on;box on; grid on;
    plot([x_min, x_max], [- lane_width / 2, - lane_width / 2], "k")
    plot([x_min, x_max], [lane_width / 2, lane_width / 2], "k--")
    plot([x_min, x_max], [lane_width * 3 / 2, lane_width * 3 / 2], "k")
    xlim([x_min, x_max]);
    ylim([y_min, y_max]);
    ego_pos = s_all(1:3, frame(i));
    put_veh(ego_pic, s_all(1:3, frame(i)));
    put_veh(cut_pic, s_all(4:6, frame(i)));
    put_veh(other_pic, s_all(7:9, frame(i)));
    put_veh(other_pic, s_all(10:12, frame(i)));
    for j = 1:5
        put_veh(other_pic, s_all(7:9, frame(i)) + [j* 30, 0, 0]' );
        put_veh(other_pic, s_all(10:12, frame(i))+ [j* 30, 0, 0]');
    end
    axis equal;
end
check_before_save(fig, full_save_path(title + "_top_view_raw"), override);
