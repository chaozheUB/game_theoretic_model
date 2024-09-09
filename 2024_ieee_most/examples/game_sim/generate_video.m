function generate_video(res, run_name, videoname, videotitle)
ego_pic.img = 0; ego_pic.alpha = 0;
cut_pic.img = 0; cut_pic.alpha = 0;
other_pic.img = 0; other_pic.alpha = 0;
% [ego_pic.img, ~, ego_pic.alpha] = imread(fullfile("most_conf_figs", "pics ", "blue_car.png"));
[ego_pic.img, ~, ego_pic.alpha] = imread(fullfile("most_conf_figs", "pics ", "green_car.png"));
[cut_pic.img, ~, cut_pic.alpha] = imread(fullfile("most_conf_figs", "pics ", "orange_car.png"));
[other_pic.img, ~, other_pic.alpha] = imread(fullfile("most_conf_figs", "pics ", "grey_car.png"));

x_all = res.results.x_all;
tsim = res.results.tsim;
veh_dim = res.params.game_param.reward.veh_dim;

x_min = -10;
x_max = 200;
y_min = -2.1;
y_max = 10.1;

%% Start video
if ispc || ismac
    writerObj = VideoWriter(videoname,'MPEG-4'); % Name it.
else
    writerObj = VideoWriter(videoname,'Motion JPEG AVI'); % Name it.
end
writerObj.FrameRate = 10; % How many frames per second.
open(writerObj); 
L = zeros(14, 1);
Lg = zeros(4, 0);
Fig = figure();
% tiledlayout(1,1,'TileSpacing','Compact','Padding','Compact');
hold on;box on;
if ~isempty(run_name)
    set(Fig, "Name", run_name)
end

set(gcf, "unit", "inches");
ps = get(gcf, "Position");
width = 13;
height = 1.5;
set(gcf, "Position", [ps(1)/2, ps(2)/2, width, height])
if ~isempty(videotitle)
    title(videotitle)
end

reduce_margin(gca)
% Plot road (once and for all)
lane_width = veh_dim.lane_width;
plot([x_min, x_max*3], [- lane_width / 2, - lane_width / 2], "k")
plot([x_min, x_max*3], [lane_width / 2, lane_width / 2], "k--")
plot([x_min, x_max*3], [lane_width * 3 / 2, lane_width * 3 / 2], "k")

x_focus =  0;
%% show every steps
steps = 1:length(x_all(1,:));
% used to show up to 10 seconds
% steps = steps(res.results.tsim <= 10);

put_veh = @(pic, pos) image(pic.img,'xdata',[pos(1),  pos(1) + veh_dim.veh_length], ...
                                    'ydata',[pos(3) - veh_dim.veh_width / 2,  pos(3) + veh_dim.veh_width / 2],...
                                    'AlphaData', pic.alpha);

for i = 1:length(steps)
    traffic_pos = x_all(7:9, steps(i));
    if (i) > 1
        delete(L);
        delete(Lg);
    end
    %box on; grid on;

    L(1) = put_veh(ego_pic, x_all(1:3, steps(i)));
    L(2) = put_veh(cut_pic, x_all(4:6, steps(i)));
    L(3) = put_veh(other_pic, x_all(7:9, steps(i)));
    L(4) = put_veh(other_pic, x_all(10:12, steps(i)));
    for j = 1:5
        L(4 + j) = put_veh(other_pic, x_all(7:9, steps(i)) + [j* 30, 0, 0]' );
        L(9 + j) = put_veh(other_pic, x_all(10:12, steps(i))+ [j* 30, 0, 0]');
    end
    
    % Lg(1)=text(x_focus + 50, 8, ['$v_{\rm ego}$', sprintf(' %.2f [m/s]', x_all(2, steps(i)))],'Color', "b", "Interpreter", "latex",'FontSize', 15);
    Lg(1)=text(x_focus + 50, 8, ['$v_{\rm ego}$', sprintf(' %.2f [m/s]', x_all(2, steps(i)))],'Color', "g", "Interpreter", "latex",'FontSize', 15);
    Lg(2)=text(x_focus + 90, 8, ['$v_{\rm cut-in}$', sprintf(' %.2f [m/s]', x_all(5, steps(i)))],'Color', "#D95319", "Interpreter", "latex",'FontSize', 15);  
    Lg(3)=text(x_focus + 130, 8, ['$v_{\rm traffic}$', sprintf(' %.2f [m/s]', x_all(8, steps(i)))], "Interpreter", "latex",'FontSize', 15); 
    Lg(4)=text(x_focus + 20, 8, ['Time', sprintf(' %.1f [s]', tsim(steps(i)))], "Interpreter", "latex",'FontSize', 15); 
    % axis equal;
    % x_focus = max(0, traffic_pos(1) - x_max / 2);
    x_focus = max(0, traffic_pos(1) - x_all(7, 1)-10);
    xlim(x_focus + [x_min, x_max]);
    ylim([y_min, y_max]);
    % axis tight
    frame = getframe(Fig); 
    writeVideo(writerObj, frame);
end

close(writerObj); % Saves the movie.
close(Fig);
end

function reduce_margin(ax)
    %% smaller margin
    % make sure all titles are already generated, otherwise will be removed
    % when removing margin
    % https://www.mathworks.com/matlabcentral/answers/369399-removing-the-grey-margin-of-a-plot#answer_293285
    outerpos = ax.OuterPosition;
    ti = ax.TightInset; 
    left = outerpos(1) + ti(1);
    bottom = outerpos(2) + ti(2);
    ax_width = outerpos(3) - ti(1) - ti(3);
    ax_height = outerpos(4) - ti(2) - ti(4);
    ax.Position = [left bottom ax_width ax_height];
end
