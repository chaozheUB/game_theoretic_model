function generate_multiple_videos(res, videoname)
    ego_pic = cell(1, 3);
    pic.img = 0; pic.alpha = 0;
    [pic.img, ~, pic.alpha] = imread(fullfile("most_conf_figs", "pics ", "blue_car.png"));
    ego_pic{1} = pic;
    [pic.img, ~, pic.alpha] = imread(fullfile("most_conf_figs", "pics ", "red_car.png"));
    ego_pic{2} = pic;
    [pic.img, ~, pic.alpha] = imread(fullfile("most_conf_figs", "pics ", "green_car.png"));
    ego_pic{3} = pic;
    colors = {"b-", "r--", "g:", "k:"};
    colors_text = {"b", "r", "g", "k"};
    
    cut_pic.img = 0; cut_pic.alpha = 0;
    other_pic.img = 0; other_pic.alpha = 0;
    [cut_pic.img, ~, cut_pic.alpha] = imread(fullfile("most_conf_figs", "pics ", "orange_car.png"));
    [other_pic.img, ~, other_pic.alpha] = imread(fullfile("most_conf_figs", "pics ", "grey_car.png"));

    num_runs = size(res, 1);
    veh_dim = res{1, 1}.params.game_param.reward.veh_dim;
    
    x_min = -10;
    x_max = 200;
    y_min = -2.1;
    y_max = 10.1;
    
    %% Start video
    if ispc || ismac
        writerObj = VideoWriter(videoname, 'MPEG-4'); % Name it.
    else
        writerObj = VideoWriter(videoname, 'Motion JPEG AVI'); % Name it.
    end
    writerObj.FrameRate = 10; % How many frames per second.
    open(writerObj); 
    L = zeros(15, num_runs);
    Lg = zeros(4, num_runs);
    Fig = figure();
    
    set(gcf, "unit", "inches");
    ps = get(gcf, "Position");
    width = 13;
    height = 6;
    set(gcf, "Position", [ps(1)/2, ps(2)/2, width, height])

    ax = zeros(num_runs + 1, 1);
    ax(1) = subplot(num_runs + 1,1,1);
    hold on; box on;grid on;
    title("Energy Consumption Profile");

    for iter = 1:num_runs
        result = res{iter, 1};
        x_all = result.results.x_all;
        tsim = result.results.tsim;
        w_ego_cum = result.results.w_ego_cum;

        if iter == 1
            steps_length = length(x_all);
        else
            steps_length = min(steps_length, length(x_all));
        end
        
        subplot(num_runs + 1, 1, 1);
        plot(tsim, w_ego_cum, colors{iter});
        plot(tsim(end), w_ego_cum(end), colors{iter}, 'Marker', 'x', "MarkerSize", 10);
        ax(iter + 1) = subplot(num_runs + 1, 1, iter + 1);
        hold on; box on;
        title(res{iter, 2});
        reduce_margin(gca);
        % Plot road (once and for all)
        lane_width = veh_dim.lane_width;
        plot([x_min, x_max*3], [- lane_width / 2, - lane_width / 2], "k")
        plot([x_min, x_max*3], [lane_width / 2, lane_width / 2], "k--")
        plot([x_min, x_max*3], [lane_width * 3 / 2, lane_width * 3 / 2], "k")
    end
    subplot(num_runs + 1, 1, 1)
    energy_xlimits = xlim;
    energy_ylimits = ylim;
    reduce_margin(gca);
    steps = 1:steps_length;
    % steps = 1;
    put_veh = @(pic, pos) image(pic.img,'xdata',[pos(1),  pos(1) + veh_dim.veh_length], ...
                                        'ydata',[pos(3) - veh_dim.veh_width / 2,  pos(3) + veh_dim.veh_width / 2],...
                                        'AlphaData', pic.alpha);
    x_focus = 0;
    for i = 1:length(steps)

        for iter = 1:num_runs
            if (i) > 1
                delete(L(:, iter));
                delete(Lg(:, iter));
            end
            %box on; grid on;
    
            result = res{iter, 1};
            x_all = result.results.x_all;
            tsim = result.results.tsim;
            w_ego_cum = result.results.w_ego_cum;
    
            axes(ax(1));
            % plot(tsim(1:steps(i)), w_ego_cum(1:steps(i)), colors{iter});
            L(15, iter) = plot(tsim(steps(i)), w_ego_cum(steps(i)), colors{iter}, 'Marker', 'o', "MarkerSize", 10);
            
            traffic_pos = x_all(7:9, steps(i));
            axes(ax(iter + 1));
            L(1, iter) = put_veh(ego_pic{iter}, x_all(1:3, steps(i)));
            L(2, iter) = put_veh(cut_pic, x_all(4:6, steps(i)));
            L(3, iter) = put_veh(other_pic, x_all(7:9, steps(i)));
            L(4, iter) = put_veh(other_pic, x_all(10:12, steps(i)));
            for j = 1:5
                L(4 + j, iter) = put_veh(other_pic, x_all(7:9, steps(i)) + [j* 30, 0, 0]' );
                L(9 + j, iter) = put_veh(other_pic, x_all(10:12, steps(i))+ [j* 30, 0, 0]');
            end
            
            Lg(1, iter)=text(x_focus + 50, 8, ['$v_{\rm ego}$', sprintf(' %.2f [m/s]', x_all(2, steps(i)))],'Color', colors_text{iter}, "Interpreter", "latex",'FontSize', 15);
            Lg(2, iter)=text(x_focus + 90, 8, ['$v_{\rm cut-in}$', sprintf(' %.2f [m/s]', x_all(5, steps(i)))],'Color', "#D95319", "Interpreter", "latex",'FontSize', 15);  
            Lg(3, iter)=text(x_focus + 130, 8, ['$v_{\rm traffic}$', sprintf(' %.2f [m/s]', x_all(8, steps(i)))], "Interpreter", "latex",'FontSize', 15); 
            Lg(4, iter)=text(x_focus + 20, 8, ['Time', sprintf(' %.1f [s]', tsim(steps(i)))], "Interpreter", "latex",'FontSize', 15); 
            % axis equal;
            x_focus = max(0, traffic_pos(1) - x_all(7, 1)-10);
            xlim(x_focus + [x_min, x_max]);
            ylim([y_min, y_max]);
        end
        axes(ax(1));
        xlim(energy_xlimits);
        ylim(energy_ylimits);
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
    ti(2) = ti(2) * 1.5;
    left = outerpos(1) + ti(1);
    bottom = outerpos(2) + ti(2);
    ax_width = outerpos(3) - ti(1) - ti(3);
    ax_height = outerpos(4) - ti(2) - ti(4);
    ax.Position = [left bottom ax_width ax_height];
end
