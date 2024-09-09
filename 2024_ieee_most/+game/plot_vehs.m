function fig = plot_vehs(vehs, dim, name_ext, ax)
    % for simplicity only has vehs x and y
    colors = {'b', 'r', 'g', 'k', 'c', 'm', 'y'};
    if isempty(ax)
        fig = figure();
        set(fig, "Name", "veh position" + name_ext)
        % set(gcf, "unit", "inches");
        % ps = get(gcf, "Position");
        % width = 15;
        % height = 9;
        % set(gcf, "Position", [ps(1), ps(2), width, height])
    else
        axes(ax)
        fig = ax;
    end
    hold on;box on;
    n = length(vehs) / 3;
    legend_text = cell(n, 1);
    obj_handles = zeros(n, 1);
    for i = 1:n
        x = vehs((i - 1) * 3 + 1:i * 3);
        [handles, ~] = BoxDraw(x(1), x(3), 0.0, dim, colors{i});
        obj_handles(i) = handles;
        if ~isempty(ax)
            legend_text{i} = sprintf("%s car %.d", name_ext, i);
        else
            legend_text{i} = sprintf("car %.d", i);
        end
    end
    legend(obj_handles, legend_text)
    hold off;
end

function [handle, Box] = BoxDraw(x, y, psi, para, color)
    L = para.veh_length; % length of the box
    W = para.veh_width; % width of the box
    d = para.veh_d; % distance from the (x,y) to the rear bumper
    
    Pr=[x-d*cos(psi);y-d*sin(psi)];
    Pf=[x+(L-d)*cos(psi);y+(L-d)*sin(psi)];
    
    Prl=Pr+W/2.*[-sin(psi); cos(psi)]; %rear left wheel center
    Prr=Pr-W/2.*[-sin(psi); cos(psi)]; %rear right wheel center
    Pfl=Pf+W/2.*[-sin(psi); cos(psi)]; %front left wheel center
    Pfr=Pf-W/2.*[-sin(psi); cos(psi)]; %front right wheel center
    
    linex=[Prl(1),Prr(1),Pfr(1),Pfl(1),Prl(1)];
    liney=[Prl(2),Prr(2),Pfr(2),Pfl(2),Prl(2)];
    Box=[linex(1:4);liney(1:4)];
    handle(1)=plot(linex, liney, color, 'LineWidth',2);
end