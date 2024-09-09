function res = get_w_ego_cum(res)
    ego_param = res.params.ego_param;
    tsim = res.results.tsim;
    simlen = length(tsim);
    u_ego_all = res.results.u_ego_all;
    x_all = res.results.x_all;
    if ego_param.q == 0
        a_ego = [u_ego_all;u_ego_all(end)];
    else
        a_ego = u_ego_all(1: simlen);
    end
    v_ego = x_all(2, :);
    w_tmp = max(a_ego(:) + ego_param.ar + ego_param.cr * v_ego(:).^2, 0) .* v_ego(:);
    w_ego_cum = cumtrapz(tsim, w_tmp);
    res.results.w_ego_cum = w_ego_cum;
end
