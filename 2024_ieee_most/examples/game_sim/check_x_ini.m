function is_ok = check_x_ini(x_ini, delta_a, print_info)
    if print_info
        fprintf("Checking initial conditions specific to 4 car cases. Make sure the maximum deceleration can avoid collision. \n");
    end
    v0 = x_ini(2);
    v1 = x_ini(5);
    v2 = x_ini(8);
    v3 = x_ini(11);
    s0 = x_ini(1);
    s1 = x_ini(4);
    s2 = x_ini(7);
    s3 = x_ini(10);
    is_ok = true;
    if (v1^2 - v2^2) / 2 / delta_a >= s2 - s1
        is_ok = false;
        if print_info
            fprintf("Distance between veh 1 and veh 2 too close, will crash. \n");
            fprintf("veh 2 needs to be at least %.1f ahead of veh 1. \n", (v1^2 - v2^2) / 2 / delta_a);
        end
    end
    if (v3^2 - v0^2) / 2 / delta_a >= s3 - s0
        is_ok = false;
        if print_info
            fprintf("Distance between veh 0 and veh 3 too close, will crash. \n");
            fprintf("veh 3 needs to be at least %.1f ahead of veh 0. \n", (v3^2 - v0^2) / 2 / delta_a);
        end
    end
end