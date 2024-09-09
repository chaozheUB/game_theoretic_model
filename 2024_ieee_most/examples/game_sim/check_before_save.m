function check_before_save(fig, target_name, override)
    if exist(target_name, "file")
        disp(target_name);
        fprintf(" already exisits. \n");
        if override
            fprintf(" Will override. \n");
        end
    else
        override = 1;
    end
    if override
        saveas(fig, target_name);
    end
end