function [REV_SPD] = get_rev_speed(bearing,target_bearing,MAX_OMEGA)
    if imabsdiff(bearing,target_bearing) > 0.5 %2
        REV_SPD = MAX_OMEGA*0.1; %0.1
    else
        REV_SPD = MAX_OMEGA*0.0025;
    end
end

