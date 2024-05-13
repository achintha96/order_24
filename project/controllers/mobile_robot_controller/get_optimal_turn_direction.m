function [turn_direction] = get_optimal_turn_direction(bearing,target_bearing)
    
    dif_accw = (bearing - target_bearing);
    dif_ccw_cost = (360 - dif_accw);
    if dif_accw<0
        dif_accw = -dif_accw;
        dif_ccw_cost = -dif_ccw_cost;
    end
    
    if dif_accw > dif_ccw_cost
        turn_direction = 1;
    else
        turn_direction = -1;
    end
end

