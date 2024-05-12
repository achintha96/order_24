function [ccw_turn] = get_optimal_turn_direction(bearing,target_bearing)
    accw_cost = 0;
    ccw_cost = 0;
    if bearing>target_bearing
        accw_cost = bearing - target_bearing
        ccw_cost = 360 - bearing + target_bearing
    else
        ccw_cost = target_bearing - bearing
        accw_cost = 360 - target_bearing + bearing
    end

    if accw_cost > ccw_cost
        ccw_turn = 1
    else
        ccw_turn = -1
    end
end

