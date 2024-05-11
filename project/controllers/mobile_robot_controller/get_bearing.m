function [bearing] = get_bearing(orientation,previous_bearing)

  rad = atan2(orientation(2), orientation(1));
  bearing = (rad - 1.5708) / pi * 180.0;
  if bearing < 0.0
    bearing = bearing + 360.0;
  end
  bearing = round(bearing*10000);
  bearing = bearing/10000;
  % if bearing == 360
  if imabsdiff(bearing,360)<0.5
    bearing=0;
  end

  % if imabsdiff(bearing,previous_bearing)>10
    % bearing = previous_bearing;
  % end

end

