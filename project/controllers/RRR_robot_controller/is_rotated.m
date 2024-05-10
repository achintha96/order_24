function [speed,status] = find_IK(limit,current)

if imabsdiff(limit, current) < 0.01
  %wb_motor_set_velocity(motor_1, 0);
  speed = 0;
  status = 1;
else
  status = 0;
  if limit > current
    speed = 0.1;
  else
    speed = -0.1;
  end
end

end

