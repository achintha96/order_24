function [v1,v2,v3,v4] = spin_right(speed,coefficient)
    v1 = speed*coefficient;
    v2 = -speed*coefficient;
    v3 = speed*coefficient;
    v4 = -speed*coefficient;
end

