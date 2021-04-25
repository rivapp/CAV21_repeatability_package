function [x, done] = dynamics(x_0, u)
%MC_DYNAMICS Summary of this function goes here
%   Detailed explanation goes here

vel = x_0(2) + 0.0015 * u - 0.0025 * cos(3 * x_0(1));
pos = x_0(1) + vel;

if vel >= 0.07
    vel = 0.07;
elseif vel <= -0.07
    vel = -0.07;
end

if pos < -1.2
    pos = -1.2;
    vel = 0;
end

x = [pos; vel];

done = 0;

if pos > 0.45
    done = 1;

end

