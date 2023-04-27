function [p1,p2] = second_order_poles(overshoot,settling_time)

    zeta = sqrt((log(overshoot)^2)/((pi^2)+log(overshoot)^2));

    w_n = 4/(settling_time*zeta);
    p1 = -zeta*w_n + w_n*sqrt(zeta^2-1);
    p2 = -zeta*w_n - w_n*sqrt(zeta^2-1);
end

