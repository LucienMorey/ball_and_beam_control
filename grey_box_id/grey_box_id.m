
k_theta_0 = 1.0;
k_theta_dot_0 = 1.0;
k_v_0 = 2.0;

function_params = {'k_theta',     k_theta_0;
                   'k_theta_dot', k_theta_dot_0;
                   'k_v',         k_v_0};

function_type = 'c';

sys = idgrey(@motor_dynamics,function_params,function_type);

idgrey