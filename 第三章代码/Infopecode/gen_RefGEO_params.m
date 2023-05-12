function [v, T, a, omega] = gen_RefGEO_params(mu, R)
    v = sqrt(mu/R);
    T = 2 * pi / sqrt(mu) * R^(3/2);
    a = v^2 / R;
    omega = sqrt(mu / R^3);
end