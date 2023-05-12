function gen_params(s1,s3)
    global params
    s2 = 0;
    rp11 = 1e-5;
    rp22 = 1e-5;
    rp33 = 1e-5;
    [params.Qf,params.Pf,params.Rp,params.Re,params.R,...
        params.Number_of_iterations] = gen_Weighting(s1,s2,s3,rp11,rp22,rp33);
end