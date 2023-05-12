function  [Qf,Pf,Rp,Re,R,Number_of_iterations] = gen_Weighting(s1,s2,s3,rp11,rp22,rp33)

    Qf11 = eye(3,3) * s1;
    Qf12 = eye(3,3) * s2;
    Qf21 = Qf12;
    Qf22 = eye(3,3) * s3;
    Qf = [ Qf11     Qf12;
           Qf21     Qf22 ];
    Pf = Qf;

    Rp = [ rp11     0       0;
            0      rp22     0;
            0       0       rp33 ];
        
    re11 = sqrt(2) * rp11;
    re22 = sqrt(2) * rp22;
    re33 = sqrt(2) * rp33;
    Re = [ re11     0       0;
            0      re22     0;
            0       0       re33 ];
  
    R = gen_R(Rp, Re);
    %ÀîÑÅÆÕÅµ·òµü´ú·¨
    Number_of_iterations = 20;
end