clc;clear    
initialize
    %global params
     
     A = params.A;
     B = params.B;
    
    k1 = 1; 
    k2 = 1; 
    k3 = 1;
    k4 = 10;
    
    g = 9.78;
    Tei = 0.0004*g;
    Tpi = 0.0006*g;
    xx = [-4.5*1000; -15*1000; 2*1000; -3; 7.5; 0];
    step = 1;
    tau_int = 0:step:1809.38644367395;
    lamda_tf = [0.102635537813599;0.0286604207557571;-0.00950301199190852;0;0;0];
    
    options = odeset('AbsTol', 1e-12, 'RelTol', 1e-8);
    [~,lamda_aut] = ode45(@(t,lamda)lamda_dot(t,lamda),tau_int,lamda_tf,options);
    lamda_t = lamda_aut(end:-1:1,:)
    
    num = size(lamda_t,1);
    
    for i = 1:num
    lamdat = lamda_t(i,:);
    
    salphap = lamdat(5)/sqrt(lamdat(4)^2+lamdat(5)^2);
    calphap = lamdat(4)/sqrt(lamdat(4)^2+lamdat(5)^2);
    sbetap = lamdat(6)/sqrt(lamdat(4)^2+lamdat(5)^2+lamdat(6)^2);
    cbetap = sqrt(lamdat(4)^2+lamdat(5)^2)/sqrt(lamdat(4)^2+lamdat(5)^2+lamdat(6)^2);
    salphae = salphap
    calphae = calphap
    sbetae = sbetap
    cbetae = cbetap
    alhpa(i)=atan2(salphae,calphae)*180/pi;
    beta(i) = atan2(sbetae,cbetae)*180/pi;
    ae = Tei .* [cbetae*calphae;cbetae*salphae;sbetae];
    ap = Tpi .* [cbetap*calphap;cbetap*salphap;sbetap];
    A
    xx
    U = ae - ap
    if isnan(U)
        U = (Tei - Tpi).*ones(3,1);
    end
    dd = A * xx;
    dx = A * xx + B * U
    
    xx = xx + step*dx
    xx_d_int(i,:) = dd;
    xx_dd_int(i,:) = dx(4:6);
    xx_acc(i,:) = U;
    xx_vec(i,:) = xx(4:6);
    xx_int(i,:) = xx;
    
    end
    
%     xx
%     if isnan(xx)
%         xx=[100000;100000;100000;100000;100000;100000];
%     end
    
    %x_tf = xx
    R = sqrt(xx_int(:,1).^2+xx_int(:,2).^2+xx_int(:,3).^2)
    figure
    plot(tau_int(1:end),R/1000)
    figure
    plot(tau_int(1:end),alhpa)
    figure
    plot(tau_int(1:end),beta)
function dlamda = lamda_dot(t,lamda)
    global params
    A = params.A;
    dlamda = A' * lamda;
end


    
    %Htf = Hamilton_tf(pop(1),pop(2),pop(3),x_tf);%!!!!pop£¨4£©Ê±¼ä£¿£¿;
    %J = k1 * x_tf(1)^2 + k2 * x_tf(2)^2 + k3 * x_tf(3)^2 + k4 * (Htf+1)^2