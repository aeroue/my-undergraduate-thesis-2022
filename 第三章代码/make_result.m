function result = make_result(X, Xe, Xp, Ve, Vp, Ue, Up, J, tau_int)
    global params
    f1 = 'X';
    value1 = {Xe./1000;Xp./1000};
    f2 = 'V';
    value2 = {Ve;Vp}; 
    f3 = 'control';
    value3 = {Ue';Up'};
    f4 = 'cost';
    value4 = {J;-J};
    f5 = 'relative_X';
    value5 = X(:,1:3)'./1000;
    f6 = 'relative_V';
    value6 = X(:,4:6)'; 
    f7 = 'tau_int';
    value7 = tau_int;
    result = struct(f1,value1,f2,value2,f3,value3,f4,value4,f5,value5,...
        f6,value6,f7,value7);
end
    