function [conv,dis] = gen_pe_distance(p1,p2)
    a = size(p1,2);
    n = 1;
    for i = 1:a
        for j = 1:a
        conv(i,j) = sqrt(sum((p1(:,i) - p2(:,j)).^2));
        dis(:,n) = p1(:,i) - p2(:,j);
        n = n + 1;
        end
    end
end
        