function [choice1,choice2] = gen_match_result(conv)
     %res=Hungary(conv);
     B=Hungary(conv);
     [choice1,b]=linear_assignment(conv,B);
     [choice2_row,choice2_col] = ind2sub(size(conv),choice1);
     choice2 = [choice2_row;choice2_col];
end
function res=Hungary(N)
%输入的矩阵应N*N的
    [a,~]=size(N);
%第一步每一行减去当前行最小值
    for ii = 1:a
        N(ii,:)= N(ii,:)-min( N(ii,:));
    end
%第二步每一列减去当前列最小值
    for ii = 1:a
        N(:,ii)=  N(:,ii)-min( N(:,ii));
    end
    num=0;
    while num~=a
        [num,N_min,del_hang,del_lie]=line_count(N);
        if num ~=a
            for ii=1:a
                if del_hang(ii)~=ii
                    N(ii,:) =  N(ii,:)-N_min;
                end
                if del_lie(ii)==ii
                    N(:,ii) =  N(:,ii)+N_min;
                end
            end
        else
            res=N;
        end
    end
end
function [place,res]=linear_assignment(M,N)
%N是n维矩阵,N是经过Hungary处理的
%M是未处理前的
[a,~]=size(N);
x=0;
place=zeros(1,a);
res=zeros(1,a);
judge=zeros(1,a);
while find(N==0)
    for ii=1:a
    judge(ii)=length(find(N(ii,:)==0));
    end
    judge(find(judge==0))=[];
    if min(judge)==1
     for ii=1:a
        if length(find(N(ii,:)==0))==1     %先选出行中只有1个0
            x=x+1;
            place(x)=ii+(find(N(ii,:)==0)-1)*a; %得到矩阵中的位置
            h=find(N(ii,:)==0);
            N(ii,:)=1./zeros(1,a);
            N(:,h)=1./zeros(a,1);
        end
     end
    end
    
    for ii=1:a
    judge(ii)=length(find(N(ii,:)==0));
    end
    judge(find(judge==0))=[];
    
    if min(judge)==2
       x=x+1;
    q=find(N==0);
    place(x)=q(1);
    N(mod(q(1),a),:)=1./zeros(1,a);
    N(:,fix(q(1)/a)+1)=1./zeros(a,1);  
    end
end
[place,~]=sort(place);
for ii=1:length(place)
    res(ii)=M(place(ii));
end
end
function [num,M_min,del_hang,del_lie]=line_count(M)
[a,~]=size(M);
num=0;
h=0;
del_hang=zeros(a,1);
del_lie=zeros(a,1);
for ii=1:a
    del=ii-h;
    [~,b]=size(find(M(del,:)==0));
    if   b>= 2
        M(del,:)=[];
        h=h+1;
        del_hang(ii)=ii;    %得到被覆盖的行数
        num=num+1;
    end
end
l=0;
for ii=1:a
    del=ii-l;
    [b,~]=size(find(M(:,del)==0));
    if  b >=1
        M(:,del)=[];
        l=l+1;
        del_lie(ii)=ii;    %得到被覆盖的列数
        num=num+1;
    end
end
M_min=min(min(M));
end