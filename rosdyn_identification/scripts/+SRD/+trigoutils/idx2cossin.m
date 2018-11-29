function f=idx2cossin(idxs,q)
for index=1:length(idxs)
    if idxs(index)==0
        f(index,1)=sym(1);
    elseif mod(idxs(index),2)==1
        tmp=floor(idxs(index)/2)+1;
        f(index,1)=cos(tmp*q);
    else
        tmp=floor(idxs(index)/2);
        f(index,1)=sin(tmp*q);
    end
end