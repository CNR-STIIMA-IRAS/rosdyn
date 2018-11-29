function [c,f]=trigocomb(v1,v2)
    c=1;
    f=[];
    for index=1:length(v1)
        v1i=v1(index);
        v2i=v2(index);
        if v2i<v1i
            tmp=v2i;
            v2i=v1i;
            v1i=tmp;
        end
        if     and(v1i==0) %1* X =X
            f(:,index)=1;
        elseif and(v1i==1,v2i==1) %cos * cos
            tmp1=f;
            tmp1(:,index)=0;
            tmp2=f;
            tmp2(:,index)=3;
            f=[tmp1;tmp2];
            c=[c/2;c/2];
        elseif and(v1i==1,v2i==2) %sin * cos
            f(:,index)=4;
            c=c/2;
        elseif and(v1i==2,v2i==2) %sin * sin
            tmp1=f;
            tmp1(:,index)=0;
            tmp2=f;
            tmp2(:,index)=3;
            f=[tmp1;tmp2];
            c=[c/2;-c/2];
        elseif and(v1i==1,v2i==3) %cos*cos2
        elseif and(v1i==2,v2i==3) %sin*cos2
        elseif and(v1i==3,v2i==3) %cos*cos2
        elseif and(v1i==1,v2i==4) %cos*sin2
        elseif and(v1i==2,v2i==4) %sin*sin2
        elseif and(v1i==3,v2i==4) %cos2*sin2
        elseif and(v1i==4,v2i==4) %sin2*sin2
        
        else
            error(sprintf('[ trigocomb ] v1i=%d,v2i=%d not implemented yet',v1i,v2i))
        end
    end
end