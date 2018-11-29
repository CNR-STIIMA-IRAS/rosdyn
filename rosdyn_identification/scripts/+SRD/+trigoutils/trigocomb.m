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
    if (v1i==0) %1*X=X;
        f(:,index)=v2i;
    elseif and(v1i==1,v2i==1) %f1=cos(q), f2=cos(q), f1*f2=cos(2*q)/2 + 1/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=0;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=3;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==1,v2i==2) %f1=cos(q), f2=sin(q), f1*f2=sin(2*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=4;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2);];
    elseif and(v1i==1,v2i==3) %f1=cos(q), f2=cos(2*q), f1*f2=cos(3*q)/2 + cos(q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=1;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=5;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==1,v2i==4) %f1=cos(q), f2=sin(2*q), f1*f2=sin(3*q)/2 + sin(q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=2;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=6;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==1,v2i==5) %f1=cos(q), f2=cos(3*q), f1*f2=cos(2*q)/2 + cos(4*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=3;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=7;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==1,v2i==6) %f1=cos(q), f2=sin(3*q), f1*f2=sin(2*q)/2 + sin(4*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=4;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=8;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==1,v2i==7) %f1=cos(q), f2=cos(4*q), f1*f2=cos(3*q)/2 + cos(5*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=5;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=9;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==1,v2i==8) %f1=cos(q), f2=sin(4*q), f1*f2=sin(3*q)/2 + sin(5*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=6;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=10;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==1,v2i==9) %f1=cos(q), f2=cos(5*q), f1*f2=cos(4*q)/2 + cos(6*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=7;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=11;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==2,v2i==2) %f1=sin(q), f2=sin(q), f1*f2=1/2 - cos(2*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=0;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=3;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(-1/2);];
    elseif and(v1i==2,v2i==3) %f1=sin(q), f2=cos(2*q), f1*f2=sin(3*q)/2 - sin(q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=2;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=6;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(-1/2); c*(1/2);];
    elseif and(v1i==2,v2i==4) %f1=sin(q), f2=sin(2*q), f1*f2=cos(q)/2 - cos(3*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=1;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=5;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(-1/2);];
    elseif and(v1i==2,v2i==5) %f1=sin(q), f2=cos(3*q), f1*f2=sin(4*q)/2 - sin(2*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=4;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=8;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(-1/2); c*(1/2);];
    elseif and(v1i==2,v2i==6) %f1=sin(q), f2=sin(3*q), f1*f2=cos(2*q)/2 - cos(4*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=3;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=7;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(-1/2);];
    elseif and(v1i==2,v2i==7) %f1=sin(q), f2=cos(4*q), f1*f2=sin(5*q)/2 - sin(3*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=6;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=10;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(-1/2); c*(1/2);];
    elseif and(v1i==2,v2i==8) %f1=sin(q), f2=sin(4*q), f1*f2=cos(3*q)/2 - cos(5*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=5;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=9;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(-1/2);];
    elseif and(v1i==2,v2i==9) %f1=sin(q), f2=cos(5*q), f1*f2=sin(6*q)/2 - sin(4*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=8;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=12;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(-1/2); c*(1/2);];
    elseif and(v1i==3,v2i==3) %f1=cos(2*q), f2=cos(2*q), f1*f2=cos(4*q)/2 + 1/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=0;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=7;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==3,v2i==4) %f1=cos(2*q), f2=sin(2*q), f1*f2=sin(4*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=8;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2);];
    elseif and(v1i==3,v2i==5) %f1=cos(2*q), f2=cos(3*q), f1*f2=cos(5*q)/2 + cos(q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=1;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=9;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==3,v2i==6) %f1=cos(2*q), f2=sin(3*q), f1*f2=sin(5*q)/2 + sin(q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=2;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=10;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==3,v2i==7) %f1=cos(2*q), f2=cos(4*q), f1*f2=cos(2*q)/2 + cos(6*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=3;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=11;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==3,v2i==8) %f1=cos(2*q), f2=sin(4*q), f1*f2=sin(2*q)/2 + sin(6*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=4;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=12;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==3,v2i==9) %f1=cos(2*q), f2=cos(5*q), f1*f2=cos(3*q)/2 + cos(7*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=5;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=13;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==4,v2i==4) %f1=sin(2*q), f2=sin(2*q), f1*f2=1/2 - cos(4*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=0;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=7;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(-1/2);];
    elseif and(v1i==4,v2i==5) %f1=sin(2*q), f2=cos(3*q), f1*f2=sin(5*q)/2 - sin(q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=2;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=10;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(-1/2); c*(1/2);];
    elseif and(v1i==4,v2i==6) %f1=sin(2*q), f2=sin(3*q), f1*f2=cos(q)/2 - cos(5*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=1;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=9;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(-1/2);];
    elseif and(v1i==4,v2i==7) %f1=sin(2*q), f2=cos(4*q), f1*f2=sin(6*q)/2 - sin(2*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=4;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=12;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(-1/2); c*(1/2);];
    elseif and(v1i==4,v2i==8) %f1=sin(2*q), f2=sin(4*q), f1*f2=cos(2*q)/2 - cos(6*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=3;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=11;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(-1/2);];
    elseif and(v1i==4,v2i==9) %f1=sin(2*q), f2=cos(5*q), f1*f2=sin(7*q)/2 - sin(3*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=6;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=14;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(-1/2); c*(1/2);];
    elseif and(v1i==5,v2i==5) %f1=cos(3*q), f2=cos(3*q), f1*f2=cos(6*q)/2 + 1/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=0;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=11;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==5,v2i==6) %f1=cos(3*q), f2=sin(3*q), f1*f2=sin(6*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=12;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2);];
    elseif and(v1i==5,v2i==7) %f1=cos(3*q), f2=cos(4*q), f1*f2=cos(7*q)/2 + cos(q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=1;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=13;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==5,v2i==8) %f1=cos(3*q), f2=sin(4*q), f1*f2=sin(7*q)/2 + sin(q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=2;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=14;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==5,v2i==9) %f1=cos(3*q), f2=cos(5*q), f1*f2=cos(2*q)/2 + cos(8*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=3;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=15;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==6,v2i==6) %f1=sin(3*q), f2=sin(3*q), f1*f2=1/2 - cos(6*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=0;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=11;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(-1/2);];
    elseif and(v1i==6,v2i==7) %f1=sin(3*q), f2=cos(4*q), f1*f2=sin(7*q)/2 - sin(q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=2;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=14;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(-1/2); c*(1/2);];
    elseif and(v1i==6,v2i==8) %f1=sin(3*q), f2=sin(4*q), f1*f2=cos(q)/2 - cos(7*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=1;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=13;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(-1/2);];
    elseif and(v1i==6,v2i==9) %f1=sin(3*q), f2=cos(5*q), f1*f2=sin(8*q)/2 - sin(2*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=4;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=16;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(-1/2); c*(1/2);];
    elseif and(v1i==7,v2i==7) %f1=cos(4*q), f2=cos(4*q), f1*f2=cos(8*q)/2 + 1/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=0;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=15;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==7,v2i==8) %f1=cos(4*q), f2=sin(4*q), f1*f2=sin(8*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=16;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2);];
    elseif and(v1i==7,v2i==9) %f1=cos(4*q), f2=cos(5*q), f1*f2=cos(9*q)/2 + cos(q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=1;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=17;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    elseif and(v1i==8,v2i==8) %f1=sin(4*q), f2=sin(4*q), f1*f2=1/2 - cos(8*q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=0;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=15;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(-1/2);];
    elseif and(v1i==8,v2i==9) %f1=sin(4*q), f2=cos(5*q), f1*f2=sin(9*q)/2 - sin(q)/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=2;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=18;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(-1/2); c*(1/2);];
    elseif and(v1i==9,v2i==9) %f1=cos(5*q), f2=cos(5*q), f1*f2=cos(10*q)/2 + 1/2
        f_new=[];
        tmp_f=f;
        tmp_f(:,index)=0;
        f_new=[f_new;tmp_f];
        tmp_f=f;
        tmp_f(:,index)=19;
        f_new=[f_new;tmp_f];
        f=f_new;
        c=[ c*(1/2); c*(1/2);];
    else
        error('[ trigocomb ] v1i=%d,v2i=%d not implemented yet',v1i,v2i);
    end
end
end