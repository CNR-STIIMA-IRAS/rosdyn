clear all;close all;clc;

syms q
v1i=2;
v2i=2;

stringa=sprintf('if (v1i==0) %%1*X=X;\n\tf(:,index)=v2i;\n');

n=9;
for v1i=1:n
    for v2i=v1i:n
        f1=SRD.trigoutils.idx2cossin(v1i,q);
        f2=SRD.trigoutils.idx2cossin(v2i,q);
        
        prodotto=f1*f2;
        
        idx=0;
        while logical(prodotto~=0)
            fi=SRD.trigoutils.idx2cossin(idx,q);
            idx=idx+1;
            if idx==1
                c(idx)=1/sym(2*pi)*int(prodotto*fi,q,0,2*sym(pi));
            else
                c(idx)=1/sym(pi)*int(prodotto*fi,q,0,2*sym(pi));
            end
            prodotto=simplify(prodotto-c(idx)*fi);
        end
        idxs=find(c~=0)-1;
        c=c(c~=0);
        stringa=sprintf('%selseif and(v1i==%d,v2i==%d) %%f1=%s, f2=%s, f1*f2=%s \n',stringa,v1i,v2i,char(f1),char(f2),char(c*SRD.trigoutils.idx2cossin(idxs,q)));
        stringa=sprintf('%s\tf_new=[];\n',stringa);
        for index=1:length(idxs)
            stringa=sprintf('%s\ttmp_f=f;\n\ttmp_f(:,index)=%d;\n\tf_new=[f_new;tmp_f];\n',stringa,idxs(index));
        end
        stringa=sprintf('%s\tf=f_new;\n',stringa);
        if ~and(isscalar(c),c==1)
            
            stringa=sprintf('%s\tc=[',stringa);
            for index=1:length(c)
                stringa=sprintf('%s c*(%s);',stringa,char(c(index)));
            end
            stringa=sprintf('%s];\n',stringa);
        end
    end
end
stringa=sprintf('%selse\n\terror(''[ trigocomb ] v1i=%%d,v2i=%%d not implemented yet'',v1i,v2i);\nend',stringa);
disp(stringa)