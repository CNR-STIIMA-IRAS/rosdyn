function vector=reduceLinConstraints(vector)
q=vector(1,1).coord;
dcdq=jac(vector,q);

LinComb=[];
lin_rows=[];
nlin_rows=[];
one=SRD.one(1,q);
for index=1:length(q)
    tmpq(index,1)=one*q(index);
end
for index=1:size(vector,1)
    [C,F]=SRD.trigoutils.common_matrix(dcdq(index,:));
    [IM,T]=SRD.sym.indipendent_columns(C);
    if size(T,2)==1
        lin_rows(end+1)=index;
        linrel=tmpq.'*T;
        syms K
        tmpfun=simplify(subs(simplify(vector(index).toSym),linrel.toSym,K,1));
        K=solve(tmpfun,K);
        if isempty(K)
            nlin_rows(end+1)=index;
        else
            LinComb(end+1,:)=[T.' K];
        end
        
    else
        nlin_rows(end+1)=index;
    end
end
if ~isempty(LinComb)
    [LinComb]=SRD.sym.indipendent_columns(LinComb.').';
    linrel=([tmpq.' one]*LinComb.').';
    
    vector=[ linrel;vector(nlin_rows)];
end
end