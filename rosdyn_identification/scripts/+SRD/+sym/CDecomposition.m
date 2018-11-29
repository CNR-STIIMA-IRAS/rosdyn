function [Cdecomposed,F]=CDecomposition(C,q,qp,qpp,verbose)
if nargin<4
    verbose=false;
end
if verbose
    counter=1;
end
Cqpp=jacobian(C,qpp);
C=simplify(C-Cqpp*qpp);
C=C(logical((C~=0)));
F=[qpp];


for index=1:length(qp)
    Cdecomposed{index}=Cqpp(:,index);
    if verbose
        fprintf('[ CDecomposition ] %d\n',counter)
        counter=counter+1;
    end
end

if isempty(C)
  C=0;
end
Cq=jacobian(C,q);
C=simplify(C-Cq*q);
C=C(logical((C~=0)));
if isempty(C)
  C=0;
end

for idx1=1:length(q)
    F(end+1,1)=q(idx1);
    if verbose
        fprintf('[ CDecomposition ] %d\n',counter)
        counter=counter+1;
    end
    Cdecomposed{end+1}=Cq(:,end);
end

Cqp=sym([]);
for idx1=1:length(qp)
    for idx2=idx1:length(qp)
        if verbose
            fprintf('[ CDecomposition ] %d\n',counter)
            counter=counter+1;
        end
        if idx1==idx2
            Cqp=diff(diff(C,qp(idx1)),qp(idx2))/2;
        else
            Cqp=diff(diff(C,qp(idx1)),qp(idx2));
        end
        C=simplify(C-Cqp*qp(idx1)*qp(idx2));
        C=C(logical((C~=0)));
        F(end+1,1)=qp(idx1)*qp(idx2);
        Cdecomposed{end+1}=Cqp;
    end
end
Cconst=C;
Cdecomposed{end+1}=Cconst;
F(end+1)=1;
if verbose
    fprintf('[ CDecomposition ] done\n')
end
end
