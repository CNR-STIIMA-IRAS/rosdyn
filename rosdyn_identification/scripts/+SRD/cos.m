function TF = cos( q,ext_coord )
%COS Summary of this function goes here
%   Detailed explanation goes here

isexpr=0; 
if nargin<2
    ext_coord=q;
end
if strcmp(class(q),'sym')
    isexpr=~logical(symvar(q)==q);%=1 if q is an expression of more than a ext_coord(i)
    if isempty(isexpr)
        isexpr=0;
        idx=[];
    elseif ~isexpr
        idx=find(logical(ext_coord==q));
    end
else
    idx=q;
end

if isexpr
    vars=symvar(q);
    expr=char(expand(cos(q)));
    for index=1:length(vars)
        eval(sprintf('syms %s',char(vars(index))));
        expr=strrep(expr,...
            sprintf('cos(%s)',char(vars(index))),...
            sprintf('SRD.cos(%s,ext_coord)',char(vars(index))));
        expr=strrep(expr,...
            sprintf('sin(%s)',char(vars(index))),...
            sprintf('SRD.sin(%s,ext_coord)',char(vars(index))));
    end
    eval(sprintf('TF=%s;',expr));
else
    if isempty(idx)
        tmp_f=zeros(1,length(ext_coord));
        TF=SRD.trigofun(cos(q),tmp_f,ext_coord);
    else
        tmp_f=zeros(1,length(ext_coord));
        tmp_f(idx)=1;
        TF=SRD.trigofun(1,tmp_f,ext_coord);
    end
end
end
