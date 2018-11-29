function TF = one( q,ext_coord )
%ONE Summary of this function goes here
%   Detailed explanation goes here
if nargin<2
    ext_coord=q;
end
if strcmp(class(q),'sym')
    idx=find(logical(ext_coord==q));
else
    idx=q;
end
tmp_f=zeros(1,length(ext_coord));
TF=SRD.trigofun(1,tmp_f,ext_coord);
end

