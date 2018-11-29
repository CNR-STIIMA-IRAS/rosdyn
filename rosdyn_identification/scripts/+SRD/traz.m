function TF = traz( q,ext_coord )
if nargin<2
    ext_coord=q;
end
% if nargin<3
%     fixed=0;
% end
% if strcmp(class(q),'sym')
%     idx=find(logical(ext_coord==q));
% else
%     idx=q;
% end

%M=[eye(3) [q;0;0];zeros(1,3) 1];


TF=SRD.eye(4,ext_coord);
tz=SRD.one(q,ext_coord)*q;
TF(3,4)=tz;

end

