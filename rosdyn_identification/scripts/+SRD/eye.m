function [ TF ] = eye( n,ext_coord)
%EYE Summary of this function goes here
%   Detailed explanation goes here

if nargin<2
    ext_coord=[];
end

for idx_r=1:n
    for idx_c=1:n
        
        if isempty(ext_coord)
            tmp_f=0;
        else
            tmp_f=zeros(1,length(ext_coord));
        end
        TF(idx_r,idx_c)=SRD.trigofun(sym(idx_r==idx_c),tmp_f,ext_coord);
    end
end
end

