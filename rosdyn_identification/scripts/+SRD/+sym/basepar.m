function [T,IM]=basepar(C,par,verbose)
if nargin<3
    verbose=false;
end
J=jacobian(C,par);
[IM,T]=SRD.sym.indipendent_columns(J,verbose);
%%  