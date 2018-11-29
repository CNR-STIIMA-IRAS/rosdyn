classdef trigofun
    properties (SetAccess = protected )
        c
        f
        coord
    end
    methods
        function TF=trigofun(ext_c,ext_f,ext_coord)
            if nargin==0
                TF.c=[];
                TF.f=[];
                TF.coord=[];
            elseif and(nargin==1,strcmp(class(ext_c),'SRD.trigofun')) % copy obj
                TF.c=ext_c.c;
                TF.f=ext_c.f;
                TF.coord=ext_c.coord;
            elseif and(nargin==1,strcmp(class(ext_c),'sym')) %void TF from coord
                TF.c=[];
                TF.f=[];
                TF.coord=ext_c;
            elseif and(nargin==3,isstr(ext_f))
                TF.coord=ext_coord;
                if strcmp(ext_f,'cos')
                    TF.c=sym(1);
                    TF.f=zeros(1,length(TF.coord));
                    idx=TF.sym2idx(ext_c);
                    TF.f(idx)=1;
                elseif strcmp(ext_f,'sin')
                    TF.c=sym(1);
                    TF.f=zeros(1,length(TF.coord));
                    idx=TF.sym2idx(ext_c);
                    TF.f(idx)=2;
                elseif strcmp(ext_f,'one')
                    TF.c=sym(1);
                    TF.f=zeros(1,length(TF.coord));
                elseif strcmp(ext_f,'eye4')
                    TF=SRD.trigofun(1,'one',TF.coord)*eye(4);
                else
                    error('string not recognized')
                end
            else
                TF.c=sym(ext_c);
                TF.f=ext_f;
                TF.coord=ext_coord;
            end
        end %trigofun

        function disp(TF)
            disp(TF.toSym)
%         function disp(TF)
%             c=TF.c;
%             f=TF.f;
%             coord=TF.coord;
%             if isscalar(TF)
%                 for index=1:length(c)
%                     if logical(c(index)~=0)
%                         if sum(f(index,:))==0
%                             str='1';
%                         else
%                             str='';
%                             for index_f=1:length(f(index,:))
%                                 if f(index,index_f)==0
%                                     flag=1;
%                                     %str=[str,'1'];
%                                 elseif mod(f(index,index_f),2)==1
%                                     flag=0;
%                                     tmp=floor(f(index,index_f)/2)+1;
%                                     if tmp==1
%                                         str=[str,sprintf('cos(%s) ',char(coord(index_f)))];
%                                     else
%                                         str=[str,sprintf('cos(%d%s) ',tmp,char(coord(index_f)))];
%                                     end
%                                 else
%                                     flag=0;
%                                     tmp=floor(f(index,index_f)/2);
%                                     if tmp==1
%                                         str=[str,sprintf('sin(%s) ',char(coord(index_f)))];
%                                     else
%                                         str=[str,sprintf('sin(%d%s) ',tmp,char(coord(index_f)))];
%                                     end
%                                 end
%                             end
%                         end
%                         fprintf('( %s ) * ( %s)',char(c(index)),str);
%                         if index<length(c)
%                             fprintf(' + ');
%                         else
%                             fprintf('\n');
%                         end
%                     else %logical(c(index)~=0)
%                         if length(c)==1
%                             fprintf('0\n');
%                         end
%                     end %logical(c(index)~=0)
%                 end
%             else
%                 for idx_r=1:size(TF,1)
%                     for idx_c=1:size(TF,2)
%                         fprintf('Row %d / Col %d\n',idx_r,idx_c)
%                         disp(TF(idx_r,idx_c))
%                     end
%                 end
%             end
        end %disp
        
        function TF3=mtimes(TF1,TF2)
            if isempty(TF1)
               tmp_f=zeros(1,length(TF2.coord));
               TF1=SRD.trigofun(0,tmp_f,TF2.coord);
            end
            if isempty(TF2)
               tmp_f=zeros(1,length(TF1.coord));
               TF2=SRD.trigofun(0,tmp_f,TF1.coord);
            end
            
            if and(isscalar(TF1),isscalar(TF2));
                if isnumeric(TF1)
                    TF3=TF2;
                    TF3.c=TF3.c*TF1;
                elseif isnumeric(TF2)
                    TF3=TF1;
                    TF3.c=TF3.c*TF2;
                elseif strcmp(class(TF1),'sym')
                    TF3=TF2;
                    TF3.c=TF3.c*TF1;
                elseif strcmp(class(TF2),'sym')
                    TF3=TF1;
                    TF3.c=TF3.c*TF2;
                else
                    TF3=scalar_prod(TF1,TF2);
                end
            else %matrix product
                s1=size(TF1);
                s2=size(TF2);
                if s1(2)~=s2(1)
                    if isscalar(TF1)
                        for idx_r=1:s2(1)
                            for idx_c=1:s2(2)
                                TF3(idx_r,idx_c)=TF1*TF2(idx_r,idx_c);
                            end
                        end
                        TF3.homogenize();
                    elseif isscalar(TF2)
                        if or(strcmp(class(TF2),'double'),strcmp(class(TF2),'sym'))
                            TF3=TF1;
                            for idx_r=1:s1(1)
                                for idx_c=1:s1(2)
                                    TF3(idx_r,idx_c).c=TF1(idx_r,idx_c).c*TF2;
                                end
                            end
                        else
                            for idx_r=1:s1(1)
                                for idx_c=1:s1(2)
                                    TF3(idx_r,idx_c)=TF1(idx_r,idx_c)*TF2;
                                end
                            end
                        end
                        TF3.homogenize();
                    else
                        error('Inner matrix dimensions must agree. ');
                    end
                else %s1(2)==s2(1)
                    if strcmp(class(TF1),'SRD.trigofun')
                        coord=TF1(1,1).coord;
                    else
                        coord=TF2(1,1).coord;
                    end
                    zero=SRD.zero(coord);
                    TF3=repmat(zero,s1(1),s2(2));
                    for idx_r=1:s1(1)
                        for idx_c=1:s2(2)
                            %tmp=SRD.trigofun(coord);
                            for idx1=1:s1(2)
                                %tmp=tmp+TF1(idx_r,idx1)*TF2(idx1,idx_c);
                                TF3(idx_r,idx_c)=TF3(idx_r,idx_c)+TF1(idx_r,idx1)*TF2(idx1,idx_c);
                            end
                            %TF3(idx_r,idx_c)=tmp;
                        end
                    end
                end
            end
        end %mtimes
        
        function TF3=times(TF1,TF2)
            if size(TF1)~=size(TF2)
                error('Matrix dimensions must agree. ');
            end
            for idx_r=1:size(TF1,1)
                for idx_c=1:size(TF1,2)
                    TF3(idx_r,idx_c)=TF1(idx_r,idx_c)*TF2(idx_r,idx_c);
                end
            end
        end %times
        
        function TF3=plus(TF1,TF2)
           if isempty(TF1)
               tmp_f=zeros(1,length(TF2.coord));
               TF1=SRD.trigofun(0,tmp_f,TF2.coord);
            end
            if isempty(TF2)
               tmp_f=zeros(1,length(TF1.coord));
               TF2=SRD.trigofun(0,tmp_f,TF1.coord);
            end
            
            s1=size(TF1);
            s2=size(TF2);
            if s1~=s2
                if and(s1(1)==1,s1(2)==1)
                    TF3=TF1*ones(size(TF2))+TF2;
                elseif and(s2(1)==1,s2(2)==1)
                    TF3=TF1+TF2*ones(size(TF2));
                else
                    error('Matrix dimensions must agree. ');
                end
            elseif and(s1(1)==1,s1(2)==1)
                if isnumeric(TF1)
                    TF3=SRD.trigofun([],[],TF2.coord);
                    TF3.c=[TF1+TF2.c];
                    TF3.f=TF2.f;
                elseif isnumeric(TF2)
                    TF3=SRD.trigofun([],[],TF1.coord);
                    TF3.c=[TF1.c+TF2];
                    TF3.f=TF1.f;
                else
                    TF3=SRD.trigofun([],[],TF1.coord);
                    %TF3.c=[TF1.c;TF2.c];
                    TF3.c=vertcat(TF1.c,TF2.c);
                    TF3.f=[TF1.f;TF2.f];
                    TF3=TF3.order();
                    TF3=TF3.purge();
                end
            else
                for idx_r=1:s1(1)
                    for idx_c=1:s1(2)
                        TF3(idx_r,idx_c)=TF1(idx_r,idx_c)+TF2(idx_r,idx_c);
                    end
                end
            end
        end %plus
        
        function TF3=minus(TF1,TF2)
            s1=size(TF1);
            s2=size(TF2);
            if s1~=s2
                if and(s1(1)==1,s1(2)==1)
                    TF3=TF1*ones(size(TF2))-TF2;
                elseif and(s2(1)==1,s2(2)==1)
                    TF3=TF1-TF2*ones(size(TF2));
                else
                    error('Matrix dimensions must agree. ');
                end
            elseif and(s1(1)==1,s1(2)==1)
                if isnumeric(TF1)
                    TF3=SRD.trigofun([],[],TF2.coord);
                    TF3.c=[TF1-TF2.c];
                elseif isnumeric(TF2)
                    TF3=SRD.trigofun([],[],TF1.coord);
                    TF3.c=[TF1.c-TF2];
                else
                    TF3=SRD.trigofun([],[],TF1.coord);
                    TF3.c=vertcat(TF1.c,-TF2.c);
                    TF3.f=[TF1.f;TF2.f];
                    TF3=TF3.order();
                    TF3=TF3.purge();
                end
            else
                for idx_r=1:s1(1)
                    for idx_c=1:s1(2)
                        TF3(idx_r,idx_c)=TF1(idx_r,idx_c)-TF2(idx_r,idx_c);
                    end
                end
            end
        end %minus
        
        function TF2=uminus(TF1)
            TF2=-1*TF1;
        end %uminus
        
        function TF2=mpower(TF,n)
            if or(size(TF,1)~=size(TF,2),~isscalar(n))
                error('Inputs must be a scalar and a square matrix.\nTo compute elementwise POWER, use POWER (.^) instead.');
            end
            if n==0
                TF2=SRD.eye(size(TF,1),TF(1,1).coord);
            else
                if mod(n,1)
                    warning('n is truncked to the previous integer'),
                end
                if n>0
                    TF2=TF;
                    for index=1:(n-1)
                        TF2=TF2*TF;
                    end
                else
                    TF2=inv(TF)^(-n);
                end
            end
        end %mpower
        
        function TF3=mrdivide(TF1,TF2)
            if and(isscalar(TF1),isscalar(TF2))
                if or(...
                        and(strcmp(class(TF1),'SRD.trigofun'),strcmp(class(TF2),'sym')),...
                        and(strcmp(class(TF1),'SRD.trigofun'),strcmp(class(TF2),'double')))
                    TF3=TF1;
                    TF3.c=TF3.c/sym(TF2);
                end
            end
        end %mrdivide

        function detTF=det(TF)
            if (size(TF,1)~=size(TF,2))
                error('Matrix must be square. ')
            end
            if isempty(TF)
                detTF=1;
            elseif size(TF,1)==1
                detTF=TF(1,1);
            elseif size(TF,1)==2
                detTF=TF(1,1)*TF(2,2)-TF(2,1)*TF(1,2);
            else
                detTF=SRD.zero(1,TF(1,1).coord);
                n=size(TF,1);
                for index=1:n
                    detTF=detTF+TF(index,n)*det(TF([1:(index-1) (index+1:end)],1:end-1))*(-1)^(index+n);
                end
            end
        end %det

        function [adjTF,detTF]=inv(TF)
            if (size(TF,1)~=size(TF,2))
                error('Matrix must be square.');
            end
            if size(TF,1)==1
                adjTF=SRD.one(1,TF.coord);
                detTF=TF;
            else
                detTF=det(TF);
                for idx_r=1:size(TF,1)
                    for idx_c=1:size(TF,1)
                        cofTF(idx_r,idx_c)=(-1)^(idx_r+idx_c)*det(TF([1:idx_r-1 idx_r+1:end],[1:idx_c-1 idx_c+1:end]));
                    end
                end
                adjTF=cofTF';
            end
        end %end
        
        function idx=sym2idx(TF,q)
            if strcmp(class(q),'sym')
                for index=1:length(q)
                    idx(index,1)=find(logical(TF(1,1).coord==q(index)));
                end
            else
                idx=q;
            end
        end %sym2idx
        
        function sym_var=idx2sym(TF,q)
            sym_var=TF.coord(q);
        end %idx2sym
        
        function TF=purge(TF,simpl)
            if nargin<2
                simpl=false;
            end
            if isscalar(TF)
                if ~isempty(TF.f)
                    if simpl
                        for index=1:size(TF.f,1);
                            if ~isempty(TF.c(index))
                                TF.c(index)=simplify(TF.c(index));
                            end
                        end
                    end
                    idxs=find(TF.c~=0);
%                     if isempty(idxs);
%                         TF.c=sym(0);
%                         TF.f=zeros(1,length(TF.coord));
%                     else
                        if length(idxs)<size(TF.f,1)
                            TF.c=TF.c(idxs);
                            TF.f=TF.f(idxs,:);
                        end
%                     end
                end
            else
                for idx_r=1:size(TF,1)
                    for idx_c=1:size(TF,2)
                        TF(idx_r,idx_c)=TF(idx_r,idx_c).purge(simpl);
                    end
                end
            end
        end %purge
        
        function TF3=scalar_prod(TF1,TF2)
            %TF3=SRD.trigofun(TF1.coord);
            tmp_c=[];
            tmp_f=[];
            if ~isempty(TF1.f) && ~isempty(TF2.f)
                if size(TF1.f,1)<size(TF2.f,1)
                    for index_1=1:size(TF1.f,1)
                        tmp_c1=TF1.c(index_1);
                        for index_2=1:size(TF2.f,1)
                            [c,f]=SRD.trigoutils.trigocomb(TF1.f(index_1,:),TF2.f(index_2,:));
                            %tmp2=tmp_c1*TF2.c(index_2);
                            tmp2=mtimes(tmp_c1,TF2.c(index_2));
                            tmp_c=vertcat(tmp_c,c*tmp2);
                            tmp_f=[tmp_f;f];
                        end
                    end
                else
                    for index_1=1:size(TF2.f,1)
                        tmp_c1=TF2.c(index_1);
                        for index_2=1:size(TF1.f,1)
                            [c,f]=SRD.trigoutils.trigocomb(TF2.f(index_1,:),TF1.f(index_2,:));
                            %tmp2=tmp_c1*TF2.c(index_2);
                            tmp2=mtimes(tmp_c1,TF1.c(index_2));
                            tmp_c=vertcat(tmp_c,c*tmp2);
                            tmp_f=[tmp_f;f];
                        end
                    end
                end
            end
            TF3=SRD.trigofun(tmp_c,tmp_f,TF1.coord);
            TF3=TF3.order();
        end % scalar_prod
        
        function TF=order(TF)
            if isscalar(TF)
                [tmp,ia,ib]=unique(TF.f,'rows');
                if size(tmp,1)~=size(TF.f,1)
                    TF.f=tmp;
                    tmp_c=TF.c;
                    TF.c=tmp_c(ia);
                    for index=1:size(TF.f,1)
                        idxs=find(ib-index==0);
                        if length(idxs)>1
                            TF.c(index,1)=sum(tmp_c(idxs));
                        end
                    end
                else
                    TF.f=tmp;
                    TF.c=TF.c(ia);
                end
                %TF=TF.purge();
            else
                for idx_r=1:size(TF,1)
                    for idx_c=1:size(TF,2)
                        TF(idx_r,idx_c)=TF(idx_r,idx_c).order;
                    end
                end
            end
        end %order
        
        function TF=homogenize(TF)
            if isscalar(TF)
                if isempty(TF.coord)
                    error('coord not defined')
                end
            else
                coord=[];
                for idx_r=1:size(TF,1)
                    for idx_c=1:size(TF,2)
                        if and(isempty(coord),~isempty(TF(idx_r,idx_c).coord))
                            coord=TF(idx_r,idx_c).coord;
                        elseif ~isempty(TF(idx_r,idx_c).coord)
                            if size(coord)~=size(TF(idx_r,idx_c).coord)
                                error('the values coord are not homogeneous');
                            end
                            if coord~=TF(idx_r,idx_c).coord
                                error('the values coord are not homogeneous');
                            end
                        end
                    end
                end
                for idx_r=1:size(TF,1)
                    for idx_c=1:size(TF,2)
                        if isempty(TF(idx_r,idx_c).coord)
                            TF(idx_r,idx_c).coord=coord;
                        end
                    end
                end
            end
        end %homogenize
        
        function TFder=diff(TF,qi)
            if isscalar(TF)
                if strcmp(class(qi),'sym')
                    idx_qi=find(logical(TF.coord==qi));
                    %if isempty(idx_qi)
                    %    error('qi does not belong to TF.coord')
                    %end
                else
                    idx_qi=qi;
                    qi=idx2sym(TF,idx_qi);
                end
                % TF=c(q)*f(q)
                % TFder=dc/dq*f(q)+c*df/dq
                
                if ~isempty(idx_qi)
                    TFder=SRD.trigofun(TF);
                    idxs=find(TF.f(:,idx_qi)==0);
                    if ~isempty(idxs)
                        TFder.c(idxs)=0;
                    end
                    
                    idxs=find(mod(TF.f(:,idx_qi),2)==1);
                    if ~isempty(idxs)
                        tmpidx=floor(TF.f(idxs,idx_qi)/2)+1;
                        TFder.c(idxs)=-tmpidx.*TF.c(idxs);
                        TFder.f(idxs,idx_qi)=TF.f(idxs,idx_qi)+1;
                    end
                    
                    idxs=find((mod(TF.f(:,idx_qi),2)==0).*(TF.f(:,idx_qi)>0));
                    if ~isempty(idxs)
                        tmpidx=floor(TF.f(idxs,idx_qi)/2);
                        TFder.c(idxs)=tmpidx.*TF.c(idxs);
                        TFder.f(idxs,idx_qi)=TF.f(idxs,idx_qi)-1;
                    end
                else
                    TFder=SRD.zero(1,TF(1,1).coord);
                end
                dcdq=diff(TF.c,qi);
                TFder2=SRD.trigofun(dcdq,TF.f,TF.coord);
                TFder=TFder+TFder2;
                
                
                TFder=TFder.order;
            else
                for idx_r=1:size(TF,1)
                    for idx_c=1:size(TF,2);
                        TFder(idx_r,idx_c)=diff(TF(idx_r,idx_c),qi);
                    end
                end
                
            end
        end %trigodiff
        
        function TFgrad=grad(TF,qi)
            if nargin<2
                qi=1:length(TF.coord);
            else
                qi=sym2idx(TF,qi);
            end
            for index=1:length(qi)
                TFgrad(index,1)=TF.diff(qi(index));
            end
        end %grad
        
        function TFjac=jac(TF,qi)
            if nargin<2
                %idx_qi=1:length(TF.coord);
                qi=TF(1,1).coord;
            else
                %qi=sym2idx(TF,qi)
            end
            if isvector(TF)
                zero=SRD.zero(1,TF(1,1).coord);
                TFjac=repmat(zero,length(TF),length(qi));
                for idx=1:length(TF)
                    for index=1:length(qi)
                        TFjac(idx,index)=TF(idx).diff(qi(index));
                    end
                end
            else
                error('TF is not a vector');
            end
        end %jac
        
        function [TFreduced,T]=reduce(TF,verbose)
            if nargin<2
                verbose=0;
            end
            [C,F]=SRD.trigoutils.common_matrix(TF);
            if rank(C)==size(C,2)
                if verbose
                    fprintf('[ reduce ] TF is full rank\n');
                end
                TFreduced=TF;
                T=sym(eye(size(C,2)));
            else
            [IM,T]=SRD.sym.indipendent_columns(C);
            for index=1:size(IM,2)
                if verbose 
                    fprintf('..column %d of %d..\n',index,size(IM,2));
                end
                TFreduced(index,1)=SRD.trigofun(IM(:,index),F,TF(1,1).coord);
            end
            TFreduced=SRD.trigoutils.reduceLinConstraints(TFreduced);
            end
        end %reduce (TFreduced=TF*T)
            
        function simplify(TF)
            disp(simplify(toSym(TF)));
        end % simplify
        
        function sym_var=toSym(TF)
            sym_var=sym(0);
            if isscalar(TF)
                for idx=1:size(TF.f,1)
                    tmp=sym(1);
                    for idx_f=1:size(TF.f,2)
                        if TF.f(idx,idx_f)==0
                            tmp=tmp;
                        elseif mod(TF.f(idx,idx_f),2)==1
                            tmpidx=floor(TF.f(idx,idx_f)/2)+1;
                            tmp=tmp*cos(tmpidx*TF.coord(idx_f));
                        else
                            tmpidx=floor(TF.f(idx,idx_f)/2);
                            tmp=tmp*sin(tmpidx*TF.coord(idx_f));
                        end
                    end
                    if ~isempty(TF.c(idx))
                        sym_var=sym_var+tmp*TF.c(idx);
                    end
                end
            else
                for idx_r=1:size(TF,1)
                    for idx_c=1:size(TF,2)
                        sym_var(idx_r,idx_c)=TF(idx_r,idx_c).toSym();
                    end
                end
            end
        end % toSym
        
        function TF=skew(v)
            zero=SRD.zero(1,v(1,1).coord);
            TF = [  zero   -v(3)  v(2);
              v(3)  zero    -v(1);
             -v(2) v(1)   zero];
        end %skew
        
        function TF=vex(v)
            TF(1,1)=v(3,2);
            TF(2,1)=v(1,3);
            TF(3,1)=v(2,1);
        end %vex
    end
    
end