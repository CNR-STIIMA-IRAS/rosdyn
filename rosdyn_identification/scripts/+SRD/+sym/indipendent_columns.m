function [indipendent_matrix,T]=indipendent_columns(matrix,verbose)

if nargin<2
    verbose=0;
end
indexes=[];
indipendent_matrix=[];
T=sym([]);
tmp_rank=0;
max_rank=rank(matrix);
num_col=size(matrix,2);
for index=1:num_col
    test=0;
    if and(tmp_rank<=max_rank,sum(logical(matrix(:,index)==0))<length(matrix(:,index)))
        if rank(matrix(:,[indexes index]))>tmp_rank
            test=1;
        end
    end
    if test
        tmp_rank=tmp_rank+1;
        indipendent_matrix=[indipendent_matrix matrix(:,index)];
        if nargout>1
            T(index,end+1)=1;
        end
        indexes=[indexes index];
    else
        if ~isempty(indipendent_matrix)
            if nargout>1
                if sum(logical(matrix(:,index)==0))<length(matrix(:,index))
                    T(index,:)=linsolve(indipendent_matrix,matrix(:,index)).';
                else
                    T(index,:)=0;
                end
            end
        end
    end
    if verbose
        fprintf('%d of %d\n',index,num_col)
    end
end

