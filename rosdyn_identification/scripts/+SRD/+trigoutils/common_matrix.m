function [C,F]=common_matrix(vector)
union_array=vector(1,1).f;
if isrow(vector)
    vector=vector';
end
for index=2:length(vector)
    union_array = union(union_array,vector(index,1).f,'rows');
end
C=sym(zeros(size(union_array,1),length(vector)));
for index=1:length(vector)
    [~,idxs]=ismember(union_array,vector(index,1).f,'rows');
    if sum(idxs~=0)>0
        C(idxs~=0,index)=vector(index,1).c;
    end
end
F=union_array;