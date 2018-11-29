function T=children_matrix(parents,children,links_name)

T=zeros(length(links_name));

for index=1:length(links_name)
    idxs=find(strcmp(parents,links_name{index}))';
    for idx=idxs
        T(index, strcmp(links_name,children{idx}))=1;
    end
end