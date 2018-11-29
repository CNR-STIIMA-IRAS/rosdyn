function T=parents_matrix(parents,children,links_name,joints)

T=zeros(length(links_name));

for index=1:length(links_name)
    idxs=find(strcmp(children,links_name{index}))';
    for idx=idxs
        T(index, strcmp(links_name,parents{idx}))=1;
    end
end