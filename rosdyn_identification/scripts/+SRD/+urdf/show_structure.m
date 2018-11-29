function [xyzrpz_num,axis_num]=show_structure(links_name,joints,link_name,space_number,parents,children)
if nargin<4
    space_number=0;
end
if nargin<5
    for index=1:length(joints)
        [parents{index,1},children{index,1}]=SRD.urdf.joint_family(joints(index));
    end
end

if nargin<3
    flag=1;
else
    flag=isempty(link_name);
end

if flag
    % remove children with parents outside the links_name list
    tmp_idxs=[];
    for index=1:length(parents)
        if ~isempty(find(strcmp(links_name,parents{index})==1,1))
            tmp_idxs=[tmp_idxs index];
        end
    end
    parents={parents{tmp_idxs}}';
    children={children{tmp_idxs}}';
    joints=joints(tmp_idxs);
    for index=1:length(links_name)
        if isempty(find(strcmp(children,links_name{index})==1,1))
            link_name=links_name{index};
            break
        end
    end
end

spaces=repmat(' ',1,space_number);
fprintf('%s:\n',link_name)
idxs_joints=find(strcmp(parents,link_name))';
max_string_length=0;
xyzrpy={};
for idx_joint=idxs_joints
    axis_dir='';
    for idx_children=1:length(joints(idx_joint).Children)
        if strcmp(joints(idx_joint).Children(idx_children).Name,'origin')
            for idx_attributes=1:length(joints(idx_joint).Children(idx_children).Attributes)
                if strcmp(joints(idx_joint).Children(idx_children).Attributes(idx_attributes).Name,'rpy')
                    rpy=joints(idx_joint).Children(idx_children).Attributes(idx_attributes).Value;
                    xyzrpz_num(idx_joint,4:6)=str2num(rpy);
                elseif strcmp(joints(idx_joint).Children(idx_children).Attributes(idx_attributes).Name,'xyz')
                    xyz=joints(idx_joint).Children(idx_children).Attributes(idx_attributes).Value;
                    xyzrpz_num(idx_joint,1:3)=str2num(xyz);
                end
            end
        elseif strcmp(joints(idx_joint).Children(idx_children).Name,'axis')
            axis_dir=joints(idx_joint).Children(idx_children).Attributes.Value;
            axis_num(idx_joint,1:3)=str2num(axis_dir);
        else
            joint_name=joints(idx_joint).Attributes(1).Value;
        end
    end
    if isempty(axis_dir)
        axis_dir='- - -';
    end
    xyzrpy{end+1,1}=sprintf('%s* %s, xyz=%s rpy=%s axis=%s ',spaces,joint_name,xyz,rpy,axis_dir);
    if length(xyzrpy{end,1})>max_string_length
        max_string_length=length(xyzrpy{end,1});
    end
end

for idx=1:length(xyzrpy)
    child_name=children{idxs_joints(idx)};
    if sum(strcmp(links_name,child_name))
        fprintf('%s& ',xyzrpy{idx})
        SRD.urdf.show_structure(links_name,joints,child_name,max_string_length-1)
    end
end