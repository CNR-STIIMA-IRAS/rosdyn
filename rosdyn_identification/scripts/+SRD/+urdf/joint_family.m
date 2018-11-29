function [parent,child]=joint_family(joint)
parent=[];
child=[];
for index=1:length(joint.Children)
    if strcmp(joint.Children(index).Name,'child')
        if ~isempty( joint.Children(index).Attributes)
            child=joint.Children(index).Attributes.Value;
        elseif  isscalar(joint.Children(index).Children)
            child=joint.Children(index).Children.Data;
        else
            error('child name do not found')
        end
    elseif strcmp(joint.Children(index).Name,'parent')
        if ~isempty( joint.Children(index).Attributes)
            parent=joint.Children(index).Attributes.Value;
        elseif  isscalar(joint.Children(index).Children)
            parent=joint.Children(index).Children.Data;
        else
            error('parent name do not found')
        end
    end
end
