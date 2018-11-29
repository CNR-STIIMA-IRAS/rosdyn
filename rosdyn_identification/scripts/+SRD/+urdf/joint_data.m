function data=joint_data(joints)


for idx_joint=1:length(joints)
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
      axis_num(idx_joint,1:3)=[0 0 0];
   end
end
data.axis=axis_num;
data.xyzrpy=xyzrpz_num;