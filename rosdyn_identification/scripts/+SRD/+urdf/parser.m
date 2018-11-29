function [joints,links,trasmissions,gazebo,coupling_joints,robot]=parser(urdf_file,worldname)
a=xml2struct(urdf_file);

%load xmlfile
%%
for index=1:length(a)
    if strcmp(a(index).Name,'robot')
        robot=a(index);
    end
end
%%
indexes_link=[];
indexes_joint=[];
indexes_coupling_joint=[];
indexes_transmission=[];
indexes_gabezo=[];

for index=1:length(robot.Children)
    if strcmp(robot.Children(index).Name,'link')
        indexes_link=[indexes_link;index];
    elseif strcmp(robot.Children(index).Name,'joint')
        indexes_joint=[indexes_joint;index];
    elseif strcmp(robot.Children(index).Name,'transmission')
        indexes_transmission=[indexes_transmission;index];
    elseif strcmp(robot.Children(index).Name,'gazebo')
        indexes_gabezo=[indexes_gabezo;index];
        %     else
        %         robot.Children(index).Name
    end
end
%%

links=robot.Children(indexes_link)';
joints=robot.Children(indexes_joint)';
trasmissions=robot.Children(indexes_transmission)';
gazebo=robot.Children(indexes_gabezo)';

coupling_joints=[];
for index=1:length(gazebo)
    if isempty(gazebo(index).Attributes)
        for idx=1:length(gazebo(index).Children)
            if strcmp(gazebo(index).Children(idx).Name,'joint')
                coupling_joints=[coupling_joints;gazebo(index).Children(idx)];
            end
        end
    end
end