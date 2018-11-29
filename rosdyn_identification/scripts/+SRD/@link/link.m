classdef link < matlab.mixin.Copyable
    properties (SetAccess = protected )
        Parents
        Robot
        Children
        Description
        Label
        
        Mrel
        Mabs
        AngJacobian
        LinJacobian
        LinVel
        LinAcc
        AngVel
        AngAcc
        
        InertiaMatrix
        Mass
        FirstMoments
        
        %JointType
        %Motorized
        ground
        velComputed=false;
        KinEnComputed=false;
        GravPotEnComputed=false;
        nomass=false;
    end
    methods
        function obj=link(extRobot,extParents,extMrel,extLabel,extDescription,extNomass)
            if nargin>=6
                obj.nomass=extNomass;
            end
            if nargin<5
                extDescription='';
            end
            if nargin<4
                extLabel='';
            end
            if isempty(extParents)
                obj.ground=1;
                obj.Parents=[];
                obj.Robot=extRobot;
                if isempty(extRobot.LinkGround)
                    setground(extRobot,obj);
                else
                    error('The robot has already a ground');
                end
            else
                obj.Robot=extRobot;
                obj.ground=0;
                obj.Mrel=extMrel;
                obj.Parents=extParents;
                if length(extParents)>1
                    extRobot.setClosedChain(1);
                end
                for index=1:length(extParents)
                    if isempty(extParents(index).Children)
                        extParents(index).Children=obj;
                    else
                        extParents(index).Children(end+1,1)=obj;
                    end
                end
            end
            obj.Parents=extParents;
            obj.Label=extLabel;
            obj.Description=extDescription;
            
            variables={'m',...
                'mcx','mcy','mcz',...
                'Ixx','Ixy','Ixz',...
                'Iyy','Iyz',...
                'Izz'};
            
            if and(~obj.ground,~obj.nomass)
                for index_var=1:length(variables)
                    eval(['syms ',variables{index_var},extLabel]);
                    eval([variables{index_var},'i=',variables{index_var},extLabel,';']);
                end
                obj.Mass=mi;
                obj.FirstMoments=[mcxi;mcyi;mczi];
                obj.InertiaMatrix=[Ixxi,Ixyi,Ixzi;
                    Ixyi,Iyyi,Iyzi;
                    Ixzi,Iyzi,Izzi];
                
                obj.Robot.addParameters([mi;mcxi;mcyi;mczi;Ixxi;Ixyi;Ixzi;Iyyi;Iyzi;Izzi]);
            end
            
            % Pos FKINE
            if obj.ground
                if ~isempty(extMrel)
                    warning('Mrel will be not taken into account for ground frame');
                end
                obj.Mabs=SRD.eye(4,extRobot.Joints);
            else
                obj.Mabs=obj.Parents(1).Mabs*obj.Mrel{1};
                for index=2:length(obj.Parents)
                    tmpM=obj.Parents(index).Mabs*obj.Mrel{index};
                    tmpM=tmpM.purge(1);
                    eq=obj.Mabs-tmpM;
                    if obj.Robot.Planar
                        extRobot.addLoops(eq([5 13:14]).')
                    else
                        extRobot.addLoops(eq([5:7 9:11 13:15]).')
                        %extRobot.addLoops(eq([9:11 13:15]).')
                    end
                end
                %.....
            end
            
        end %link
        
        function link_handle=findLink(obj,string)
            if strcmp(obj.Label,string)
                link_handle=obj;
            elseif ~isempty(obj.Children)
                for index=1:length(obj.Children)
                    link_handle=findLink(obj.Children(index),string);
                    if ~isempty(link_handle)
                        break
                    end
                end
            else
                link_handle=[];
            end
        end %link_handle
        
        function setMassPoint(obj)
           obj.InertiaMatrix=sym(zeros(size(obj.InertiaMatrix)));
           obj.FirstMoments=sym(zeros(size(obj.FirstMoments)));
        end
        
        function computeVel(obj)
            if obj.Robot.verbose
                fprintf('[link %s [ computeVel ] ...',obj.Label);
            end
            if and(~obj.ground,~obj.velComputed)
                obj.velComputed=true;
                Rrel=obj.Mrel{1}(1:3,1:3);
                Orel=obj.Mrel{1}(1:3,4);
                Oabs=obj.Mabs(1:3,4);
                Father=obj.Parents(1);
                if Father.velComputed==0
                    Father.computeVel;
                end
                R_Father=obj.Parents(1).Mabs(1:3,1:3);
                q=Orel(1).coord;
                
                for index_joint=1:length(q)
                    RelAngJacobian(:,index_joint)=(vex(diff(Rrel,q(index_joint))*Rrel.'));
                    LinJacobian_Translation(:,index_joint)=(R_Father*(diff(Orel,q(index_joint)))); % moviment due to translation
                    LinJacobian_Rotation(:,index_joint)=(skew(Father.AngJacobian(:,index_joint))*R_Father*Orel);
                end
                obj.LinJacobian=Father.LinJacobian+LinJacobian_Translation+LinJacobian_Rotation;
                obj.AngJacobian=Father.AngJacobian+R_Father*RelAngJacobian;
                
                obj.LinVel=obj.LinJacobian*obj.Robot.DJoints;
                obj.AngVel=obj.AngJacobian*obj.Robot.DJoints;
                
                
            elseif obj.ground
                q=obj.Mabs(1,1).coord;
                zero=SRD.zero(1,q);
                obj.AngJacobian=repmat(zero,3,length(q));
                obj.LinJacobian=repmat(zero,3,length(q));
                obj.velComputed=true;
            end
            if obj.Robot.verbose
                fprintf('done\n');
            end
            for idx=1:length(obj.Children)
                obj.Children(idx).computeVel;
            end
            
        end %computeVel
        
        function KinEn=KineticEnergy(obj)
            if obj.KinEnComputed
                KinEn=0;
            else
                if obj.nomass
                    obj.KinEnComputed=true;
                    KinEn=0;
                else
                    if obj.Robot.verbose
                        fprintf('[link %s [ KineticEnergy ] ...',obj.Label);
                    end
                    if obj.ground
                        KinEn=0;
                    else
                        R=obj.Mabs(1:3,1:3);
                        I=R*obj.InertiaMatrix*R.';
                        m=obj.Mass;
                        mc=obj.FirstMoments;
                        w=obj.AngVel;
                        v=obj.LinVel;
                        RotKinEnergy=(w.'*I*w*sym(1/2));
                        TraKinEnergy=(v.'*v)*(1/2*m)+v.'*cross(w,R*mc);
                        KinEn=RotKinEnergy+TraKinEnergy;
                        obj.KinEnComputed=true;
                    end
                end
            end
            if obj.Robot.verbose
                fprintf('done\n');
            end
            for index=1:length(obj.Children);
                KinEn=KinEn+obj.Children(index).KineticEnergy;
            end
            
        end %KineticEnergy
        
        function GravPotEn=GravPotEnergy(obj)
            if obj.GravPotEnComputed
                GravPotEn=SRD.zero(coord);
            else
                if obj.nomass
                    GravPotEn=0;
                    obj.GravPotEnComputed=true;
                else
                    if obj.Robot.verbose
                        fprintf('[link %s [ GravPotEnergy ] ...',obj.Label);
                    end
                    if obj.ground
                        GravPotEn=0;
                    else
                        R=obj.Mabs(1:3,1:3);
                        O=obj.Mabs(1:3,4);
                        mc=obj.FirstMoments;
                        m=obj.Mass;
                        % GravPotEn=-obj.Robot.Gravity.'*(m*O+R*mc);
                        GravPotEn=-(O*m+R*mc)'*obj.Robot.Gravity;
                        obj.GravPotEnComputed=true;
                    end
                end
            end
            if obj.Robot.verbose
                fprintf('done\n');
            end
            for index=1:length(obj.Children);
                GravPotEn=GravPotEn+obj.Children(index).GravPotEnergy;
            end
            
        end %GravPotEnergy
        
        function delete(obj)
            for index=1:length(obj.Children)
                if obj.Children(index).isvalid
                    delete(obj.Children(index))
                end
            end
        end %delete
        
    end
end



% 	FathersId		= Vettore Identificativi Padri (input, f X 1)
% 	ChildrenId		= Vettore Identificativi Figli (c X 1
% 	----MDHpar			= Matrice parametri MDH (input, f X 4 (alpha,d,theta,r))
% 	Mrel			= Celle con Matrici M relative ai padri (f celle di 4 X 4)
% 	Mabs			= Matrice M assoluta (4 X 4)
% 	----MDHparCL		= Matrice parametri MDH che individuano la posizione delle origini di Mrel{f} (con f>1) rispetto a Mrel{1} espresse nel s.d.r. di Mrel{1} (input, (f-1) X 4 (alpha,d,theta,r))
% 	LinVel			= Vettore Velocità lineare (3 X 1)
% 	AngVel			= Vettore Velocità angolare (3 X 1)
% 	KinEnergy		= Energia Cinetica (1 X 1)
% 	GravPotEnergy	= Energia Gravitazionale (1 X 1)
% 	InertiaMatrix	= Matrice di inerzia (3 X 3)
% 	mass			= massa  (1 X 1)
% 	JointType		= 0 ROT/1 TRA/[] non specificato/-1 no motore  (input, 1 X 1)
% 	LagrVars 		= lista delle variabili di lagrange presenti nel link
% 	Description		= descrizione (input, stringa)
% 	label           = nome del link