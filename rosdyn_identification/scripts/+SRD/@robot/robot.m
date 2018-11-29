classdef robot < matlab.mixin.Copyable
    properties (SetAccess = protected )
        Description
        Label

        LinkGround
        ClosedChain
        Loops
        
        dcdq_dep
        dcdq_ind
        Ddcdq_dep
        Ddcdq_ind
        det_dcdq_dep
        adj_dcdq_dep
        
        KinEnergy
        GravPotEnergy
        ElasPotEnergy
        Gravity
        
        Joints
        DJoints
        DDJoints
        MotJoints
        
        tau
        tauOC
        tau_den
        
        Parameters=[];
        Treduced
        
        KinEnComputed=false
        GravPotEnComputed=false
        loopReduced=false;
        verbose=false;
        OCcomputed=false;
        Planar
    end
    methods
        function obj=robot(extJoints,extMotJoints,extLabel,extDescription,extGravity,extVerbose,extPlanar)
            if nargin>=7
                obj.Planar=extPlanar;
            end
            if nargin>=6
                obj.verbose=true;
            end
            if nargin<5
                syms gx gy gz
                extGravity=[gx;gy;gz];
            end
            if nargin<4
                extDescription='';
            end
            if nargin<3
                extLabel='';
            end
            obj.ClosedChain=0;
            if isrow(extJoints);
                obj.Joints=extJoints.';
            else
                obj.Joints=extJoints;
            end
            
            
            obj.MotJoints=extMotJoints;
            obj.Label=extLabel;
            obj.Description=extDescription;
            obj.Gravity=extGravity;
            
            obj.DJoints=sym(zeros(size(obj.Joints)));
            obj.DDJoints=sym(zeros(size(obj.Joints)));
            for idx=1:length(extJoints)
                eval(['syms D',char(obj.Joints(idx))]);
                eval(['syms DD',char(obj.Joints(idx))]);
                eval(['obj.DJoints(idx,1)=D',char(obj.Joints(idx)),';']);
                eval(['obj.DDJoints(idx,1)=DD',char(obj.Joints(idx)),';']);
            end
        end %robot
        
        function link_obj=addlink(obj,extParents,extMrel,extLabel,extDescription,extNoMass)
            if nargin<6
                extNoMass=false;
            end
            
            if nargin<4
                extLabel='';
            end
            if nargin<5
                extDescription='';
            end
            if iscell(extParents)
                for index=1:length(extParents)
                    extParentsLink(index,1)=obj.findLink(extParents{index});
                end
            elseif isstr(extParents)
                extParentsLink=obj.findLink(extParents);
            else
                extParentsLink=extParents;
            end
            link_obj=SRD.link(obj,extParentsLink,extMrel,extLabel,extDescription,extNoMass);
            
            if obj.verbose
                fprintf(' [robot: %s] added link %s\n',obj.Label,extLabel);
            end
        end %addlink
        
        function computeVel(obj)
            obj.LinkGround.computeVel;
        end %computeVel
        
        function setground(obj,link)
            obj.LinkGround=link;
        end %setground
        
        function setClosedChain(obj,value)
            obj.ClosedChain=value;
        end %setClosedChain
        
        function addLoops(obj,eq)
            if isempty(obj.Loops)
                obj.Loops=eq;
            else
                obj.Loops=[obj.Loops;eq];
                obj.Loops.order();
            end
        end %addLoops
        
        function addParameters(obj,par)
            obj.Parameters=[obj.Parameters;par];
        end
        
        function setMotorJoints(obj,MotJoints)
            obj.MotJoints=MotJoints;
        end %setMotorJoints

        function link_handle=findLink(obj,string)
            
            link_handle=obj.LinkGround.findLink(string);
            if isempty(link_handle)
                error('Cannot find ''%s'' link label\n',string);
            end
        end %findLink
        
        function computeKineticEnergy(obj)
            if ~obj.KinEnComputed
                obj.KinEnergy=obj.LinkGround.KineticEnergy;
                obj.KinEnComputed=true;
            end
        end %computeKineticEnergy
        
        function computeGravPotEnergy(obj)
            if ~obj.GravPotEnComputed
                obj.GravPotEnergy=obj.LinkGround.GravPotEnergy;
                obj.GravPotEnComputed=true;
            end
        end %computeGravPotEnergy
        
        function computeElasPotEnergy(obj,K,offset)
            if exist('K','var')
                if exist('offset','var')
                    obj.ElasPotEnergy=(obj.Joints-offset).'*K*(obj.Joints-offset)/2;
                else
                    obj.ElasPotEnergy=SRD.one(obj.Joints(1),obj.Joints)* obj.Joints.'*K*obj.Joints/2;
                end
            else
                obj.ElasPotEnergy=SRD.zero(obj.Joints(1),obj.Joints);
            end
        end %computeElasPotEnergy
        
        function computeLagrangeEquation(obj)

            if obj.ClosedChain
                obj.velLoops;
            end
            q=obj.Joints;
            qp=obj.DJoints;
            qpp=obj.DDJoints;
            q=q(:); %serve per ovviare a un baco
            qp=qp(:); %serve per ovviare a un baco
            qpp=qpp(:); %serve per ovviare a un baco

            if not(obj.OCcomputed)
                obj.OCcomputed=true;
                K=obj.KinEnergy;
                GP=obj.GravPotEnergy;
                EP=obj.ElasPotEnergy;
                %
                obj.tauOC=SRD.zero(1,q);
                for index=1:length(obj.Joints)
                    if obj.verbose
                        fprintf('[ computeLagrangeEquation ] torque %d ...',index);
                    end
                    tmp_K=diff(K,qp(index));
                    first_term=SRD.zero(1,obj.Joints);
                    for index2=1:length(obj.Joints)
                        first_term=first_term+diff(tmp_K,qp(index2))*qpp(index2)+diff(tmp_K,q(index2))*qp(index2);
                    end
                    obj.tauOC(index,1)=first_term+diff(GP+EP-K,q(index));
                    if obj.verbose
                        fprintf('done\n');
                    end
                end
            end
            if obj.ClosedChain
                if obj.verbose
                    fprintf('[ computeLagrangeEquation ] closed-chain torques..' );
                end
                    
                obj.tau_den=repmat(obj.det_dcdq_dep,length(obj.tauOC),1);
                obj.tau=repmat(SRD.zero(1,q),length(obj.tauOC),1);
                obj.tau(obj.MotJoints==1)=obj.tauOC(obj.MotJoints==1)*obj.det_dcdq_dep-(obj.dcdq_ind')*(obj.adj_dcdq_dep')*obj.tauOC(obj.MotJoints==0);
                
            else
                
                obj.tau=obj.tauOC;
                obj.tau_den=repmat(SRD.one(1,q),length(obj.tau),1);
            end
        end %computeLagrangeEquation
        
        function velLoops(obj)
            if ~obj.loopReduced
                if obj.verbose
                    fprintf('[ computeLagrangeEquation ] reduceLoops..' );
                end
                obj.reduceLoops;
                if obj.verbose
                    fprintf('done\n');
                end
            end
            
            dcdq=jac(obj.Loops,q);
            idxs=find(obj.MotJoints==0);
            if sum(obj.MotJoints==0)~=length(obj.Loops)
                warning('the number of constraints is not correct');
            end
            dcdq=dcdq(1:length(idxs),:);
            obj.dcdq_dep=dcdq(:,obj.MotJoints==0);
            [obj.adj_dcdq_dep,obj.det_dcdq_dep]=inv(obj.dcdq_dep);
            obj.dcdq_ind=dcdq(:,obj.MotJoints==1);
            % XXX RIVEDERE GESTIONE VELOCITA'
        end %velLoops
        
        function reduceLoops(obj)
            obj.Loops=obj.Loops.reduce(obj.verbose);
%            obj.Loops=obj.Loops([3 5 6]);
            %obj.Loops=obj.
            obj.loopReduced=true;
        end %reduceLoops
        
        function computeBaseParameters(obj)
            q  =obj.Joints;
            qp =obj.DJoints;
            qpp=obj.DDJoints;
            if obj.verbose
                fprintf('[ computeBaseParameters ] purge tau..');
            end
            obj.tau=obj.tau.purge;
            if obj.verbose
                fprintf('done..');
            end
            T={};
            IM={};
            Ttot=[];
            if obj.Planar
                nlink=size(obj.Parameters,1)/10;
                indexes=[];
                for index=1:nlink;
                    indexes=[indexes,[1:3 10]+(index-1)*10];
                end
                obj.Parameters=obj.Parameters(indexes);
            end
            
            for index=1:length(obj.tau);
                if obj.verbose 
                    fprintf('..tau %d of %d..\n',index,length(obj.tau));
                end
                if obj.MotJoints(index)==1
                    C=obj.tau(index).c;
                    [Cdecompose,Fun]=SRD.sym.CDecomposition(C,q,qp,qpp,obj.verbose);
                    %%
                    c=sym([]);
                    for index2=1:length(Cdecompose)
                        tmpc=Cdecompose{index2};
                        if ~isempty(tmpc)
                            c=vertcat(c,tmpc(tmpc~=0));
                        end
                    end
                    [T{index},IM{index}]=SRD.sym.basepar(c,obj.Parameters,obj.verbose);
                    %%
                    Ttot=[Ttot;T{index}.'];
                else
                    %                    C=obj.tauOC(index).c;
                end
            end
            [IM,obj.Treduced]=SRD.sym.indipendent_columns(Ttot,obj.verbose);
        end %computeBaseParameters
        
        function delete(obj)
            if obj.LinkGround.isvalid
                delete(obj.LinkGround)
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