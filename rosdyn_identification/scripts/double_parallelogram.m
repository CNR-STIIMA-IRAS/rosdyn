ccc;


syms qm1 q2 q3 q4 q5 q6 q7 
syms gy l1 l2 l3 l4 l5 l6 l7 p1 p2
q=[qm1 q2 q3 q4 q5 q6 q7];
Motorized=[1 0 0 0 0 0 0];
verbose=1;
planar=1;

debug_idx=0;
Robot2=SRD.robot(q,Motorized,'fourbar','',[0;gy;0],verbose,planar);
Robot2.addlink([],{},'ground');

debug_idx=debug_idx+1;
fprintf('step %d\n',debug_idx); %1
%
M1=SRD.rotz(qm1,q);
Robot2.addlink('ground',{M1},'_link1');

debug_idx=debug_idx+1;
fprintf('step %d\n',debug_idx); %2
%
M2=SRD.trax(l1,q)*SRD.rotz(q2,q);
Robot2.addlink('_link1',{M2},'_link2');

debug_idx=debug_idx+1;
fprintf('step %d\n',debug_idx); %3
%
M3=SRD.trax(l2,q)*SRD.rotz(q3,q);
Robot2.addlink('_link2',{M3},'_link3');

debug_idx=debug_idx+1;
fprintf('step %d\n',debug_idx); %4
%
M4=SRD.trax(l3,q)*SRD.rotz(q4,q);
M5=SRD.trax(p1,q);
Robot2.addlink({'_link3','ground'},{M4,M5},'_link4');

debug_idx=debug_idx+1;
fprintf('step %d\n',debug_idx); %5

%
syms alpha
fixed=1;
M6=SRD.rotz(alpha,q,fixed)*SRD.trax(l3,q)*SRD.rotz(q5,q);
Robot2.addlink({'_link4'},{M6},'_link5'); 

debug_idx=debug_idx+1;
fprintf('step %d\n',debug_idx); %6
%
M7=SRD.trax(l6,q)*SRD.rotz(q6,q);
Robot2.addlink('_link5',{M7},'_link6'); 

debug_idx=debug_idx+1;
fprintf('step %d\n',debug_idx); %7
%
M8=SRD.trax(l7,q)*SRD.rotz(q7,q);
M9=SRD.trax(p1+p2,q);
Robot2.addlink({'_link6','ground'},{M8,M9},'_link7'); %8

debug_idx=debug_idx+1;
fprintf('step %d\n',debug_idx); %9



Robot2.computeVel

debug_idx=debug_idx+1;
fprintf('step %d\n',debug_idx); %10
Robot2.computeKineticEnergy

debug_idx=debug_idx+1;
fprintf('step %d\n',debug_idx); %11
Robot2.computeGravPotEnergy

debug_idx=debug_idx+1;
fprintf('step %d\n',debug_idx); %12
Robot2.computeLagrangeEquation
save('double_new3');
debug_idx=debug_idx+1;
fprintf('step %d\n',debug_idx); %14
Robot2.computeBaseParameters

debug_idx=debug_idx+1;
fprintf('step %d\n',debug_idx); %14
BP=Robot2.Treduced.'*Robot2.Parameters