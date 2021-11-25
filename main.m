
%Master script for program execution

%STEP1: Take input of geometry of structure: [Beam_length ,Coulumn Height]
%Executing app for taking layout info
app_Layout=Layout; 
waitfor(app_Layout);
%L,H now contain Beam Length and Column Height, Node coordinate contains
%coordinates for 12 Nodes

%%STEP2: Tlake input for section geometry of beam and column
%Executing beamprop
app_beamprop=beamprop;
waitfor(app_beamprop);
Beam_props=[L ;E; G; Area; Ixx; Iyy; Izz];
app_columnrop=columnprop;
waitfor(app_columnrop);
Column_props=[H; E; G; Area; Ixx; Iyy; Izz];


%%STEP 3: Defining Nodes and Members(as pair of nodes) 
Elements=[1 2; 2 3; 3 4; 4 5;5 6;6 1;1 7; 2 8; 3 9; 4 10; 5 11; 6 12];
    %Element(i,:) contains [Node1 Node2] of ith elements
Nodes=[];
Members=[];
    %Defining a master node at floor slab center
Master_node=Node(0,[0;0;0]);
    %Now defining all other nodes
for i=1:length(Node_Coordinates)
    Nodes=[Nodes; Node(i,reshape(Node_Coordinates(i,:),3,1))];
end

 %Now defining members with properties
    %First beams
    for i=1:6
        N1=Nodes(Elements(i,1));
        N1.get_pos();
        N2=Nodes(Elements(i,2));
        Members=[Members; Beam(N1,N2,Beam_props(2),Beam_props(3),Beam_props(4),Beam_props(5),Beam_props(6),Beam_props(7))];
    end
    %Then Columns
    for i=7:12
        N1=Nodes(Elements(i,1));
        N2=Nodes(Elements(i,2));
        Members=[Members; Beam(N1,N2,Column_props(2),Column_props(3),Column_props(4),Column_props(5),Column_props(6),Column_props(7))];
    end
    

%%STEP 5: Applying Nodal Loads
Nodes(1).set_Load([10000;10000;0;0;0;0]);   %=10000 N

%%STEP 6: Applying support conditions
for i=7:12
    Nodes(i).set_restrains([1;1;1;1;1;1]);   %Fully fixed all 6 DOF restrained
end

Equib_Loads=zeros(6,1);
for i=1:length(Nodes)
    Nod_Pos=Nodes(i).get_pos();
    Loads=Nodes(i).get_Load();
    
    Restr=Nodes(i).get_restrain();
    for j=1:6
        if(Restr(j)~=1)
        Equib_Loads(j)=Equib_Loads(j)+Loads(j);
        end 
    end
    Equib_Loads([4:6])=Equib_Loads([4:6])+cross(Nod_Pos,Loads([1:3]));
   
end

%%STEP 7: Numbering DOFs
    %First enslaving floor DOFs for rotation and translation
    Enslaved_DOFs=[1;1;0;0;0;1];    %x,y translation and thetaz
    for i=1:6
    Nodes(i).set_slaved_DOF(Enslaved_DOFs);   %Enslaved x,y, translation and z rotation
    Nodes(i).set_master_pos(Master_node.get_pos());     %Set Master_node as master of ith node
    end
    DOF_counter=1;
    %Next numbering Master Floor DOFs
    Master_node=Master_node.set_Association([1;2;-1;-1;-1;3]); %Only two translation[ux,uy] and one rotation(uz) taken
    Master_Node_DOFs=[1;2;-1;-1;-1;3];
    DOF_counter=DOF_counter+3;
    
    %Then numbering unrestrained and unslaved DOFs
    Count_Pj=0;
    for i=1:length(Nodes)
        
        restr=Nodes(i).get_restrain();
        slaved_dofs=Nodes(i).get_slaved_DOF();
        Associate=Nodes(i).get_Association();
        for j=1:length(restr)
            if(restr(j)==0 && slaved_dofs(j)==0)
            Associate(j)=DOF_counter;
            DOF_counter=DOF_counter+1;
            Count_Pj=Count_Pj+1;
            end
        end
        Nodes(i).set_Association(Associate);
    end
    DOF_counter;
    %Next numbering restrained and unslaved
    for i=1:length(Nodes)
        
        restr=Nodes(i).get_restrain();
        slaved_dofs=Nodes(i).get_slaved_DOF();
        Associate=Nodes(i).get_Association();
        for j=1:length(restr)
            if(restr(j)==1 && slaved_dofs(j)==0)
            Associate(j)=DOF_counter;
            DOF_counter=DOF_counter+1;
            end
        end
        
        Nodes(i).set_Association(Associate);
       
    end
    DOF_counter;
    %Finally associating slaved DOFs with master floor node DOFs
    for i=1:length(Nodes)
        
        restr=Nodes(i).get_restrain();
        slaved_dofs=Nodes(i).get_slaved_DOF();
        Associate=Nodes(i).get_Association();
        count_slaved_DOF=1;
        for j=1:length(restr)
            if(slaved_dofs(j)==1)
            Associate(j)=count_slaved_DOF;
            count_slaved_DOF=count_slaved_DOF+1;
            end
        end
      
        Nodes(i).set_Association(Associate);
    end
   
%%STEP 8: Assembling total stiffness matrix
N=DOF_counter-1;     %K_TS size
p=Count_Pj+3;        %UnrestrainedDOF+ Floor_DOFs size
K_TS=zeros(N,N);

for i=1:length(Members)
    this_member=Members(i);
    Association=this_member.get_association();
    k_g=this_member.get_global_K();  %Member global stiffness matrix
    C=this_member.get_C();
    K_g_star=C'*(k_g*C);           %Rigid body slab transformation
    for m=1:length(K_g_star)
        for n=m:length(K_g_star)  
            K_TS(Association(m),Association(n))=K_TS(Association(m),Association(n))+K_g_star(m,n);
            if(m~=n)
            K_TS(Association(n),Association(m))=K_TS(Association(n),Association(m))+K_g_star(m,n);
            end
        end
    end 
end

%%STEP 9: Forming P* and U* vector
P_star=zeros(N,1);   %Nodal Force vector
U_star=zeros(N,1);   %Nodal Disp vector
for i=1:length(Nodes)
    Association=Nodes(i).get_Association();
    Nodal_Disp=Nodes(i).get_Disp();
    Nodal_Force=Nodes(i).get_Load();
    C_nodal=Nodes(i).get_C();
    P_star(Association)=P_star(Association)+C_nodal'*Nodal_Force;    
    U_star(Association)=U_star(Association)+C_nodal'*Nodal_Disp; 
end
   

%%STEP 10: Partitioning K_TS ,P* and U* vector

Kpp=K_TS(1:p,1:p);
Kpx=K_TS(1:p,p+1:end);
Kxp=K_TS(p+1:end,1:p);
Kxx=K_TS(p+1:end,p+1:end);

Pp_star=P_star(1:p);
Ux_star=U_star(p+1:end);


%%STEP 11: Solving P*-U* relation using choleski inverse
% """Solving force-displacement equations
%     [Kpp]{Up*}+[Kpx]{Ux*}={Pp*}
%     [Kxp]{Up*}+[Kxx]{Ux*}={Px*}
% 
%     {Up*}=[Kpp]^(-1){[Pp*]-[Kpx][Ux*]}     #Unknown forces 
%     {Px*}=[Kxp]{Up*}+[Kxx]{Ux*}            #Unknown Reactions
% """

Up_star=Inverse_matrix_using_Cholesky(Kpp)*(Pp_star-Kpx*Ux_star);
Px_star=Kxp*Up_star+Kxx*Ux_star;
U_star=[Up_star;Ux_star];
P_star=[Pp_star;Px_star];

%%STEP 12: Updating Nodes with calculated values
    %First Master slab node
Master_node.set_Disp([U_star(1);U_star(2);0;0;0;U_star(3)]);
    %Now other nodes
for i=1:length(Nodes)
    Association=Nodes(i).get_Association();
    C_nodal=Nodes(i).get_C();
    Nodes(i).set_Disp(C_nodal*U_star(Association));
    Nodes(i).set_Load(C_nodal*P_star(Association));
end

%%STEP 13: Force Equilibrium check
Equib_Reactions=zeros(6,1);
for i=1:length(Nodes)
    Nod_Pos=Nodes(i).get_pos();
    Loads=Nodes(i).get_Load();
    F=zeros(3,1);
    Restr=Nodes(i).get_restrain();
    for j=1:length(Restr)
        if(Restr(j)==1)
        Equib_Reactions(j)=Equib_Reactions(j)+Loads(j);
        if(j<4)
        F(j)=F(j)+Loads(j);
        end
        end
    end
    Equib_Reactions([4:6])=Equib_Reactions([4:6])+cross(Nod_Pos,F);
end

%%STEP 14: Printing output results to file
out_file=fopen('output.txt','w');

%First Nodal displacements
fprintf(out_file,"\n||NODAL DISPLACEMENTS||\n");
fprintf(out_file,"\nNode\t\t Ux(mm)\t\t\tUy(mm)\t\t\tUz(mm)\t\t\tTheta_x(rad)\tTheta_y(rad)\tTheta_z(rad)\n");
fprintf(out_file,"0[Slab]\t");
Master_node_disp=Master_node.get_Disp();
Master_node_disp(1:3)=Master_node_disp(1:3)*1000;
fprintf(out_file,"%12.6f\t",Master_node_disp);
fprintf(out_file,"\n");
for i=1:length(Nodes)
    Nodal_Disp=Nodes(i).get_Disp();
    Nodal_Disp(1:3)=Nodal_Disp(1:3)*1000; %Changing to mm
    fprintf(out_file,string(i)+"\t\t");
    fprintf(out_file,"%12.6f\t",Nodal_Disp);
    fprintf(out_file,"\n");
end

%Then Member end forces
fprintf(out_file,"\n\n||MEMBER FORCES||\n");
fprintf(out_file,"\nMember\tNode\t  Fx(kN)\t\t\tFy(kN)\t\t\tFz(kN)\t\t\tMx(kNm)\t\t\tMy(kNm)\t\t\tMz(kNm)\n");
fprintf(out_file,"\n");
for i=1:length(Members)
    Mem_force_vec=Members(i).get_internal_force();
    Mem_nodes=Members(i).get_nodes();
    fprintf(out_file,string(i)+"\t\t"+string(Mem_nodes(1).get_ID())+"\t");
    fprintf(out_file,"%12.6f\t",Mem_force_vec([1:6])*0.001);
    fprintf(out_file,"\n");
    fprintf(out_file,"\t\t"+string(Mem_nodes(2).get_ID())+"\t");
    fprintf(out_file,"%12.6f\t",Mem_force_vec([7:12])*0.001);
    fprintf(out_file,"\n");
end

%Then Reactions 
fprintf(out_file,"\n\n||Reactions||\n");
fprintf(out_file,"Node\t\tFx(kN)\t\t\tFy(kN)\t\t\tFz(kN)\t\t\tMx(kNm)\t\t\tMy(kNm)\t\t\tMz(kNm)\n");
fprintf(out_file,"\n");
for i=1:length(Nodes)
    Nodal_Force=Nodes(i).get_Load();
    Nodal_Restr=Nodes(i).get_restrain();
    if(sum(Nodal_Restr)~=0)
    fprintf(out_file,string(i)+"\t\t");
    for j=1:length(Nodal_Restr)
        if(Nodal_Restr(j)==1)
        fprintf(out_file,"%12.6f\t",Nodal_Force(j)*0.001);
        else
        fprintf(out_file,"\t\t\t\t");
        end
    end
    fprintf(out_file,"\n");
    end
end

%Then Equilibrium Check
fprintf(out_file,"\n\n||Equilibrium Check||\n");
fprintf(out_file,"L/C\t\t\t\t\tFx(kN)\t\t\tFy(kN)\t\t\tFz(kN)\t\t\tMx(kNm)\t\t\tMy(kNm)\t\t\tMz(kNm)\n");
fprintf(out_file,"\nLoads\t\t\t");
fprintf(out_file,"%12.6f\t",Equib_Loads*0.001);
fprintf(out_file,"\nReactions\t\t");
fprintf(out_file,"%12.6f\t",Equib_Reactions*0.001);
fprintf(out_file,"\nDifference\t\t");
fprintf(out_file,"%12.6f\t",(Equib_Loads+Equib_Reactions)*0.001);