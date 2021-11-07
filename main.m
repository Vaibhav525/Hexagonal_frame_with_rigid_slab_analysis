
%Master script for program execution

%STEP1: Take input of geometry of structure: [Beam_length ,Coulumn Height]
%Executing app for taking layout info
app_Layout=Layout; 
waitfor(app_Layout);
%L,H now contain Beam Length and Column Height, Node coordinate contains
%coordinates for 12 Nodes

%%STEP2: Take input for section geometry of beam and column
%Executing beamprop
app_beamprop=beamprop;
waitfor(app_beamprop);
Beam_props=[L ;E; G; Area; Ixx; Iyy; Izz];
app_columnrop=columnprop;
waitfor(app_columnrop);
Column_props=[H; E; G; Area; Ixx; Iyy; Izz];


%%STEP 3: Defining Members and Nodes
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
Nodes(1).set_Load([10;0;0;0;0;0]);

%%STEP 6: Applying support conditions
for i=7:12
    Nodes(i).set_restrains([1;1;1;1;1;1]);   %Fully fixed all 6 DOF restrained
end

%%STEP 7: Numbering DOFs
    %First enslaving floor DOFs for rotation and translation
    Enslaved_DOFs=[1;1;0;0;0;1];    %x,y translation and thetaz
    for i=1:6
    Nodes(i).set_slaved_DOF(Enslaved_DOFs);   %Enslaved x,y, translation and z rotation
    end
    DOF_counter=1;
    %Next numbering Master Floor DOFs
    Master_node=Master_node.set_Association([1;2;3;-1;-1;-1]); %Only two translation and one rotation taken
    Master_Node_DOFs=[1;2;3];
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
p=Count_Pj+3;           %UnrestrainedDOF+ Floor_DOFs size
K_TS=zeros(N,N);

%Defining rigid body transformation matrix ,C matrix 
Pos_Master=Master_node.get_pos();
xj=Pos_Master(1);
yj=Pos_Master(2);
C=eye(12,12);
C(1,6)=-yj;
C(7,12)=-yj;
C(2,6)=xj;
C(8,12)=xj;

for i=1:length(Members)
    this_member=Members(i);
    Association=this_member.get_association();
    k_g=this_member.get_global_K();  %Member global stiffness matrix
    K_g_star=C'*(k_g*C);        %Rigid body slab transformation
    K_TS(Association,Association)=K_TS(Association,Association) +K_g_star; %Assembling                           %Assembly
end

%%STEP 9: Forming P and U vector
P=zeros(N,1);   %Nodal Force vector
U=zeros(N,1);    %Nodal Disp vector
for i=1:length(Nodes)
    Association=Nodes(i).get_Association();
    Nodal_Disp=Nodes(i).get_Disp();
    Nodal_Force=Nodes(i).get_Load();
    P(Association)=P(Association)+Nodal_Force;
    U(Association)=U(Association)+Nodal_Disp; 
end
   

%%STEP 10: Partitioning K_TS ,P and U vector

Kpp=K_TS(1:p,1:p);
Kpx=K_TS(1:p,p+1:end);
Kxp=K_TS(p+1:end,1:p);
Kxx=K_TS(p+1:end,p+1:end);

Pp=P(1:p);
Ux=U(p+1:end);


%%STEP 11: Solving P-U relation using choleski inverse
% """Solving force-displacement equations
%     [Kpp]{Up}+[Kpx]{Ux}={Pp}
%     [Kxp]{Up}+[Kxx]{Ux}={Px}
% 
%     {Up}=[Kpp]^(-1){[Pp]-[Kpx][Ux]}     #Unknown forces 
%     {Px}=[Kxp]{Up}+[Kxx]{Ux}            #Unknown Reactions
% """

Up=Inverse_matrix_using_Cholesky(Kpp)*(Pp-Kpx*Ux);
Px=Kxp*Up+Kxx*Ux;
U=[Up;Ux];
P=[Pp;Px];
%%STEP 12: Updating Nodes with calculated values
for i=1:length(Nodes)
    Association=Nodes(i).get_Association();
    Nodal_Disp=Nodes(i).get_Disp();
    Nodal_Force=Nodes(i).get_Load();
    P(Association)=P(Association)+Nodal_Force;
    U(Association)=U(Association)+Nodal_Disp; 
end
