
%Master script for program execution
clc;
close all;
%STEP 0: Defining a matrix inversion function, Node class and a Beam class

%Node(Also Provided in separate file, since matlab doesn't support multiple class in one script)
% classdef Node < handle
% %    """Node
% %         -id= node id
% %         -Pos: [x,y] Coordinate Position
% %         -Disp: [ux,uy],( =[None,None] if unknown)
% %         -Restrained[i]==1 if restrained 0 if unrestrained
% %         -Load Vector=[Px,Py]    (Default [0,0])
% %         -Association :list ->Association[i]=Structure DOF corresponding
% %         to ith local DOF, -1 default uninitialised
% %         """
%     %%Author :Vaibhav Bihani, 2018CE10169, IITD
%    properties(SetAccess=private)
%   
%               id    (1,1) int64
%               Pos   (3,1) double
%               Disp_vec  (6,1) double 
%               Restrain  (6,1) int64   %0 for unrestrained DOF, 1 for restrained DOF
%               Load      (6,1) double 
%               Association (6,1) int64 
%               is_slave  (1,1) logical
%               master_pos   (3,1) double  %Master Node position
%               Slave     (6,1) int64  %0 for free, 1 for slaved
%    end
% 
%    methods (Access = public)
%        function obj = Node(myid,myPos,myDisp_vec,myResrain,myLoad, myAssociation,mySlave)
%        arguments
%        myid     (1,1) int64=0
%        myPos    (3,1) double=[0.0;0.0;0.0]
%        myDisp_vec   (6,1) double=[0.0;0.0;0.0;0.0;0.0;0.0]
%        myResrain    (6,1) int64=[0;0;0;0;0;0]
%        myLoad       (6,1) double=[0.0;0.0;0.0;0.0;0.0;0.0]
%        myAssociation (6,1) int64=[-1;-1;-1;-1;-1;-1]
%        mySlave     (6,1) int64 =[0;0;0;0;0;0]
%        end
%        obj.id=myid;
%        obj.Pos=myPos;
%        obj.Disp_vec=myDisp_vec;
%        obj.Restrain=myResrain;
%        obj.Load=myLoad;
%        obj.Association=myAssociation;
%        obj.Slave=mySlave;
%        obj.master_pos=myPos;
%        obj.is_slave=false;
%        end
% 
% 
%        function obj=set_pos(obj,Pos)
%            arguments
%            obj (1,1) Node
%            Pos (3,1) double
%            end
%            obj.Pos=Pos;
%        end
%        function out=get_pos(obj)
%             out=obj.Pos;
%        end
%        function set_master_pos(obj,Master_pos) 
%            arguments
%            obj  (1,1) Node
%            Master_pos (3,1) double
%            end
%            obj.is_slave=true;
%            obj.master_pos=Master_pos;
%        end
% 
%        function out=get_master_pos(obj) 
%            out=obj.master_pos;
%        end
%        function out=get_ID(obj)
%            out=obj.id;
%        end
%        function obj=set_Disp(obj,Disp)
%            arguments
%            obj  (1,1) Node
%            Disp (6,1) double
%            end
%            obj.Disp_vec=Disp;
%        end
%        function out=get_Disp(obj)
%            out=obj.Disp_vec;
%        end
%        function obj=set_Load(obj,Load)
%            obj.Load=Load;
%        end
%        function out=get_Load(obj)
%            out=obj.Load;
%        end
%        function obj=set_restrains(obj,Restrains)
%            arguments
%            obj (1,1) Node
%            Restrains (6,1) int64 
%            end
%            obj.Restrain=Restrains;
%        end
%        function out=get_restrain(obj)
%            out=obj.Restrain;
%        end
%            function obj=set_slaved_DOF(obj,Slaved_DOF)
%            arguments
%            obj (1,1) Node
%            Slaved_DOF (6,1) int64 
%            end
%            obj.Slave=Slaved_DOF;
%        end
%        function out=get_slaved_DOF(obj)
%            out=obj.Slave;
% 
%        end
%        function obj=set_Association(obj,association)
%            arguments
%            obj  (1,1) Node
%            association (6,1) int64
%            end
%            obj.Association=association;
%        end
%        function out=get_Association(obj)
%            out=obj.Association;
%        end
%        function out=get_new_pos(obj,scale)
%            arguments
%            obj  (1,1) Node
%            scale (1,1) int64
%            end
%            prev_pos=obj.Pos;
%            Displace=obj.Disp_vec;
%            out=prev_pos+scale.*Displace;
%        end
%        function out=get_C(obj)
%            %Returns C matrix if slaved, I if unslaved
%            out=eye(6,6);
%            pos_node=obj.get_pos();
%            if(obj.is_slave==true)
%                 pos_master=obj.master_pos();
%                 yj=pos_node(2)-pos_master(2);
%                 xj=pos_node(1)-pos_master(1);
%                 out(1,6)=-1*yj;
%                 out(2,6)=xj;
%            end
%        end
%    end
% end
% 


%####Beam(Also Provided in separate file, since matlab doesn't support multiple class in one script)
% classdef Beam <handle
% %     """Bar Element
% %        
% %         -E,G,A,Ixx,Iyy,Izz,L  (Material and Geometrical properties)
% %         -K[12x12] (Local stiffness matrix)
% %         -N1, N2 :Nodes of element"""
% 
%         properties(SetAccess=private)
%      
%            N1   (1,1) Node      %Near Node
%            N2   (1,1) Node      %Far Node
%            E    (1,1) double    %young's modulus N/m^2
%            G    (1,1) double    %Shear Modulus N/m^2
%            A    (1,1) double    %Area of c/s m^2
%            L    (1,1) double    %Beam Length N/m^2
%            Ixx  (1,1) double    %J ,Torisional Constant N/m^2
%            Iyy  (1,1) double    %Iyy Moment of inertia y m^4
%            Izz  (1,1) double    %Izz Moment of inertia z m^4
%            T    (12,12) double    %Member transformation matrix
%         end
%         
%         methods
%             function obj =Beam(myN1,myN2,E,G,A,Ixx,Iyy,Izz,alpha)
%             arguments
%             myN1  (1,1) Node
%             myN2  (1,1) Node         %defaults of ISHB400
%             E    (1,1) double=2e11    %young's modulus N/m^2
%             G    (1,1) double=0.769e11    %Shear Modulus N/m^2
%             A    (1,1) double=0.00966    %Area of c/s m^2
%             Ixx  (1,1) double=0.000048118
%             Iyy  (1,1) double=0.000027283
%             Izz  (1,1) double=0.000020835
%             alpha (1,1) double=0      %Rotation of y-z axis in beam local coordinate,
%             end
%             
%             obj.L=abs(norm(myN2.get_pos()-myN1.get_pos()));
%             obj.E=E;
%             obj.A=A;
%             obj.G=G;
%             obj.Ixx=Ixx;
%             obj.Iyy=Iyy;
%             obj.Izz=Izz;
%             obj.N1=myN1;
%             obj.N2=myN2;
%             %Beam Orientation calculations
%             N2_pos=myN2.get_pos();
%             N1_pos=myN1.get_pos();
%             Cx=(N2_pos(1)-N1_pos(1))/obj.L;
%             Cy=(N2_pos(2)-N1_pos(2))/obj.L;
%             Cz=(N2_pos(3)-N1_pos(3))/obj.L;
%             Cxy=sqrt(Cx*Cx+Cy*Cy);
%             if(Cxy~=0.0)
%                 cos_beta=Cx/Cxy;
%                 sin_beta=Cy/Cxy;
%                 cos_gamma=Cxy;
%                 sin_gamma=Cz;
%             else
%                 cos_beta=1;
%                 sin_beta=0;
%                 cos_gamma=0;
%                 if(Cz>0)
%                 sin_gamma=1;
%                 else
%                 sin_gamma=-1;
%                 end     
%             end
%             cos_alpha=cos(alpha*pi/180);
%             sin_alpha=sin(alpha*pi/180);
%             R_beta=[cos_beta sin_beta 0;-sin_beta cos_beta 0; 0 0 1];
%             R_gamma=[cos_gamma 0 sin_gamma;0 1 0;-sin_gamma 0 cos_gamma]; 
%             R_alpha=[1 0 0; 0 cos_alpha sin_alpha; 0 -sin_alpha cos_alpha];
%             R=R_alpha*R_gamma*R_beta;
%             zero=zeros(3,3);
%             obj.T=[R zero zero zero;zero R zero zero;zero zero R zero;zero zero zero R];
%          
%             end
% 
%             function K_L=get_local_K(obj)
%            
%                 EA_L=obj.E*obj.A/obj.L;
%                 GJ_L=obj.G*obj.Ixx/obj.L;
%                 EIyy_L=obj.E*obj.Iyy/(obj.L);
%                 EIyy_L2=obj.E*obj.Iyy/(obj.L^2);
%                 EIyy_L3=obj.E*obj.Iyy/(obj.L^3);
%                 EIzz_L=obj.E*obj.Izz/(obj.L);
%                 EIzz_L2=obj.E*obj.Izz/(obj.L^2);
%                 EIzz_L3=obj.E*obj.Izz/(obj.L^3);
%                 
%                 K_L=[EA_L      0       0       0       0       0       -1*EA_L      0      0       0       0       0;
%                      0      12*EIzz_L3 0       0       0    6*EIzz_L2      0   -12*EIzz_L3 0       0       0    6*EIzz_L2;
%                      0         0    12*EIyy_L3 0    -6*EIyy_L2 0           0        0  -12*EIyy_L3 0    -6*EIyy_L2 0;
%                      0         0       0     GJ_L      0       0           0        0      0    -1*GJ_L    0       0;
%                      0         0  -6*EIyy_L2   0     4*EIyy_L  0           0        0   6*EIyy_L2  0     2*EIyy_L  0;
%                      0      6*EIzz_L2  0       0       0    4*EIzz_L       0   -6*EIzz_L2  0       0       0    2*EIzz_L;
%                   -1*EA_L      0       0       0       0       0          EA_L      0      0       0       0       0;
%                      0    -12*EIzz_L3  0       0       0   -6*EIzz_L2      0    12*EIzz_L3 0       0       0   -6*EIzz_L2;
%                      0         0   -12*EIyy_L3 0    6*EIyy_L2  0           0        0   12*EIyy_L3 0     6*EIyy_L2 0;
%                      0         0       0  -1*GJ_L      0       0           0        0      0       GJ_L    0       0;
%                      0         0  -6*EIyy_L2   0     2*EIyy_L  0           0        0   6*EIyy_L2  0     4*EIyy_L  0;
%                      0      6*EIzz_L2  0       0       0    2*EIzz_L       0   -6*EIzz_L2  0       0       0    4*EIzz_L;
%                   
%        
%                     ];
%                
%             end
%             
%             function K_g=get_global_K(obj)
%                 K_l=obj.get_local_K();
%                 K_g=obj.T'*K_l*obj.T;
%                
%             end
% 
%             function Disp=get_Disp(obj)
%                 Disp=[obj.N1.get_Disp(); obj.N2.get_Disp()];
%             end
%             function F=get_F(obj)
%                 %Get member force vector
%                 F=[obj.N1.get_Load(); obj.N2.get_Load()];
%             end
%             function Nodes=get_nodes(obj)
%                 %Get member force vector
%                 Nodes=[obj.N1; obj.N2];
%             end
%             function out=get_C(obj)
%                 %Get member force vector
%                 out=[obj.N1.get_C() zeros(6,6);zeros(6,6) obj.N2.get_C()];
%             end
%             function Association=get_association(obj)
%              
%                 %Get member force vector
%                 Association=[obj.N1.get_Association; obj.N2.get_Association];
%             end
%             function f=get_internal_force(obj)
%                 Local_Disp=obj.T*obj.get_Disp();
%                 %Get member internal force vector
%                 f=obj.get_local_K()*Local_Disp;
%             end
%            
%         end
% 
% end

%STEP1: Take input of geometry of structure: [Beam_length ,Coulumn Height]
%Executing app for taking layout info

prompt={'Beam Length(m)','Column Height(m)'};
title='Frame Layout Dimensions';
defaultans={'1','2'};
numlines=[1 50];
dimen=inputdlg(prompt,title,numlines,defaultans);
L=str2double(dimen{1});
H=str2double(dimen{2});
a=sqrt(3)/2;
Node_Coordinates=[-L 0.0 H;-0.5*L a*L H;0.5*L a*L H;L 0.0 H;
                   0.5*L -a*L H;-0.5*L -a*L H;-L 0.0 0.0;
                   -0.5*L a*L 0.0;0.5*L a*L 0.0;L 0.0 0.0;
                   0.5*L -a*L 0.0;-0.5*L -a*L 0.0;];

%L,H now contain Beam Length and Column Height, Node coordinate contains
%coordinates for 12 Nodes

%%STEP2: Tlake input for section geometry of beam and column
%Executing beamprop
movegui('west')
imshow('Local-axis-of-a-beam-x-is-the-longitudinal-direction-y-is-perpendicular-to-x-and-z-is.jpg');
prompt={'Area(m2)','Youngs Modulus(E)(N/m2)','Shear Modulus(G)(N/m2)', 'J or Ixx(m4)', 'Iyy(m4)' ,'Izz(m4)','Alpha(Local axis Rotation)(deg)'};
title='Beam Sectional Properties';
defaultans={'0.00987','1.99948e11','7.69e10','5.59e-7','0.000027283','0.00028086','0'};
numlines=[1 75];
dimen=inputdlg(prompt,title,numlines,defaultans);
Area=str2double(dimen{1});
E=str2double(dimen{2});
G=str2double(dimen{3});
Ixx=str2double(dimen{4});
Iyy=str2double(dimen{5});
Izz=str2double(dimen{6});
alpha=str2double(dimen{7});
Beam_props=[E; G; Area; Ixx; Iyy; Izz;alpha];

%Column
prompt={'Area(m2)','Youngs Modulus(E)(N/m2)','Shear Modulus(G)(N/m2)', 'J or Ixx(m4)', 'Iyy(m4)' ,'Izz(m4)','Alpha(Local axis Rotation)(deg)'};
title='Column Sectional Properties';
defaultans={'0.00987','1.99948e11','7.69e10','5.59e-7','0.000027283','0.00028086','0'};
numlines=[1 75];
dimen=inputdlg(prompt,title,numlines,defaultans);
Area=str2double(dimen{1});
E=str2double(dimen{2});
G=str2double(dimen{3});
Ixx=str2double(dimen{4});
Iyy=str2double(dimen{5});
Izz=str2double(dimen{6});
alpha=str2double(dimen{7});
Column_props=[E; G; Area; Ixx; Iyy; Izz;alpha];


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
        Members=[Members; Beam(N1,N2,Beam_props(1),Beam_props(2),Beam_props(3),Beam_props(4),Beam_props(5),Beam_props(6),Beam_props(7))];
    end
    %Then Columns
    for i=7:12
        N1=Nodes(Elements(i,1));
        N2=Nodes(Elements(i,2));
        Members=[Members; Beam(N1,N2,Column_props(1),Column_props(2),Column_props(3),Column_props(4),Column_props(5),Column_props(6),Column_props(7))];
    end
    

%%STEP 5: Applying Nodal Loads



More=1;
while(More~=0)
prompt={'Node[1-6]','Fx(N)','Fy(N)','Fz(N)','Mx(Nm)','My(Nm)','Mz(Nm)'};
title='Nodal Loads';
defaultans={'1','10000','0','0','0','0','0'};
numlines=[1 75];
dimen=inputdlg(prompt,title,numlines,defaultans);
Node_num=str2num(dimen{1});
Fx=str2double(dimen{2});
Fy=str2double(dimen{3});
Fz=str2double(dimen{4});
Mx=str2double(dimen{5});
My=str2double(dimen{6});
Mz=str2double(dimen{7});
Load=[Fx,Fy,Fz,Mx,My,Mz];
Nodes(Node_num).set_Load(Load);   
prompt={'Enter 1 to apply more load 0 to exit'};
title='More Loads?';
defaultans={'1'};
numlines=[1 75];
More_load=inputdlg(prompt,title,numlines,defaultans);
More=str2num(More_load{1});

end

%%STEP 6: Applying support conditions
for i=7:12
    Nodes(i).set_restrains([1;1;1;1;1;1]);   %Fully fixed all 6 DOF restrained
end

%Calculating Loads for equilibrium check later
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

%Plotting Structure    

fig= figure(2);
hold on
plot3(Node_Coordinates(1:6,1),Node_Coordinates(1:6,2),Node_Coordinates(1:6,3),Color='k',LineWidth=1);
plot3(Node_Coordinates([1,6],1),Node_Coordinates([1,6],2),Node_Coordinates([1,6],3),Color='k',LineWidth=1);
for i=1:6
plot3(Node_Coordinates([i,i+6],1),Node_Coordinates([i,i+6],2),Node_Coordinates([i,i+6],3),Color='k',LineWidth=1);
end
text(Node_Coordinates(1:12,1),Node_Coordinates(1:12,2),Node_Coordinates(1:12,3).*1.1,['  1';'  2';'  3';'  4';'  5';'  6';'  7';'  8';'  9';' 10';' 11'; ' 12' ],color='b');
for i=1:length(Members)
    This_mem_Nodes=Members(i).get_nodes();
    Pos=0.5*(This_mem_Nodes(1).get_pos()+This_mem_Nodes(2).get_pos());
    text(Pos(1),Pos(2)*1.05,Pos(3),num2str(i),color='r');
end

view(20,20);
hold off



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

Res="Output.txt generated as shown"
type output.txt


function [A_inv]=Inverse_matrix_using_Cholesky(A)
    %A = input("Enter A matrix (In this format'[a,b;c,d]')= ");
    %This is useful for symmetric matrix only
    if det(A)==0
        fprintf("Det is ZERO.\nSo inverse is not possible.")
    else
        N = length(A);
        L = zeros(N,N);
        U = zeros(N,N);
        IL = zeros(N,N);
        IU = zeros(N,N);
        L(1,1) = sqrt(A(1,1));
        U(1,1) = L(1,1);
        for a=2:N
            L(a,1) = A(a,1)/L(1,1);
            U(1,a) = L(a,1);
        end
        for i=2:N
            for j=i:N
               if i==j
                   L(j,i) = sqrt(A(j,i)-(L(j,1:i-1)*U(1:i-1,j)));
                   U(j,i) = L(j,i);
               else
                   L(j,i) = (A(j,i)-(L(j,1:i-1)*U(1:i-1,i)))/L(i,i);
                   U(i,j) = L(j,i);
               end
            end
        end
        for i=1:N
            for j=i:N
                if i==j
                    IL(j,i) = 1/(L(j,i));
                    IU(j,i) = IL(i,j);
                else
                    IL(j,i) = (-L(j,i:j-1)*IL(i:j-1,i))/(L(j,j));
                    IU(i,j) = IL(j,i);
                end
            end
        end
        A_inv = IU*IL;
    end
end