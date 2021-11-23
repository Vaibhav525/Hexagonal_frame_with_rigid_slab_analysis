classdef Beam <handle
%     """Bar Element
%        
%         -E,G,A,Ixx,Iyy,Izz,L  (Material and Geometrical properties)
%         -K[12x12] (Local stiffness matrix)
%         -N1, N2 :Nodes of element"""

        properties(SetAccess=private)
     
           N1   (1,1) Node      %Near Node
           N2   (1,1) Node      %Far Node
           E    (1,1) double    %young's modulus N/m^2
           G    (1,1) double    %Shear Modulus N/m^2
           A    (1,1) double    %Area of c/s m^2
           L    (1,1) double    %Beam Length N/m^2
           Ixx  (1,1) double    %J ,Torisional Constant N/m^2
           Iyy  (1,1) double    %Iyy Moment of inertia y m^4
           Izz  (1,1) double    %Izz Moment of inertia z m^4
           T    (12,12) double    %Member transformation matrix
        end
        
        methods
            function obj =Beam(myN1,myN2,E,G,A,Ixx,Iyy,Izz,alpha)
            arguments
            myN1  (1,1) Node
            myN2  (1,1) Node
            %defaults of ISHB400
            E    (1,1) double=2e11    %young's modulus N/m^2
            G    (1,1) double=0.769e11    %Shear Modulus N/m^2
            A    (1,1) double=0.00966    %Area of c/s m^2
            Ixx  (1,1) double=0.000048118
            Iyy  (1,1) double=0.000027283
            Izz  (1,1) double=0.000020835
            alpha (1,1) double=0.0      %Rotation of y-z axis in beam local coordinate,
            end
            
            obj.L=norm(myN2.get_pos()-myN1.get_pos());
            obj.E=E;
            obj.A=A;
            obj.G=G;
            obj.Ixx=Ixx;
            obj.Iyy=Iyy;
            obj.Izz=Izz;
            obj.N1=myN1;
            obj.N2=myN2;
            %Beam Orientation calculations
            N2_pos=myN2.get_pos();
            N1_pos=myN1.get_pos();
            Cx=(N2_pos(1)-N1_pos(1))/obj.L;
            Cy=(N2_pos(2)-N1_pos(2))/obj.L;
            Cz=(N2_pos(3)-N1_pos(3))/obj.L;
            Cxy=sqrt(Cx*Cx+Cy*Cy);
            if(Cxy~=0.0)
                cos_beta=Cx/Cxy;
                sin_beta=Cy/Cxy;
                cos_gamma=Cxy;
                sin_gamma=Cz;
            else
                cos_beta=1;
                sin_beta=0;
                cos_gamma=0;
                if(Cz>0)
                sin_gamma=1;
                else
                sin_gamma=-1;
                end     
            end
            cos_alpha=cos(alpha*pi/180);
            sin_alpha=sin(alpha*pi/180);
            R_beta=[cos_beta sin_beta 0;-sin_beta cos_beta 0; 0 0 1];
            R_gamma=[cos_gamma 0 sin_gamma;0 1 0;-sin_gamma 0 cos_gamma]; 
            R_alpha=[1 0 0; 0 cos_alpha sin_alpha; 0 -sin_alpha cos_alpha];
            R=R_alpha*R_gamma*R_beta;
            zero=zeros(3,3);
            obj.T=[R zero zero zero;zero R zero zero;zero zero R zero;zero zero zero R];
         
            end

            function K_L=get_local_K(obj)
           
                EA_L=obj.E*obj.A/obj.L;
                GJ_L=obj.G*obj.Ixx/obj.L;
                EIyy_L=obj.E*obj.Iyy/(obj.L);
                EIyy_L2=obj.E*obj.Iyy/(obj.L^2);
                EIyy_L3=obj.E*obj.Iyy/(obj.L^3);
                EIzz_L=obj.E*obj.Izz/(obj.L);
                EIzz_L2=obj.E*obj.Izz/(obj.L^2);
                EIzz_L3=obj.E*obj.Izz/(obj.L^3);
                
                K_L=[EA_L      0       0       0       0       0       -1*EA_L      0      0       0       0       0;
                     0      12*EIzz_L3 0       0       0    6*EIzz_L2      0   -12*EIzz_L3 0       0       0    6*EIzz_L2;
                     0         0    12*EIyy_L3 0    -6*EIyy_L2 0           0        0  -12*EIyy_L3 0    -6*EIyy_L2 0;
                     0         0       0     GJ_L      0       0           0        0      0    -1*GJ_L    0       0;
                     0         0  -6*EIyy_L2   0     4*EIyy_L  0           0        0   6*EIyy_L2  0     2*EIyy_L  0;
                     0      6*EIzz_L2  0       0       0    4*EIzz_L       0   -6*EIzz_L2  0       0       0    2*EIzz_L;
                  -1*EA_L      0       0       0       0       0          EA_L      0      0       0       0       0;
                     0    -12*EIzz_L3  0       0       0   -6*EIzz_L2      0    12*EIzz_L3 0       0       0   -6*EIzz_L2;
                     0         0   -12*EIyy_L3 0    6*EIyy_L2  0           0        0   12*EIyy_L3 0     6*EIyy_L2 0;
                     0         0       0  -1*GJ_L      0       0           0        0      0       GJ_L    0       0;
                     0         0  -6*EIyy_L2   0     2*EIyy_L  0           0        0   6*EIyy_L2  0     4*EIyy_L  0;
                     0      6*EIzz_L2  0       0       0    2*EIzz_L       0   -6*EIzz_L2  0       0       0    4*EIzz_L;
                  
       
                    ];
            end
            
            function K_g=get_global_K(obj)
                K_l=obj.get_local_K();
                K_g=obj.T'*(K_l*obj.T);
            end

            function Disp=get_Disp(obj)
                Disp=[obj.N1.get_Disp(); obj.N2.get_Disp()];
            end
            function F=get_F(obj)
                %Get member force vector
                F=[obj.N1.get_Load(); obj.N2.get_Load()];
            end
            function Nodes=get_nodes(obj)
                %Get member force vector
                Nodes=[obj.N1; obj.N2];
            end
            function out=get_C(obj)
                %Get member force vector
                out=[obj.N1.get_C() zeros(6,6);zeros(6,6) obj.N2.get_C()];
            end
            function Association=get_association(obj)
             
                %Get member force vector
                Association=[obj.N1.get_Association; obj.N2.get_Association];
            end
            function f=get_internal_force(obj)
                Local_Disp=obj.T'*obj.get_Disp();
                %Get member internal force vector
                f=obj.get_local_K()*Local_Disp;
            end
           
        end

end