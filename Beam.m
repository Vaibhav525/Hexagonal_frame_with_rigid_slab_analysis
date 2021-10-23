classdef Beam
%     """Bar Element
%        
%         -E,G,A,Ixx,Iyy,Izz,L  (Material and Geometrical properties)
%         -K[12x12] (Local stiffness matrix)
%         -R [3x3] orientation matrix [x';y',z'] direction cosines from
%         global [x y z] #Need consideration for principal axes 
%         -N1, N2 :Nodes of element"""

        properties(SetAccess=private)
     
           N1   (1,1) Node      %Near Node
           N2   (1,1) Node      %Far Node
           E    (1,1) double    %young's modulus N/m^2
           G    (1,1) double    %Shear Modulus N/m^2
           A    (1,1) double    %Area of c/s m^2
           L    (1,1) double    %Beam Length N/m^2
           R    (3,3) double    %Nodal Transformation Matrix
           Ixx  (1,1) double    %J ,Torisional Constant N/m^2
           Iyy  (1,1) double    %Iyy Moment of inertia y m^4
           Izz  (1,1) double    %Izz Moment of inertia z m^4
           
           
        end
        
        methods
            function obj =Beam(myN1,myN2,E,G,A,L,R,Ixx,Iyy,Izz)
            arguments
            myN1  (1,1) Node
            myN2  (1,1) Node
            %defaults of ISHB400
            
            E    (1,1) double=2e11    %young's modulus N/m^2
            G    (1,1) double=0.769e11    %Shear Modulus N/m^2
            A    (1,1) double=0.00966    %Area of c/s m^2
            L     (1,1) double=1.0
            R    (3,3) double=[1 0 0;0 1 0; 0 0 1]
            Ixx  (1,1) double=0.000048118
            Iyy  (1,1) double=0.000027283
            Izz  (1,1) double=0.000020835
            
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
            Dir_cosine_x=(myN2.get_pos()-myN1.get_pos())'/obj.L;
            Dir_cosine_y=[0 1 0];   %Assumed for now
            Dir_cosine_z=[0 0 1];   %Assumed for now
            obj.R=[Dir_cosine_x;Dir_cosine_y;Dir_cosine_z];
         
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


        end

end