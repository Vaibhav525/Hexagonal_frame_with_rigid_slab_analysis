classdef Node < handle
%    """Node
%         -id= node id
%         -Pos: [x,y] Coordinate Position
%         -Disp: [ux,uy],( =[None,None] if unknown)
%         -Restrained[i]==1 if restrained 0 if unrestrained
%         -Load Vector=[Px,Py]    (Default [0,0])
%         -Association :list ->Association[i]=Structure DOF corresponding
%         to ith local DOF, -1 default uninitialised
%         """
    %%Author :Vaibhav Bihani, 2018CE10169, IITD
   properties(SetAccess=private)
  
              id    (1,1) int64
              Pos   (3,1) double
              Disp_vec  (6,1) double 
              Restrain  (6,1) int64   %0 for unrestrained DOF, 1 for restrained DOF
              Load      (6,1) double 
              Association (6,1) int64 
              is_slave  (1,1) logical
              master_pos   (3,1) double  %Master Node position
              Slave     (6,1) int64  %0 for free, 1 for slaved
   end

   methods (Access = public)
       function obj = Node(myid,myPos,myDisp_vec,myResrain,myLoad, myAssociation,mySlave)
       arguments
       myid     (1,1) int64=0
       myPos    (3,1) double=[0.0;0.0;0.0]
       myDisp_vec   (6,1) double=[0.0;0.0;0.0;0.0;0.0;0.0]
       myResrain    (6,1) int64=[0;0;0;0;0;0]
       myLoad       (6,1) double=[0.0;0.0;0.0;0.0;0.0;0.0]
       myAssociation (6,1) int64=[-1;-1;-1;-1;-1;-1]
       mySlave     (6,1) int64 =[0;0;0;0;0;0]
       end
       obj.id=myid;
       obj.Pos=myPos;
       obj.Disp_vec=myDisp_vec;
       obj.Restrain=myResrain;
       obj.Load=myLoad;
       obj.Association=myAssociation;
       obj.Slave=mySlave;
       obj.master_pos=myPos;
       obj.is_slave=false;
       end


       function obj=set_pos(obj,Pos)
           arguments
           obj (1,1) Node
           Pos (3,1) double
           end
           obj.Pos=Pos;
       end
       function out=get_pos(obj)
            out=obj.Pos;
       end
       function set_master_pos(obj,Master_pos) 
           arguments
           obj  (1,1) Node
           Master_pos (3,1) double
           end
           obj.is_slave=true;
           obj.master_pos=Master_pos;
       end

       function out=get_master_pos(obj) 
           out=obj.master_pos;
       end
       function out=get_ID(obj)
           out=obj.id;
       end
       function obj=set_Disp(obj,Disp)
           arguments
           obj  (1,1) Node
           Disp (6,1) double
           end
           obj.Disp_vec=Disp;
       end
       function out=get_Disp(obj)
           out=obj.Disp_vec;
       end
       function obj=set_Load(obj,Load)
           obj.Load=Load;
       end
       function out=get_Load(obj)
           out=obj.Load;
       end
       function obj=set_restrains(obj,Restrains)
           arguments
           obj (1,1) Node
           Restrains (6,1) int64 
           end
           obj.Restrain=Restrains;
       end
       function out=get_restrain(obj)
           out=obj.Restrain;
       end
           function obj=set_slaved_DOF(obj,Slaved_DOF)
           arguments
           obj (1,1) Node
           Slaved_DOF (6,1) int64 
           end
           obj.Slave=Slaved_DOF;
       end
       function out=get_slaved_DOF(obj)
           out=obj.Slave;

       end
       function obj=set_Association(obj,association)
           arguments
           obj  (1,1) Node
           association (6,1) int64
           end
           obj.Association=association;
       end
       function out=get_Association(obj)
           out=obj.Association;
       end
       function out=get_new_pos(obj,scale)
           arguments
           obj  (1,1) Node
           scale (1,1) int64
           end
           prev_pos=obj.Pos;
           Displace=obj.Disp_vec;
           out=prev_pos+scale.*Displace;
       end
       function out=get_C(obj)
           %Returns C matrix if slaved, I if unslaved
           out=eye(6,6);
           pos_node=obj.get_pos();
           if(obj.is_slave==true)
                pos_master=obj.master_pos();
                yj=pos_node(2)-pos_master(2);
                xj=pos_node(1)-pos_master(1);
                out(1,6)=-1*yj;
                out(2,6)=xj;
           end
       end
   end
end
