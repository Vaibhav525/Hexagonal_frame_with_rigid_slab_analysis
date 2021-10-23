classdef Node
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
              Disp_vec  (6,1) double =[0.0;0.0;0.0;0.0;0.0;0.0]
              Restrain  (6,1) int64  =[0;0;0;0;0;0] %0 for unrestrained DOF, 1 for restrained DOF
              Load      (6,1) double =[0.0;0.0;0.0;0.0;0.0;0.0]
              Association (6,1) int64 =[-1;-1;-1;-1;-1;-1]

   end

   methods (Access = public)
       function obj = Node(myid,myPos,myDisp_vec,myResrain,myLoad, myAssociation)
       arguments
       myid     (1,1) int64=0
       myPos    (3,1) double=[0.0;0.0;0.0]
       myDisp_vec   (6,1) double=[0.0;0.0;0.0;0.0;0.0;0.0]
       myResrain    (6,1) int64=[0;0;0;0;0;0]
       myLoad       (6,1) double=[0.0;0.0;0.0;0.0;0.0;0.0]
       myAssociation (6,1) int64=[-1;-1;-1;-1;-1;-1]
       end
       obj.id=myid;
       obj.Pos=myPos;
       obj.Disp_vec=myDisp_vec;
       obj.Restrain=myResrain;
       obj.Load=myLoad;
       obj.Association=myAssociation;
      
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
       function obj=set_Association(obj,association)
           arguments
           obj  (1,1) Node
           association (6,1) int64
           end
           obj.Disp_vec=association;
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
       
   end
end
