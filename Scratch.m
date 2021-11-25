

More=1;
while(More~=0)
prompt={'Node[1-6]','Fx(N)','Fy(N)','Fz(N)','Mx(Nm)','My(Nm)','Mz(Nm)'};
title='Nodal Loads';
defaultans={'1','10000','0','0','0','0','0'};
numlines=[1 75];
dimen=inputdlg(prompt,title,numlines,defaultans);
Node=str2double(dimen{1});
Fx=str2double(dimen{2});
Fy=str2double(dimen{3});
Fz=str2double(dimen{4});
Mx=str2double(dimen{5});
My=str2double(dimen{6});
Mz=str2double(dimen{7});
Load=[Fx,Fy,Fz,Mx,My,Mz];
prompt={'Enter 1 to apply more load 0 to exit'};
title='More Loads?';
defaultans={'1'};
numlines=[1 75];
More_load=inputdlg(prompt,title,numlines,defaultans);
More=str2num(More_load{1});
end
% fig= figure(1);
% hold on
% plot3(Node_coordinates(1:6,1),Node_coordinates(1:6,2),Node_coordinates(1:6,3),Color='k',LineWidth=1);
% plot3(Node_coordinates([1,6],1),Node_coordinates([1,6],2),Node_coordinates([1,6],3),Color='k',LineWidth=1);
% for i=1:6
% plot3(Node_coordinates([i,i+6],1),Node_coordinates([i,i+6],2),Node_coordinates([i,i+6],3),Color='k',LineWidth=1);
% end
% view(45,45);
% hold off

