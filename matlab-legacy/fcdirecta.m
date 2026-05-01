function fcdirecta(q1,q2,q3,q4,q5,q6,handles)
cla

L1=5;  L2=5; L3=5;

% set(handles.text2,'string',[num2str(q1),'°']);
% % set(handles.edit1,'string',q1);
% 
% set(handles.text6,'string',[num2str(q2),'°']);
% % set(handles.edit2,'string',q2);
% 
% set(handles.text7,'string',[num2str(q3),'°']);
% % set(handles.edit3,'string',q3);


ardno=evalin('base','ardno');
if ardno==1
a=evalin('base','a');
qa1=round((q1));
if qa1>180
    qa1=180;
elseif q1<0
    qa1=0;
end
qa2=round(abs(q2-180));
if qa2>180
    qa2=180;
elseif qa2<0
    qa2=0;
end
qa3=round(q3+90);
if qa3>180
    qa3=180;
elseif qa3<0
    qa3=0;
end
a.servoWrite(3,qa1)
a.servoWrite(5,qa2)
a.servoWrite(6,qa3)
a.servoWrite(9,round(q5+90))
a.servoWrite(10,round(q4+90))
% a.servoWrite(11,round(q6+80))
end


q1=deg2rad(q1);
q2=deg2rad(q2);
q3=deg2rad(q3);
q4=deg2rad(q4);
q5=deg2rad(q5);
q6=deg2rad(q6);

A1=fDH(q1,L1,0,pi/2);
A2=fDH(q2,0,L2,0);
A3=fDH(q3,0,L3,0);
A4=fDH(0,0,0,q4);
A5=fDH(q5,0,0,-pi/2);
A6=fDH(q6,0,0,0);
% A6=[cos(q6) 0 sin(q6) 0; 0 1 0 0; -sin(q6) 0 cos(q6) 0; 0 0 0 1];
T=A1*A2*A3*A4*A5*A6;

x1=A1(1,4);
y1=A1(2,4);
z1=A1(3,4);

A21=A1*A2;
x2=A21(1,4);
y2=A21(2,4);
z2=A21(3,4);

px=T(1,4);
py=T(2,4);
pz=T(3,4);

R=T;

alpha=atan2(R(3,2),R(2,2));
beta=atan2(-R(1,2),sqrt((R(2,2))^2+(R(3,2))^2));
gama=atan2(R(1,3),R(1,1));

graficar=evalin('base','graficar');

if graficar==1
plot3(handles.axes1,[0 x1], [0 y1], [0 z1], '-ro','linewidth',4,'MarkerEdgeColor','black'); hold on;
plot3(handles.axes1,[x1 x2],[y1 y2], [z1 z2], '-bo','linewidth',4,'MarkerEdgeColor','black'); hold on;
plot3(handles.axes1,[x2 px],[y2 py], [z2 pz], '-go','linewidth',4,'MarkerEdgeColor','black'); hold on;

espacio=evalin('base','espacio');
if espacio==1
set(handles.text8,'String',['Px: ',num2str(red((px))),', Py: ',num2str(red((py))),', Pz: ',num2str(red((pz)))]);
set(handles.text2,'string',[num2str(round(rad2deg(q1))),'°']);
set(handles.edit1,'string',round(rad2deg(q1)));

set(handles.text6,'string',[num2str(round(rad2deg(q2))),'°']);
set(handles.edit2,'string',round(rad2deg(q2)));

set(handles.text7,'string',[num2str(round(rad2deg(q3))),'°']);
set(handles.edit3,'string',round(rad2deg(q3)));

set(handles.text15,'string',[num2str(round(rad2deg(q4)))]);
set(handles.edit4,'string',round(rad2deg(q4)));

set(handles.text16,'string',[num2str(round(rad2deg(q5)))]);
set(handles.edit5,'string',round(rad2deg(q5)));

set(handles.text17,'string',[num2str(round(rad2deg(q6)))]);
set(handles.edit6,'string',round(rad2deg(q6)));

else
    set(handles.text2,'string',red(px));
    set(handles.edit1,'string',red(px));
    
    set(handles.text6,'string',red(py));
    set(handles.edit2,'string',red(py));

    set(handles.text7,'string',red(pz));
    set(handles.edit3,'string',red(pz));
    
%     set(handles.text15,'string',[num2str(red(rad2deg(alpha)))]);
%     set(handles.edit4,'string',red(rad2deg(alpha)));
% 
%     set(handles.text16,'string',[num2str(red(rad2deg(beta)))]);
%     set(handles.edit5,'string',red(rad2deg(beta)));
% 
%     set(handles.text17,'string',[num2str(red(rad2deg(gama)))]);
%     set(handles.edit6,'string',red(rad2deg(gama)));
    
    set(handles.text8,'String',['q1: ',num2str(round(q1)),', q2: ',num2str(round(q2)),', q3: ',num2str(round(q3))]);
end

xlim(handles.axes1,[-15 15])
ylim(handles.axes1,[-15 15])
zlim(handles.axes1,[-10 20])

xlabel(handles.axes1,'Eje X');
ylabel(handles.axes1,'Eje Y');
zlabel(handles.axes1,'Eje Z');

axes(handles.axes1);
rotate3d ON
grid ON

% % Sistema 0
% quiver3(0,0,0,2,0,0,'color','r','Linewidth',2);text(2,0,0,'X{_0}');
% quiver3(0,0,0,0,2,0,'color','g','Linewidth',2);text(0,2,0,'X{_0}');
% quiver3(0,0,0,0,0,2,'color','b','Linewidth',2);text(0,0,2,'X{_0}');
% 
% % Sistema 1
% quiver3(x1,y1,z1,2*A1(1,1),2*A1(2,1),2*A1(3,1),'r','Linewidth',2);text(2*A1(1,1),2*A1(2,1),2*A1(3,1)+z1,'X{_1}');
% quiver3(x1,y1,z1,2*A1(1,2),2*A1(2,2),2*A1(3,2),'g','Linewidth',2);text(2*A1(1,2),2*A1(2,2),2*A1(3,2)+z1,'X{_1}');
% quiver3(x1,y1,z1,2*A1(1,3),2*A1(2,3),2*A1(3,3),'b','Linewidth',2);text(2*A1(1,3),2*A1(2,3),2*A1(3,3)+z1,'X{_1}');
% 
% % Sistema 2
% quiver3(x2,y2,z2,2*A21(1,1),2*A21(2,1),2*A21(3,1),'r','Linewidth',2);text(2*A21(1,1)+x2,2*A21(2,1)+y2,2*A21(3,1)+z2,'X{_2}');
% quiver3(x2,y2,z2,2*A21(1,2),2*A21(2,2),2*A21(3,2),'g','Linewidth',2);text(2*A21(1,2)+x2,2*A21(2,2)+y2,2*A21(3,2)+z2,'X{_2}');
% quiver3(x2,y2,z2,2*A21(1,3),2*A21(2,3),2*A21(3,3),'b','Linewidth',2);text(2*A21(1,3)+x2,2*A21(2,3)+y2,2*A21(3,3)+z2,'X{_2}');
% 
% % Sistema 3
% quiver3(px,py,pz,2*T(1,1),2*T(2,1),2*T(3,1),'r','Linewidth',2);text(2*T(1,1)+px,2*T(2,1)+py,2*T(3,1)+pz,'X{_3}');
% quiver3(px,py,pz,2*T(1,2),2*T(2,2),2*T(3,2),'g','Linewidth',2);text(2*T(1,2)+px,2*T(2,2)+py,2*T(3,2)+pz,'X{_3}');
% quiver3(px,py,pz,2*T(1,3),2*T(2,3),2*T(3,3),'b','Linewidth',2);text(2*T(1,3)+px,2*T(2,3)+py,2*T(3,3)+pz,'X{_3}');


r1=T*[2; 0; 0; 1];
r2=T*[2; -1; 0; 1];
r3=T*[2; 1; 0; 1];
r4=T*[4; -1; 0; 1];
r5=T*[4; 1; 0; 1];

plot3(handles.axes1,[px r1(1)], [py r1(2)], [pz r1(3)],'-k','linewidth',2); hold on;
plot3(handles.axes1,[r2(1) r3(1)], [r2(2) r3(2)], [r2(3) r3(3)],'-k','linewidth',2); hold on;
plot3(handles.axes1,[r2(1) r4(1)], [r2(2) r4(2)], [r2(3) r4(3)],'-k','linewidth',2); hold on;
plot3(handles.axes1,[r3(1) r5(1)], [r3(2) r5(2)], [r3(3) r5(3)],'-k','linewidth',2); hold on;
pause(0.001);
end

Q=[red(rad2deg(q1)),red(rad2deg(q2)),red(rad2deg(q3)),red(rad2deg(q4)),red(rad2deg(q5)),red(rad2deg(q6))];
% Q=[rad2deg(q1),rad2deg(q2),rad2deg(q3),rad2deg(q4),rad2deg(q5),rad2deg(q6)];
P=[red(px),red(py),red(pz),alpha,beta,gama];
assignin('base','Q',Q);
assignin('base','P',P);
assignin('base','A1',A1);
assignin('base','A2',A2);
assignin('base','A3',A3);
