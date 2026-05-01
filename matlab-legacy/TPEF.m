function TPEF(l)

for j=1:l
	P=evalin('base','P');
	PEF=evalin('base','PEF');
    angulos=evalin('base','angulos');
	PEF(l,:)=[P(1) P(2) P(3)];
    angulos(l+1,:)=[P(4) P(5) P(6)];
	if (j<l)&&(j>1)
	    plot3([PEF(j,1) PEF(j+1,1)],[PEF(j,2) PEF(j+1,2)],[PEF(j,3) PEF(j+1,3)],'y','linewidth',4);
    end
	assignin('base','PEF',PEF);
    assignin('base','angulos',angulos);
 end
pause(0.01);
l=l+1;
assignin('base','l',l);