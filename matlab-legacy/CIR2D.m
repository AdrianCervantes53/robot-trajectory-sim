function CIR2D(pf1,pf2,pf3,pa1,pa2,plano,handles)

        P=evalin('base','P');
        PEF=[P(1) P(2) P(3)];
        assignin('base','PEF',PEF);

        po1=P(1);
        po2=P(2);
        po3=P(3);
        q4=P(4);
        q5=P(5);
        q6=P(6);
        
        syms D E F
        E1=po1^2+po2^2+D*po1+E*po2+F;
        E2=pa1^2+pa2^2+D*pa1+E*pa2+F;
        E3=pf1^2+pf2^2+D*pf1+E*pf2+F;
        
        resultado=solve(E1,E2,E3);
        D=double(resultado.D);
        E=double(resultado.E);
        F=double(resultado.F);
        
        radio=(1/2)*sqrt(D^2+E^2-4*F);
        xc=-D/2;
        yc=-E/2;
        
        theta1=rad2deg(atan2(po2-yc,po1-xc))
        theta2=rad2deg(atan2(pa2-yc,pa1-xc))
        theta3=rad2deg(atan2(pf2-yc,pf1-xc))
        
        if (theta3>theta2)&&(theta2>theta1) %1
            k=+1; 
            cambio=4;
        elseif (theta1>theta2)&&(theta2>theta3) %2
            k=-1;
            cambio=4;
        elseif (theta1>theta3)&&(theta3>theta2) %3
            k=+1;
            theta2=theta2+360;
            cambio=0;
        elseif (theta2>theta3)&&(theta3>theta1) %4
            k=-1;            
            theta2=theta2-360;
            cambio=1;
        elseif (theta2>theta1)&&(theta1>theta3) %5
            k=1;            
            theta3=theta3+360;
            cambio=2;
        elseif (theta3>theta1)&&(theta1>theta2) %6
            k=-1;            
            theta3=theta3-360;
            cambio=3;
        end

        l=2;
        
        for theta=theta1:k:theta2
           px=radio*cosd(theta)+xc;
           py=radio*sind(theta)+yc;           
           if plano==1
               L=fcinversa(px,py,P(3),P(4),P(5),P(6),handles);
           end
           if plano==2
               L=fcinversa(px,P(2),py,P(4),P(5),P(6),handles);
           end
           if plano==3
               L=fcinversa(P(1),px,py,P(4),P(5),P(6),handles);
           end
           fcdirecta(L(1),L(2),L(3),q4,q5,q6,handles);
           
           for j=1:l
                P=evalin('base','P');
                PEF=evalin('base','PEF');
                PEF(l,:)=[P(1) P(2) P(3)];
                if (j<l)&&(j>1)
                    plot3([PEF(j,1) PEF(j+1,1)],[PEF(j,2) PEF(j+1,2)],[PEF(j,3) PEF(j+1,3)],'y','linewidth',3);
                end
                assignin('base','PEF',PEF);
             end
            pause(0.01);
            l=l+1;
        end
        
        if cambio==0
            theta2=theta2-360;
        end
        if cambio==1
            theta2=theta2+360;
        end

        for theta=theta2:k:theta3
           px=radio*cosd(theta)+xc;
           py=radio*sind(theta)+yc;
           
           if plano==1
               L=fcinversa(px,py,P(3),P(4),P(5),P(6),handles);
           end
           if plano==2
               L=fcinversa(px,P(2),py,P(4),P(5),P(6),handles);
           end
           if plano==3
               L=fcinversa(P(2),px,py,P(4),P(5),P(6),handles);
           end
           fcdirecta(L(1),L(2),L(3),q4,q5,q6,handles);
           
           for j=1:l
                P=evalin('base','P');
                PEF=evalin('base','PEF');
                PEF(l,:)=[P(1) P(2) P(3)];
                if (j<l)&&(j>1)
                    plot3([PEF(j,1) PEF(j+1,1)],[PEF(j,2) PEF(j+1,2)],[PEF(j,3) PEF(j+1,3)],'y','linewidth',3);
                end
                assignin('base','PEF',PEF);
             end
            pause(0.05);
            l=l+1;
        end