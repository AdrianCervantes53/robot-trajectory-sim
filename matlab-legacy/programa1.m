function varargout = programa1(varargin)
% PROGRAMA1 MATLAB code for programa1.fig
%      PROGRAMA1, by itself, creates a new PROGRAMA1 or raises the existing
%      singleton*.
%
%      H = PROGRAMA1 returns the handle to a new PROGRAMA1 or the handle to
%      the existing singleton*.
%
%      PROGRAMA1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PROGRAMA1.M with the given input arguments.
%
%      PROGRAMA1('Property','Value',...) creates a new PROGRAMA1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before programa1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to programa1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDEs Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help programa1

% Last Modified by GUIDE v2.5 01-Dec-2021 18:40:50

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @programa1_OpeningFcn, ...
                   'gui_OutputFcn',  @programa1_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before programa1 is made visible.
function programa1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to programa1 (see VARARGIN)

% Choose default command line output for programa1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes programa1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);
Q=[90,90,0,0,0,0];
P=[0,0,15,0,0,0];
assignin('base','Q',Q);
assignin('base','P',P);
cont=0;
assignin('base','cont',cont);
T=2;
assignin('base','T',T);
angulos=[P(4) P(5) P(6)];
assignin('base','angulos',angulos);
PEF=[P(1) P(2) P(3)];
assignin('base','PEF',PEF);
graficar=1;
assignin('base','graficar',graficar);
l=2;
assignin('base','l',l);
Vel=50;
assignin('base','Vel',Vel)
ardno=0;
assignin('base','ardno',ardno)
g=0;
assignin('base','g',g)
plano=0;
assignin('base','plano',plano)
paux=[0 0];
assignin('base','paux',paux)
PTR=[0 0 0 0 0 0 0 0 0 0];
assignin('base','PTR',PTR)
trayectoria=0;
assignin('base','trayectoria',trayectoria)
espacio=1;
assignin('base','espacio',espacio)



% --- Outputs from this function are returned to the command line.
function varargout = programa1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla
op=get(handles.radiobutton2,'value');
modo=get(handles.radiobutton10,'value');

P=evalin('base','P');
Q=evalin('base','Q');
angulos=[deg2rad(Q(4)),deg2rad(Q(5)),deg2rad(Q(6))];
assignin('base','angulos',angulos)
q4=str2double(get(handles.edit4,'string'));
q5=str2double(get(handles.edit5,'string'));
q6=str2double(get(handles.edit6,'string'));
Q(4)=q4;
Q(5)=q5;
Q(6)=q6;

% assignin('base','P',P)

if modo==1
    if op==1
        q1=str2double(get(handles.edit1,'string'));
        q2=str2double(get(handles.edit2,'string'));
        q3=str2double(get(handles.edit3,'string'));

        fcdirecta(q1,q2,q3,q4,q5,q6,handles);
        
        P=evalin('base','P');
        set(handles.text2,'value',q1);
        set(handles.text6,'value',q2);
        set(handles.text7,'value',q3);
        set(handles.text15,'value',q4);
        set(handles.text16,'value',q5);
        set(handles.text17,'value',q6);
        set(handles.text8,'String',['Px: ',num2str(red(P(1))),', Py: ',num2str(red(P(2))),', Pz: ',num2str(red(P(3)))]);

    else 
        px=str2double(get(handles.edit1,'string'));
        py=str2double(get(handles.edit2,'string'));
        pz=str2double(get(handles.edit3,'string'));
        A=str2double(get(handles.edit4,'string'));
        B=str2double(get(handles.edit5,'string'));
        C=str2double(get(handles.edit6,'string'));

%         set(handles.text2,'value',px);
%         set(handles.text6,'value',py);
%         set(handles.text7,'value',pz);
%         set(handles.text15,'value',A);
%         set(handles.text16,'value',B);
%         set(handles.text17,'value',C);

        P=evalin('base','P')
        F=fcinversa(px,py,pz,deg2rad(A),deg2rad(B),deg2rad(C),handles);
        
        q1=F(1);
        q2=F(2);
        q3=F(3);
        fcdirecta(F(1),F(2),F(3),rad2deg(F(4)),rad2deg(F(5)),rad2deg(F(6)),handles);

        set(handles.text15,'value',A);
        set(handles.text16,'value',B);
        set(handles.text17,'value',C);
        
        set(handles.text2,'string',num2str(red(px)));
        set(handles.edit1,'string',red(px));

        set(handles.text6,'string',num2str(red(py)));
        set(handles.edit2,'string',red(py));

        set(handles.text7,'string',num2str(red(pz)));
        set(handles.edit3,'string',red(pz));
        
        set(handles.text8,'String',['q1: ',num2str(round(q1)),', q2: ',num2str(round(q2)),', q3: ',num2str(round(q3))]);

    end
else
    mPTP=get(handles.radiobutton6,'value');
    linea=get(handles.radiobutton7,'value');
    cir2D=get(handles.radiobutton8,'value');
    cir3D=get(handles.radiobutton9,'value');
    
    if mPTP==1       %========================MOVIMIENTO PTP=======================================================================
        if op==1    %ESPACIO ARTICULAR
            pf1=str2double(get(handles.edit1,'string'));
            pf2=str2double(get(handles.edit2,'string'));
            pf3=str2double(get(handles.edit3,'string'))-90;
            
        else    %ESPACIO CARTESIANO
            pf1=str2double(get(handles.edit1,'string'));
            pf2=str2double(get(handles.edit2,'string'));
            pf3=str2double(get(handles.edit3,'string'));

            Qf=fcinversa(pf1,pf2,pf3,P(4),P(5),P(6),handles);
            pf1=Qf(1);
            pf2=Qf(2);
            pf3=Qf(3);
        end
        PTP(pf1,pf2,pf3,handles);
    end
    
    if linea==1     %========================MOVIMIENTO LINEA=======================================================================
        
        if op==1    %ESPACIO ARTICULAR
            graficar=0;
            assignin('base','graficar',graficar);
            q1=str2double(get(handles.edit1,'string'));
            q2=str2double(get(handles.edit2,'string'));
            q3=str2double(get(handles.edit3,'string'))-90;
            q4=str2double(get(handles.edit4,'string'));
            q5=str2double(get(handles.edit5,'string'));
            q6=str2double(get(handles.edit6,'string'));
            
            Ptemporal=evalin('base','P');
            fcdirecta(q1,q2,q3,q4,q5,q6,handles);
            
            P=evalin('base','P');
            pf1=red(P(1));
            pf2=red(P(2));
            pf3=red(P(3));
            assignin('base','P',Ptemporal)
            graficar=1;
            assignin('base','graficar',graficar);
            
        else    %ESPACIO CARTESIANO

            pf1=str2double(get(handles.edit1,'string'));
            pf2=str2double(get(handles.edit2,'string'));
            pf3=str2double(get(handles.edit3,'string'));
        end
        LIN(pf1,pf2,pf3,handles)
    end
    
    if cir2D==1     %========================MOVIMIENTO CIRCULO2D=============================================================
        
        plano=get(handles.popupmenu1,'value')
        assignin('base','plano',plano)
        if op==1    %ESPACIO ARTICULAR
            graficar=0;
            assignin('base','graficar',graficar);
            
            q1=str2double(get(handles.edit1,'string'));
            q2=str2double(get(handles.edit2,'string'));
            q3=str2double(get(handles.edit3,'string'))-90;
            
            if plano==1
                qa1=str2double(get(handles.edit7,'string'));
                qa2=str2double(get(handles.edit8,'string'));
                qa3=q3;
           end
           if plano==2
                qa1=str2double(get(handles.edit7,'string'));
                qa2=str2double(get(handles.edit8,'string'));
                qa3=q2;
           end
           if plano==3
                qa1=str2double(get(handles.edit7,'string'));
                qa2=str2double(get(handles.edit8,'string'));
                qa3=q1;
           end
            
            Ptemporal=evalin('base','P');
            
            fcdirecta(qa1,qa2,qa3,q4,q5,q6,handles);
            P=evalin('base','P');
            pa1=P(1);
            pa2=P(2);
            pa3=P(3);
            
            fcdirecta(q1,q2,q3,q4,q5,q6,handles);
            P=evalin('base','P');
            pf1=P(1);
            pf2=P(2);
            pf3=P(3);
            
            assignin('base','P',Ptemporal)
            
            graficar=1;
            assignin('base','graficar',graficar);
            
        else    %ESPACIO CARTESIANO
            if plano==1
                pf1=str2double(get(handles.edit1,'string'));
                pf2=str2double(get(handles.edit2,'string'));
                pf3=str2double(get(handles.edit3,'string'));
           end
           if plano==2
                pf1=str2double(get(handles.edit1,'string'));
                pf2=str2double(get(handles.edit3,'string'));
                pf3=str2double(get(handles.edit2,'string'));
           end
           if plano==3
                pf1=str2double(get(handles.edit2,'string'));
                pf2=str2double(get(handles.edit3,'string'));
                pf3=str2double(get(handles.edit1,'string'));
           end
        
            pa1=str2double(get(handles.edit7,'string'));
            pa2=str2double(get(handles.edit8,'string'));
        end
        
        paux=[pa1 pa2];
        assignin('base','paux',paux)
        CIR2D(pf1,pf2,pf3,pa1,pa2,plano,handles)
        
    end
    if cir3D==1     %========================MOVIMIENTO CIRCULO3D=============================================================
        
    end
end
        


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

%==================================Segundo Slider=========================%
cla
op=get(handles.radiobutton2,'value');
if op==1
    q1=get(handles.slider1,'value');
    q2=get(handles.slider4,'value');
    q3=get(handles.slider5,'value');
    q4=get(handles.slider8,'value');
    q5=get(handles.slider9,'value');
    q6=get(handles.slider10,'value');
    fcdirecta(q1,q2,q3,q4,q5,q6,handles);
    
    set(handles.text2,'string',num2str(round(q1)));
    set(handles.edit1,'string',round(q1));

    set(handles.text6,'string',num2str(round(q2)));
    set(handles.edit2,'string',round(q2));

    set(handles.text7,'string',num2str(round(q3)));
    set(handles.edit3,'string',round(q3));
    
else 
    px=get(handles.slider1,'value');
    py=get(handles.slider4,'value');
    pz=get(handles.slider5,'value');
    q4=get(handles.slider8,'value');
    q5=get(handles.slider9,'value');
    q6=get(handles.slider10,'value');
    
    P=evalin('base','P')
    F=fcinversa(px,py,pz,P(4),P(5),P(6),handles);
    q1=F(1);
    q2=F(2);
    q3=F(3);
    fcdirecta(q1,q2,q3,q4,q5,q6,handles);
    
    set(handles.text2,'string',num2str(red(px)));
    set(handles.edit1,'string',red(px));

    set(handles.text6,'string',num2str(red(py)));
    set(handles.edit2,'string',red(py));

    set(handles.text7,'string',num2str(red(pz)));
    set(handles.edit3,'string',red(pz));
    
    set(handles.text8,'String',['q1: ',num2str(round(q1)),', q2: ',num2str(round(q2)),', q3: ',num2str(round(q3))]);

end


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

set(hObject,'min',0);
set(hObject,'max',180);
set(hObject,'value',90);
set(hObject,'SliderStep',[(1/180) (1/180)]);

% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

%==================================Tercer Slider==========================%
cla
op=get(handles.radiobutton2,'value');
if op==1
    q1=get(handles.slider1,'value');
    q2=get(handles.slider4,'value');
    q3=get(handles.slider5,'value');
    q4=get(handles.slider8,'value');
    q5=get(handles.slider9,'value');
    q6=get(handles.slider10,'value');
    fcdirecta(q1,q2,q3,q4,q5,q6,handles);
    
    set(handles.text2,'string',num2str(round(q1)));
    set(handles.edit1,'string',round(q1));

    set(handles.text6,'string',num2str(round(q2)));
    set(handles.edit2,'string',round(q2));

    set(handles.text7,'string',num2str(round(q3)));
    set(handles.edit3,'string',round(q3));
    
else 
    px=get(handles.slider1,'value');
    py=get(handles.slider4,'value');
    pz=get(handles.slider5,'value');
    q4=get(handles.slider8,'value');
    q5=get(handles.slider9,'value');
    q6=get(handles.slider10,'value');
    
    P=evalin('base','P')
    F=fcinversa(px,py,pz,P(4),P(5),P(6),handles);
    q1=F(1);
    q2=F(2);
    q3=F(3);
    fcdirecta(q1,q2,q3,q4,q5,q6,handles);
    
    set(handles.text2,'string',red(px));
    set(handles.edit1,'string',red(px));

    set(handles.text6,'string',red(py));
    set(handles.edit2,'string',red(py));

    set(handles.text7,'string',red(pz));
    set(handles.edit3,'string',red(pz));
    
    set(handles.text8,'String',['q1: ',num2str(round(q1)),', q2: ',num2str(round(q2)),', q3: ',num2str(round(q3))]);
end


% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

set(hObject,'min',-90);
set(hObject,'max',90);
set(hObject,'value',0);
set(hObject,'SliderStep',[(1/180) (1/180)]);


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1


function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when selected object is changed in uipanel1.
function uipanel1_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in uipanel1 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)

op=get(handles.radiobutton2,'value');
espacio=op
assignin('base','espacio',espacio)
if op==1    %ESPACIO ARTICULAR
    px=str2double(get(handles.edit1,'string'));
    py=str2double(get(handles.edit2,'string'));
    pz=str2double(get(handles.edit3,'string'));
    A=str2double(get(handles.edit4,'string'));
    B=str2double(get(handles.edit5,'string'));
    C=str2double(get(handles.edit6,'string'));

%     P=evalin('base','P');
%     Q=fcinversa(px,py,pz,P(4),P(5),P(6),handles);
    Q=fcinversa(px,py,pz,deg2rad(A),deg2rad(B),deg2rad(C),handles);

    q1=Q(1);
    q2=Q(2);
    q3=Q(3);
    q4=rad2deg(Q(4));
    q5=rad2deg(Q(5));
    q6=rad2deg(Q(6));
    fcdirecta(q1,q2,q3,q4,q5,q6,handles)
    

    set(handles.text2,'string',q1);
    set(handles.edit1,'string',q1);
    

    set(handles.text6,'string',round(q2));
    set(handles.edit2,'string',round(q2));
    

    set(handles.text7,'string',round(q3));
    set(handles.edit3,'string',round(q3));
    
    set(handles.text1,'string','q1');
    set(handles.text4,'string','q2');
    set(handles.text5,'string','q3');

    
    set(handles.text8,'String',['Px: ',num2str(red(px)),', Py: ',num2str(red(py)),', Pz: ',num2str(red(pz))]);

    
else    %ESPACIO CARTESIANO
    Q=evalin('base','Q');
    
    q1=round(str2double(get(handles.edit1,'string')));
    q2=round(str2double(get(handles.edit2,'string')));
    q3=round(str2double(get(handles.edit3,'string')));
    q4=round(str2double(get(handles.edit4,'string')));
    q5=round(str2double(get(handles.edit5,'string')));
    q6=round(str2double(get(handles.edit6,'string')));
    
%     q1=Q(1);
%     q2=Q(2);
%     q3=Q(3);
%     q4=Q(4);
%     q5=Q(5);
%     q6=Q(6);  
    fcdirecta(q1,q2,q3,q4,q5,q6,handles)
      
    P=evalin('base','P');
    px=P(1);
    py=P(2);
    pz=P(3);
    
    set(handles.text2,'string',red(px));
    set(handles.edit1,'string',red(px));
    
    set(handles.text6,'string',red(py));
    set(handles.edit2,'string',red(py));

    set(handles.text7,'string',red(pz));
    set(handles.edit3,'string',red(pz));
    
    set(handles.text1,'string','Px');
    set(handles.text4,'string','Py');
    set(handles.text5,'string','Pz');

    
    set(handles.text8,'String',['q1: ',num2str(round(q1)),', q2: ',num2str(round(q2)),', q3: ',num2str(round(q3))]);

end



% --- Executes on slider movement.
function slider8_Callback(hObject, eventdata, handles)
% hObject    handle to slider8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    Q=evalin('base','Q');
    q1=get(handles.slider1,'value');
    q2=get(handles.slider4,'value');
    q3=get(handles.slider5,'value');
    q4=get(handles.slider8,'value');
    q5=get(handles.slider9,'value');
    q6=get(handles.slider10,'value');
    fcdirecta(q1,q2,q3,q4,q5,q6,handles);
    set(handles.text15,'string',[num2str(q4),'�']);

% --- Executes during object creation, after setting all properties.
function slider8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

set(hObject,'min',-90);
set(hObject,'max',90);
set(hObject,'value',0);
set(hObject,'SliderStep',[(1/180) (1/180)]);


% --- Executes on slider movement.
function slider9_Callback(hObject, eventdata, handles)
% hObject    handle to slider9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    Q=evalin('base','Q');
    q1=get(handles.slider1,'value');
    q2=get(handles.slider4,'value');
    q3=get(handles.slider5,'value');
    q4=get(handles.slider8,'value');
    q5=get(handles.slider9,'value');
    q6=get(handles.slider10,'value');
    fcdirecta(q1,q2,q3,q4,q5,q6,handles);
    set(handles.text16,'string',[num2str(q5),'�']);

% --- Executes during object creation, after setting all properties.
function slider9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

set(hObject,'min',-90);
set(hObject,'max',90);
set(hObject,'value',0);
set(hObject,'SliderStep',[(1/180) (1/180)]);

% --- Executes on slider movement.
function slider10_Callback(hObject, eventdata, handles)
% hObject    handle to slider10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    Q=evalin('base','Q');
    q1=get(handles.slider1,'value');
    q2=get(handles.slider4,'value');
    q3=get(handles.slider5,'value');
    q4=get(handles.slider8,'value');
    q5=get(handles.slider9,'value');
    q6=get(handles.slider10,'value');
    fcdirecta(q1,q2,q3,q4,q5,q6,handles);
    set(handles.text17,'string',[num2str(q6),'�']);

% --- Executes during object creation, after setting all properties.
function slider10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

set(hObject,'min',-90);
set(hObject,'max',90);
set(hObject,'value',0);
set(hObject,'SliderStep',[(1/180) (1/180)]);



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% GUARDAR POSICIONES
Q=evalin('base','Q');
cont=evalin('base','cont');
angulos=evalin('base','angulos');
PEF=evalin('base','PEF');
g=evalin('base','g');
plano=evalin('base','plano');
paux=evalin('base','paux');
P=evalin('base','P');
PTR=evalin('base','PTR');
trayectoria=evalin('base','trayectoria');

    mPTP=get(handles.radiobutton6,'value');
    linea=get(handles.radiobutton7,'value');
    cir2D=get(handles.radiobutton8,'value');
    cir3D=get(handles.radiobutton9,'value');
if mPTP == 1
    t=1;
elseif linea == 1
    t=2;
elseif cir2D == 1;
    t=3;
elseif cir3D == 1;
    t=4;
end

cont=cont+1;

if cont==1
    PTR=[PEF(1,1) PEF(1,2) PEF(1,3) angulos(1,1) angulos(1,2) angulos(1,3) paux(1) paux(2) plano g];
    cont=cont+1;
end

trayectoria(cont,:)=t;
if t==1
    PTR(cont,:)=[Q(1) Q(2) Q(3) P(4) P(5) P(6) paux(1) paux(2) plano g];
else
    PTR(cont,:)=[P(1) P(2) P(3) P(4) P(5) P(6) paux(1) paux(2) plano g];
end

assignin('base','trayectoria',trayectoria)
assignin('base','PTR',PTR)
assignin('base','cont',cont);

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% P=evalin('base','P');
cont=evalin('base','cont');
trayectoria=evalin('base','trayectoria');
PTR=evalin('base','PTR');
for t=1:cont
    P=evalin('base','P')
    P(4)=rad2deg(PTR(t,4));
    P(5)=rad2deg(PTR(t,5));
    P(6)=rad2deg(PTR(t,6));
    assignin('base','P',P)
    if trayectoria(t)==1           %============PTP
%         F=fcinversa(PTR(t,1),PTR(t,2),PTR(t,3),PTR(t,4),PTR(t,5),PTR(t,6),handles);
        
        PTP(PTR(t,1),PTR(t,2),PTR(t,3),handles)
        pause(0.5)
        g=PTR(t,10);
        assignin('base','g',g)
        gripper(handles)
        
    elseif trayectoria(t)==2       %============Linea
        
        LIN(PTR(t,1),PTR(t,2),PTR(t,3),handles)
        pause(0.5)
        g=PTR(t,10);
        assignin('base','g',g)
        gripper(handles)
        
    elseif trayectoria(t)==3       %============circulo2D
        
        CIR2D(PTR(t,1),PTR(t,2),PTR(t,3),PTR(t,7),PTR(t,8),PTR(t,9),handles)
        pause(0.5)
        g=PTR(t,10);
        assignin('base','g',g)
        
        gripper(handles)
        
    end
    pause(1)
end
    
% --- Executes on button press in radiobutton6.
function radiobutton6_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton6


% --- Executes on button press in radiobutton7.
function radiobutton7_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton7


% --- Executes on button press in radiobutton8.
function radiobutton8_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton8


% --- Executes on button press in radiobutton9.
function radiobutton9_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton9


% --- Executes when selected object is changed in uipanel7.
function uipanel7_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in uipanel7 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)
cir2D=get(handles.radiobutton8,'value');
if cir2D==1
    set(handles.popupmenu1,'enable','on')
    set(handles.edit7,'enable','on')
    set(handles.edit8,'enable','on')
else
    set(handles.popupmenu1,'enable','off')
    set(handles.edit7,'enable','off')
    set(handles.edit8,'enable','off')
end


% --- Executes on button press in checkbox2.
function checkbox2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox2



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when selected object is changed in uipanel8.
function uipanel8_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in uipanel8 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)
op=get(handles.radiobutton10,'value');
Q=evalin('base','Q');

if op==1    %MODO MANUAL
    set(handles.pushbutton9,'enable','on')
    set(handles.pushbutton10,'enable','on')
    set(handles.pushbutton11,'enable','on')
    set(handles.pushbutton12,'enable','on')
    set(handles.pushbutton13,'enable','on')
    set(handles.pushbutton14,'enable','on')
    set(handles.pushbutton15,'enable','on')
    set(handles.pushbutton16,'enable','on')
    set(handles.pushbutton17,'enable','on')
    set(handles.pushbutton18,'enable','on')
    set(handles.pushbutton19,'enable','on')
    set(handles.pushbutton20,'enable','on')
    set(handles.text2,'enable','on')
    set(handles.text6,'enable','on')
    set(handles.text7,'enable','on')
    set(handles.text15,'enable','on')
    set(handles.text16,'enable','on')
    set(handles.text17,'enable','on')
%     set(handles.radiobutton6,'enable','off')
%     set(handles.radiobutton7,'enable','off')
%     set(handles.radiobutton8,'enable','off')
%     set(handles.radiobutton9,'enable','off')
else
    set(handles.pushbutton9,'enable','off')
    set(handles.pushbutton10,'enable','off')
    set(handles.pushbutton11,'enable','off')
    set(handles.pushbutton12,'enable','off')
    set(handles.pushbutton13,'enable','off')
    set(handles.pushbutton14,'enable','off')
    set(handles.pushbutton15,'enable','off')
    set(handles.pushbutton16,'enable','off')
    set(handles.pushbutton17,'enable','off')
    set(handles.pushbutton18,'enable','off')
    set(handles.pushbutton19,'enable','off')
    set(handles.pushbutton20,'enable','off')
    set(handles.text2,'enable','off')
    set(handles.text6,'enable','off')
    set(handles.text7,'enable','off')
    set(handles.text15,'enable','off')
    set(handles.text16,'enable','off')
    set(handles.text17,'enable','off')
%     set(handles.radiobutton6,'enable','on')
%     set(handles.radiobutton7,'enable','on')
%     set(handles.radiobutton8,'enable','on')
%     set(handles.radiobutton9,'enable','on')
    
        
end



% --- Executes on slider movement.
function slider11_Callback(hObject, eventdata, handles)
% hObject    handle to slider11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

Vel=evalin('base','Vel');
Vel=get(handles.slider11,'value');
assignin('base','Vel',Vel)
set(handles.edit11,'string',Vel)

% --- Executes during object creation, after setting all properties.
function slider11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

set(hObject,'min',0)
set(hObject,'max',100)
set(hObject,'value',50)
set(hObject,'Sliderstep',[1/100 1/100])


function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ardno=1;
assignin('base','ardno',ardno)

try
a=arduino('COM1')
assignin('base','a',a);
catch
    a=evalin('base','a');
end
a.servoAttach(3)
a.servoAttach(5)
a.servoAttach(6)
a.servoAttach(9)
a.servoAttach(10)
a.servoAttach(11)


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear all ports
a=evalin('base','a');
a.servoDetach(3)
a.servoDetach(5)
a.servoDetach(6)
a.servoDetach(9)
a.servoDetach(10)
a.servoDetach(11)

% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in togglebutton1.
function togglebutton1_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton1

g=get(hObject,'value');
assignin('base','g',g)
pause(0.1)
gripper(handles);


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla

Q=evalin('base','Q');
op=get(handles.radiobutton2,'value');   %=================-1===================
if op==1
    q1=Q(1)-2;
    if q1<0
        Q(1)=0;
    else
        Q(1)=q1;
    end
        q1=Q(1);
        fcdirecta(Q(1),Q(2),Q(3),Q(4),Q(5),Q(6),handles);
        set(handles.text2,'string',num2str(round(q1)));
        set(handles.edit1,'string',round(q1));
else
    P=evalin('base','P');
    P(1)=P(1)-0.5;
    F=fcinversa(P(1),P(2),P(3),P(4),P(5),P(6),handles);

    fcdirecta(F(1),F(2),F(3),F(4),F(5),F(6),handles);
    P=evalin('base','P');
    px=P(1);
    set(handles.text2,'string',num2str(red(px)));
    set(handles.edit1,'string',red(px));
    set(handles.text8,'String',['q1: ',num2str(round(F(1))),', q2: ',num2str(round(F(2))),', q3: ',num2str(round(F(3)))]);
end

% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla

Q=evalin('base','Q');
op=get(handles.radiobutton2,'value');   %====================-2===================
if op==1
    q2=Q(2)-2;
    if q2<0
        Q(2)=0;
    else
        Q(2)=q2;
    end
        q2=Q(2);
        fcdirecta(Q(1),Q(2),Q(3),Q(4),Q(5),Q(6),handles);
        set(handles.text6,'string',num2str(round(q2)));
        set(handles.edit2,'string',round(q2));
else
    P=evalin('base','P');
    P(2)=P(2)-0.5;
    F=fcinversa(P(1),P(2),P(3),P(4),P(5),P(6),handles);

    fcdirecta(F(1),F(2),F(3),F(4),F(5),F(6),handles);
    P=evalin('base','P');
    py=P(2);
    set(handles.text6,'string',num2str(red(py)));
    set(handles.edit2,'string',red(py));
    set(handles.text8,'String',['q1: ',num2str(round(F(1))),', q2: ',num2str(round(F(2))),', q3: ',num2str(round(F(3)))]);
end

% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla

Q=evalin('base','Q');
op=get(handles.radiobutton2,'value');   %=======================-3===================
if op==1
    q3=Q(3)-2;
    if q3<-90
        Q(3)=-90;
    else
        Q(3)=q3;
    end
        q3=Q(3);
        fcdirecta(Q(1),Q(2),Q(3),Q(4),Q(5),Q(6),handles);
        set(handles.text7,'string',num2str(round(q3+90)));
        set(handles.edit3,'string',round(q3+90));
else
    P=evalin('base','P');
    P(3)=P(3)-0.5;
    F=fcinversa(P(1),P(2),P(3),P(4),P(5),P(6),handles);

    fcdirecta(F(1),F(2),F(3),F(4),F(5),F(6),handles);
    P=evalin('base','P');
    pz=P(3);
    set(handles.text7,'string',num2str(red(pz)));
    set(handles.edit3,'string',red(pz));
    set(handles.text8,'String',['q1: ',num2str(round(F(1))),', q2: ',num2str(round(F(2))),', q3: ',num2str(round(F(3)))]);
end

% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla

Q=evalin('base','Q');
op=get(handles.radiobutton2,'value');   %=======================-4===================
if op==1
    q4=Q(4)-2;
    if q4<-90
        Q(4)=-90;
    else
        Q(4)=q4;
    end
        q4=Q(4);
        fcdirecta(Q(1),Q(2),Q(3),Q(4),Q(5),Q(6),handles);
        set(handles.text15,'string',num2str(red(q4)));
        set(handles.edit4,'string',red(q4));
else
    P=evalin('base','P');
    P(4)=rad2deg(P(4))-2;
    F=fcinversa(P(1),P(2),P(3),deg2rad(P(4)),(P(5)),(P(6)),handles);

    fcdirecta(F(1),F(2),F(3),rad2deg(F(4)),rad2deg(F(5)),rad2deg(F(6)),handles);
    P=evalin('base','P');
    q4=rad2deg(P(4));
    set(handles.text15,'string',num2str(red(q4)));
    set(handles.edit4,'string',red(q4));
    set(handles.text8,'String',['q1: ',num2str(round(F(1))),', q2: ',num2str(round(F(2))),', q3: ',num2str(round(F(3)))]);
end

% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla
op=get(handles.radiobutton2,'value');   %=======================-5===================
if op==1
Q=evalin('base','Q');
    q5=Q(5)-2;
    if q5<-90
        Q(5)=-90;
    else
        Q(5)=q5;
    end
        q5=Q(5);
        fcdirecta(Q(1),Q(2),Q(3),Q(4),Q(5),Q(6),handles);
        set(handles.text16,'string',num2str(red(q5)));
        set(handles.edit5,'string',red(q5));
else
    P=evalin('base','P');
    P(5)=rad2deg(P(5))-2;
    F=fcinversa(P(1),P(2),P(3),(P(4)),deg2rad(P(5)),(P(6)),handles);

    fcdirecta(F(1),F(2),F(3),rad2deg(F(4)),rad2deg(F(5)),rad2deg(F(6)),handles);
    P=evalin('base','P');
    q5=rad2deg(P(5));
    set(handles.text16,'string',num2str(red(q5)));
    set(handles.edit5,'string',red(q5));
    set(handles.text8,'String',['q1: ',num2str(round(F(1))),', q2: ',num2str(round(F(2))),', q3: ',num2str(round(F(3)))]);
end

% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla

Q=evalin('base','Q');
op=get(handles.radiobutton2,'value');   %=======================-6===================
if op==1
    q6=Q(6)-2;
    if q6<-90
        Q(6)=-90;
    else
        Q(6)=q6;
    end
        q6=Q(6);
        fcdirecta(Q(1),Q(2),Q(3),Q(4),Q(5),Q(6),handles);
        set(handles.text17,'string',num2str(red(q6)));
        set(handles.edit6,'string',red(q6));
else
    P=evalin('base','P');
    P(6)=rad2deg(P(6))-2;
    F=fcinversa(P(1),P(2),P(3),(P(4)),(P(5)),deg2rad(P(6)),handles);

    fcdirecta(F(1),F(2),F(3),rad2deg(F(4)),rad2deg(F(5)),rad2deg(F(6)),handles);
    P=evalin('base','P');
    q6=rad2deg(P(6));
    set(handles.text17,'string',num2str(red(q6)));
    set(handles.edit6,'string',red(q6));
    set(handles.text8,'String',['q1: ',num2str(round(F(1))),', q2: ',num2str(round(F(2))),', q3: ',num2str(round(F(3)))]);
end

% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla

Q=evalin('base','Q');
op=get(handles.radiobutton2,'value');   %=======================+1===================
if op==1
    q1=Q(1)+2;
    if q1>180
        Q(1)=180;
    else
        Q(1)=q1;
    end
        q1=Q(1);
        fcdirecta(Q(1),Q(2),Q(3),Q(4),Q(5),Q(6),handles);
        set(handles.text2,'string',num2str(round(q1)));
        set(handles.edit1,'string',round(q1));
else
    P=evalin('base','P');
    P(1)=P(1)+0.5;
    F=fcinversa(P(1),P(2),P(3),P(4),P(5),P(6),handles);

    fcdirecta(F(1),F(2),F(3),F(4),F(5),F(6),handles);
    P=evalin('base','P');
    px=P(1);
    set(handles.text2,'string',num2str(red(px)));
    set(handles.edit1,'string',red(px));
    set(handles.text8,'String',['q1: ',num2str(round(F(1))),', q2: ',num2str(round(F(2))),', q3: ',num2str(round(F(3)))]);
end

% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla

Q=evalin('base','Q');
op=get(handles.radiobutton2,'value');   %=======================+2===================
if op==1
    q2=Q(2)+2;
    if q2>180
        Q(2)=180;
    else
        Q(2)=q2;
    end
        q2=Q(2);
        fcdirecta(Q(1),Q(2),Q(3),Q(4),Q(5),Q(6),handles);
        set(handles.text6,'string',num2str(round(q2)));
        set(handles.edit2,'string',round(q2));
else
    P=evalin('base','P');
    P(2)=P(2)+0.5;
    F=fcinversa(P(1),P(2),P(3),P(4),P(5),P(6),handles);

    fcdirecta(F(1),F(2),F(3),F(4),F(5),F(6),handles);
    P=evalin('base','P');
    py=P(2);
    set(handles.text6,'string',num2str(red(py)));
    set(handles.edit2,'string',red(py));
    set(handles.text8,'String',['q1: ',num2str(round(F(1))),', q2: ',num2str(round(F(2))),', q3: ',num2str(round(F(3)))]);
end

% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla

Q=evalin('base','Q');
op=get(handles.radiobutton2,'value');   %=======================+3===================
if op==1
    q3=Q(3)+2;
    if q3>90
        Q(3)=90;
    else
        Q(3)=q3;
    end
        q3=Q(3);
        fcdirecta(Q(1),Q(2),Q(3),Q(4),Q(5),Q(6),handles);
        set(handles.text7,'string',num2str(round(q3+90)));
        set(handles.edit3,'string',round(q3+90));
else
    P=evalin('base','P');
    P(3)=P(3)+0.5;
    F=fcinversa(P(1),P(2),P(3),P(4),P(5),P(6),handles);

    fcdirecta(F(1),F(2),F(3),F(4),F(5),F(6),handles);
    P=evalin('base','P');
    pz=P(3);
    set(handles.text7,'string',num2str(red(pz)));
    set(handles.edit3,'string',red(pz));
    set(handles.text8,'String',['q1: ',num2str(round(F(1))),', q2: ',num2str(round(F(2))),', q3: ',num2str(round(F(3)))]);
end

% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla

Q=evalin('base','Q');
op=get(handles.radiobutton2,'value');   %=======================+4===================
if op==1
    q4=Q(4)+2;
    if q4>90
        Q(4)=90;
    else
        Q(4)=q4;
    end
        q4=Q(4);
        fcdirecta(Q(1),Q(2),Q(3),Q(4),Q(5),Q(6),handles);
        set(handles.text15,'string',num2str(red(q4)));
        set(handles.edit4,'string',red(q4));
else
    P=evalin('base','P');
    P(4)=rad2deg(P(4))+2
    F=fcinversa(P(1),P(2),P(3),deg2rad(P(4)),(P(5)),(P(6)),handles);

    fcdirecta(F(1),F(2),F(3),rad2deg(F(4)),rad2deg(F(5)),rad2deg(F(6)),handles);
    P=evalin('base','P');
    q4=rad2deg(P(4));
    set(handles.text15,'string',num2str(red(q4)));
    set(handles.edit4,'string',red(q4));
    set(handles.text8,'String',['q1: ',num2str(round(F(1))),', q2: ',num2str(round(F(2))),', q3: ',num2str(round(F(3)))]);
end

% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla
Q=evalin('base','Q');   %=======================+5===================
op=get(handles.radiobutton2,'value');
if op==1
    q5=Q(5)+2;
    if q5>90
        Q(5)=90;
    else
        Q(5)=q5;
    end
        q5=Q(5);
        fcdirecta(Q(1),Q(2),Q(3),Q(4),Q(5),Q(6),handles);
        set(handles.text16,'string',num2str(red(q5)));
        set(handles.edit5,'string',red(q5));
    else
    P=evalin('base','P');
    P(5)=rad2deg(P(5))+2;
    F=fcinversa(P(1),P(2),P(3),(P(4)),deg2rad(P(5)),(P(6)),handles);

    fcdirecta(F(1),F(2),F(3),rad2deg(F(4)),rad2deg(F(5)),rad2deg(F(6)),handles);
    P=evalin('base','P');
    q5=rad2deg(P(5));
    set(handles.text16,'string',num2str(red(q5)));
    set(handles.edit5,'string',red(q5));
    set(handles.text8,'String',['q1: ',num2str(round(F(1))),', q2: ',num2str(round(F(2))),', q3: ',num2str(round(F(3)))]);
end
        
% --- Executes on button press in pushbutton20.
function pushbutton20_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla

Q=evalin('base','Q');   %=======================+6===================
op=get(handles.radiobutton2,'value');
if op==1
    q6=Q(6)+2;
    if q6>90
        Q(6)=90;
    else
        Q(6)=q6;
    end
        q6=Q(6);
        fcdirecta(Q(1),Q(2),Q(3),Q(4),Q(5),Q(6),handles);
        set(handles.text17,'string',num2str(red(q6)));
        set(handles.edit6,'string',red(q6));
else
    P=evalin('base','P');
    P(6)=rad2deg(P(6))+2;
    F=fcinversa(P(1),P(2),P(3),(P(4)),(P(5)),deg2rad(P(6)),handles);

    fcdirecta(F(1),F(2),F(3),rad2deg(F(4)),rad2deg(F(5)),rad2deg(F(6)),handles);
    P=evalin('base','P');
    q6=rad2deg(P(6));
    set(handles.text17,'string',num2str(red(q6)));
    set(handles.edit6,'string',red(q6));
    set(handles.text8,'String',['q1: ',num2str(round(F(1))),', q2: ',num2str(round(F(2))),', q3: ',num2str(round(F(3)))]);
end
        
% --- Executes on button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

PTP(90,90,0,handles)


% --- Executes on button press in pushbutton22.
function pushbutton22_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cont=0;
assignin('base','cont',cont);
PTR=[0 0 0 0 0 0 0 0 0 0];
assignin('base','PTR',PTR)
trayectoria=0;
assignin('base','trayectoria',trayectoria)


% --- Executes on button press in pushbutton23.
function pushbutton23_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% save('rutina.mat','PTR','cont','trayectoria')
PTR=evalin('base','PTR');
cont=evalin('base','cont');
trayectoria=evalin('base','trayectoria');
save rutina.mat PTR cont trayectoria
% save('acont','cont')
% save('atrayectoria','trayectoria')


% --- Executes on button press in pushbutton24.
function pushbutton24_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

load('rutina.mat')

assignin('base','PTR',PTR)
assignin('base','cont',cont)
assignin('base','trayectoria',trayectoria)
