function varargout = Six_R_Robot(varargin)
% SIX_R_ROBOT MATLAB code for Six_R_Robot.fig
%      SIX_R_ROBOT, by itself, creates a new SIX_R_ROBOT or raises the existing
%      singleton*.
%
%      H = SIX_R_ROBOT returns the handle to a new SIX_R_ROBOT or the handle to
%      the existing singleton*.
%
%      SIX_R_ROBOT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SIX_R_ROBOT.M with the given input arguments.
%
%      SIX_R_ROBOT('Property','Value',...) creates a new SIX_R_ROBOT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Six_R_Robot_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Six_R_Robot_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Six_R_Robot

% Last Modified by GUIDE v2.5 20-Dec-2019 02:22:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Six_R_Robot_OpeningFcn, ...
                   'gui_OutputFcn',  @Six_R_Robot_OutputFcn, ...
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


% --- Executes just before Six_R_Robot is made visible.
function Six_R_Robot_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Six_R_Robot (see VARARGIN)

% Choose default command line output for Six_R_Robot
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Six_R_Robot wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Six_R_Robot_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function Theta_1_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_1 as text
%        str2double(get(hObject,'String')) returns contents of Theta_1 as a double


% --- Executes during object creation, after setting all properties.
function Theta_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_2_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_2 as text
%        str2double(get(hObject,'String')) returns contents of Theta_2 as a double


% --- Executes during object creation, after setting all properties.
function Theta_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_3_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_3 as text
%        str2double(get(hObject,'String')) returns contents of Theta_3 as a double


% --- Executes during object creation, after setting all properties.
function Theta_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_4_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_4 as text
%        str2double(get(hObject,'String')) returns contents of Theta_4 as a double


% --- Executes during object creation, after setting all properties.
function Theta_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_5_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_5 as text
%        str2double(get(hObject,'String')) returns contents of Theta_5 as a double


% --- Executes during object creation, after setting all properties.
function Theta_5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_6_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_6 as text
%        str2double(get(hObject,'String')) returns contents of Theta_6 as a double


% --- Executes during object creation, after setting all properties.
function Theta_6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pos_X_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_X as text
%        str2double(get(hObject,'String')) returns contents of Pos_X as a double


% --- Executes during object creation, after setting all properties.
function Pos_X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pos_Z_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_Z as text
%        str2double(get(hObject,'String')) returns contents of Pos_Z as a double


% --- Executes during object creation, after setting all properties.
function Pos_Z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_Forward.
function btn_Forward_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Th_1 = str2double(handles.Theta_1.String)*pi/180;
Th_2 = str2double(handles.Theta_2.String)*pi/180;
Th_3 = str2double(handles.Theta_3.String)*pi/180;
Th_4 = str2double(handles.Theta_4.String)*pi/180;
Th_5 = str2double(handles.Theta_5.String)*pi/180;
Th_6 = str2double(handles.Theta_6.String)*pi/180;

angle_offset = [0 90 90 -90 0 0]*pi/180;

L_1 = 0;
L_2 = 80;
L_3 = 70;
L_4 = 50;
L_5 = 0;
L_6 = 0;

L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
L(4) = Link([0 0 L_4 -pi/2]);
L(5) = Link([0 0 L_5 pi/2]);
L(6) = Link([0 L_6 0 0]);

Robot = SerialLink(L);
Robot.name = 'Six_R_Robot';
Robot.plot([Th_1 Th_2 Th_3 Th_4 Th_5 Th_6]);

T = Robot.fkine([Th_1 Th_2 Th_3 Th_4 Th_5 Th_6]);
handles.Pos_X.String = num2str(floor(T.t(1)));
handles.Pos_Y.String = num2str(floor(T.t(2)));
handles.Pos_Z.String = num2str(floor(T.t(3)));


if exist('theta_initial') == 0
    theta_initial = angle_offset;
end

Th_path{1} = linspace(theta_initial(1),Th_1,20);
Th_path{2} = linspace(theta_initial(2),Th_2,20);
Th_path{3} = linspace(theta_initial(3),Th_3,20);
Th_path{4} = linspace(theta_initial(4),Th_4,20);
Th_path{5} = linspace(theta_initial(5),Th_5,20);
Th_path{6} = linspace(theta_initial(6),Th_6,20);


% Initial Position
P_path = [];
for i = 1:20
    P_path = [P_path fkine4DOF(   DH, [Th_path{1}(i) Th_path{2}(i) Th_path{3}(i) Th_path{4}(i)])];
end

% Target Position
% P = fkine4DOF(DH, [Th_1 Th_2 Th_3 Th_4], angle_offset);
% P_path{1} = linspace(P_initial(1),P(1), 20);
% P_path{2} = linspace(P_initial(2),P(2), 20);
% P_path{3} = linspace(P_initial(3),P(3), 20);

plot3(P_path(1,:), P_path(2,:), P_path(3,:), 'Color', [1 0 0], 'Linewidth', 2);
hold on;
xlim([-20 20])
ylim([-20 20])
zlim([-10 30])

for i = 1:20
    Robot.plot([Th_path{1}(i) Th_path{2}(i) pi/2 Th_path{3}(i) Th_path{4}(i) pi/2, 'scale', 0.5, 'perspective'])
end

hold off;
P = fkine4DOF(DH, [Th_1 Th_2 Th_3 Th_4], angle_offset);
P = eval(P)
handles.x.String = num2str(round(P(1),3));
handles.y.String = num2str(round(P(2),3));
handles.z.String = num2str(round(P(3),3));





% --- Executes on button press in btn_Inverse.
function btn_Inverse_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
PX = str2double(handles.Pos_X.String);
PY = str2double(handles.Pos_Y.String);
PZ = str2double(handles.Pos_Z.String);

L_1 = 0;
L_2 = 80;
L_3 = 70;
L_4 = 50;
L_5 = 0;
L_6 = 0;

L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
L(4) = Link([0 0 L_4 -pi/2]);
L(5) = Link([0 0 L_5 pi/2]);
L(6) = Link([0 L_6 0 0]);


Robot = SerialLink(L);
Robot.name = 'Six_R_Robot';

T = [ 1 0 0 PX;
      0 1 0 PY;
      0 0 1 PZ;
      0 0 0 1];
  
J = Robot.ikine(T,[0 0 0],'mask',[1 1 1 0 0 0])*180/pi;
handles.Theta_1.String = num2str(floor(J(1)));
handles.Theta_2.String = num2str(floor(J(2)));
handles.Theta_3.String = num2str(floor(J(3)));
handles.Theta_4.String = num2str(floor(J(4)));
handles.Theta_5.String = num2str(floor(J(5)));
handles.Theta_6.String = num2str(floor(J(6)));

Robot.plot(J*pi/180);



function Pos_Y_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_Y as text
%        str2double(get(hObject,'String')) returns contents of Pos_Y as a double


% --- Executes during object creation, after setting all properties.
function Pos_Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
