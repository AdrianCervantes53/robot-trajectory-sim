function gripper(handles)

g=evalin('base','g');

ardno=evalin('base','ardno');
if ardno==1
    a=evalin('base','a');
    if g==0
        a.servoWrite(11,80)
    elseif g==1
        a.servoWrite(11,145)
    end
end