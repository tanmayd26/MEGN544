function [eepos] = forwardkinematics(pose,a,b)

c_124 = cos(pose(1)+pose(2)+pose(4));
s_124 = sin(pose(1)+pose(2)+pose(4));

eepos = [c_124*cos(pose(5))*cos(pose(6))-s_124*sin(pose(6)) -s_124*cos(pose(6))-c_124*cos(pose(5))*sin(pose(6)) -c_124*sin(pose(5)) cos(pose(1)+pose(2))*b+cos(pose(1))*a;
    s_124*cos(pose(5))*cos(pose(6))+c_124*sin(pose(6)) c_124*cos(pose(6))-s_124*cos(pose(5))*sin(pose(6)) -s_124*sin(pose(5)) sin(pose(1))*sin(pose(2))*b+sin(pose(1))*a;
    cos(pose(6))*sin(pose(5)) -sin(pose(5))*sin(pose(6)) cos(pose(5)) pose(3);
    0 0 0 1];