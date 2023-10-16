% createLink: Creates a structure 
%
% R = angleAxis2Rot(k, theta) Provides us the 3-by-3 roation matrix for the  angle 
% axis represenation for a rotation of theta about k vector 
%
% output1 = Structure
% output2 = description of what the second output is/means include units if appropriate
%
% input1 = DH parameter a (meters)
% input2 = DH parameter d (meters)
% input3 = DH parameter alpha (radians)
% input4 = joint angle provided by encoder (radians)
% input5 = The number of radians (or meters for prismatic) different between the 
% encoder orientation and the DH zero-angle. θ_DH = Θ -Θ_offset (equivalent
% equation for prismatic). 
% input6 = the position of the link s center of mass 
% input7 = link mass (kg)
% input8 = link mass moment of inertia (kg m^2)
%
% Tanmay Desai
% 10922557
% MEGN 544A
% October 22nd 2023
function L = createLink(a, d, alpha, theta, offset,centOfMass, mass, inertia)



if isempty(theta)
   
   L.a = a;
   L.d = d;
   L.alpha = alpha;
   L.theta = [];
   L.offset = offset;
   L.mass = mass;
   L.inertia = inertia;
   L.com = centOfMass;
   L.isRotary = 1;
    
elseif isempty(d)
    L.a = a;
   L.d = d;
   L.alpha = alpha;
   L.theta = [];
   L.offset = offset;
   L.mass = mass;
   L.inertia = inertia;
   L.com = centOfMass;
   L.isRotary = 0;
else
    L.a = a;
   L.d = d;
   L.alpha = alpha;
   L.theta = theta;
   L.offset = offset;
   L.mass = mass;
   L.inertia = inertia;
   L.com = centOfMass;
   L.isRotary = -1;
    
end
