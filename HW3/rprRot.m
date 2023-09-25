function[R] = rprRot(phi,theta)

R = rotZ(phi)*rotY(theta)*rotZ(phi);

end