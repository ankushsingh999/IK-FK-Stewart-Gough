% Velocity Jacobian
function J = jacobianV1(P)
[l n R s] = IK1(P);
J = [n(:,1)' , cross(R * s(:,1), n(:,1))';
     n(:,2)' , cross(R * s(:,2), n(:,2))';
     n(:,3)' , cross(R * s(:,3), n(:,3))';
     n(:,4)' , cross(R * s(:,4), n(:,4))';
     n(:,5)' , cross(R * s(:,5), n(:,5))';
     n(:,6)' , cross(R * s(:,6), n(:,6))'];
 end