% Forward Kinematics
function FK1
% Step #1: Initial Guess
P0 = [0 0 150 0 0 0]';
%Declaring the leg lengths 
lg = [250.1730, 247.7072, 253.3073, 277.6336, 278.4548, 254.3322]';

P(:,1) = P0;
i = 2;
dl = 1;
while dl > 0.0001
   % Step #2
   J = jacobianV1(P(:,i-1));

   % Step #3: Calculate T
   a = P(4,i-1)*pi/180;
   b = P(5,i-1)*pi/180;
   c = P(6,i-1)*pi/180;
   B = [0    -sin(a) sin(b)*cos(a);
        0     cos(a) sin(b)*sin(a);
        1     0      cos(b)];
   T = [eye(3)       zeros(3,3)
        zeros(3,3)   B];

   % Step #4: IK
   [l n R s] = IK1(P(:,i-1));

   Dl = lg - l';
   dl = norm(Dl , 2);

   % Step #5
   P(:,i) = P(:,i-1) + pinv(J*T) * Dl;
   %Dp = norm(P(:,1) - P(:,1) , 2);
  i = i+1;

end
P(:,i-1)
[l n R s] = IK1(P(:,i-1));
