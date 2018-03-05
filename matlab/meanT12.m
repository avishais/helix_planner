load('T12options.mat')
%%
T = zeros(4,4);
for i = 1:6
T = T + T12{i};
end
T=T/6;
r = T(1:3,4);
%%
clear e
for i = 1:6
    e(i,:) = rotm2eul(T12{i}(1:3,1:3));
end
e = mean(e);

R = eul2rotm(e);

T(1:3,1:3) = R;

%%
eul = (rotm2eul(T(1:3,1:3)))+[pi 0 0]
%%
% Average displacement between robot base frames (in meters)
% ( x , y , z ) = ( 1.0921 , 0.0034 , 0.00010 ) m

% Z (looking up into the base's axis)
% mean 3.6824 <-- this is the important number if we assume the table is a flat surface.
tz = eul(1); %psi
% Y (right hand rule direction, off to side)
% mean 0.8164
ty = eul(2);%deg2rad(0.8164); % theta
% X (looking straight 'forward')
% mean 1.0382 
tx = eul(3);%deg2rad(1.0382); %phi

Rz = @(t) [cos(t) -sin(t) 0 0; sin(t) cos(t) 0 0; 0 0 1 0; 0 0 0 1];
Ry = @(t) [cos(t) 0 sin(t) 0; 0 1 0 0; -sin(t) 0 cos(t) 0; 0 0 0 1];
Rx = @(t) [1 0 0 0; 0 cos(t) -sin(t) 0; 0 sin(t) cos(t) 0;0 0 0 1];
Rt = @(p) [eye(3) p; 0 0 0 1];
Rzyx = Rz(tz+pi)*Ry(ty)*Rx(tx);%*Rz(pi);

p2 = r;
Tpose2 = Rt(p2)*Rzyx;
