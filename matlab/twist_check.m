% twist check

D = load('../robot_path_twist.txt');

plot(rad2deg(D(:,[6 12])),'.-');