errorx = input("Enter world x error: ");
errory = input("Enter world y error: ");
errorz = input("Enter world z error: ");

errorWorld = [errorx; errory; errorz];  

rotation = zeros(1,3);
rotation(3) = input("Enter roll (radians): ");
rotation(2) = input("Enter pitch (radians): ");
rotation(1) = input("Enter yaw (radians): ");

rotm = transpose(eul2rotm(rotation));        
errorBody = mtimes(rotm,errorWorld);           

disp("Body frame error:");
disp(errorBody);