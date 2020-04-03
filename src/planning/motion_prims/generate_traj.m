function points=generate_traj(l1,R,l2,x_i,y_i,theta_i,x_f,y_f,theta_f,numofsamples)

% compute the total length to move
L=abs(l1)+abs(l2)+abs(R*(theta_f-theta_i));
%generate samples
dtheta=theta_i;
length2=0;
points = zeros(numofsamples,3);
for i = 1:numofsamples                                        
    dL = L*(i-1)/(numofsamples-1);
    if(dL < abs(l1))
        if(l1>0)
            points(i,:) = [x_i + dL*cos(theta_i) ...
                y_i + dL*sin(theta_i) ...
                theta_i];
        else
            points(i,:) = [x_i - dL*cos(theta_i) ...
                y_i - dL*sin(theta_i) ...
                theta_i];
        end
    else
        if(dL<(L-abs(l2)))
            if(theta_i<theta_f)
                dtheta = dtheta+(L/(numofsamples-1))/abs(R);
                points(i,:) = [x_i + l1*cos(theta_i) + R*(sin(dtheta) - ...
                    sin(theta_i)) y_i + l1*sin(theta_i) - R*(cos(dtheta) -...
                    cos(theta_i)) dtheta];
            else
                dtheta = dtheta-(L/(numofsamples-1))/abs(R);
                points(i,:) = [x_i + l1*cos(theta_i) + R*(sin(dtheta) - ...
                    sin(theta_i)) y_i + l1*sin(theta_i) - R*(cos(dtheta) -...
                    cos(theta_i)) dtheta];
            end
        else
            if(l2>0)
                points(i,:) = [x_i + l1*cos(theta_i) + R*(sin(theta_f) - ...
                    sin(theta_i)) + length2*cos(theta_f) y_i + ...
                    l1*sin(theta_i) - R*(cos(theta_f) - cos(theta_i)) + ...
                    length2*sin(theta_f) theta_f];
                length2=length2+(L/(numofsamples-1));
            else
                points(i,:) = [x_i + l1*cos(theta_i) + R*(sin(theta_f) - ...
                    sin(theta_i)) + length2*cos(theta_f) y_i + ...
                    l1*sin(theta_i) - R*(cos(theta_f) - cos(theta_i)) + ...
                    length2*sin(theta_f) theta_f];
                length2=length2-(L/(numofsamples-1));
            end
        end
    end
end

% compute final pose error
errorxy = [x_f - points(numofsamples,1) ...
    y_f - points(numofsamples,2)];
interpfactor = [0:1/(numofsamples-1):1];
points(:,1) = points(:,1) + errorxy(1)*interpfactor';
points(:,2) = points(:,2) + errorxy(2)*interpfactor';
    
