function [x y]=gen_position(R,res,theta_i,theta_f)
theta_f=(theta_f)*pi/180;
theta_i=(theta_i)*pi/180;
x=R*abs(sin(theta_f)-sin(theta_i))/res;
if theta_f<theta_i
    y=R*(cos(theta_f)-cos(theta_i))/res;
else
    y=-R*(cos(theta_f)-cos(theta_i))/res;
end

x=round(x);
y=round(y);