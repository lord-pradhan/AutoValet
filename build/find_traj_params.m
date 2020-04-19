function [l1 R l2 status]=find_traj_params(theta_i,theta_f,delta_x,delta_y,Rmin)

A_eq=[[cos(theta_i) sin(theta_f)-sin(theta_i) cos(theta_f)]; ...
      [sin(theta_i) -(cos(theta_f)-cos(theta_i)) sin(theta_f)]];
b_eq=[delta_x;delta_y];
A_ineq=zeros(3,3);
b_ineq=zeros(3,1);
f=zeros(3,1);
nx=cos(theta_i);
ny=sin(theta_i);
norm=sqrt(delta_x^2+delta_y^2);
if(nx*delta_x/norm+ny*delta_y/norm>0)
    % positive l1
    A_ineq(1,:)=[-1000 0 0];
    f(1)=1;
else
    % negative l1
    A_ineq(1,:)=[1000 0 0];
    f(1)=-1;
end
if(nx*delta_x/norm+ny*delta_y/norm>0)
    % positive l2
    A_ineq(3,:)=[0 0 -1];
    f(3)=1;
else
    % negative l2
    A_ineq(3,:)=[0 0 1];
    f(3)=-1;
end
nx=cos(theta_i+pi/2);
ny=sin(theta_i+pi/2);
if(nx*delta_x/norm+ny*delta_y/norm>0)
    % positive R
    A_ineq(2,:)=[0 -1 0];
    b_ineq(2)=-Rmin;
    f(2)=0;
else
    % negative R
    A_ineq(2,:)=[0 1 0];
    b_ineq(2)=-Rmin;
    f(2)=0;
end
[x,fval,exitflag] = linprog(f,A_ineq,b_ineq,A_eq,b_eq);
if exitflag~=1
    warning(['Impossible to find a feasible solution for initial heading '...
         ,num2str(theta_i),', final heading ',num2str(theta_f), ...
         ' and displacement (',num2str(delta_x),',',num2str(delta_y),')']);
    l1=0;
    R=0;
    l2=0;
    status=0;
else
    l1=x(1);
    R=x(2);
    l2=x(3);
    status=1;
end