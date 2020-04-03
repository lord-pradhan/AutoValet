function primitives=generate_primitives(resolution,num_angles,points,costs,Rmin)

primitives.resolution=resolution;
primitives.num_angles=num_angles;
primitives.num_prim=size(points,1);
primitives.num_samples=32;
primitives.trajectories=[];

for angleind = 1:num_angles
    %iterate over primitives    
    for primind = 1:size(points,1)
        %current angle
        currentangle = (angleind-1)*2*pi/num_angles;
        primitives.trajectories((angleind-1)*size(points,1)+...
            primind).start_angle=angleind-1;% in discretized states
        primitives.trajectories((angleind-1)*size(points,1)+...
            primind).id=primind-1;
                
        traj_points(primind,1)=points(primind,1)*cos(currentangle)-...
            points(primind,2)*sin(currentangle);
        traj_points(primind,2)=points(primind,1)*sin(currentangle)+...
            points(primind,2)*cos(currentangle);
        traj_points(primind,3)=points(primind,3)+currentangle;
 
        % find the closest point on the resolution grid
        res_points(primind,1)=round(traj_points(primind,1)/resolution)*...
            resolution;
        res_points(primind,2)=round(traj_points(primind,2)/resolution)*...
            resolution;
        res_points(primind,3)=traj_points(primind,3);
        
        primitives.trajectories((angleind-1)*size(points,1)+...
            primind).endpose=zeros(1,3);
        primitives.trajectories((angleind-1)*size(points,1)+...
            primind).endpose(1:2)=round(res_points(primind,1:2)./resolution);
        primitives.trajectories((angleind-1)*size(points,1)+...
            primind).endpose(3)=round(rem(res_points(primind,3)*...
            num_angles/(2*pi),num_angles));
        primitives.trajectories((angleind-1)*size(points,1)+...
            primind).cost=costs(primind);
 
        if(points(primind,3)==0)% straight segments
            l1=points(primind,1)/2;
            l2=points(primind,1)/2;
            R=0;
            status=1;
        else
            [l1 R l2 status]=find_traj_params(currentangle,...
                res_points(primind,3),res_points(primind,1),...
                res_points(primind,2),Rmin);
        end
        if(status==1)
            primitives.trajectories((angleind-1)*size(points,1)+...
                primind).points=generate_traj(l1,R,l2,0,0,currentangle,...
                res_points(primind,1),res_points(primind,2),...
                res_points(primind,3),primitives.num_samples);
        
            plot(primitives.trajectories((angleind-1)*size(points,1)+...
                primind).points(:,2),primitives.trajectories((angleind-1)*...
                size(points,1)+primind).points(:,1));
            grid on
            hold on;
        else
            primitives.trajectories((angleind-1)*size(points,1)+...
                primind).points=[];
        end
    end
end
