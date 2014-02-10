function T = random_pose( Dangl,Dpos )
%RANDOM_POSE generates a random pose with a max variation in all angles and poses
%  random_pose( Dangl,Dpos )
%  generate poses with angles in btw [-Dpos/2,Dpos/2]
%  and [-Dangl/2,Dangl/2]

if (nargin() == 2)
    T= [rpy2r(Dangl*(0.5-(rand([1,3])))),Dpos*(0.5-(rand(3,1)));[0 0 0 1]];
elseif  (nargin() == 1)
    T= [rpy2r(Dangl*(0.5-(rand([1,3])))),[0 0 0]';[0 0 0 1]];
else
    error('must have 1 or 2 args');
end

end

