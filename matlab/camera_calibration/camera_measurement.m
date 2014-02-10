function cn_T_mn = camera_measurement( w_T_mr,w_T_cr,noise_ang,noise_pos   )
%CAMERA_MEASUREMENT returns the nominal pose of the marker w.r.t. the camera frame.
%   parameters:
%   w_T_mr: nominal position of the marker
%   w_T_cr: nominal position of the camera
%   noise_ang and noise_pos: parameters that modulate noise on measuremnt 

if (nargin() == 2) 
   T_noise=eye(4);
elseif  (nargin() == 4) 
    T_noise=random_pose(noise_ang,noise_pos);
else 
   error('must have 4 or 2 args'); 
end 

 cn_T_mn= inv(w_T_cr)*w_T_mr*T_noise;
end

