function xdot = dys(t,x,I,tau_r)

q = x(1:4);
qn = norm(q);
q = q/qn;
w = x(5:7);

%Calculate omega derivative from Euler's equation + torques
wdot = I\(-hat(w)*I*w + tau_r);

%Calculate quaternion derivative from omega
qdot = .5*[hat(q(1:3)) + q(4)*eye(3); -q(1:3)']*w; %- (qn-1)*q;

xdot = [qdot; wdot];

end
