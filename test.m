
% h = animatedline();
% for K = 0:0.01:1
%     [Fx,Fy] = tire_dyn(K,0.5,10, 12000,12000,0.2);
%     addpoints(h, K, Fx);
% end

[Fx,Fy] = tire_dyn(1.2,0.5,10, 12000,12000,0.2)
% [Fx,Fy] = tire_dyn_test(-1.2,0.5,10, 12000,12000,0.2)