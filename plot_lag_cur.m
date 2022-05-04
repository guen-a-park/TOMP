function plot_lag_cur(phy,NE,tf)

hold on    
for ii = 1 : NE
    
    lb = tf / NE * (ii-1);
    ub = tf / NE * ii;
    
    data = phy(ii,1:4);
    tttemp = lb + (ub-lb).*[0, 0.1550510257216822, 0.6449489742783178, 1];
    
    ytemp = lagrange(tttemp, data, [lb:0.001:ub]);
    plot([lb:0.001:ub],ytemp);

end