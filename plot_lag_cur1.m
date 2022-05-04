function plot_lag_cur1(phy,NE,tf)
hold on    
for ii = 1 : NE
    
    lb = tf / NE * (ii-1);
    ub = tf / NE * ii;
    
    data = phy(ii,1:3);
    tttemp = lb + (ub-lb).*[0.1550510257216822, 0.6449489742783178, 1];
    
    plot(tttemp , data, 'r');
    
    if (ii >= 2)
        tttemp = lb + (ub-lb).*[0, 0.1550510257216822];
        data = [phy(ii-1,3), data(1,1)];
        plot(tttemp , data, 'r');
    end

end