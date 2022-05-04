set(0,'DefaultLineLineWidth',2.5);

load v.txt
v = (reshape(v,round(length(v)/NE),NE))';
subplot(2,3,1)
box on
plot_lag_cur(v,NE,tf)
xlabel('Time / sec','Fontsize',20,'FontWeight','bold')
ylabel('v(t) / (m/s)','Fontsize',20,'FontWeight','bold')
set(gca,'FontSize',12,'FontWeight','bold')
axis tight
grid on


load p.txt
p = (reshape(p,round(length(p)/NE),NE))';
subplot(2,3,2)
box on
plot_lag_cur(p.*180./pi,NE,tf);
xlabel('Time / sec','Fontsize',20,'FontWeight','bold')
ylabel('\phi(t) / deg','Fontsize',20,'FontWeight','bold')
set(gca,'FontSize',12,'FontWeight','bold')
grid on
axis tight
grid on


load t.txt
theta = (reshape(t,round(length(t)/NE),NE))';
subplot(2,3,3)
plot_lag_cur(theta.*180./pi,NE,tf)
xlabel('Time / sec','Fontsize',20,'FontWeight','bold')
ylabel('\theta(t) / deg','Fontsize',20,'FontWeight','bold')
set(gca,'FontSize',12,'FontWeight','bold')
axis tight
grid on


load a.txt
a = (reshape(a,round(length(a)/NE),NE))';
subplot(2,3,4)
box on
plot_lag_cur(a,NE,tf)
xlabel('Time / sec','Fontsize',20,'FontWeight','bold')
ylabel('a(t) / (m/s^2)','Fontsize',20,'FontWeight','bold')
set(gca,'FontSize',12,'FontWeight','bold')
axis tight
grid on





set(0,'DefaultLineLineWidth',1.5);
load o.txt
o = (reshape(o,round(length(o)/NE),NE))';
subplot(2,3,5)
box on
plot_lag_cur1(o,NE,tf)
xlabel('Time / sec','Fontsize',20,'FontWeight','bold')
ylabel('\omega(t) / (rad/s)','Fontsize',20,'FontWeight','bold')
set(gca,'FontSize',12,'FontWeight','bold')
axis tight
grid on



load jerk.txt
jerk = (reshape(jerk,round(length(jerk)/NE),NE))';
subplot(2,3,6)
box on
plot_lag_cur1(jerk,NE,tf)
xlabel('Time / sec','Fontsize',20,'FontWeight','bold')
ylabel('jerk(t) / (m/s^3)','Fontsize',20,'FontWeight','bold')
set(gca,'FontSize',12,'FontWeight','bold')
axis tight
grid on