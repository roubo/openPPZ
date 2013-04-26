clear();
getf('mb_utils.sci');

load('averaged_ramp_geared.dat','av_throttle','av_rpm')   

t_part = av_throttle(20:90);
r_part = av_rpm(20:90);

p0=[0.00001; 20000];
Z = [r_part; t_part];
//[p, err] = datafit(err_mot_lin, Z, p0); 

[p, err] = datafit(err_mot_ric, Z, p0) 
//p = p0;

xbasc();
//subplot(3,1,1);
xtitle('Rpms vs Throttle');
plot2d(av_throttle,av_rpm);
plot2d(t_part,r_part, 4);


r_fit = [];
for i=1:length(av_throttle)
  //  r_fit = [r_fit mot_lin(av_throttle(i), p)];
  r_fit = [r_fit mot_ric(av_throttle(i), p)];
end

plot2d(av_throttle,r_fit, 3);