clc;
clear;
g0 = 9.7803266714;
load t;
[t,Gyro_merr(:,1)] = ode45(@gyro_markov,t,0.01/57.3*randn(1));
Gyro_werr(:,1) = 0.01/57.3*randn(N,1);

[t,Gyro_merr(:,2)] = ode45(@gyro_markov,t,0.01/57.3*randn(1));
Gyro_werr(:,2) = 0.01/57.3*randn(N,1);

[t,Gyro_merr(:,3)] = ode45(@gyro_markov,t,0.01/57.3*randn(1));
Gyro_werr(:,3) = 0.01/57.3*randn(N,1);

[t,Acc_merr(:,1)] = ode45(@acc_markov,t,1e-3*g0*rand(1));

[t,Acc_merr(:,2)] = ode45(@acc_markov,t,1e-3*g0*rand(1));

[t,Acc_merr(:,3)] = ode45(@acc_markov,t,1e-3*g0*rand(1));
save Gyro_werr;
save Gyro_merr;
save Acc_merr;