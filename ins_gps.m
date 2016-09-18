clear;
clc;
global step;
global Rm;
global Rn;
global weie;
imu = load('imu.txt');
pos_vel = load('pos_vel.txt');
atti = load('dat_atti.txt');
% a = find(atti(:,4)<0);
% atti(a,4) = atti(a,4) + 2*pi;
Re=6378137;
g0 = 9.7803266714;
e = 0.0033528106647474807;
weie = 0.7292115147e-4;
deg_rad = pi/180;
N = size(imu,1);
dpr2rps = (pi/180)/3600;
step = 0.01;

t(:,1) = step*imu(:,1);
%带误差的惯性测量元件和初值
save t;
load Gyro_merr;
load Gyro_werr;
load Acc_merr;

wbib(:,1) = imu(:,2) + Gyro_merr(:,1) + Gyro_werr(:,1);
wbib(:,2) = imu(:,3) + Gyro_merr(:,2) + Gyro_werr(:,2);
wbib(:,3) = imu(:,4) + Gyro_merr(:,3) + Gyro_werr(:,3);
gyro_mvari = [(0.01/57.3)^2 (0.01/57.3)^2 (0.01/57.3)^2];
gyro_wvari = [(0.01/57.3)^2 (0.01/57.3)^2 (0.01/57.3)^2];


fb(:,1) = imu(:,5) + Acc_merr(:,1);
fb(:,2) = imu(:,6) + Acc_merr(:,2);
fb(:,3) = imu(:,7) + Acc_merr(:,3);
acc_mvari = [(1e-3*g0)^2 (1e-3*g0)^2 (1e-3*g0)^2];
% 
% save('wbib_error.txt','wbib','-ascii');
% save('fb_error.txt','fb','-ascii');
ins_pos = zeros(N,3);
ins_vel = zeros(N,3);
ins_atti = zeros(N,3);

ins_pos(1,1) = pos_vel(1,6);
ins_pos(1,2) = pos_vel(1,5);
ins_pos(1,3) = pos_vel(1,7);

ins_vel(1,1) = pos_vel(1,2);
ins_vel(1,2) = pos_vel(1,3);
ins_vel(1,3) = pos_vel(1,4);

ins_atti(1,1) = atti(1,2);
ins_atti(1,2) = atti(1,3);
ins_atti(1,3) = atti(1,4);


%理想状况的初值

% wbib(:,1:3) = imu(:,2:4);
% fb(:,1:3) = imu(:,5:7);
% ins_pos = zeros(N,3);
% ins_vel = zeros(N,3);
% ins_atti = zeros(N,3);
% 
% ins_pos(1,1) = 48*deg_rad;
% ins_pos(1,2) = 125*deg_rad;
% ins_pos(1,3) = 5000;
% 
% ins_vel(1,1) = 0;
% ins_vel(1,2) = 0;
% ins_vel(1,3) = 0;
% 
% ins_atti(1,1) = 5*deg_rad;
% ins_atti(1,2) = 5*deg_rad;
% ins_atti(1,3) = 5*deg_rad;
%initialize attitude matrix
atti = ins_atti(1,:);
cnb = a2cnb(atti);
cbn = cnb';
%initialize quart number
q = cnb2quat(cnb);
%initialize position matrix

cen = pos2cen(ins_pos(1,:));
cne = cen';

G = zeros(3,1);
wbnb = zeros(3,1);

%gps位置带误差的模拟
gps_error(:,1:2) = 20*randn(N,2)/Re;
gps_error(:,3) = 40*randn(N,1);
gps_pos(:,1) = pos_vel(:,6) + gps_error(:,1);
gps_pos(:,2) = pos_vel(:,5) + gps_error(:,2);
gps_pos(:,3) = pos_vel(:,7) + gps_error(:,3);
% save gps_pos.txt gps_pos -ascii;
%gps速度带误差模拟
gps_vel_error = 15*randn(N,3);
gps_vel = pos_vel(:,2:4) + gps_vel_error;
%量测噪声方差矩阵


%卡尔曼滤波初始化
X = zeros(18,1);    %状态变量初始化
qq = 0.1*dpr2rps;

Q = diag([gyro_wvari, gyro_mvari, acc_mvari]);

P(1:18,1:18) = diag([1 1 30,20 20 20, 0.1 0.1 0.1,  ...
    0.001 0.001 0.001, 0.001 0.001 0.001 0.001 0.001 0.001]);    %误差方差矩阵

H = zeros(3,18);
H(1:3,1:3) = diag([1 1 1]);

GG = zeros(18,9);
GG(13:18,4:9) = eye(6,6);

R = diag(std(gps_error).^2);    
%经过卡尔曼滤波后位速姿
ins_gps_pos = zeros(N,3);
ins_gps_atti = zeros(N,3);
ins_gps_vel = zeros(N,3);
ins_gps_pos(1,:) = ins_pos(1,:);
ins_gps_atti(1,:) = ins_atti(1,:);
ins_gps_vel(1,:) = ins_vel(1,:);
for i = 2 : N
    cbn0 = cbn; %前一时刻的姿态变换矩阵
%     cnb0 = cnb;
    Rm  = Re + ins_gps_pos(i-1,3);
    Rn  = Re + ins_gps_pos(i-1,3);
    
%     Rm  = Re + ins_pos(i-1,3);
%     Rn  = Re + ins_pos(i-1,3);
    g = g0*(1 + 0.00193185138639*cen(3,3)^2)/sqrt(1-0.00669437999013*cen(3,3)^2);
    G(3,1) = -g;
    %地固系相对惯性系在导航系角速度
    wnie(1,1) = 0;
    wnie(2,1) = weie*cen(2,3);
    wnie(3,1) = weie*cen(3,3);
    %导航系相对地固系角速度在导航系角速度
    wnen(1,1) = -ins_gps_vel(i-1,2)/Rm;
    wnen(2,1) = ins_gps_vel(i-1,1)/Rn;
    wnen(3,1) = ins_gps_vel(i-1,1)*tan(ins_gps_pos(i-1,1))/Rn;
    
%     wnen(1,1) = -ins_vel(i-1,2)/Rm;
%     wnen(2,1) = ins_vel(i-1,1)/Rn;
%     wnen(3,1) = ins_vel(i-1,1)*tan(ins_pos(i-1,1))/Rn;
    fn = cbn*fb(i-1,:)';
    %update velocity
    dvt = fn - cross((2*wnie + wnen),ins_gps_vel(i-1,:)') + G;
%     dvt = fn - cross((2*wnie + wnen),ins_vel(i-1,:)') + G;
    ins_vel(i,:) = ins_vel(i-1,:) + step*dvt';
    %update position matrix
    cen = update_cen(cen,wnen);
    %update position
    pos = cen2pos(cen);
%     ins_pos(i,3) = ins_pos(i-1,3) + step*ins_vel(i-1,3); 
    pos(3) = ins_pos(i-1,3) + step*ins_vel(i-1,3); 
    ins_pos(i,:) = pos;
%     ins_pos(i,3) = gps_pos(i,3);
    %update quart number   
    wbnb = wbib(i-1,:)' - cnb*(wnie + wnen);
    q = update_quat(q,wbnb);
    
    %update attitude matrix
    cnb = quat2cnb(q);
    cbn = cnb';
    atti = cnb2atti(cnb);
    ins_atti(i,:) = atti;
    %update attitude
    %卡尔曼滤波
    F = getf(fn,ins_vel(i-1,:),cbn0,ins_pos(i-1,1));
%     GG(4:6,1:3)   =  cbn0;
    GG(7:9,1:3) = cbn0;
  
    A = eye(18,18) + F*step + F*F*step*step/2;
    B = (eye(18,18)+step*F/2)*GG*step;
%     M = GG*Q*GG';
%     QQ = M*step + (F*M + (F*M)')*step*step/2;
%     H(1:3,7:9) = diag([1 1 1]);
%     H(4:6,1:3) = diag([1 1 1]);
    %过程转移矩阵离散化
%     A = eye(15,15) + F*step;
%     B = (eye(15,15)+step*F/2)*GG*step;
%     量测量
    Z(1:3,1) = (ins_pos(i,:) - gps_pos(i,:))';
    %     Z(4:6,1) = (ins_vel(i,:) - gps_vel(i,:))';
%     标准卡尔曼滤波
%     %filtering
    
    [X,P] = kf(A,B*Q*B',P,X,H,R,Z);

    ins_gps_pos(i,:) = ins_pos(i,:) - X(1:3,1)';
    ins_gps_vel(i,:) = ins_vel(i,:) - X(4:6,1)';
    
    phiX = [0 -X(9,1) X(8,1);
        X(9,1) 0 -X(7,1);
        -X(8,1) X(7,1) 0];
    cbn = (eye(3,3) + phiX)*cbn;
    cnb = cbn';
    ins_gps_atti(i,:) = cnb2atti(cnb);
   
end


figure;
subplot(311);
plot(t,(ins_pos(:,1) - pos_vel(:,6))*57.3,t,(ins_gps_pos(:,1) - pos_vel(:,6))*57.3,'r');
subplot(312);
plot(t,(ins_pos(:,2) - pos_vel(:,5))*57.3,t,(ins_gps_pos(:,2) - pos_vel(:,5))*57.3,'r');
subplot(313);
plot(t,ins_pos(:,3) - pos_vel(:,7),t,ins_gps_pos(:,3) - pos_vel(:,7),'r');

figure;
subplot(311);
plot(t,ins_vel(:,1) - pos_vel(:,2),t,ins_gps_vel(:,1) - pos_vel(:,2),'r');
subplot(312);
plot(t,ins_vel(:,2) - pos_vel(:,3),t,ins_gps_vel(:,2) - pos_vel(:,3),'r');
subplot(313);
plot(t,ins_vel(:,3) - pos_vel(:,4),t,ins_gps_vel(:,3) - pos_vel(:,4),'r');

figure;
subplot(311);
plot(t,(ins_atti(:,1) - atti(:,2))*57.3,t,(ins_gps_atti(:,1) - atti(:,2))*57.3,'r');
subplot(312);
plot(t,(ins_atti(:,2) - atti(:,3))*57.3,t,(ins_gps_atti(:,2) - atti(:,3))*57.3,'r');
subplot(313);
plot(t,(ins_atti(:,3) - atti(:,4))*57.3,t,(ins_gps_atti(:,3) - atti(:,4))*57.3,'r');

% figure;
% subplot(311);
% plot(t,ins_pos(:,1) - pos_vel(:,6));
% subplot(312);
% plot(t,ins_pos(:,2) - pos_vel(:,5));
% subplot(313);
% plot(t,ins_pos(:,3) - pos_vel(:,7));
% 
% figure;
% subplot(311);
% plot(t,ins_vel(:,1) - pos_vel(:,2));
% subplot(312);
% plot(t,ins_vel(:,2) - pos_vel(:,3));
% subplot(313);
% plot(t,ins_vel(:,3) - pos_vel(:,4));
% 
% 
% figure;
% subplot(311);
% plot(t,ins_atti(:,1) - atti(:,2));
% subplot(312);
% plot(t,ins_atti(:,2) - atti(:,3));
% subplot(313);
% plot(t,ins_atti(:,3) - atti(:,4));