% Script by: Nicholas Granlund
close all
clear all
clc

% Parameters
mu = [0; 10];
Sigma = [0.3, 0; 0, 8];
level = 3;
npoints = 128;

A = [1, 0.5; 0, 1];

% 
[ xy_q ] = sigmaEllipse2D( mu, Sigma, level, npoints );

% Affine transformation
[mu_y, Sigma_y] = affineGaussianTransform(mu, Sigma, A, [0; 0])
[ xy_z ] = sigmaEllipse2D( mu_y, Sigma_y, level, npoints );


% Plot
figure()
plot(xy_q(1,:),xy_q(2,:),'linewidth',2)
axis equal
hold on
grid on
plot(mu(1), mu(2), '*')

plot(xy_z(1,:),xy_z(2,:),'linewidth',2)
axis equal
hold on
grid on
plot(mu_y(1), mu_y(2), '*')
legend('3\sigma - curve for q', 'mean q','3\sigma - curve for z','mean z')
xlabel('x_1'); ylabel('x_2')

%% 2 a)
close all
clear all
clc

% Define gaussian variable x
mu_x = 0;
sigma_x = 2;
N = 50000;

% Analytically calculate z
[mu_z1, sigma_z1] = affineGaussianTransform(mu_x, sigma_x, 3, 0)
x_s1 = mvnrnd(mu_z1, sigma_z1, N)';


% Nummerically calculate z
[mu_z2, sigma_z2, ~] = approxGaussianTransform(mu_x, sigma_x, @(x) 3*x, N)
x_s2 = mvnrnd(mu_z2, sigma_z2, N)';


figure()
axi = -15:1:15;
subplot(2,1,1)
histogram(x_s1,50,'Normalization','pdf')
hold on
plot(axi,normpdf(axi,mu_z1,sqrt(sigma_z1)))
title('Analytical pdf')


subplot(2,1,2)
histogram(x_s2,50,'Normalization','pdf')
hold on
plot(axi,normpdf(axi,mu_z2,sqrt(sigma_z2)))
title('approximated pdf')



%% 2 b)
close all
clear all
clc

% Define gaussian variable x
mu_x = 0;
sigma_x = 2;
N = 50000;

% Nummerically calculate z
[mu_z2, sigma_z2, ~] = approxGaussianTransform(mu_x, sigma_x, @(x) x.^3, N)
x_s2 = mvnrnd(mu_z2, sigma_z2, N).^3;

% plot
figure()
axi = -10:0.1:10;
histogram(x_s2,100,'Normalization','pdf')
hold on
plot(axi,normpdf(axi,mu_z2,sqrt(sigma_z2)),'linewidth',2)
title('approximated pdf')








%% 3 a)
close all
clear all
clc


% function 
h = @(x) 5*x + 7;
N = 10000;
y = zeros(1,N);

for i=1:N
    % sample h(x)
    x = rand() -0.5;
    h_sample = h(x);
    
    % sample r
    r_sample = mvnrnd(0, 1^2, 1);

    y(i) = h_sample + r_sample;
end


% plot
figure()
axi = -0:0.1:15;
histogram(y,100,'Normalization','pdf')
hold on

dist = normpdf(axi, mean(y) , sqrt(var(y)));
plot(axi,dist,'linewidth',2)
mean(y)












%% 4 a)
close all
clear all
clc

% Draw samples
N = 100000;
y = zeros(1,N);

for i=1:N
    % sample theta
    theta_sample = randsample([-1 1],1);
    
    % sample w
    w_sample = mvnrnd(0, 0.5^2,1);

    y(i) = theta_sample + w_sample;
end

% plot
figure()
axi = -3:0.1:3;
histogram(y,100,'Normalization','pdf')
hold on

dist = 0.5*normpdf(axi, -1 , 0.5) + 0.5*normpdf(axi, 1 , 0.5);
plot(axi,dist,'linewidth',2)
title('p(y) = 0.5p(\theta) + 0.5p(w)','fontsize',12)








