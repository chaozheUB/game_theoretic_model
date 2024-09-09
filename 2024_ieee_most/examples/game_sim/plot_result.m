function plot_result(results)
Util.plot_setup();
color_pacc = [0.929411764705882,0.694117647058824,0.125490196078431];
color_ego_lead = "b";
% [wpacc, spacc, hpacc, vpacc, apacc, pred_err_pacc, tsim] = Util.load_result(result_name);
% wpacc = results.w;
% spacc = results.s;
hpacc = results.h;
vpacc = results.v;
apacc = results.a;
tsim = results.t;
pred_err_s_pacc = results.s1pred_err;
% s1 = results.s1;
v1 = results.v1;


tmax = tsim(end);
figure;
subplot(4,1,1);
plot(tsim, hpacc, 'color', color_pacc); hold on;

pbaspect([3,1,1]);
xlim([0, tmax]);
ylim([0, 120]);
grid on;

subplot(4,1,2);
plot(tsim, vpacc, 'color', color_pacc); hold on;
plot(tsim, v1, 'color', color_ego_lead); hold on;
pbaspect([3,1,1]);
xlim([0, tmax]);
ylim([0, 40]);
grid on;

subplot(4,1,3);
plot(tsim, apacc, 'color', color_pacc); hold on;
pbaspect([3,1,1]);
xlim([0, tmax]);
ylim([-8, 4]);
yticks(-8: 4: 4);
grid on;

% subplot(4,1,4);
% plot(tsim, nh_est, 'color', color_pccc); hold on;
% yline(LL-2, 'k--', 'LineWidth', 1.5);
% xlim([0, tmax]);
% ylim([0, 6]);
% yticks([0, 4]);
% pbaspect([3,1,1]);
% grid on;



%% h - v diagram
set(0,'defaultAxesFontSize', 14);
grey = 0.618 * [1,1,1];
hrange = [0, 120];
vrange = [0, 40];
hst=5;
kappa=0.6;
vmax=35;
hgo=vmax/kappa+hst;
V=@(h) (hst<=h & h<=hgo)*kappa.*(h-hst)+(h>hgo).*vmax;
Tmin = 0.67;
dmin = 3;
T = 1.67;
dst = 5;
hvrange = hrange(1): 0.01: hrange(end);

figure;
plot(hpacc, vpacc, 'color', color_pacc); hold on;
plot(hpacc(1), vpacc(1), '-x', 'color', color_pacc); hold on;
plot(hvrange, min((hvrange - dmin) / Tmin, vmax), '--', 'color', grey);
plot(hvrange, min((hvrange - dst) / T, vmax), 'k--');
grid on;
pbaspect([2,1,1]);
xlim(hrange);
ylim(vrange);

%% Prediction Error
figure;
set(0,'defaultAxesFontSize', 26);
nlevel = 20;
cmin = -220;
cmax = 220;
subplot(1,2,1);
[~,h] = contourf(pred_err_s_pacc, nlevel);
clim([cmin, cmax]);
colorbar('XTick', -200: 100: 200);
set(h,'LineColor','none');
xticks([1, 1000, 2000, 3000, 4000]);
xticklabels(["0", "100", "200", "300", "400"]);
yticks([1, 40, 80, 120, 160]);
yticklabels(["0", "4", "8", "12", "16"]);
pbaspect([2,1,1]);
end