% About 4-5 seconds on average

M = dlmread('tests_80x60_5000.txt');

figure1 = figure(1);
axes1 = axes('Parent',figure1,'PlotBoxAspectRatio',[1 1 1]);
xlim(axes1,[-180 180]);
ylim(axes1,[-180 180]);
zlim(axes1,[-180 180]);
view(axes1,[136 26]);
hold(axes1,'all');

%scatter3(M(:,1),M(:,2),M(:,3),'b');
std(M)

% Between 6-8 seconds

M = dlmread('tests_160x120_5000.txt');
hold('on')
scatter3(M(:,1),M(:,2),M(:,3),'r');
std(M)

% Between 13-19 seconds

M = dlmread('tests_320x240_5000.txt');

scatter3(M(:,1),M(:,2),M(:,3),'g');
std(M)

% Between 37-54 seconds

M = dlmread('tests_640x480_5000.txt');

scatter3(M(:,1),M(:,2),M(:,3),'g');
std(M)