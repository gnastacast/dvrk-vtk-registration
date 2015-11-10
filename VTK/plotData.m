files = {'tests_80x60_5000.txt', ... % About 4-5 seconds on average
         'tests_160x120_5000.txt', ... % Between 6-8 seconds
         'tests_320x240_5000.txt', ... % Between 13-19 seconds
         'tests_640x480_5000.txt'}; % Between 37-54 seconds

%figure1 = figure();
%axes1 = axes('Parent',figure1,'PlotBoxAspectRatio',[1 1 1]);
%xlim(axes1,[-180 180]);
%ylim(axes1,[-180 180]);
%zlim(axes1,[-180 180]);
%view(axes1,[136 26]);
%hold(axes1,'all');

close('all')
percentages = [];

for fileIdx = 1:length(files)
    M = dlmread(files{fileIdx});
    mad(M,1)
    wrong = 0;
    right = 0;
    for idx = 1:length(M)
        if (abs(M(idx,:)-[10 -130 -170])<[1 1 1])
            right = right+1;
        else
            wrong = wrong+1;
        end
    end
    figure();
    subplot(3,1,1);
    hist(M(:,1),1000);
    title(files{fileIdx}(7:end-9))
    subplot(3,1,2);
    hist(M(:,2),1000);
    subplot(3,1,3);
    hist(M(:,3),1000);
    
    %scatter3(M(:,1),M(:,2),M(:,3))
    percentages = [percentages,right/length(M)];
end

figure();
bar(percentages)
set(gca,'XTickLabel',{'80x60','160x120','320x240','640x480'})
ylabel('Ratio within 1 degree')
xlabel('Resolution')
<<<<<<< HEAD
title('Results of registration from 100 random orientations')
=======
title('Results of registration from 100 random orientations')
>>>>>>> f18e40d0116b5fb903da6c483b85c0afdf0d7c6f
