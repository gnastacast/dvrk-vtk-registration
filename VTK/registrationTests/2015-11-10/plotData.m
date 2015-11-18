%close('all')
clear;
percentages = [];

actual = dlmread('reg_actual.txt');
manual = dlmread('reg_manual.txt');
anneal = dlmread('reg_anneal.txt');

numReg = length(actual);
numAnneal = length(anneal)/numReg;

figure();
for trialIdx = 1:numReg
    medDev(trialIdx,:) = mad(anneal((trialIdx-1)*numAnneal+1:trialIdx*numAnneal,1:6));
    wrong = 0;
    right = 0;
    for idx = (trialIdx-1)*numAnneal+1:trialIdx*numAnneal
        if (abs(anneal(idx,:)-actual(trialIdx))<[1 1 1 1 1 1])
            right = right+1;
        else
            wrong = wrong+1;
        end
    end
    plot = subplot(numReg,3,trialIdx*3);
    bar(medDev(trialIdx,:));
    set(gca,'XTickLabel',{'dX','dY','dZ','tX','tY','tZ'})
    %ylim(plot,[0 1.5]);
    plot2 = subplot(numReg,3,trialIdx*3-1);
    errors(trialIdx,:) = abs(manual(trialIdx,:)-actual(trialIdx,:));
    posError = mean(errors(trialIdx,1:3));
    rotError = mean(errors(trialIdx,1:3));
    bar([posError rotError]);
    set(gca,'XTickLabel',{'Position Error','Rotation Error'})
    ylim(plot2,[0 8]);
    
    plot3 = subplot(numReg,3,trialIdx*3-2);
    hold('on')
    positionErrors = [];
    for idx = (trialIdx-1)*numAnneal+1:trialIdx*numAnneal
        positionErrors = horzcat(positionErrors, anneal(idx,1:3)-actual(trialIdx,1:3));
    end
    hist(positionErrors,50);
    ylim(plot3,[0 50]);
    %xlim(plot3,[-2 2]);
    hold('off')
     %title(files{trialIdx}(7:end-9))
      
%     %scatter3(M(:,1),M(:,2),M(:,3))
%     percentages = [percentages,right/length(M)];
end
% 
% figure();
% subplot(2,1,1)
% hold('on')
% positionErrors = [];
% for trialIdx = 1:numReg
%     for idx = (trialIdx-1)*numAnneal+1:trialIdx*numAnneal
%         positionErrors = horzcat(positionErrors, anneal(idx,1:3)-actual(trialIdx,1:3));
%     end
% end
% hist(positionErrors);
% 
% subplot(2,1,2)
% hold('on')
% rotationErrors = [];
% for trialIdx = 1:numReg
%     if (trialIdx == 6 || trialIdx == 10)
%         continue
%     end
%     for idx = (trialIdx-1)*numAnneal+1:trialIdx*numAnneal
%         rotationErrors = horzcat(rotationErrors, anneal(idx,4:6)-actual(trialIdx,4:6));
%     end
% end
% hist(rotationErrors);