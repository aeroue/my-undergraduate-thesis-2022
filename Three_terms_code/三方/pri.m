    
for i = 1:14
        h=figure(i);
        set(h,'PaperPositionMode','manual');
        set(h,'PaperUnits','points');
        set(h,'PaperPosition',[0,0,350,262.5]);%恰当选择尺寸
        set(gca,'FontSize',14);
        filename = ['results_ab正常',num2str(i)];
        print(h,['./111/',filename,'.jpg'],'-r600','-djpeg');%-r600可改为300dpi分辨率
    end