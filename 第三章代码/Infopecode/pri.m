
    for i = 1:19
        h=figure(i);
        set(h,'PaperPositionMode','manual');
        set(h,'PaperUnits','points');
        set(h,'PaperPosition',[0,0,400,300]);%恰当选择尺寸
        filename = ['无限机动',num2str(i)];
        print(h,['./111/',filename,'.jpg'],'-r600','-djpeg');%-r600可改为300dpi分辨率
    end