function writeAnimation(M, name)
    minx = 100000;
    miny = 100000;
    for i = 1:length(M)
        if size(M(i).cdata, 1) < minx
            minx = size(M(i).cdata, 1);
        end
        if size(M(i).cdata, 2) < miny
            miny = size(M(i).cdata, 2);
        end
    end
    for i = 1:length(M)
        if size(M(i).cdata, 1) > minx
            M(i).cdata = M(i).cdata(1:minx,:,:);
        end
        if size(M(i).cdata, 2) > miny
            M(i).cdata = M(i).cdata(:,1:miny,:);
        end
    end
    myVideo = VideoWriter(name, 'Uncompressed AVI');
    myVideo.FrameRate = 10;
    open(myVideo);
    writeVideo(myVideo, M);
    close(myVideo);
end