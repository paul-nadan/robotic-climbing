% USAGE: saveAnimation(FRAMES, 'Filename')
function saveAnimation(M, name, framerate)
    if nargin < 3
        framerate = 10;
    end
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
    myVideo = VideoWriter(['results/animations/' name], 'Uncompressed AVI');
    myVideo.FrameRate = framerate;
    open(myVideo);
    writeVideo(myVideo, M);
    close(myVideo);
end