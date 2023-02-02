% Code is binary values for: [Tail (1), back legs (6), body joint (3), 
% middle legs (6), body joint (3), front legs (6)]
% body width/height are [3x1], leg length is [3x2], knee ratio is [3x2],
% tail length is scalar
function config = walker(code, w, h, l, k, t)
    if ~isnumeric(code)
        code = code == '1';
    end
    N = 1+(length(code)-7)/9;
    config.bodies = {getSegment(w(1), h(1))};
    config.count = 0;
    config.joints = zeros(3, 3, sum(code));
    parent = 1;
    
    % Tail
    if code(1)
        config.count(parent) = config.count(parent)+1;
        config = [config, 0];
        config.joints(:,:,1) = NaN;
    end
    
    % Iterate over segments
    for seg = 1:N
        
    end
end

function body = getSegment(w, h)
    body = [-w/2,w/2,w/2,-w/2; h/2,h/2,-h/2,-h/2; 0,0,0,0];
end