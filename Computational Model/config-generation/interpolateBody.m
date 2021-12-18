function body = interpolateBody(vertices, n)
    d = 1/n:1/n:1;
    body = zeros(3, size(vertices, 2)*n);
    for i = 1:size(vertices, 2)
        p1 = vertices(:,i);
        p2 = vertices(:,1+mod(i,size(vertices,2)));
        body(:,1+(i-1)*n:i*n) = p1 + (p2-p1).*d;
    end
end