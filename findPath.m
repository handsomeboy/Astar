function path = findPath(cameFrom, currIdx,fScore)
%���յ����ҵ����

if  isinf(fScore(currIdx))
    path = [];
else
    path = currIdx;
    
    while cameFrom(path(1)) 
        path = [cameFrom(path(1)), path];
    end
end