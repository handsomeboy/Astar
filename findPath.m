function path = findPath(cameFrom, currIdx,fScore)
%从终点逐渐找到起点

if  isinf(fScore(currIdx))
    path = [];
else
    path = currIdx;
    
    while cameFrom(path(1)) 
        path = [cameFrom(path(1)), path];
    end
end