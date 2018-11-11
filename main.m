% 我的A*算法
% jack0077555@163.com
% 个人网站：www.deanhan.com
% 个人公众号：深度机器学习
% 参考： https://en.wikipedia.org/wiki/A*_search_algorithm

clear all
clc
%% 人工  设置地图和障碍
rows = 21;
cols = 28;      %行数和列数

map=ones(rows,cols);%行数和列数

map(4,4:8)=0;
map(8,4:8)=0;
map(4:8,8)=0;
map(12,1:end-1)=0;
map(2:end-1,19)=0;%墙

start=[6,6];
goal=[17,13];%设置起点和终点

%%
startIndex=sub2ind([rows,cols],start(1),start(2));
goalIndex=sub2ind([rows,cols],goal(1),goal(2));

stepLen1=10;%水平或垂直的cost
stepLen2=14;%斜向的cost

%% 相关的矩阵

closeMat = false(rows,cols);%闭mat
openMat  = false(rows,cols);%开mat


gScore = inf(rows,cols);
fScore = inf(rows,cols);%初始化全部为inf

cameFrom =zeros(rows,cols);%节点的父节点，线性坐标

%% 设置启发式距离矩阵


[Y,X]=meshgrid(1:cols,1:rows);

hScore=stepLen1 * max(abs(X-goal(1)),  abs(Y-goal(1))); %这里用到的是L-inf范数

%% 设置起点

openMat(startIndex)=true;%开始的时候只有起点在open里面

gScore(startIndex)=0;                  % 起点的g值
fScore(startIndex)=hScore(startIndex); % 起点的f值


while any(openMat(:))   %只要openMat不为空
    
    
    
    [~,currIdx]=min(fScore(:));%找到f值最小的点
    
    
    if goalIndex==currIdx %找到全局
        path = findPath(cameFrom, currIdx, fScore) ;%找到路径
        break;
    end
    
    [currRow,currCol]=ind2sub([rows,cols],currIdx);%当前的坐标转化为2维索引
    
    
    openMat(currIdx)  = false;
    closeMat(currIdx) = true;%从open中取出，放入close中
    
    fScore(currIdx)  = inf; %f值要变为无穷，在进行f值比较时，不考虑
    
    
    
    for i=currRow-1:currRow+1    %考虑current点的周围
        
        if (i<1 || i>rows)  %非法index
            continue;
        end
        
        for j=currCol-1:currCol+1  %行和列
            if   (j<1 || j>cols)  %非法index
                continue;
            end
            
            if ~map(i,j) %墙
                continue;
            end
            
            if (i==currRow) && (j==currCol)  %排除自身
                continue;
            end
            
            
            % 开始判断
            
            if closeMat(i,j)  %不在close里面
                continue;
            end
            
            direct = abs(i-currRow) + abs(j-currCol);%用于在x和y方向分别走了多少
            
            if direct<=1
                temp=gScore(currRow,currCol) + stepLen1; %不同方向成本不一样
            else
                temp=gScore(currRow,currCol) + stepLen2;
            end
            
            if ~openMat(i,j) %不在open里面
                openMat(i,j)=true;
            else
                if  temp >= gScore(i,j) %在open里面，但是值比较大
                    continue;
                end
            end
            
            
            cameFrom(i,j)=currIdx; %（i,j）来源于currIdx
            
            gScore(i,j) = temp;
            fScore(i,j) = gScore(i,j)+ hScore(i,j);  %设置新的g和f值
            
        end
    end
    
end

%% 绘制图形

cmap = [0,0,0;...
    1,1,1;...
    1,0,0;... %r
    0,0,1;... %g
    1,1,0;... %b
    ];%颜色卡，每一行对应一个颜色


colormap(cmap);%设置颜色卡


map=map+1;%颜色编号从0开始

map(goalIndex)=3;
map(startIndex)=4;
map(path(2:end-1))=5;%看色卡编号


h=image(0.5,0.5,map);%map(1,1)像素值落在坐标的的哪个位置



set(gca,'xtick',[0:cols+1])
set(gca,'ytick',[0:rows+1])
axis equal
grid on
axis tight
