% �ҵ�A*�㷨
% jack0077555@163.com
% ������վ��www.deanhan.com
% ���˹��ںţ���Ȼ���ѧϰ
% �ο��� https://en.wikipedia.org/wiki/A*_search_algorithm

clear all
clc
%% �˹�  ���õ�ͼ���ϰ�
rows = 21;
cols = 28;      %����������

map=ones(rows,cols);%����������

map(4,4:8)=0;
map(8,4:8)=0;
map(4:8,8)=0;
map(12,1:end-1)=0;
map(2:end-1,19)=0;%ǽ

start=[6,6];
goal=[17,13];%���������յ�

%%
startIndex=sub2ind([rows,cols],start(1),start(2));
goalIndex=sub2ind([rows,cols],goal(1),goal(2));

stepLen1=10;%ˮƽ��ֱ��cost
stepLen2=14;%б���cost

%% ��صľ���

closeMat = false(rows,cols);%��mat
openMat  = false(rows,cols);%��mat


gScore = inf(rows,cols);
fScore = inf(rows,cols);%��ʼ��ȫ��Ϊinf

cameFrom =zeros(rows,cols);%�ڵ�ĸ��ڵ㣬��������

%% ��������ʽ�������


[Y,X]=meshgrid(1:cols,1:rows);

hScore=stepLen1 * max(abs(X-goal(1)),  abs(Y-goal(1))); %�����õ�����L-inf����

%% �������

openMat(startIndex)=true;%��ʼ��ʱ��ֻ�������open����

gScore(startIndex)=0;                  % ����gֵ
fScore(startIndex)=hScore(startIndex); % ����fֵ


while any(openMat(:))   %ֻҪopenMat��Ϊ��
    
    
    
    [~,currIdx]=min(fScore(:));%�ҵ�fֵ��С�ĵ�
    
    
    if goalIndex==currIdx %�ҵ�ȫ��
        path = findPath(cameFrom, currIdx, fScore) ;%�ҵ�·��
        break;
    end
    
    [currRow,currCol]=ind2sub([rows,cols],currIdx);%��ǰ������ת��Ϊ2ά����
    
    
    openMat(currIdx)  = false;
    closeMat(currIdx) = true;%��open��ȡ��������close��
    
    fScore(currIdx)  = inf; %fֵҪ��Ϊ����ڽ���fֵ�Ƚ�ʱ��������
    
    
    
    for i=currRow-1:currRow+1    %����current�����Χ
        
        if (i<1 || i>rows)  %�Ƿ�index
            continue;
        end
        
        for j=currCol-1:currCol+1  %�к���
            if   (j<1 || j>cols)  %�Ƿ�index
                continue;
            end
            
            if ~map(i,j) %ǽ
                continue;
            end
            
            if (i==currRow) && (j==currCol)  %�ų�����
                continue;
            end
            
            
            % ��ʼ�ж�
            
            if closeMat(i,j)  %����close����
                continue;
            end
            
            direct = abs(i-currRow) + abs(j-currCol);%������x��y����ֱ����˶���
            
            if direct<=1
                temp=gScore(currRow,currCol) + stepLen1; %��ͬ����ɱ���һ��
            else
                temp=gScore(currRow,currCol) + stepLen2;
            end
            
            if ~openMat(i,j) %����open����
                openMat(i,j)=true;
            else
                if  temp >= gScore(i,j) %��open���棬����ֵ�Ƚϴ�
                    continue;
                end
            end
            
            
            cameFrom(i,j)=currIdx; %��i,j����Դ��currIdx
            
            gScore(i,j) = temp;
            fScore(i,j) = gScore(i,j)+ hScore(i,j);  %�����µ�g��fֵ
            
        end
    end
    
end

%% ����ͼ��

cmap = [0,0,0;...
    1,1,1;...
    1,0,0;... %r
    0,0,1;... %g
    1,1,0;... %b
    ];%��ɫ����ÿһ�ж�Ӧһ����ɫ


colormap(cmap);%������ɫ��


map=map+1;%��ɫ��Ŵ�0��ʼ

map(goalIndex)=3;
map(startIndex)=4;
map(path(2:end-1))=5;%��ɫ�����


h=image(0.5,0.5,map);%map(1,1)����ֵ��������ĵ��ĸ�λ��



set(gca,'xtick',[0:cols+1])
set(gca,'ytick',[0:rows+1])
axis equal
grid on
axis tight
