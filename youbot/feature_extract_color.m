%demo程序演示红色物体的识别方法
%关闭所有窗口
close all;
%获取并显示图像
figure(1)
I = imread('sim.png');
imshow(I)
%设定颜色阈值
diff_R = 100;
Image=I;
%获取颜色矩阵
RP_R=Image(:,:,1); 
RP_G=Image(:,:,2); 
RP_B=Image(:,:,3);
%红色区域分析
XYR=255*((RP_R-RP_G)>diff_R&(RP_R-RP_B)>diff_R); 
%绘制黑白图
figure(2)
imshow(uint8(XYR));
%转为二值图
bw=im2bw(XYR);
%寻找红色区域
[r c]=find(bw==1)
%寻找红色区域的最小外接矩形  
[rectx,recty,area,perimeter] = minboundrect(c,r,'a');
%根据外接矩形顶点绘制直线框 
line(rectx(:),recty(:),'color','r');
%显示外接矩形顶点坐标
rectx
recty
% end
