%demo������ʾ��ɫ�����ʶ�𷽷�
%�ر����д���
close all;
%��ȡ����ʾͼ��
figure(1)
I = imread('sim.png');
imshow(I)
%�趨��ɫ��ֵ
diff_R = 100;
Image=I;
%��ȡ��ɫ����
RP_R=Image(:,:,1); 
RP_G=Image(:,:,2); 
RP_B=Image(:,:,3);
%��ɫ�������
XYR=255*((RP_R-RP_G)>diff_R&(RP_R-RP_B)>diff_R); 
%���ƺڰ�ͼ
figure(2)
imshow(uint8(XYR));
%תΪ��ֵͼ
bw=im2bw(XYR);
%Ѱ�Һ�ɫ����
[r c]=find(bw==1)
%Ѱ�Һ�ɫ�������С��Ӿ���  
[rectx,recty,area,perimeter] = minboundrect(c,r,'a');
%������Ӿ��ζ������ֱ�߿� 
line(rectx(:),recty(:),'color','r');
%��ʾ��Ӿ��ζ�������
rectx
recty
% end
