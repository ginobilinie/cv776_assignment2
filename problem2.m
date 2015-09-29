%This function is written to solve the radial distortion problem
%Dong Nie, dongnie@cs.unc.edu
function problem2()
%do some initialization
inDataPath='data/';
inImagePath ='P1Output/';
files = dir(fullfile(inImagePath,'*.jpg'));
outPath ='P2Output/';
originalWidth = 250;
originalHeight = 400;
frameCnt = 75;
frameFrequency = 15;

%load data and prepare for video generation
load([inDataPath,'data.mat']);
videoFile=VideoWriter('distortion.avi');
videoFile.FrameRate=frameFrequency;
open(videoFile);

%calculate depths of foreground object
meanDepth=mean(ForegroundPointCloudRGB(3,:));
minDepth=min(ForegroundPointCloudRGB(3,:));
originalDepth=meanDepth*(1-originalWidth/originalHeight);
camStride=(minDepth - originalDepth)/(frameCnt - 1);

%get focal length and center
focalX=K(1,1);
focalY=K(2,2);
centerX=K(1,3);
centerY=K(2,3);

%some settings for distortion
k1=-1;
kStride=2/(frameCnt-1);%k changes [-1,1]

for currFrame=1:length(files)
 tic
 mat=imread([inImagePath, files(currFrame).name]);
 mat = im2double(mat);
[m, n]= size(mat(:,:, 1));
[x, y]= meshgrid(1:n, 1:m);
%normalize
normU=(x-centerX)/focalX;
normV=(y-centerY)/focalY;
%compute u' and v' by Brown's distortion model
r2=normU.^2 + normV.^2;
uPrime =(1 + k1 * r2).* normU;
vPrime =(1 + k1 * r2).* normV;
%compute x' and y'
xPrime=round(uPrime*focalX + centerX);
yPrime=round(vPrime*focalY + centerY);
idx=xPrime>0 & xPrime<=n & yPrime>0 & yPrime<=m;

%copy pixels from input image (Problem1 output image) to distorted
%images channel by channel
tempR=mat(:,:,1);
ind=sub2ind(size(tempR),y(idx),x(idx));
indPrime=sub2ind(size(tempR),yPrime(idx),xPrime(idx));
distortR=zeros(m,n);
distortR(ind)=tempR(indPrime);

tempG=mat(:,:,2);
ind=sub2ind(size(tempG),y(idx),x(idx));
indPrime=sub2ind(size(tempG),yPrime(idx),xPrime(idx));
distortG=zeros(m,n);
distortG(ind)=tempG(indPrime);

tempB=mat(:,:,3);
ind=sub2ind(size(tempB),y(idx),x(idx));
indPrime=sub2ind(size(tempB),yPrime(idx),xPrime(idx));
distortB=zeros(m,n);
distortB(ind)=tempB(indPrime);

distortMat=cat(3,distortR,distortG,distortB);
%writethecurrentframeimage
imwrite(distortMat,[outPath,files(currFrame).name]);
imshow(distortMat);
frame=getframe(gca);
writeVideo(videoFile,frame);

%updatecamerapositionandfocallength
camPosition=-(currFrame-1)*camStride-originalDepth;
focalX=focalX*(1-camStride/(meanDepth+camPosition));
focalY=focalY*(1-camStride/(meanDepth+camPosition));
%updatedistortioncoefficient
k1=k1+kStride;
toc
end

close(videoFile);

return
