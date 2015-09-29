%This function is written to solve the dooly zoon problem
%the idea is simple: find the relative positions, and compute frame by frame 
%Dong Nie, dongnie@cs.unc.edu
function problem1()
%some initilization
frameCnt=75;
frameFrequency=15;
inPath='./data/';
outPath='./P1Output/';
originalWidth=250
originalHeight=400;
targetWidth=400;
targetHeight=640;

%load data
inFile=[inPath,'data.mat'];
load(inFile);
datasets={BackgroundPointCloudRGB,ForegroundPointCloudRGB};

%prepare videoWriter, I first write as avi format, and then convert to wmv
videoFile=VideoWriter('dollyzoom.avi');
videoFile.FrameRate = frameFrequency; 
open(videoFile);

%compute depth information of foreground object, also, decide the position information
minDepth=min(ForegroundPointCloudRGB(3, :));
meanDepth=mean(ForegroundPointCloudRGB(3, :));
originalDepth=meanDepth*(1-originalWidth/targetWidth);
originalPosition=[0 0 -originalDepth]';

%begin to compute frame by frame
stride=(minDepth - originalDepth)/(frameCnt - 1);
strideVec=[0 0 -stride]';
E=eye(3);
for currFrame=1:frameCnt
    tic
    %compute the current position information, and form the project matrix
    currPosition=(currFrame - 1) * strideVec + originalPosition;
    M=K*[E,currPosition];%the required project matrix
    %compute the frame
    im=PointCloud2Image(M,datasets,crop_region,filter_size);
    %write the frame
    outFile=[outPath,sprintf('outFrame%d.jpg',currFrame)];
    imwrite(im,outFile);
    imshow(im);%you can comment out this sentence
    fprintf('Now we are generating the %d frame\n',currFrame);
    frame=getframe(gca);
    writeVideo(videoFile,frame);
    %update focal length information in calibration matrix
    K(1,1)=K(1,1)*(1-stride/(meanDepth+currPosition(3)));
    K(2,2)=K(2,2)*(1-stride/(meanDepth + currPosition(3)));
    toc
end
close(videoFile);
end