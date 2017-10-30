% Create video out of list of jpgs
clear 
clc

%% thanks to N. Masserey for his work!
load('stereoParams1.mat');


%% Folder with all the image files you want to create a movie from, choose this folder using:
Left = 'C:\Users\alexa\Documents\EPFL\Rover code\Left';
Right = 'C:\Users\alexa\Documents\EPFL\Rover code\Right';

% Verify that all the images are in the correct time order, this could be useful if you were using any kind of time lapse photography. We can do that by using dir to map our images and create a structure with information on each file.
jpegFiles = dir(strcat(Left,'/left_*.jpg'));
jpegFiles2 = dir(strcat(Right,'/right_*.jpg'));

% Sort by date from the datenum information.
S = [jpegFiles(:).datenum]; 
[S,S] = sort(S);
jpegFilesS = jpegFiles(S);

S2 = [jpegFiles2(:).datenum]; 
[S2,S2] = sort(S2);
jpegFilesS2 = jpegFiles2(S2);
% The sub-structures within jpegFilesS is now sorted in ascending time order.
% Notice that datenum is a serial date number, for example, if you would like to get the time difference in hours between two images you need to subtract their datenum values and multiply by 1440.

% Create a VideoWriter object, in order to write video data to an .avi file using a jpeg compression.
VideoFile = 'timelapse2.avi';
writerObj = VideoWriter(VideoFile);

% Define the video frames per second speed (fps)
fps = 30; 
writerObj.FrameRate = fps;

% Open file for writing video data
open(writerObj);

% Running over all the files, converting them to movie frames using im2frame and writing the video data to file using writeVideo
for t = 5:(length(jpegFilesS2)-10)
    %reading the images: left & right
     left = imread(strcat(Left,'/',jpegFilesS(t).name));
     right = imread(strcat(Right,'/',jpegFilesS2(t-4).name));
     
     %enhencing contrast to improve the quality of the disparity map
     lefthis = histeq(left);
     righthis = histeq(right);
     
     %undistort the effect of the camera lenses
     imgRectLeft = undistortImage(lefthis, stereoParams1.CameraParameters1);
     imgRectRight = undistortImage(righthis, stereoParams1.CameraParameters2);
     %figure, imshow(imgRectLeft), figure, imshow(imgRectRight);
        
     [stereoRectLeft, stereoRectRight] = rectifyStereoImages(imgRectLeft, imgRectRight, stereoParams1);
     %figure(2),  imshow(stereoRectLeft), axis on, figure(3), imshow(stereoRectRight), axis on;
     I1 = rgb2gray(stereoRectLeft); I2 = rgb2gray(stereoRectRight);
     
     minDisp = 0;
     maxDisp = 32;
     disparityRange = [minDisp, maxDisp];
     disparityMap = disparity(I1, I2, 'BlockSize', 15, 'DisparityRange',disparityRange);
     
     figure(1);
     subplot (1,2,1);
     imshow(disparityMap, disparityRange, 'InitialMagnification', 50,  'Colormap', colormap('jet'));
     %colormap('jet'); 
     colorbar;
     title('Disparity Map');
     subplot (1,2,2);
     imshow(left)
     Frame = getframe(gcf);
     
     
     %point cloud
     point3D = reconstructScene(disparityMap, stereoParams1);
     % conversion to meters from millimeters
     point3D = point3D / 1000;
     % Plot points from minZ to maxZ, meaning the range of the object to detect
     % on the 3D reconstruction
    z = point3D(:,:,3);
    maxZ = 5;
    minZ = 1;
    zdisp = z;
    zdisp(z < minZ | z > maxZ)= NaN;
    point3Ddisp = point3D;
    point3Ddisp(:,:,3) = zdisp;
    figure('rend','opengl','pos',[10 10 1440 1080])

    pcshow(point3Ddisp, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    xlabel('X'); 
    ylabel('Y'); 
    zlabel('Z'); 
    xlim([-1.5,1.5])
    ylim([-1.5,0.5])
    zlim([2.5,4.5])

    grid on;
    Frame1 = getframe(gcf);
    [X, map] = frame2im(Frame1);

    
    close all

    %break 
    writeVideo(writerObj,Frame);
end

% Close the file after writing the video data 
close(writerObj);