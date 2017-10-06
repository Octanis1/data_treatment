% Create video out of list of jpgs
clear 
clc

% thanks to N. Masserey for his work!
load('stereoParams3.mat');


% Folder with all the image files you want to create a movie from, choose this folder using:
ImagesFolder = '/Volumes/Ephemeral/Octanis1-ROS/octanis1_mission_stereocamera_export/octanis1_stereo_image_dump/jpg';

% Verify that all the images are in the correct time order, this could be useful if you were using any kind of time lapse photography. We can do that by using dir to map our images and create a structure with information on each file.
jpegFiles = dir(strcat(ImagesFolder,'/left_*.jpg'));
jpegFiles2 = dir(strcat(ImagesFolder,'/right_*.jpg'));

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
VideoFile = strcat(ImagesFolder,'/timelapse.avi');
writerObj = VideoWriter(VideoFile);

% Define the video frames per second speed (fps)
fps = 30; 
writerObj.FrameRate = fps;

% Open file for writing video data
open(writerObj);

% Running over all the files, converting them to movie frames using im2frame and writing the video data to file using writeVideo
for t = 5:(length(jpegFilesS2)-10)
     left = imread(strcat(ImagesFolder,'/',jpegFilesS(t).name));
     right = imread(strcat(ImagesFolder,'/',jpegFilesS2(t-4).name));
     
     
     imgRectLeft = undistortImage(left, stereoParams3.CameraParameters1);
     imgRectRight = undistortImage(right, stereoParams3.CameraParameters2);
        
     [stereoRectLeft, stereoRectRight] = rectifyStereoImages(imgRectLeft, imgRectRight, stereoParams3);
     I1 = rgb2gray(stereoRectLeft); I2 = rgb2gray(stereoRectRight);
     
     minDisp = 0;
     maxDisp = 32;
     disparityRange = [minDisp, maxDisp];
     disparityMap = disparity(I1, I2, 'BlockSize', 15, 'DisparityRange',disparityRange);
     
     %figure;
     %imshow(disparityMap, disparityRange, 'InitialMagnification', 50);
     %colormap('jet'); colorbar;
     %title('Disparity Map');
     %Frame = getframe(gcf);
     %close all;
     
     %point cloud
     point3D = reconstructScene(disparityMap, stereoParams3);
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
    Frame = getframe(gcf);


    
    close all

    %break 
     writeVideo(writerObj,Frame);
end

% Close the file after writing the video data 
close(writerObj);