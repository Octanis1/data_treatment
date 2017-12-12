%Before running script import data from laserdata and mavlinkgpsrawintdata
%csv in this order!!!
%files with SEMICOLON delimiter as column vectors

%% Init fixed transform for Laser - GPS
rotmLIDAR = quat2rotm([0.924909,0,0.380188,0]);
offsetLIDAR = [0.22*ones(360,1) 0.05*ones(360,1) -0.06*ones(360,1)];

%% Import Laser data (laserdata)
laserdata = nan(length(rangedatam),360);
for i=1:length(rangedatam)
    laserdata(i,:) = str2double(strsplit(rangedatam{i},','));
end

%% Interpolate GPS Data to fit time and transform to local frame NED (using mavlinkgpsrawintdata)
j = 1;
latitude = nan(length(Time),1);
longitude = nan(length(Time),1);
offsetGPS = nan(length(Time),3);
for i=1:length(Time)
    while((Time(i) > Time1(j)) && (j < length(Time1)))
        j = j + 1;
        if j > length(Time1)
            j = length(Time1);
        end
    end
    %interpolate data
    if j < length(Time1)
        latitude(i) = pi / 180E7 * (Latitude1E7deg(j) + (Latitude1E7deg(j) - Latitude1E7deg(j + 1)) * (Time(i) - Time1(j)) / (Time1(j) - Time1(j + 1)));
        longitude(i) = pi / 180E7 * (Longitude1E7deg(j) + (Longitude1E7deg(j) - Longitude1E7deg(j + 1)) * (Time(i) - Time1(j)) / (Time1(j) - Time1(j + 1)));
    else
        latitude(i) = pi / 180E7 * (Latitude1E7deg(j) + (Latitude1E7deg(j-1) - Latitude1E7deg(j)) * (Time(i) - Time1(j)) / (Time1(j - 1) - Time1(j)));
        longitude(i) = pi / 180E7 * (Longitude1E7deg(j) + (Longitude1E7deg(j-1) - Longitude1E7deg(j)) * (Time(i) - Time1(j)) / (Time1(j - 1) - Time1(j)));
    end
    %calculate position in local frame NED
    %assumption altitude is at WGS84 polarradius, simplification!
    Rearth = 6356752; %m
    offsetGPS(i,:) = [(latitude(i)-latitude(1))*Rearth,sin(latitude(1))*Rearth*(longitude(i) - longitude(1)),0];
end


%% Apply rotation, convert to cartesian and add GPS offset
%offsetdGPSSim = [ones(360,1)*0.05 zeros(360,2)];

rotmOdom = quat2rotm([rotationodomW rotationodomX rotationodomY rotationodomZ]);

laserdataRect=nan(360,3,size(laserdata,1));
dotMatrix = nan(360*size(laserdata,1),3);

for i = 1:size(laserdata,1)
    laserdataRect(:,:,i) = ([cos((0:359)*pi/180)'.*laserdata(i,:)' sin((0:359)*pi/180)'.*laserdata(i,:)' 0*laserdata(i,:)']*rotmLIDAR'-offsetLIDAR)*rotmOdom(:,:,i)';
    dotMatrix(1+360*(i-1):360*i,:) = laserdataRect(:,:,i)+[ones(360,1)*offsetGPS(i,1) ones(360,1)*offsetGPS(i,2) ones(360,1)*offsetGPS(i,3)];
end

%% display data

scatter3(dotMatrix(:,1),dotMatrix(:,2),dotMatrix(:,3),'.');
%scatter3(laserdataRect(:,1,40),laserdataRect(:,2,40),laserdataRect(:,3,40),'.')
daspect([1 1 1])
%ylim([-2,2])
%zlim([-1,0.5])