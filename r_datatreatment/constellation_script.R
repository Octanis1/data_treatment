
#Script for analysing the data from the Constellation Rover__________________________________________________

getwd() #know your current working directory. If it is not the one where the data files are stored, change it with setwd("path") or set file path when loading files

#Installing packages_________________________________________________________________________________________

install.packages("ggplot2")
install.packages("ggthemes")
install.packages("psych")

#Importing packages__________________________________________________________________________________________

library(ggplot2)
library(ggthemes)
library(psych)

#Importing data______________________________________________________________________________________________

batterydata <- read.csv(file = "batterydata", header = TRUE, sep = ";")
batterydata_median <- read.csv(file = "batterydata_median", header = TRUE, sep = ";")
imudata <- read.csv(file = "imudata", header = TRUE, sep = ";")
imudata_median <- read.csv(file = "imudata_median", header = TRUE, sep = ";")
mavpressdata <- read.csv(file = "mavpressdata", header = TRUE, sep = ";")
mavpressdata_median <- read.csv(file = "median_mavpressdata", header = TRUE, sep = ";")
mavtempdata <- read.csv(file = "mavtempdata", header = TRUE, sep = ";")
mavtempdata_median <- read.csv(file = "mavtempdata_median", header = TRUE, sep = ";")
rcchanneldata <- read.csv(file = "rcchanneldata", header = TRUE, sep = ";")
rcchanneldata_median <- read.csv(file = "rcchanneldata_median", header = TRUE, sep = ";")
statusdata <- read.csv(file = "statusdata", header = TRUE, sep = ";")
tempdata <- read.csv(file = "tempdata", header = TRUE, sep = ";")
tempdata_median <- read.csv(file = "tempdata_median", header = TRUE, sep = ";")

#Data exploration_____________________________________________________________________________________________

str(batterydata_median) #Visualise structure of data set
names(batterydata_median) #Visualise variables of data set (columns)
summary(batterydata_median) #Get basic descriptive statistics of dataset per variable
describe(batterydata_median) #Get more descriptive statistics of dataset per variable

#Data visualisation___________________________________________________________________________________________

#Time series graphs

batterydata_currentin_timeseriesplot <- ggplot(batterydata_median, aes(Time, Current_in_Ampere)) + geom_point() + geom_line()
batterydata_currentin_timeseriesplot <- batterydata_currentin_timeseriesplot + theme_bw()
ggsave("./figures/batterydata_currentin_timeseriesplot.jpg")
batterydata_currentout_timeseriesplot <- ggplot(batterydata_median, aes(Time, Current_out_Ampere)) + geom_point() + geom_line()
batterydata_currentout_timeseriesplot <- batterydata_currentout_timeseriesplot + theme_bw()
ggsave("./figures/batterydata_currentout_timeseriesplot.jpg")
batterydata_solarvoltage_timeseriesplot <- ggplot(batterydata_median, aes(Time, Solar_voltage_Volt)) + geom_point() + geom_line()
batterydata_solarvoltage_timeseriesplot <- batterydata_solarvoltage_timeseriesplot + theme_bw()
ggsave("./figures/batterydata_solarvoltage_timeseriesplot.jpg")
batterydata_estsolarpower_timeseriesplot <- ggplot(batterydata_median, aes(Time, Est_solar_power_Watt)) + geom_point() + geom_line()
batterydata_estsolarpower_timeseriesplot <- batterydata_estsolarpower_timeseriesplot + theme_bw()
ggsave("./figures/batterydata_estsolarpower_timeseriesplot.jpg")

imudata_movementx_timeseriesplot <- ggplot() + 
  geom_line(data = imudata_median, aes(x = Time, y = Linear_acceleration_x, color = "x acceleration"))  + 
  geom_line(data = imudata_median, aes(x = Time, y = Angular_velocity_x, color = "x velocity")) +
  xlab('Time') +
  ylab('Velocity & acceleration')
imudata_movementx_timeseriesplot <- imudata_movementx_timeseriesplot + theme_bw()
ggsave("./figures/imudata_movementx_timeseriesplot.jpg")
imudata_movementy_timeseriesplot <- ggplot() + 
  geom_line(data = imudata_median, aes(x = Time, y = Linear_acceleration_y, color = "y acceleration"))  + 
  geom_line(data = imudata_median, aes(x = Time, y = Angular_velocity_y, color = "y velocity")) +
  xlab('Time') +
  ylab('Velocity & acceleration')
imudata_movementy_timeseriesplot <- imudata_movementy_timeseriesplot + theme_bw()
ggsave("./figures/imudata_movementy_timeseriesplot.jpg")
imudata_movementz_timeseriesplot <- ggplot() + 
  geom_line(data = imudata_median, aes(x = Time, y = Linear_acceleration_z, color = "z acceleration"))  + 
  geom_line(data = imudata_median, aes(x = Time, y = Angular_velocity_z, color = "z velocity")) +
  xlab('Time') +
  ylab('Velocity & acceleration')
imudata_movementz_timeseriesplot <- imudata_movementz_timeseriesplot + theme_bw()
ggsave("./figures/imudata_movementz_timeseriesplot.jpg")
imudata_orientation_timeseriesplot <- ggplot() +
  geom_line(data = imudata_median, aes(x = Time, y = Orientation_x, color = "x orientation")) + 
  geom_line(data = imudata_median, aes(x = Time, y = Orientation_y, color = "y orientation")) +
  geom_line(data = imudata_median, aes(x = Time, y = Orientation_z, color = "z orientation")) +
  geom_line(data = imudata_median, aes(x = Time, y = Orientation_w, color = "w orientation")) +
  xlab('Time') +
  ylab('Orientation')
imudata_orientation_timeseriesplot <- imudata_orientation_timeseriesplot + theme_bw()
ggsave("./figures/imudata_orientation_timeseriesplot.jpg")

mavpressdata_timeseriesplot <- ggplot(mavpressdata_median, aes(Time, Fluid_pressure)) + geom_point() + geom_line()
mavpressdata_timeseriesplot <- mavpressdata_timeseriesplot + theme_bw()
ggsave("./figures/mavpressdata_timeseriesplot.jpg")

rcchanneldata_humidity_timeseriesplot <- ggplot() + 
  geom_line(data = rcchanneldata_median, aes(x = Time, y = Internal_humidity_percent, color = "Internal humidity"))  + 
  geom_line(data = rcchanneldata_median, aes(x = Time, y = External_humidity_percent, color = "External humidity")) +
  xlab('Time') +
  ylab('Humidity %')
rcchanneldata_humidity_timeseriesplot <- rcchanneldata_humidity_timeseriesplot + theme_bw()
ggsave("./figures/rcchanneldata_humidity_timeseriesplot.jpg")
rcchanneldata_uvlight_timeseriesplot <- ggplot(rcchanneldata_median, aes(Time, UV_light)) + geom_point() + geom_line()
rcchanneldata_uvlight_timeseriesplot <- rcchanneldata_uvlight_timeseriesplot + theme_bw()
ggsave("./figures/rcchanneldata_uvlight_timeseriesplot.jpg")
rcchanneldata_vislight_timeseriesplot <- ggplot(rcchanneldata_median, aes(Time, Visual_light_Lux)) + geom_point() + geom_line()
rcchanneldata_vislight_timeseriesplot <- rcchanneldata_vislight_timeseriesplot + theme_bw()
ggsave("./figures/rcchanneldata_vislight_timeseriesplot.jpg")
rcchanneldata_irlight_timeseriesplot <- ggplot(rcchanneldata_median, aes(Time, IR_light)) + geom_point() + geom_line()
rcchanneldata_irlight_timeseriesplot <- rcchanneldata_irlight_timeseriesplot + theme_bw()
rcchanneldata_irlight_timeseriesplot
ggsave("./figures/rcchanneldata_irlight_timeseriesplot.jpg")
rcchanneldata_radioactivity_timeseriesplot <- ggplot(rcchanneldata_median, aes(Time, Radioactivitiy_CPM)) + geom_point() + geom_line()
rcchanneldata_radioactivity_timeseriesplot <- rcchanneldata_radioactivity_timeseriesplot + theme_bw()
ggsave("./figures/rcchanneldata_radioactivity_timeseriesplot.jpg")

tempdata_timeseriesplot <- ggplot() + 
  geom_line(data = rcchanneldata_median, aes(x = Time, y = External_temperature_C, color = "RC Channel"))  +
  geom_point(data = rcchanneldata_median, aes(x = Time, y = External_temperature_C, color = "RC Channel"))  +
  geom_line(data = mavtempdata_median, aes(x = Time, y = Temperature, color = "mavros/temp")) +
  geom_point(data = mavtempdata_median, aes(x = Time, y = Temperature, color = "mavros/temp")) +
  geom_line(data = tempdata_median, aes(x = Time, y = Temperature, color = "imu/temp")) +
  geom_point(data = tempdata_median, aes(x = Time, y = Temperature, color = "imu/temp")) +
  xlab('Time') +
  ylab('Temperature °C') +
  coord_cartesian(ylim = c(8, 21.5))
tempdata_timeseriesplot <- tempdata_timeseriesplot + theme_bw()
ggsave("./figures/tempdata_timeseriesplot.jpg")

#box plots

jpeg(filename = "./figures/batterydata_currentin_boxplot.jpg")
boxplot(batterydata_median$Current_in_Ampere, ylab = "Current in (A)")
dev.off()
jpeg(filename = "./figures/batterydata_currentout_boxplot.jpg")
boxplot(batterydata_median$Current_out_Ampere, ylab = "Current out (A)")
dev.off()
jpeg(filename = "./figures/batterydata_solarvoltage_boxplot.jpg")
boxplot(batterydata_median$Solar_voltage_Volt, ylab = "Solar voltage (V)")
dev.off()
jpeg(filename = "./figures/batterydata_estsolarpower_boxplot.jpg")
boxplot(batterydata_median$Est_solar_power_Watt, ylab = "Est. solar power (W)")
dev.off()

jpeg(filename = "./figures/imudata_angvel_boxplot.jpg")
boxplot(imudata_median$Angular_velocity_x, imudata_median$Angular_velocity_y, imudata_median$Angular_velocity_z, ylab = "Angular velocity m/s", names = c("Ang. vel. x", "Ang. vel. y", "Ang. vel. z"))
dev.off()
jpeg(filename = "./figures/imudata_orientation_boxplot.jpg")
boxplot(imudata_median$Orientation_x, imudata_median$Orientation_y, imudata_median$Orientation_z, imudata_median$Orientation_w, ylab = "Orientation", names = c("Orientation x", "Orientation y", "Orientation z", "Orientation w"))
dev.off()
jpeg(filename = "./figures/imudata_linacc_boxplot.jpg")
boxplot(imudata_median$Linear_acceleration_x, imudata_median$Linear_acceleration_y, imudata_median$Linear_acceleration_z, ylab = "Linear acceleration m/s^2", names = c("Lin. acc. x", "Lin. acc. y", "Lin. acc. z"))
dev.off()

jpeg(filename = "./figures/mavpressdata_boxplot.jpg")
boxplot(mavpressdata_median$Fluid_pressure, ylab = "Fluid pressure")
dev.off()

jpeg(filename = "./figures/rcchanneldata_humidity_boxplot.jpg")
boxplot(rcchanneldata_median$Internal_humidity_percent, rcchanneldata_median$External_humidity_percent, ylab = "Humidity %", names = c("Int. humidity", "Ext. humidity"))
dev.off()
jpeg(filename = "./figures/rcchanneldata_uvlight_boxplot.jpg")
boxplot(rcchanneldata_median$UV_light, ylab = "UV light")
dev.off()
jpeg(filename = "./figures/rcchanneldata_irlight_boxplot.jpg")
boxplot(rcchanneldata_median$IR_light, ylab = "IR light")
dev.off()
jpeg(filename = "./figures/rcchanneldata_vislight_boxplot.jpg")
boxplot(rcchanneldata_median$Visual_light_Lux, ylab = "Visual light Lux")
dev.off()
jpeg(filename = "./figures/rcchanneldata_radioactivity_boxplot.jpg")
boxplot(rcchanneldata_median$Radioactivitiy_CPM, ylab = "Radioactivity CPM")
dev.off()

jpeg(filename = "./figures/tempdata_boxplot.jpg")
boxplot(rcchanneldata_median$External_temperature_C, mavtempdata_median$Temperature, tempdata_median$Temperature, ylab = "Ext. temperature °C", names = c("RC Channel", "mavros/temp", "imu/temp"))
dev.off()

#Merge data sets for further analysis______________________________________________________________________________________

mergeddata1 <- merge(batterydata_median, imudata_median, by = "Time", all=TRUE)
mergeddata2 <- merge(mergeddata1, mavpressdata_median, by = "Time", all=TRUE)
mergeddata3 <- merge(mergeddata2, mavtempdata_median, by = "Time", all=TRUE)
mergeddata4 <- merge(mergeddata3, rcchanneldata_median, by = "Time", all=TRUE)
mergeddata <- merge(mergeddata4, tempdata_median, by = "Time", all=TRUE)

#Function to remove outliers_______________________________________________________________________________________________

remove_outliers <- function(x, na.rm = TRUE, ...) {
  qnt <- quantile(x, probs=c(.25, .75), na.rm = na.rm, ...)
  H <- 1.5 * IQR(x, na.rm = na.rm)
  y <- x
  y[x < (qnt[1] - H)] <- NA
  y[x > (qnt[2] + H)] <- NA
  y
}

#Clean temperature data in merged data set

mergeddata$Temperature.x <- remove_outliers(mergeddata$Temperature.x)
mergeddata$Temperature.y <- remove_outliers(mergeddata$Temperature.y)
mergeddata$External_temperature_C <- remove_outliers(mergeddata$External_temperature_C)

#Preliminary analysis______________________________________________________________________________________________________

jpeg(filename = "./figures/tempdata_scatterplot.jpg")
plot(mergeddata$Temperature.x, mergeddata$External_temperature_C, xlab = "mavros/temp", ylab = "RC Channel")
abline(lm_temp <- lm(mergeddata$External_temperature_C ~ mergeddata$Temperature.x))
Rsq_temp <- summary(lm_temp)$adj.r.squared
Rsq_temp <- format(Rsq_temp, digits = 4)
legend("topright", bty="n", legend=paste("R^2 = ", Rsq_temp))
dev.off()

mergeddata["Total_velocity"] <- (abs(mergeddata$Angular_velocity_x) + abs(mergeddata$Angular_velocity_y) + abs(mergeddata$Angular_velocity_z))
mergeddata["Total_acceleration"] <- (abs(mergeddata$Linear_acceleration_x) + abs(mergeddata$Linear_acceleration_y) + abs(mergeddata$Linear_acceleration_z + 9.8))
mergeddata["Current_out_Ampere_abs"] <- abs(mergeddata$Current_out_Ampere)

jpeg(filename = "./figures/velocitydata_scatterplot.jpg")
plot(mergeddata$Total_velocity, mergeddata$Current_out_Ampere_abs, xlab = "Overall velocity", ylab = "Current out")
abline(lm_vel <- lm(mergeddata$Current_out_Ampere_abs ~ mergeddata$Total_velocity))
Rsq_vel <- summary(lm_vel)$adj.r.squared
Rsq_vel <- format(Rsq_vel, digits = 4)
legend("topright", bty="n", legend=paste("R^2 = ", Rsq_temp))
dev.off()

jpeg(filename = "./figures/accelerationdata_scatterplot.jpg")
plot(mergeddata$Total_acceleration, mergeddata$Current_out_Ampere_abs, xlab = "Overall acceleration", ylab = "Current out")
abline(lm_acc <- lm(mergeddata$Current_out_Ampere_abs ~ mergeddata$Total_acceleration))
Rsq_acc <- summary(lm_acc)$adj.r.squared
Rsq_acc <- format(Rsq_acc, digits = 4)
legend("topright", bty="n", legend=paste("R^2 = ", Rsq_temp))
dev.off()

