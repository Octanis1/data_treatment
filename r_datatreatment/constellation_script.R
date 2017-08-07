
#Script for analysing the data from the Constellation Rover__________________________________________________

#note: create a folder called 'figures' in your working directory prior to running the script.

#Installing packages_________________________________________________________________________________________

install.packages("ggplot2")
install.packages("ggthemes")
install.packages("psych")

#Importing packages__________________________________________________________________________________________

library(ggplot2)
library(ggthemes)
library(psych)

#Importing data______________________________________________________________________________________________

#note: Adjust file path

batterydata <- read.csv(file = "~/octanis/data_treatment/r_datatreatment/datafiles/batterydata", header = TRUE, sep = ";")
batterydata_median <- read.csv(file = "~/octanis/data_treatment/r_datatreatment/datafiles/batterydata_median", header = TRUE, sep = ";")
imudata <- read.csv(file = "~/octanis/data_treatment/r_datatreatment/datafiles/imudata", header = TRUE, sep = ";")
imudata_median <- read.csv(file = "~/octanis/data_treatment/r_datatreatment/datafiles/imudata_median", header = TRUE, sep = ";")
mavpressdata <- read.csv(file = "~/octanis/data_treatment/r_datatreatment/datafiles/mavpressdata", header = TRUE, sep = ";")
mavpressdata_median <- read.csv(file = "~/octanis/data_treatment/r_datatreatment/datafiles/median_mavpressdata", header = TRUE, sep = ";")
mavtempdata <- read.csv(file = "~/octanis/data_treatment/r_datatreatment/datafiles/mavtempdata", header = TRUE, sep = ";")
mavtempdata_median <- read.csv(file = "~/octanis/data_treatment/r_datatreatment/datafiles/mavtempdata_median", header = TRUE, sep = ";")
rcchanneldata <- read.csv(file = "~/octanis/data_treatment/r_datatreatment/datafiles/rcchanneldata", header = TRUE, sep = ";")
rcchanneldata_median <- read.csv(file = "~/octanis/data_treatment/r_datatreatment/datafiles/rcchanneldata_median", header = TRUE, sep = ";")
statusdata <- read.csv(file = "~/octanis/data_treatment/r_datatreatment/datafiles/statusdata", header = TRUE, sep = ";")
tempdata <- read.csv(file = "~/octanis/data_treatment/r_datatreatment/datafiles/tempdata", header = TRUE, sep = ";")
tempdata_median <- read.csv(file = "~/octanis/data_treatment/r_datatreatment/datafiles/tempdata_median", header = TRUE, sep = ";")

#Function to remove outliers_______________________________________________________________________________________________

remove_outliers <- function(x, na.rm = TRUE, ...) {
  qnt <- quantile(x, probs=c(.25, .75), na.rm = na.rm, ...)
  H <- 1.5 * IQR(x, na.rm = na.rm)
  y <- x
  y[x < (qnt[1] - H)] <- NA
  y[x > (qnt[2] + H)] <- NA
  y
}

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
ggsave("./figures/rcchanneldata_irlight_timeseriesplot.jpg")
rcchanneldata_radioactivity_timeseriesplot <- ggplot(rcchanneldata_median, aes(Time, Radioactivity_CPM)) + geom_point() + geom_line()
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
  ylab('Temperature °C') 
tempdata_timeseriesplot <- tempdata_timeseriesplot + theme_bw()
ggsave("./figures/tempdata_timeseriesplot.jpg")

tempdata_timeseriesplot2 <- ggplot() + 
  geom_line(data = mavtempdata_median, aes(x = Time, y = Temperature, color = "mavros/temp")) +
  geom_point(data = mavtempdata_median, aes(x = Time, y = Temperature, color = "mavros/temp")) +
  geom_line(data = tempdata_median, aes(x = Time, y = Temperature, color = "imu/temp")) +
  geom_point(data = tempdata_median, aes(x = Time, y = Temperature, color = "imu/temp")) +
  xlab('Time') +
  ylab('Temperature °C') 
tempdata_timeseriesplot2 <- tempdata_timeseriesplot2 + theme_bw()
ggsave("./figures/tempdata_timeseriesplot2.jpg")
tempdata_timeseriesplot2

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
boxplot(rcchanneldata_median$Radioactivity_CPM, ylab = "Radioactivity CPM")
dev.off()

jpeg(filename = "./figures/tempdata_boxplot.jpg")
boxplot(rcchanneldata_median$External_temperature_C, mavtempdata_median$Temperature, tempdata_median$Temperature, ylab = "Ext. temperature °C", names = c("RC Channel", "mavros/temp", "imu/temp"))
dev.off()

#QQ plots

jpeg(filename = "./figures/batterydata_currentin_qqplot.jpg")
qqnorm(batterydata_median$Current_in_Ampere)
qqline(batterydata_median$Current_in_Ampere)
dev.off()
jpeg(filename = "./figures/batterydata_currentout_qqplot.jpg")
qqnorm(batterydata_median$Current_out_Ampere)
qqline(batterydata_median$Current_out_Ampere)
dev.off()
jpeg(filename = "./figures/batterydata_solarvoltage_qqplot.jpg")
qqnorm(batterydata_median$Solar_voltage_Volt)
qqline(batterydata_median$Solar_voltage_Volt)
dev.off()
jpeg(filename = "./figures/batterydata_estsolarpower_qqplot.jpg")
qqnorm(batterydata_median$Est_solar_power_Watt)
qqline(batterydata_median$Est_solar_power_Watt)
dev.off()

jpeg(filename = "./figures/imudata_angvelx_qqplot.jpg")
qqnorm(imudata_median$Angular_velocity_x)
qqline(imudata_median$Angular_velocity_x)
dev.off()
jpeg(filename = "./figures/imudata_angvely_qqplot.jpg")
qqnorm(imudata_median$Angular_velocity_y)
qqline(imudata_median$Angular_velocity_y)
dev.off()
jpeg(filename = "./figures/imudata_angvelz_qqplot.jpg")
qqnorm(imudata_median$Angular_velocity_z)
qqline(imudata_median$Angular_velocity_z)
dev.off()
jpeg(filename = "./figures/imudata_linaccx_qqplot.jpg")
qqnorm(imudata_median$Linear_acceleration_x)
qqline(imudata_median$Linear_acceleration_x)
dev.off()
jpeg(filename = "./figures/imudata_linaccy_qqplot.jpg")
qqnorm(imudata_median$Linear_acceleration_y)
qqline(imudata_median$Linear_acceleration_y)
dev.off()
jpeg(filename = "./figures/imudata_linaccz_qqplot.jpg")
qqnorm(imudata_median$Linear_acceleration_z)
qqline(imudata_median$Linear_acceleration_z)
dev.off()

jpeg(filename = "./figures/mavpressdata_fluidpress_qqplot.jpg")
qqnorm(mavpressdata_median$Fluid_pressure)
qqline(mavpressdata_median$Fluid_pressure)
dev.off()

jpeg(filename = "./figures/rcchanneldata_inthumidity_qqplot.jpg")
qqnorm(rcchanneldata_median$Internal_humidity_percent)
qqline(rcchanneldata_median$Internal_humidity_percent)
dev.off()
jpeg(filename = "./figures/rcchanneldata_exthumidity_qqplot.jpg")
qqnorm(rcchanneldata_median$External_humidity_percent)
qqline(rcchanneldata_median$External_humidity_percent)
dev.off()
jpeg(filename = "./figures/rcchanneldata_uvlight_qqplot.jpg")
qqnorm(rcchanneldata_median$UV_light)
qqline(rcchanneldata_median$UV_light)
dev.off()
jpeg(filename = "./figures/rcchanneldata_irlight_qqplot.jpg")
qqnorm(rcchanneldata_median$IR_light)
qqline(rcchanneldata_median$IR_light)
dev.off()
jpeg(filename = "./figures/rcchanneldata_vislight_qqplot.jpg")
qqnorm(rcchanneldata_median$Visual_light_Lux)
qqline(rcchanneldata_median$Visual_light_Lux)
dev.off()
jpeg(filename = "./figures/rcchanneldata_radioactivity_qqplot.jpg")
qqnorm(rcchanneldata_median$Radioactivity_CPM)
qqline(rcchanneldata_median$Radioactivity_CPM)
dev.off()

jpeg(filename = "./figures/tempdata_rcchannel_qqplot.jpg")
qqnorm(remove_outliers(rcchanneldata_median$External_temperature_C))
qqline(remove_outliers(rcchanneldata_median$External_temperature_C))
dev.off()
jpeg(filename = "./figures/tempdata_mavros_qqplot.jpg")
qqnorm(remove_outliers(mavtempdata_median$Temperature))
qqline(remove_outliers(mavtempdata_median$Temperature))
dev.off()
jpeg(filename = "./figures/tempdata_imu_qqplot.jpg")
qqnorm(remove_outliers(tempdata_median$Temperature))
qqline(remove_outliers(tempdata_median$Temperature))
dev.off()

#Merge data sets for further analysis______________________________________________________________________________________

mergeddata1 <- merge(batterydata_median, imudata_median, by = "Time", all=TRUE)
mergeddata2 <- merge(mergeddata1, mavpressdata_median, by = "Time", all=TRUE)
mergeddata3 <- merge(mergeddata2, mavtempdata_median, by = "Time", all=TRUE)
mergeddata4 <- merge(mergeddata3, rcchanneldata_median, by = "Time", all=TRUE)
mergeddata <- merge(mergeddata4, tempdata_median, by = "Time", all=TRUE)

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

jpeg(filename = "./figures/tempdata_scatterplot2.jpg")
plot(mergeddata$Temperature.x, mergeddata$Temperature.y, xlab = "mavros/temp", ylab = "imu/temp")
abline(lm_temp <- lm(mergeddata$Temperature.y ~ mergeddata$Temperature.x))
Rsq_temp <- summary(lm_temp)$adj.r.squared
Rsq_temp <- format(Rsq_temp, digits = 4)
legend("topright", bty="n", legend=paste("R^2 = ", Rsq_temp))
dev.off()

mergeddata["xaccelpos"] <- ifelse(mergeddata$Linear_acceleration_x < 0, NA, mergeddata$Linear_acceleration_x)
mergeddata["yaccelpos"] <- ifelse(mergeddata$Linear_acceleration_y < 0, NA, mergeddata$Linear_acceleration_y)
mergeddata["linacczero"] <- mergeddata$Linear_acceleration_z + 9.807
mergeddata["zaccelpos"] <- ifelse(mergeddata$linacczero < 0, NA, mergeddata$linacczero)
mergeddata["Total_acceleration"] <- mergeddata$xaccelpos + mergeddata$yaccelpos + mergeddata$zaccelpos
mergeddata["Total_velocity"] <- (abs(mergeddata$Angular_velocity_x) + abs(mergeddata$Angular_velocity_y) + abs(mergeddata$Angular_velocity_z))
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

#cat("\014") #to clean the console
