/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

// NOTE: This requires ICM_20948_USE_DMP to be uncommented for the DMP to function.
#include "ICM_20948.h"

ICM_20948_I2C imu;

#define WIRE_PORT Wire

ros::NodeHandle  nh;

sensor_msgs::Imu imu_msg;
const char frame_id[] = "/imu";
ros::Publisher imu_pub("/imu", &imu_msg);

bool use_dmp = true;

void setRPY(const float roll, const float pitch, const float yaw, geometry_msgs::Quaternion& quat)
{
    const float halfYaw = yaw * 0.5;
    const float halfPitch = pitch * 0.5;
    const float halfRoll = roll * 0.5;
    const float cosYaw = cos(halfYaw);
    const float sinYaw = sin(halfYaw);
    const float cosPitch = cos(halfPitch);
    const float sinPitch = sin(halfPitch);
    const float cosRoll = cos(halfRoll);
    const float sinRoll = sin(halfRoll);
    quat.x = (sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw); //x
    quat.y = (cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw); //y
    quat.z = (cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw); //z
    quat.w = (cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); //formerly yzx
}

float DegreeToRadian(const float deg)
{
    return deg * (PI / 180);
}

float MilliGToMeterSq(const float milli_g)
{
    return (milli_g / 1000) * 9.80665;
}

bool InitializeImu(const bool use_dmp)
{
    // Initialize the IMU.
    const auto imu_status = imu.begin(WIRE_PORT);
    if (imu_status != ICM_20948_Stat_Ok)
    {
        nh.logerror("Failed to initialize IMU.");
        return false;
    }

    // Check if DMP should be used.
    if (!use_dmp)
    {
        return true;
    }

    // Configure the DMP.
    imu.initializeDMP();
    if (imu.status != ICM_20948_Stat_Ok)
    {
        nh.logerror("Failed to initialize DMP.");
        return false;
    }

    // DMP sensor options are defined in ICM_20948_DMP.h
    //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
    //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
    //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
    //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
    //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
    //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

    // Enable the DMP acceleration sensor.
    imu.enableDMPSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION);
    if (imu.status != ICM_20948_Stat_Ok)
    {
        nh.logerror("Failed to initialize linear acceleration.");
        return false;
    }

    // Enable the DMP gyroscope sensor.
    imu.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE);
    if (imu.status != ICM_20948_Stat_Ok)
    {
        nh.logerror("Failed to initialize gyroscope.");
        return false;
    }

    // Enable the DMP orientation sensor.
    imu.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION);
    if (imu.status != ICM_20948_Stat_Ok)
    {
        nh.logerror("Failed to initialize orientation.");
        return false;
    }

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate / ODR ) - 1
    // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    imu.setDMPODRrate(DMP_ODR_Reg_Accel, 0); // Set to the maximum
    imu.setDMPODRrate(DMP_ODR_Reg_Gyro, 0); // Set to the maximum
    imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 0); // Set to the maximum
    //dmp_initialized &= (imu.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //dmp_initialized &= (imu.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //dmp_initialized &= (imu.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    if (imu.status != ICM_20948_Stat_Ok)
    {
        nh.logerror("Failed to set DMP ODR rate.");
        return false;
    }

    // Enable the FIFO
    imu.enableFIFO();
    if (imu.status != ICM_20948_Stat_Ok)
    {
        nh.logerror("Failed to enable FIFO.");
        return false;
    }

    // Enable the DMP
    imu.enableDMP();
    if (imu.status != ICM_20948_Stat_Ok)
    {
        nh.logerror("Failed to enable DMP.");
        return false;
    }

    // Reset DMP
    imu.resetDMP();
    if (imu.status != ICM_20948_Stat_Ok)
    {
        nh.logerror("Failed to reset DMP.");
        return false;
    }

    // Reset FIFO
    imu.resetFIFO();
    if (imu.status != ICM_20948_Stat_Ok)
    {
        nh.logerror("Failed to reset FIFO.");
        return false;
    }

    return true;
}

void setup()
{
    // Initialize ROS.
    nh.initNode();
    nh.advertise(imu_pub);

    // Get the parameters.
    int i2c_clock = 400000;
    nh.getParam("i2c_clock", &i2c_clock, 1);
    nh.getParam("use_dmp", &use_dmp, 1);

    // Initialize the I2C port.
    WIRE_PORT.begin();
    WIRE_PORT.setClock(i2c_clock);

    // Initialize the IMU.
    bool imu_initialized = false;
    while(!imu_initialized)
    {
        imu_initialized = InitializeImu(use_dmp);
        // Sleep to avoid a hot-loop.
        delay(10);
    }

    imu_msg.header.frame_id = frame_id;
}

void loop()
{
    // Check if the IMU has data.
    if (!imu.dataReady())
    {
        // Sleep to avoid a hot-loop.
        delay(1);
        nh.spinOnce();
        return;
    }

    if (!use_dmp)
    {
        // Get the raw data from the IMU.
        imu.getAGMT();

        // The imu reports the acceleration in milli g's, but the ROS IMU message expects m/s^2.
        imu_msg.linear_acceleration.x = MilliGToMeterSq(imu.accX());
        imu_msg.linear_acceleration.y = MilliGToMeterSq(imu.accY());
        imu_msg.linear_acceleration.z = MilliGToMeterSq(imu.accZ());
        imu_msg.linear_acceleration_covariance[0] = -1;

        // The imu reports the gyro in deg/sec, but the ROS IMU message expects rad/sec.
        imu_msg.angular_velocity.x = DegreeToRadian(imu.gyrX());
        imu_msg.angular_velocity.y = DegreeToRadian(imu.gyrY());
        imu_msg.angular_velocity.z = DegreeToRadian(imu.gyrZ());
        imu_msg.angular_velocity_covariance[0] = -1;

        setRPY(imu.magX(), imu.magY(), imu.magZ(), imu_msg.orientation);
        imu_msg.orientation_covariance[0] = -1;
    }
    else
    {
        // Read any DMP data waiting in the FIFO
        // Note:
        //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
        //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
        //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
        //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
        //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
        icm_20948_DMP_data_t data;
        // Was valid data available?
        while ((ICM_20948_Stat_FIFOMoreDataAvail | ICM_20948_Stat_Ok) & imu.readDMPdataFromFIFO(&data))
        {
            // Check the packet contains Accel data.
            if ((data.header & DMP_header_bitmap_Accel) > 0)
            {
                // Extract the raw accelerometer data
                imu_msg.linear_acceleration.x = (float)data.Raw_Accel.Data.X;
                imu_msg.linear_acceleration.y = (float)data.Raw_Accel.Data.Y;
                imu_msg.linear_acceleration.z = (float)data.Raw_Accel.Data.Z;
                imu_msg.linear_acceleration_covariance[0] = -1;
            }

            // Check the packet contains Gyro data.
            if ((data.header & DMP_header_bitmap_Gyro) > 0)
            {
                // Extract the raw gyro data
                imu_msg.angular_velocity.x = (float)data.Raw_Gyro.Data.X;
                imu_msg.angular_velocity.y = (float)data.Raw_Gyro.Data.Y;
                imu_msg.angular_velocity.z = (float)data.Raw_Gyro.Data.Z;
                imu_msg.angular_velocity_covariance[0] = -1;
            }

            // Check the packet contains Quat9 data.
            if ((data.header & DMP_header_bitmap_Quat9) > 0)
            {
                // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
                // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
                // The quaternion data is scaled by 2^30.

                // Scale to +/- 1
                float q1 = ((float)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                float q2 = ((float)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                float q3 = ((float)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
                float q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

                imu_msg.orientation.x = q1;
                imu_msg.orientation.y = q2;
                imu_msg.orientation.z = q3;
                imu_msg.orientation.w = q0;
                imu_msg.orientation_covariance[0] = -1;
            }
        }
    }

    // Publish the IMU data.
    imu_msg.header.stamp = nh.now();
    imu_pub.publish( &imu_msg );
    nh.spinOnce();
}
