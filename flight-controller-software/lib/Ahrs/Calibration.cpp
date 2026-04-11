#include "Calibration.h"

Calibration::Calibration(RFD900XUS& radio, LSM6DSV80X& imu) : _imu(imu), _radio(radio) {
}

void Calibration::get_offsets(Offsets& offsets) {


    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;

    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;

    int count_x = 0;
    int count_y = 0;
    int count_z = 0;

    for (int i = 0; i < AXIS_NUM; i++) {
        char current_axis = axis_to_align[i];
        _align_axis_prompt(current_axis);

        for (int j = 0; j < NUM_MEASUREMENTS; j++) {
            _imu.read(imu_data);

            switch (current_axis) {
                case 'x':
                    accel_x += (imu_data.ax - ACCEL_STATIONARY_EXPECTED_VALUE);
                    gyro_x += imu_data.gx;
                    count_x++;
                    break;

                case 'X':
                    accel_x += (imu_data.ax + ACCEL_STATIONARY_EXPECTED_VALUE);
                    gyro_x += imu_data.gx;
                    count_x++;
                    break;

                case 'y':
                    accel_y += (imu_data.ay - ACCEL_STATIONARY_EXPECTED_VALUE);
                    gyro_y += imu_data.gy;
                    count_y++;
                    break;

                case 'Y':
                    accel_y += (imu_data.ay + ACCEL_STATIONARY_EXPECTED_VALUE);
                    gyro_y += imu_data.gy;
                    count_y++;
                    break;

                case 'z':
                    accel_z += (imu_data.az - ACCEL_STATIONARY_EXPECTED_VALUE);
                    gyro_z += imu_data.gz;
                    count_z++;
                    break;

                case 'Z':
                    accel_z += (imu_data.az + ACCEL_STATIONARY_EXPECTED_VALUE);
                    gyro_z += imu_data.gz;
                    count_z++;
                    break;
            }

            timer.delay_us(CALIBRATION_TIME_DELAY);
        }
    }

    offsets.ax = accel_x / count_x;
    offsets.ay = accel_y / count_y;
    offsets.az = accel_z / count_z;

    offsets.gx = gyro_x / count_x;
    offsets.gy = gyro_y / count_y;
    offsets.gz = gyro_z / count_z;
}
void Calibration::_align_axis_prompt(char axis) {
    _radio.send_text("Align the IMU along its ");

    switch (axis) {
        case 'x':
            _radio.send_text("+X");
            break;
        case 'y':
            _radio.send_text("+Y");
            break;
        case 'z':
            _radio.send_text("+Z");
            break;
        case 'X':
            _radio.send_text("-X");
            break;
        case 'Y':
            _radio.send_text("-Y");
            break;
        case 'Z':
            _radio.send_text("-Z");
            break;
    }

    _radio.send_text(" axis, then press any key.");

    TeensyTime time;

    while (!_radio.available()) {
        _imu.read(imu_data);
        _radio.send_text("Waiting for key press...");
        time.delay_us(CALIBRATION_TIME_DELAY);
    }

    while (_radio.available()) {
        _radio.read();
    }
}

void Calibration::apply_offsets(const Offsets& offsets, LSM6DSV80X::IMU_Data& imu_data){
    imu_data.ax -= offsets.ax;
    imu_data.ay -= offsets.ay;
    imu_data.az -= offsets.az;
    imu_data.gx -= offsets.gx;
    imu_data.gy -= offsets.gy;
    imu_data.gz -= offsets.gz;
}
