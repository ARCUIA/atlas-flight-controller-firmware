#include "Calibration.h"

Calibration::Calibration(RFD900XUS& radio, LSM6DSV80X& imu)
    : _imu(imu), _radio(radio) {
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
    int gyro_count = 0;

    for (int i = 0; i < AXIS_NUM; i++) {
        char current_axis = axis_to_align[i];

        // Prompts user, waits for one keypress, then clears input.
        _align_axis_prompt(current_axis);

        for (int j = 0; j < NUM_MEASUREMENTS; j++) {
            _imu.read(imu_data);

            // Gyro bias is measured while stationary, independent of orientation.
            gyro_x += imu_data.gx;
            gyro_y += imu_data.gy;
            gyro_z += imu_data.gz;
            gyro_count++;

            switch (current_axis) {
                case 'x':
                    accel_x += imu_data.ax - ACCEL_STATIONARY_EXPECTED_VALUE;
                    count_x++;
                    break;

                case 'X':
                    accel_x += imu_data.ax + ACCEL_STATIONARY_EXPECTED_VALUE;
                    count_x++;
                    break;

                case 'y':
                    accel_y += imu_data.ay - ACCEL_STATIONARY_EXPECTED_VALUE;
                    count_y++;
                    break;

                case 'Y':
                    accel_y += imu_data.ay + ACCEL_STATIONARY_EXPECTED_VALUE;
                    count_y++;
                    break;

                case 'z':
                    accel_z += imu_data.az - ACCEL_STATIONARY_EXPECTED_VALUE;
                    count_z++;
                    break;

                case 'Z':
                    accel_z += imu_data.az + ACCEL_STATIONARY_EXPECTED_VALUE;
                    count_z++;
                    break;

                default:
                    _radio.send_message("Calibration error: unknown axis");
                    return;
            }

            timer.delay_us(CALIBRATION_TIME_DELAY);
        }
    }

    if (count_x <= 0 || count_y <= 0 || count_z <= 0 || gyro_count <= 0) {
        _radio.send_message("Calibration failed: zero sample count");
        return;
    }

    offsets.ax = accel_x / count_x;
    offsets.ay = accel_y / count_y;
    offsets.az = accel_z / count_z;

    offsets.gx = gyro_x / gyro_count;
    offsets.gy = gyro_y / gyro_count;
    offsets.gz = gyro_z / gyro_count;

    _radio.send_message("Calibration complete");
}

void Calibration::_align_axis_prompt(char axis) {
    _radio.send_message("Align the IMU along its ");

    switch (axis) {
        case 'x':
            _radio.send_message("+X");
            break;

        case 'y':
            _radio.send_message("+Y");
            break;

        case 'z':
            _radio.send_message("+Z");
            break;

        case 'X':
            _radio.send_message("-X");
            break;

        case 'Y':
            _radio.send_message("-Y");
            break;

        case 'Z':
            _radio.send_message("-Z");
            break;

        default:
            _radio.send_message("UNKNOWN");
            return;
    }

    _radio.send_message(" axis, then press any key.");

    // Wait for exactly one user confirmation.
    while (!_radio.available()) {
        timer.delay_us(10000);
    }

    // Clear all pending input bytes so the next axis does not auto-advance.
    while (_radio.available()) {
        _radio.read();
    }
}

void Calibration::apply_offsets(const Offsets& offsets, LSM6DSV80X::IMU_Data& imu_data) {
    imu_data.ax -= offsets.ax;
    imu_data.ay -= offsets.ay;
    imu_data.az -= offsets.az;

    imu_data.gx -= offsets.gx;
    imu_data.gy -= offsets.gy;
    imu_data.gz -= offsets.gz;
}