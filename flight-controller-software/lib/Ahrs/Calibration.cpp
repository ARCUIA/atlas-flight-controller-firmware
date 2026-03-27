#include "Calibration.h"

Calibration::Calibration(SerialBus& serial, LSM6DSV80X& imu) : _imu(imu), _serial(serial) {};


void Calibration::get_offsets(Offsets& offsets) {
    TeensyTime time;

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

            time.delay_us(CALIBRATION_TIME_DELAY);
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
    _serial.send_message((char*)"Align the IMU along its ");

    switch (axis) {
        case 'x':
            _serial.send_message((char*)"+X");
            break;
        case 'y':
            _serial.send_message((char*)"+Y");
            break;
        case 'z':
            _serial.send_message((char*)"+Z");
            break;
        case 'X':
            _serial.send_message((char*)"-X");
            break;
        case 'Y':
            _serial.send_message((char*)"-Y");
            break;
        case 'Z':
            _serial.send_message((char*)"-Z");
            break;
    }

    _serial.send_message((char*)" axis, then press any key.\n");

    TeensyTime time;

    while (!_serial.available()) {
        _serial.send_bytes((uint8_t*)&imu_data, sizeof(imu_data));
        _imu.read(imu_data);
        time.delay_us(CALIBRATION_TIME_DELAY);
    }

    uint8_t buffer;
    while (_serial.available()) {
        _serial.read(0, &buffer, 1);
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
