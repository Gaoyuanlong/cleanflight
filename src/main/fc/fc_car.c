/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "blackbox/blackbox.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/printf.h"

#include "config/config_eeprom.h"
#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/compass/compass.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/rx_pwm.h"
#include "drivers/pwm_output.h"
#include "drivers/adc.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/buttons.h"
#include "drivers/inverter.h"
#include "drivers/flash_m25p16.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/sdcard.h"
#include "drivers/usb_io.h"
#include "drivers/transponder_ir.h"
#include "drivers/exti.h"
#include "drivers/max7456.h"
#include "drivers/vtx_rtc6705.h"
#include "drivers/vtx_common.h"

#include "fc/config.h"
#include "fc/fc_car.h"
#include "fc/fc_msp.h"
#include "fc/fc_tasks.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/cli.h"

#include "msp/msp_serial.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "io/beeper.h"
#include "io/displayport_max7456.h"
#include "io/serial.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/motors.h"
#include "io/servos.h"
#include "io/gimbal.h"
#include "io/ledstrip.h"
#include "io/dashboard.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/transponder_ir.h"
#include "io/osd.h"
#include "io/osd_slave.h"
#include "io/displayport_msp.h"
#include "io/vtx_rtc6705.h"
#include "io/vtx_control.h"
#include "io/vtx_smartaudio.h"
#include "io/vtx_tramp.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/initialisation.h"
#include "sensors/sensors.h"
#include "sensors/sonar.h"

#include "telemetry/telemetry.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/navigation.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "io/rcsplit.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "build/build_config.h"
#include "build/debug.h"

#ifdef TARGET_PREINIT
void targetPreInit(void);
#endif

#ifdef SOFTSERIAL_LOOPBACK
serialPort_t *loopbackPort;
#endif

static CarOutputPort car_output_port;

void carInit(void) {
  motorDevConfig_t motorDevConfig;
  motorDevConfig.motorPwmRate = CAR_MOTOR_RATE;
  motorDevConfig.motorPwmProtocol = PWM_TYPE_BRUSHED;
  motorDevConfig.useUnsyncedPwm = true;
  motorDevConfig.motorPwmInversion = 0;
  motorDevConfig.ioTags[0] = timerHardware[8].tag;
  motorDevInit(&motorDevConfig, 0, 1);

  servoDevConfig_t servoDevConfig;
  servoDevConfig.servoCenterPulse = 0.0;
  servoDevConfig.servoPwmRate = CAR_SERVO_RATE;
  servoDevConfig.ioTags[0] =  timerHardware[10].tag;
  servoDevInit(&servoDevConfig);

  carOutputReset();
}

void carProcessLoop(void) {

}

void carOutputReset(void) {
  car_output_port.motor = CAR_MIN_MOTOR;
  car_output_port.servo = CAR_MID_SERVO;
  carOutput();
}

void carOutput(void) {
  if (car_output_port.motor > CAR_MAX_MOTOR) {
    car_output_port.motor = CAR_MAX_MOTOR;
  }else if (car_output_port.motor < CAR_MIN_MOTOR) {
    car_output_port.motor = CAR_MIN_MOTOR;
  }

  if (car_output_port.servo > CAR_MAX_MOTOR) {
    car_output_port.servo = CAR_MAX_MOTOR;
  }else if (car_output_port.servo < CAR_MIN_SERVO) {
    car_output_port.servo = CAR_MIN_SERVO;
  }
  pwmWriteMotor(0, car_output_port.motor + CAR_OFFSET_MOTOR);
  pwmWriteServo(0, car_output_port.servo);
}

float carGetMotor(void) {
  return car_output_port.motor;
}

float carGetServo(void) {
  return car_output_port.servo;
}

void carSetMotor(float motor) {
  if (motor > CAR_MAX_MOTOR) {
    motor = CAR_MAX_MOTOR;
  }else if (motor < CAR_MIN_MOTOR) {
    motor = CAR_MIN_MOTOR;
  }
  car_output_port.motor = motor;
}

 void carSetServo(float servo) {
  if (servo > CAR_MAX_MOTOR) {
    servo = CAR_MAX_MOTOR;
  }else if (car_output_port.servo < CAR_MIN_SERVO) {
    servo = CAR_MIN_SERVO;
  }
  car_output_port.servo = servo;
}