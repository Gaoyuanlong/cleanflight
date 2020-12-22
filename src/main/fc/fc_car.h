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

#pragma once

#define CAR_MAX_MOTOR 2000
#define CAR_MIN_MOTOR 1100
#define CAR_OFFSET_MOTOR -1000
#define CAR_MOTOR_RATE 50

#define CAR_MAX_SERVO 2000
#define CAR_MID_SERVO 1500
#define CAR_MIN_SERVO 1000
#define CAR_SERVO_RATE 50

typedef struct {
  float motor;
  float servo;
}CarOutputPort;


void carInit(void);
void carProcessLoop(void);
void carOutputReset(void);
void carOutput(void);
float carGetMotor(void);
float carGetServo(void);
void carSetMotor(float motor);
void carSetServo(float servo);
