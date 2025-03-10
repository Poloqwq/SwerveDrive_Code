
#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

#define current_fr_angle front_right_turn.GetSelectedSensorPosition()
#define current_fl_angle front_left_turn.GetSelectedSensorPosition()
#define current_bl_angle back_left_turn.GetSelectedSensorPosition()
#define current_br_angle back_right_turn.GetSelectedSensorPosition()

double speed_reverse[10];

void Robot::PID_Reset(TalonSRX* motor) {
  motor->ConfigSelectedFeedbackSensor(FeedbackDevice::Analog, 0, 0);
  motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 20, 0);
  motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 20, 0);
  motor->ConfigNominalOutputForward(0, 0);
  motor->ConfigNominalOutputReverse(0, 0);
  motor->ConfigPeakOutputForward(1, 0);
  motor->ConfigPeakOutputReverse(-1, 0);
  motor->SelectProfileSlot(0, 0);
  motor->Config_kF(0, 0, 0);
  motor->Config_kP(0, 10, 0);
  motor->Config_kI(0, 0, 0);
  motor->Config_kD(0, 10, 0);
  motor->ConfigMotionCruiseVelocity(1500, 0);
  motor->ConfigMotionAcceleration(1500, 0);
}

void Robot::PID_Reset() {
  front_left_turn.Set(ControlMode::Position, 256);
  front_right_turn.Set(ControlMode::Position, 256);
  back_left_turn.Set(ControlMode::Position, 256);
  back_right_turn.Set(ControlMode::Position, 256);
}

double distance(double x, double y) {
  return sqrt(pow(x, 2)+pow(y, 2));
}

void Robot::speed_array_process() {
  //front right spm
  speed_output[0][0] = jx + turn_speed * rcos;
  speed_output[0][1] = jy - turn_speed * rsin;
  //front left spm
  speed_output[1][0] = jx + turn_speed * rcos;
  speed_output[1][1] = jy + turn_speed * rsin;
  //back left spm
  speed_output[2][0] = jx - turn_speed * rcos;
  speed_output[2][1] = jy + turn_speed * rsin;
  //back right spm
  speed_output[3][0] = jx - turn_speed * rcos;
  speed_output[3][1] = jy - turn_speed * rsin;
}

double closest(double destin, double now, int i) {
  int k = now / 1024;
  double base_now = now - k * 1024.0;
  if(destin>base_now)
  {
    if(abs((destin-512.0)-base_now)<abs(destin-base_now))
    {
      speed_reverse[i] = -1.0;
      return now+destin-512.0-base_now;
    }
    else
    {
      speed_reverse[i] = 1.0;
      return now+destin-base_now;
    }
  }
  else
  {
    if(abs((destin+512.0)-base_now)<abs(destin-base_now))
    {
      speed_reverse[i] = -1.0;
      return now+destin+512.0-base_now;
    }
    else
    {
      speed_reverse[i] = 1.0;
      return now+destin-base_now;
    }
  }
  return k + destin;// 防漏洞措施

}

void Robot::PID_Set() {
  front_right_turn.Set(ControlMode::Position, closest(angle_position[0], current_fr_angle, 0));
  front_left_turn.Set(ControlMode::Position, closest(angle_position[1], current_fl_angle, 1));
  back_left_turn.Set(ControlMode::Position, closest(angle_position[2], current_bl_angle, 2));
  back_right_turn.Set(ControlMode::Position, closest(angle_position[3], current_br_angle, 3));
}

void Robot::speed_set() {
  front_right_spm.Set(speed[0]*speed_reverse[0]);
  front_left_spm.Set(speed[1]*speed_reverse[1]);
  back_left_spm.Set(-1*speed[2]*speed_reverse[2]);
  back_right_spm.Set(speed[3]*speed_reverse[3]);
}

void Robot::RobotInit() {
  PID_Reset(&front_left_turn);
  PID_Reset(&front_right_turn);
  PID_Reset(&back_left_turn);
  PID_Reset(&back_right_turn);

  for(int i=0;i<4;i++)
  {
    speed_reverse[i] = 1.0;
  }

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void Robot::RobotPeriodic() {
  PID_Reset(&front_left_turn);
  PID_Reset(&front_right_turn);
  PID_Reset(&back_left_turn);
  PID_Reset(&back_right_turn);
}


void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {


  jx = drivestick.GetRawAxis(0) * jx_max;
  jy = drivestick.GetRawAxis(1) * jy_max * -1.0;
  
  turn_speed = drivestick.GetRawAxis(4) * turn_max;
  
  speed_array_process();

  for(int i=0;i<4;i++)
  {
    if((-0.05 < turn_speed)&&(turn_speed < 0.05))
    {
      angle = atan2(jy, jx) / 2 / M_PI * 1024;
    }
    else
    {
      angle = atan2(speed_output[i][1], speed_output[i][0]) / 2 / M_PI * 1024;
    }
    angle_position[i] = angle+256;
    if(i==1)
    {
      angle_position[i] = angle;
    }
    speed[i] = distance(speed_output[i][1], speed_output[i][0]);
  }
  if(drivestick.GetRawButton(1))
  {
    PID_Reset();
  }
  else
  {
    PID_Set();
  }
  speed_set();

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {


}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
