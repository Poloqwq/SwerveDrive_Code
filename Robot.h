#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/controller/PIDController.h>

#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <frc2/command/CommandScheduler.h>

#include <frc/Joystick.h>

#include <math.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
  void PID_Reset(TalonSRX* motor); //reset pid constant
  void PID_Reset(); //set position to 0
  void PID_Set(); //轉向motor position set
  void speed_array_process();
  void speed_set();

  rev::CANSparkMax front_right_spm{5,rev::CANSparkMaxLowLevel::MotorType::kBrushed};
  rev::CANSparkMax front_left_spm{6,rev::CANSparkMaxLowLevel::MotorType::kBrushed};
  rev::CANSparkMax back_left_spm{7,rev::CANSparkMaxLowLevel::MotorType::kBrushed};
  rev::CANSparkMax back_right_spm{8,rev::CANSparkMaxLowLevel::MotorType::kBrushed};

  TalonSRX front_right_turn{1};
  TalonSRX front_left_turn{2};
  TalonSRX back_left_turn{3};
  TalonSRX back_right_turn{4};

  frc::Joystick drivestick{0};

  double jx, jy, turn_speed = 0, speed_output[10][2], angle_position[10], speed[10];
  double rx=0.89/*cos -> 0.665*/, rcos=0.665, ry=1.0/*sin -> 0.747*/, rsin=0.747, angle = 0;
  double jx_max = 0.6, jy_max = 0.6, turn_max = 0.45;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
