#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>

void Robot::RobotInit() {
	frc::DataLogManager::Start();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */

void Robot::RobotPeriodic() {
	frc2::CommandScheduler::GetInstance().Run();
	m_container.m_pickUp.SetDist(m_container.m_drive.GetDist() );
	m_container.m_pickUp.SetOffset(m_container.m_drive.GetOffset() );
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
	m_container.m_pickUp.ShooterFar();
	m_container.m_pickUp.ShooterOff();
	m_container.m_pickUp.PickUpRetract();
	m_container.m_pickUp.RollerOff();
	m_container.m_pickUp.IndexerOff();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
	m_container.m_pickUp.ShooterOn();
	m_container.LogData();
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif
