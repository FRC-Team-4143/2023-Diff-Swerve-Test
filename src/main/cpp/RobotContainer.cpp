#include "RobotContainer.h"
#include <frc/geometry/Translation2d.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <networktables/NetworkTableEntry.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <utility>
#include "commands/DriveGyro.h"
#include "commands/DriveLime.h"

#undef PATHWEAVER

#ifdef PATHWEAVER
#include "commands/PWSwerveControllerCommand.h"
#else
#include "commands/PPSwerveControllerCommand.h"
#endif

#include "commands/SetWheelOffsets.h"
#include "commands/ToggleDriveMode.h"
#include "commands/ZeroClimber.h"
#include "commands/ZeroYaw.h"

using namespace DriveConstants;

// ==========================================================================

RobotContainer::RobotContainer()
:	m_log{frc::DataLogManager::GetLog()}, m_drive{&m_driverController, m_log}, m_pickUp{&m_driverController}, m_climber{&m_climberController, m_log},
	m_powerDistributionPanel{},
	m_driveCommand{
		[this] {
			//auto x = -m_xspeedLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetLeftY(), DriveConstants::stickDeadBand));
			//auto y = -m_yspeedLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetLeftX(),DriveConstants::stickDeadBand));
			auto x = -frc::ApplyDeadband(m_driverController.GetLeftY(), DriveConstants::stickDeadBand);
			auto y = -frc::ApplyDeadband(m_driverController.GetLeftX(),DriveConstants::stickDeadBand);
			auto rot = -m_rotLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetRightX(), DriveConstants::stickDeadBand));

			double stickMagnitude = std::clamp(sqrt(pow(x, 2) + pow(y, 2)), -1., 1.);
			double power = 0;

			if(stickMagnitude != 0)
				power = m_powerLimiter.Calculate(stickMagnitude) / stickMagnitude;
			else
				m_powerLimiter.Reset(0);

			// Code that works


			//reset on stop
			//if(fabs(x.value()) > DriveConstants::stickDeadBand && fabs(m_driverController.GetLeftY()) < DriveConstants::stickDeadBand) m_xspeedLimiter.Reset(0);
			//if(fabs(y.value()) > DriveConstants::stickDeadBand && fabs(m_driverController.GetLeftX()) < DriveConstants::stickDeadBand) m_yspeedLimiter.Reset(0);
			if(fabs(rot.value()) > DriveConstants::stickDeadBand && fabs(m_driverController.GetRightX()) < DriveConstants::stickDeadBand) m_rotLimiter.Reset(0);

			//auto rotMod = (fabs(x.value())>.3 || fabs(y.value()) > .3) ? .5 : 1.0;
			auto rotMod = 1.0;

			if(m_driverController.GetRightTriggerAxis()) {x = std::clamp(x,-.75,.75); y = std::clamp(y,-.75,.75); }

			m_drive.Drive(
				units::meters_per_second_t(x * DriveConstants::kMaxSpeed * power),
				units::meters_per_second_t(y * DriveConstants::kMaxSpeed * power),
				units::radians_per_second_t(rot * DriveConstants::kMaxAngularSpeed * rotMod)
			);
		},
		{&m_drive}
	}
{
	// Initialize all of your commands and subsystems here

	_ConfigureButtonBindings();

	frc::DataLogManager::Start();

	frc::LiveWindow::DisableAllTelemetry();

	// Set up default drive command
	// The left stick controls translation of the robot.
	// Turning is controlled by the X axis of the right stick.
	m_drive.SetDefaultCommand(m_driveCommand);

	m_totalCurrent = wpi::log::DoubleLogEntry(m_log, "/robot/totalCurrent");
    m_batteryVoltage = wpi::log::DoubleLogEntry(m_log, "/robot/batteryVoltage");

	m_pickUpCycleCommand = new PickUpCycle(&m_pickUp,&m_driverController);
	m_pickUpCycleBounceCommand = new PickUpCycleBounce(&m_pickUp,&m_driverController);
}

// ==========================================================================

	//m_powerDistributionPanel.GetTotalCurrent();

void RobotContainer::LogData() {
	m_totalCurrent.Append(m_powerDistributionPanel.GetTotalCurrent());
	m_batteryVoltage.Append(m_powerDistributionPanel.GetVoltage());

}

// ==========================================================================

void RobotContainer::_ConfigureButtonBindings() {
	frc2::InstantCommand pickUpBounceCommand{
		[this]() { m_pickUp.PickUpBounce(); },
		{&m_pickUp}
	};

	frc2::InstantCommand pickUpExtendCommand{
		[this]() { m_pickUp.PickUpExtend(); },
		{&m_pickUp}
	};

	frc2::InstantCommand pickUpRetractCommand{
		[this]() { m_pickUp.PickUpRetract(); },
		{&m_pickUp}
	};

	frc2::InstantCommand pickUpToggleCommand{
		[this]() { m_pickUp.PickUpToggle(); },
		{&m_pickUp}
	};

	frc2::FunctionalCommand rollerInCommand{
		[]() {},
		[this]() { m_pickUp.RollerIn(); },
		[this](bool) { m_pickUp.RollerOff(); },
		[]() { return false; },
		{&m_pickUp}
	};

	frc2::FunctionalCommand rollerOutCommand{
		[]() {},
		[this]() { m_pickUp.RollerOut(); },
		[this](bool) { m_pickUp.RollerOff(); },
		[]() { return false; },
		{&m_pickUp}
	};

	frc2::FunctionalCommand indexerOnCommand{
		[]() {},
		[this]() { m_pickUp.IndexerOn(); },
		[this](bool) { m_pickUp.IndexerOff(); },
		[]() { return false; }
	};

	frc2::FunctionalCommand indexerRevCommand{
		[]() {},
		[this]() { m_pickUp.IndexerRev(); },
		[this](bool) { m_pickUp.IndexerOff(); },
		[]() { return false; }
	};

	frc2::FunctionalCommand shooterOnLimeLightCommand{
		[]() {},
		[this]() { m_pickUp.ShooterOnLimeLight(); },
		[this](bool) { m_pickUp.ShooterOff(); },
		[]() { return false; },
	};

	frc2::FunctionalCommand shooterOnCommand{
		[]() {},
		[this]() { m_pickUp.ShooterOn(); },
		[this](bool) { m_pickUp.ShooterOff(); },
		[]() { return false; },
	};

	frc2::InstantCommand shooterFasterCommand{
		[this]() { m_pickUp.ShooterFaster(); },
	};

	frc2::InstantCommand shooterSlowerCommand{
		[this]() { m_pickUp.ShooterSlower(); },
	};

	frc2::InstantCommand shooterDistToggleCommand{
		[this]() { m_pickUp.ShooterDistToggle(); },
	};

	frc2::InstantCommand nextClimberStepCommand{
		[this]() { m_climber.IndexStep(); },
	};

	frc2::InstantCommand previousClimberStepCommand{
		[this]() { m_climber.BackStep(); },
	};

	frc2::InstantCommand changeDriveModeComamnd{
		[this]() { m_drive.ToggleFieldCentric(); },
	};

	//(new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_A))->WhileHeld(rollerInCommand);
	//(new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_START))->WhenPressed(pickUpRetractCommand);
	//(new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_RB))->WhenPressed(shooterFasterCommand);
	//(new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_X))->WhileHeld(indexerOnCommand);

	frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY)
		.OnTrue(&shooterDistToggleCommand);
	frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftStick)
		.OnTrue(&changeDriveModeComamnd);
	frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB)
		.WhileTrue(&indexerRevCommand);
	frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart)
		.OnTrue(&shooterSlowerCommand);
	frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kBack)
		.OnTrue(&shooterFasterCommand);
	frc2::JoystickButton(&m_climberController, frc::XboxController::Button::kLeftBumper)
		.OnTrue(&nextClimberStepCommand);
	frc2::JoystickButton(&m_climberController, frc::XboxController::Button::kBack)
		.OnTrue(&previousClimberStepCommand);

	frc2::Trigger leftTrigger{
		[this]() {
			return m_driverController.GetLeftTriggerAxis() != 0;
		}
	};
	leftTrigger.WhileTrue(&indexerOnCommand);

	frc2::Trigger rightTrigger{
		[this]() {
			return m_driverController.GetRightTriggerAxis() != 0;
		}
	};
	//rightTrigger.WhileActiveContinous(shooterOnCommand);

	frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper).OnTrue(m_pickUpCycleCommand);
	frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper).OnTrue(m_pickUpCycleBounceCommand);

	frc::SmartDashboard::PutData("Zero Climber", new ZeroClimber(&m_climber));
	frc::SmartDashboard::PutData("Set WheelOffsets", new SetWheelOffsets(&m_drive));
	frc::SmartDashboard::PutData("Zero Yaw", new ZeroYaw(&m_drive));
}
