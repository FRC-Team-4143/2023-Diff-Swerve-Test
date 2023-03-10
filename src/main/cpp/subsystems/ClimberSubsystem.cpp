#include "subsystems/ClimberSubsystem.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>

// ============================================================================

ClimberSubsystem::ClimberSubsystem(frc::XboxController* controller, wpi::log::DataLog& log)
:	m_controller(controller),
	m_rotateLeft{ClimberConstants::kRotateLeftPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
	m_rotateRight{ClimberConstants::kRotateRightPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
	m_extendLeft{ClimberConstants::kExtendLeftPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
	m_extendRight{ClimberConstants::kExtendRightPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless},

	m_rotateLeftEncoder{m_rotateLeft.GetEncoder()},
	m_rotateRightEncoder{m_rotateRight.GetEncoder()},
	m_extendLeftEncoder{m_extendLeft.GetEncoder()},
	m_extendRightEncoder{m_extendRight.GetEncoder()},

	m_rotateLeftPidController{m_rotateLeft.GetPIDController()},
	m_rotateRightPidController{m_rotateRight.GetPIDController()},
	m_extendLeftPidController{m_extendLeft.GetPIDController()},
	m_extendRightPidController{m_extendRight.GetPIDController()},
	m_rotateLeftForwardLimit {m_rotateLeft.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed)},
	m_rotateLeftReverseLimit {m_rotateLeft.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed)},
	m_brakeSolenoidRght {frc::PneumaticsModuleType::CTREPCM, ClimberConstants::kBrakeSolenoidPortRght},
	m_brakeSolenoidLeft {frc::PneumaticsModuleType::CTREPCM, ClimberConstants::kBrakeSolenoidPortLeft},
	m_log(log)
{
	m_rotateLeft.RestoreFactoryDefaults();
	m_rotateRight.RestoreFactoryDefaults();
	m_extendLeft.RestoreFactoryDefaults();
	m_extendRight.RestoreFactoryDefaults();

	m_rotateLeft.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 100); 
	m_rotateLeft.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500); 
	m_rotateLeft.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500);
	m_rotateRight.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 100); 
	m_rotateRight.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500); 
	m_rotateRight.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500);
	m_extendLeft.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 100); 
	m_extendLeft.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500); 
	m_extendLeft.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500);
	m_extendRight.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 100); 
	m_extendRight.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500); 
	m_extendRight.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500);

	m_rotateLeftEncoder.SetPositionConversionFactor(90/17.57);
	m_rotateRightEncoder.SetPositionConversionFactor(90/17.57);
	m_extendLeftEncoder.SetPositionConversionFactor(1);
	m_extendRightEncoder.SetPositionConversionFactor(1);

	m_rotateLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	m_rotateRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	m_extendLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	m_extendRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

	m_rotateLeftPidController.SetP(ClimberConstants::kPR);
	m_rotateLeftPidController.SetI(ClimberConstants::kIR);
	m_rotateLeftPidController.SetD(ClimberConstants::kDR);
	m_rotateLeftPidController.SetIZone(ClimberConstants::kIzR);
	m_rotateLeftPidController.SetFF(ClimberConstants::kFFR);
	m_rotateLeftPidController.SetOutputRange(ClimberConstants::kMinOutputR, ClimberConstants::kMaxOutputR);

	m_rotateRightPidController.SetP(ClimberConstants::kPR);
	m_rotateRightPidController.SetI(ClimberConstants::kIR);
	m_rotateRightPidController.SetD(ClimberConstants::kDR);
	m_rotateRightPidController.SetIZone(ClimberConstants::kIzR);
	m_rotateRightPidController.SetFF(ClimberConstants::kFFR);
	m_rotateRightPidController.SetOutputRange(ClimberConstants::kMinOutputR, ClimberConstants::kMaxOutputR);

	m_extendLeftPidController.SetP(ClimberConstants::kPE);
	m_extendLeftPidController.SetI(ClimberConstants::kIE);
	m_extendLeftPidController.SetD(ClimberConstants::kDE);
	m_extendLeftPidController.SetIZone(ClimberConstants::kIzE);
	m_extendLeftPidController.SetFF(ClimberConstants::kFFE);
	m_extendLeftPidController.SetOutputRange(ClimberConstants::kMinOutputE, ClimberConstants::kMaxOutputE);

	m_extendRightPidController.SetP(ClimberConstants::kPE);
	m_extendRightPidController.SetI(ClimberConstants::kIE);
	m_extendRightPidController.SetD(ClimberConstants::kDE);
	m_extendRightPidController.SetIZone(ClimberConstants::kIzE);
	m_extendRightPidController.SetFF(ClimberConstants::kFFE);
	m_extendRightPidController.SetOutputRange(ClimberConstants::kMinOutputE, ClimberConstants::kMaxOutputE);

	m_extendLeftPidController.SetSmartMotionMaxVelocity(ClimberConstants::kMaxVel);
	m_extendLeftPidController.SetSmartMotionMinOutputVelocity(ClimberConstants::kMinVel);
	m_extendLeftPidController.SetSmartMotionMaxAccel(ClimberConstants::kMaxAcc);
	m_extendLeftPidController.SetSmartMotionAllowedClosedLoopError(ClimberConstants::kAllErr);

	m_extendRightPidController.SetSmartMotionMaxVelocity(ClimberConstants::kMaxVel);
	m_extendRightPidController.SetSmartMotionMinOutputVelocity(ClimberConstants::kMinVel);
	m_extendRightPidController.SetSmartMotionMaxAccel(ClimberConstants::kMaxAcc);
	m_extendRightPidController.SetSmartMotionAllowedClosedLoopError(ClimberConstants::kAllErr);

	m_rotateLeftForwardLimit.EnableLimitSwitch(false);
	m_rotateLeftReverseLimit.EnableLimitSwitch(false);

	m_rotateLeft.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);
	m_rotateLeft.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);

	m_brakeSolenoidRght.Set(true);
	m_brakeSolenoidLeft.Set(true);

	slowSafeMode = false;
	fastSafeMode = false;

	m_rotateLeftCurrent = wpi::log::DoubleLogEntry(log, "climberRotateLeftCurrent");
    m_rotateRightCurrent = wpi::log::DoubleLogEntry(log, "climberRotateRightCurrent");
	m_extendLeftCurrent = wpi::log::DoubleLogEntry(log, "climberExtendLeftCurrent");
	m_extendRightCurrent = wpi::log::DoubleLogEntry(log, "climberExtendRightCurrent");
	m_rotateLeftVelocity = wpi::log::DoubleLogEntry(log, "climberRotateLeftVelocity");
	m_rotateRightVelocity = wpi::log::DoubleLogEntry(log, "climberRotateRightVelocity");
	m_extendLeftVelocity = wpi::log::DoubleLogEntry(log, "climberExtendLeftVelocity");
	m_extendRightVelocity = wpi::log::DoubleLogEntry(log, "climberExtendRightVelocity");

}

bool slowClimbButtonState = false;
// ============================================================================

void ClimberSubsystem::Periodic() {
	/*
	m_rightPosition = frc::SmartDashboard::GetNumber("m_rightPosition", 0 );
	m_leftPosition = frc::SmartDashboard::GetNumber("m_leftPosition", 0 );
	m_rightExtensionPos = frc::SmartDashboard::GetNumber("m_rightExtensionPos",0 );
	m_leftExtensionPos = frc::SmartDashboard::GetNumber("m_leftExtensionPos",0 );
	*/
	if (m_controller->GetRightBumper()) {

			if(!fastSafeMode){
			m_rotateLeftPidController.SetSmartMotionMaxVelocity(ClimberConstants::kMaxVel);
			m_rotateRightPidController.SetSmartMotionMaxVelocity(ClimberConstants::kMaxVel);
			m_extendLeftPidController.SetSmartMotionMaxVelocity(ClimberConstants::kMaxVel);
			m_extendRightPidController.SetSmartMotionMaxVelocity(ClimberConstants::kMaxVel);

			fastSafeMode = true;
			slowSafeMode = false;
			}

		if (m_controller->GetStartButton()) { m_step = 0; m_newStep = true; }
		if (m_newStep) {
			m_leftPosition = m_climbSteps[m_step][0];
			m_rightPosition = m_climbSteps[m_step][1];
			m_leftExtensionPos = m_climbSteps[m_step][2];
			m_rightExtensionPos = m_climbSteps[m_step][3];
			m_newStep = false;
		}

		if (m_controller->GetYButton()) m_leftPosition += 0.5;
		if (m_controller->GetXButton()) m_leftPosition -= 0.5;

		if (m_controller->GetAButton()) m_rightPosition += 0.5;
		if (m_controller->GetBButton()) m_rightPosition -= 0.5;

		if (m_controller->GetLeftY() > 0.3) m_leftExtensionPos -= 1.0;
		if (m_controller->GetLeftY() < -0.3) m_leftExtensionPos += 1.0;

		if (m_controller->GetRightY() > 0.3) m_rightExtensionPos -= 1.0;
		if (m_controller->GetRightY() < -0.3) m_rightExtensionPos += 1.0;

		if (m_controller->GetLeftTriggerAxis() < 0.1) {
			m_rightPosition = std::clamp(m_rightPosition, -45.0, 0.0);
			m_leftPosition = std::clamp(m_leftPosition, 0.0, 45.0);
			m_rightExtensionPos = std::clamp(m_rightExtensionPos, -10.0*9/16, 265.0*9/16);
			m_leftExtensionPos = std::clamp(m_leftExtensionPos, -10.0*9/16, 265.0*9/16);
		}
/*
		frc::SmartDashboard::PutNumber("m_rightPosition", m_rightPosition);
		frc::SmartDashboard::PutNumber("m_leftPosition", m_leftPosition);
		frc::SmartDashboard::PutNumber("m_rightExtensionPos",m_rightExtensionPos);
		frc::SmartDashboard::PutNumber("m_leftExtensionPos",m_leftExtensionPos);

		frc::SmartDashboard::PutNumber("leftArmCurrent", m_extendLeft.GetOutputCurrent());
		frc::SmartDashboard::PutNumber("rightArmCurrent", m_extendRight.GetOutputCurrent());
*/
		frc::SmartDashboard::PutNumber("m_step", m_step);

		m_rotateLeftPidController.SetReference(m_leftPosition, rev::CANSparkMax::ControlType::kPosition);
		m_rotateRightPidController.SetReference(m_rightPosition, rev::CANSparkMax::ControlType::kPosition);

		m_extendLeftPidController.SetReference(m_leftExtensionPos, rev::CANSparkMax::ControlType::kSmartMotion);
		m_extendRightPidController.SetReference(m_rightExtensionPos, rev::CANSparkMax::ControlType::kSmartMotion);

	} else if(m_controller->GetRightTriggerAxis()>0.5){

			if(!slowSafeMode){
			m_rotateLeftPidController.SetSmartMotionMaxVelocity(ClimberConstants::kMaxVel/2);
			m_rotateRightPidController.SetSmartMotionMaxVelocity(ClimberConstants::kMaxVel/2);
			m_extendLeftPidController.SetSmartMotionMaxVelocity(ClimberConstants::kMaxVel/2);
			m_extendRightPidController.SetSmartMotionMaxVelocity(ClimberConstants::kMaxVel/2);
			slowSafeMode = true;
			fastSafeMode = false;
			}
		
		if (m_controller->GetStartButton()) { m_step = 0; m_newStep = true; }
		if (m_newStep) {
			m_leftPosition = m_climbSteps[m_step][0];
			m_rightPosition = m_climbSteps[m_step][1];
			m_leftExtensionPos = m_climbSteps[m_step][2];
			m_rightExtensionPos = m_climbSteps[m_step][3];
			m_newStep = false;
		}

		if (m_controller->GetYButton()) m_leftPosition += 0.5;
		if (m_controller->GetXButton()) m_leftPosition -= 0.5;

		if (m_controller->GetAButton()) m_rightPosition += 0.5;
		if (m_controller->GetBButton()) m_rightPosition -= 0.5;

		if (m_controller->GetLeftY() > 0.3) m_leftExtensionPos -= 1.0;
		if (m_controller->GetLeftY() < -0.3) m_leftExtensionPos += 1.0;

		if (m_controller->GetRightY() > 0.3) m_rightExtensionPos -= 1.0;
		if (m_controller->GetRightY() < -0.3) m_rightExtensionPos += 1.0;

		if (m_controller->GetLeftTriggerAxis() < 0.1) {
			if (m_rightPosition > 0 && m_leftPosition < 45) {
				m_leftPosition += m_rightPosition;
				m_rightPosition = 0;
			}
			if (m_leftPosition < 0.0 && m_rightPosition > -45.0) {
				m_rightPosition += m_leftPosition;
				m_leftPosition = 0.0;
			}
			m_rightPosition = std::clamp(m_rightPosition, -45.0, 0.0);
			m_leftPosition = std::clamp(m_leftPosition, 0.0, 45.0);
			m_rightExtensionPos = std::clamp(m_rightExtensionPos, -10.0*9/16, 265.0*9/16);
			m_leftExtensionPos = std::clamp(m_leftExtensionPos, -10.0*9/16, 265.0*9/16);
		}

/*
		frc::SmartDashboard::PutNumber("m_rightPosition", m_rightPosition);
		frc::SmartDashboard::PutNumber("m_leftPosition", m_leftPosition);
		frc::SmartDashboard::PutNumber("m_rightExtensionPos",m_rightExtensionPos);
		frc::SmartDashboard::PutNumber("m_leftExtensionPos",m_leftExtensionPos);
		frc::SmartDashboard::PutNumber("leftArmCurrent", m_extendLeft.GetOutputCurrent());
		frc::SmartDashboard::PutNumber("rightArmCurrent", m_extendRight.GetOutputCurrent());
*/
		frc::SmartDashboard::PutNumber("m_step", m_step);

		m_rotateLeftPidController.SetReference(m_leftPosition, rev::CANSparkMax::ControlType::kPosition);
		m_rotateRightPidController.SetReference(m_rightPosition, rev::CANSparkMax::ControlType::kPosition);

		m_extendLeftPidController.SetReference(m_leftExtensionPos, rev::CANSparkMax::ControlType::kSmartMotion);
		m_extendRightPidController.SetReference(m_rightExtensionPos, rev::CANSparkMax::ControlType::kSmartMotion);
	} else {
		m_rotateLeft.Set(0);
		m_rotateRight.Set(0);
		m_extendLeft.Set(0);
		m_extendRight.Set(0);
		slowSafeMode = false;
		fastSafeMode = false;
	}
/*
	frc::SmartDashboard::PutNumber("Rotate Left Pos", _GetLeftRotationPosition());
	frc::SmartDashboard::PutNumber("Rotate Right Pos", _GetRightRotationPosition());
	frc::SmartDashboard::PutNumber("Extend Left Pos", m_extendLeftEncoder.GetPosition());
	frc::SmartDashboard::PutNumber("Extend Right Pos", m_extendRightEncoder.GetPosition());
*/

double rotateLeftCurrent = m_rotateLeft.GetOutputCurrent();
double rotateRightCurrent = m_rotateRight.GetOutputCurrent();
double extendLeftCurrent = m_extendLeft.GetOutputCurrent();
double extendRightCurrent = m_extendRight.GetOutputCurrent();

auto rotateLeftVelocity = m_rotateLeftEncoder.GetVelocity();
auto rotateRightVelocity = m_rotateRightEncoder.GetVelocity();
auto extendLeftVelocity = m_extendLeftEncoder.GetVelocity();
auto extendRightVelocity = m_extendRightEncoder.GetVelocity();

m_rotateLeftCurrent.Append(rotateLeftCurrent);
m_rotateRightCurrent.Append(rotateRightCurrent);
m_extendLeftCurrent.Append(extendLeftCurrent);
m_extendRightCurrent.Append(extendRightCurrent);
m_rotateLeftVelocity.Append(rotateLeftVelocity);
m_rotateRightVelocity.Append(rotateRightVelocity);
m_extendLeftVelocity.Append(extendLeftVelocity);
m_extendRightVelocity.Append(extendRightVelocity);

}

// ============================================================================

void ClimberSubsystem::IndexStep(){
	if (m_step < m_numSteps) {
		m_step++;
		m_newStep = true;
	}
}

// ============================================================================

void ClimberSubsystem::BackStep(){
	if(m_step > 0){
		m_step = m_step - 1;
		m_newStep = true;
	}
}

// ============================================================================

void ClimberSubsystem::ZeroClimber(){
	m_rotateLeftEncoder.SetPosition(0);
	m_rotateRightEncoder.SetPosition(0);
	m_extendLeftEncoder.SetPosition(0);
	m_extendRightEncoder.SetPosition(0);
	m_step = 0; 
	m_leftPosition = m_climbSteps[m_step][0];
	m_rightPosition = m_climbSteps[m_step][1];
	m_leftExtensionPos = m_climbSteps[m_step][2];
	m_rightExtensionPos = m_climbSteps[m_step][3];
	m_newStep = false;
}

// ============================================================================

double ClimberSubsystem::_GetLeftRotationPosition(){
	return m_rotateLeftEncoder.GetPosition();
}

// ============================================================================

double ClimberSubsystem::_GetRightRotationPosition(){
	return m_rotateRightEncoder.GetPosition();
}

// ============================================================================
