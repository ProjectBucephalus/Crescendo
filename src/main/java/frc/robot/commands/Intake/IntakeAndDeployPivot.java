package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake.IndexerState;
import frc.robot.subsystems.Intake.IntakeStatus;
import frc.robot.subsystems.Pivot.PivotPosition;

/**
 * Intake deploy command
 * 
 * @author 5985
 */
public class IntakeAndDeployPivot extends Command {
    public boolean isFinished = false;
    Pivot s_Pivot;
    Intake s_Intake;
    private XboxController xbox;

    public IntakeAndDeployPivot(Pivot s_Pivot, Intake s_Intake, XboxController xbox) {
        this.s_Pivot = s_Pivot;
        this.s_Intake = s_Intake;
        this.xbox = xbox;
    }

    @Override
    public void initialize() {
        s_Intake.setDriverXbox(xbox);
    }

    @Override
    public void end(boolean interrupted) {
        

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Pivot.setPosition(PivotPosition.DEPLOYED);
        s_Intake.setIntakeStatus(IntakeStatus.IN_WITH_BEAM_BREAK);
        s_Intake.rumbleWithNote(true);
        
    }

    public boolean isFinished() {
        return true;
    }
}
