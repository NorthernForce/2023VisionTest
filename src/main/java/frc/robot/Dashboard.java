package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* Because the SmartDashboard just isn't enough */
public class Dashboard {
    private SendableChooser<Command> chooser;
    public Dashboard()
    {
    }
    public void clearDashboard()
    {
        for (String key : SmartDashboard.getKeys())
        {
            SmartDashboard.delete(key);
        }
    }
    public void putNumber(String key, double object)
    {
        SmartDashboard.putNumber(key, object);
    }
    public DoubleSupplier getSupplier(String key)
    {
        return () -> SmartDashboard.getNumber(key, 0);
    }
    public void displayCommands(Command... commands)
    {
        chooser = new SendableChooser<>();
        chooser.setDefaultOption(commands[0].getName(), commands[0]);
        for (int i = 1; i < commands.length; i++)
        {
            chooser.addOption(commands[i].getName(), commands[i]);
        }
    }
    public Command getSelection()
    {
        return chooser.getSelected();
    }
    public void delete(String key)
    {
        SmartDashboard.delete(key);
    }
    public void putData(String key, Sendable data)
    {
        SmartDashboard.putData(key, data);
    }
}