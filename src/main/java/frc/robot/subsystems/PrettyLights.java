// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Random;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PrettyLights extends SubsystemBase {
    private SerialPort m_serialPort = null;
    private final BooleanSupplier m_noteIntook;
    private Timer m_timer = new Timer();
    private Image m_currentImage = Image.EMPTY;

    public enum Image {
        EMPTY,
        HAPPY,
        MONEY,
        WINK,
        SMILE_NOTE,
        OCTO_NOTE,
    }

    public PrettyLights(BooleanSupplier noteIntook) {
        m_timer.start();
        m_noteIntook = noteIntook;

        try {
            m_serialPort = new SerialPort(19200, Port.kUSB);
        } catch (Exception e) {
            System.out.println("LEDs are crying! " + e);
        }
    }

    public void setImage(Image image) {
        if (m_serialPort == null)
            return;
        
        m_currentImage = image;

        byte imageByte = 0x00;
        // howwww do i do a hashmap
        switch (image) {
            default:
            case EMPTY:
                imageByte = 0x00;
                break;
            case HAPPY:
                imageByte = 0x01;
                break;
            case MONEY:
                imageByte = 0x02;
                break;
            case WINK:
                imageByte = 0x03;
                break;
            case SMILE_NOTE:
                imageByte = 0x04;
                break;
            case OCTO_NOTE:
                imageByte = 0x05;
                break;
        }

        byte[] buffer = { imageByte };
        m_serialPort.write(buffer, 1);
    }

    private Image randomImage() {
        return Image.values()[new Random().nextInt(Image.values().length)];
    }

    @Override
    public void periodic() {
        Image targetImage = Image.OCTO_NOTE;
        if (m_noteIntook.getAsBoolean()) targetImage = Image.SMILE_NOTE;
        if (targetImage == m_currentImage) return;
        setImage(targetImage);
    }
}