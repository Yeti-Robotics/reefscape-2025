package frc.robot.util.controllerUtils;

import edu.wpi.first.wpilibj.XboxController;

import java.util.Map;

public class ControllerContainer {
    private final Controller[] controllers;

    public ControllerContainer(Map<Integer, Controller.ControllerType> controllers) {
        this.controllers = new Controller[controllers.size()];

        controllers.forEach((port, type) -> {
            switch (type) {
                case CUSTOM:
                    this.controllers[port] = new CustomController(port);
                    break;
                case XBOX:
                    this.controllers[port] = new CustomXboxController(port);
                    break;
            }
        });
    }

    public Controller get(int controllerNumber) {
        return controllers[controllerNumber];
    }

    public Controller[] getControllers() {
        return controllers;
    }

    private static class CustomController extends Controller {
        public CustomController(int port) {
            super(port);
        }

        @Override
        public double getLeftY() {
            return -super.getRawAxis(0);
        }

        @Override
        public double getLeftX() {
            return super.getRawAxis(1);
        }

        @Override
        public double getRightY() {
            return -super.getRawAxis(2);
        }

        @Override
        public double getRightX() {
            return super.getRawAxis(3);
        }
    }

    private static class CustomXboxController extends Controller {
        public CustomXboxController(int port) {
            super(port);
        }

        @Override
        public double getLeftY() {
            return -super.getRawAxis(XboxController.Axis.kLeftY.value);
        }

        @Override
        public double getLeftX() {
            return super.getRawAxis(XboxController.Axis.kLeftX.value);
        }

        @Override
        public double getRightY() {
            return -super.getRawAxis(XboxController.Axis.kRightY.value);
        }

        @Override
        public double getRightX() {
            return super.getRawAxis(XboxController.Axis.kRightX.value);
        }
    }
}
