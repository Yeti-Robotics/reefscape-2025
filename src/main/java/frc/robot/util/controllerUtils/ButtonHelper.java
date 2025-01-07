package frc.robot.util.controllerUtils;


import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

/**
 * Creates, stores, and manages buttons.
 * All buttons created by ButtonHelper are MultiButtons
 */
public class ButtonHelper {
    private static final int maxAxis = 32;
    private static final int maxPOV = 16;
    private final Controller[] controllers;
    /**
     * Map of controllers and their associated button mappings
     */
    private final HashMap<Controller, HashMap<Byte, MultiButton>> buttonMaps = new HashMap<>();
    private Controller controller;

    public ButtonHelper(Controller[] controllers) {
        this.controllers = controllers;
        for (Controller c : controllers) {
            buttonMaps.put(c, new HashMap<>());
        }

        controller = controllers[0];
    }

    /**
     * Returns a string representing the button's ButtonType and its real port number
     *
     * @param buttonID the buttons unique 8 bit ID
     * @return the button's type and port
     */
    public static String buttonIDToString(byte buttonID) {
        String type;
        int port = (buttonID & 0xFF);
        byte binaryType = (byte) ((buttonID & 0xFF) >> 6);

        switch (binaryType) {
            case 0b00000000:
                type = "Button";
                port = buttonID;
                break;
            case 0b00000001:
                type = "Axis";
                port = port ^ 0b01000000;
                if (port >= maxAxis) {
                    port -= maxAxis;
                }
                break;
            case 0b00000010:
                type = "POV";
                port = port ^ 0b10000000;
                if (port >= maxPOV) {
                    port %= maxPOV;
                }
                break;
            default:
                type = "UNKNOWN";
                break;
        }

        return String.format("Button Type: %s || Port: %d\n", type, port);
    }

    private MultiButton createButton(
            BooleanSupplier supplier,
            byte buttonID,
            int layer,
            Command command,
            MultiButton.RunCondition runCondition) {

        if (buttonID == (byte) 0b11111111) {
            System.out.println("FAILED to create button!!!");
            return null;
        }

        if (buttonMaps.get(controller).containsKey(buttonID)) {
            buttonMaps.get(controller).get(buttonID).addLayer(layer, command, runCondition);
            return buttonMaps.get(controller).get(buttonID);
        }

        MultiButton multiButton = new MultiButton(
                supplier,
                buttonID,
                layer,
                command,
                runCondition);

        buttonMaps.get(controller).put(buttonID, multiButton);
        return multiButton;
    }

    private MultiButton createButton(
            BooleanSupplier supplier,
            byte buttonID) {

        if (buttonID == (byte) 0b11111111) {
            System.out.println("FAILED to create button!!!");
            return null;
        }

        if (buttonMaps.get(controller).containsKey(buttonID)) {
            return buttonMaps.get(controller).get(buttonID);
        }

        MultiButton multiButton = new MultiButton(
                supplier,
                buttonID);

        buttonMaps.get(controller).put(buttonID, multiButton);
        return multiButton;
    }

    /**
     * Creates a MultiButton from a normal push-type button on a controller
     *
     * @param buttonPort   the button's port number
     * @param layer        the layer the command will be assigned to on the button
     * @param command      a command to run
     * @param runCondition the condition to run the command
     * @return instance of the MultiButton created. Does not have to be stored.
     */
    public MultiButton createButton(
            int buttonPort,
            int layer,
            Command command,
            MultiButton.RunCondition runCondition) {


        return createButton(
                () -> controller.getRawButton(buttonPort),
                getButtonID(buttonPort),
                layer,
                command,
                runCondition);
    }

    /**
     * Creates a MultiButton from a normal push-type button on a controller
     * This method does not bind any commands. It is just to instantiate a button.
     *
     * @param buttonPort the button's port number
     * @return instance of the MultiButton created. Does not have to be stored.
     */
    public MultiButton createButton(int buttonPort) {

        return createButton(
                () -> controller.getRawButton(buttonPort),
                getButtonID(buttonPort));
    }

    /**
     * Creates a MultiButton from an analog axis on a controller
     *
     * @param axisPort     the axis's port number
     * @param layer        the layer the command will be assigned to on the button
     * @param command      a command to run
     * @param runCondition the condition to run the command
     * @param threshold    a value from -1.0 to 1.0 that triggers the button
     * @return instance of the MultiButton created. Does not have to be stored.
     */
    public MultiButton createAxisButton(
            int axisPort,
            int layer,
            Command command,
            MultiButton.RunCondition runCondition,
            double threshold) {
        BooleanSupplier axisSupplier = () -> {
            if (threshold < 0.0) {
                return controller.getRawAxis(axisPort) < threshold;
            }
            return controller.getRawAxis(axisPort) > threshold;
        };

        return createButton(
                axisSupplier,
                getButtonID(axisPort, threshold < 0.0),
                layer,
                command,
                runCondition);
    }

    /**
     * Creates a MultiButton from an analog axis on a controller
     * This method does not bind any commands. It is just to instantiate a button.
     *
     * @param axisPort  the axis's port number
     * @param threshold a value from -1.0 to 1.0 that triggers the button
     * @return instance of the MultiButton created. Does not have to be stored.
     */
    public MultiButton createAxisButton(
            int axisPort,
            double threshold) {
        BooleanSupplier axisSupplier = () -> {
            if (threshold < 0.0) {
                return controller.getRawAxis(axisPort) < threshold;
            }
            return controller.getRawAxis(axisPort) > threshold;
        };

        return createButton(
                axisSupplier,
                getButtonID(axisPort, threshold < 0.0));
    }

    /**
     * Creates a MultiButton from a POV button on a controller
     *
     * @param povPort      the POV's port number
     * @param layer        the layer the command will be assigned to on the button
     * @param command      a command to run
     * @param runCondition the condition to run the command
     * @param direction    the direction of the POV
     * @return instance of the MultiButton created. Does not have to be stored.
     */
    public MultiButton createPOVButton(
            int povPort,
            POVDirections direction,
            int layer,
            Command command,
            MultiButton.RunCondition runCondition) {
        BooleanSupplier povSupplier = () -> controller.getPOV(povPort) == direction.value;

        return createButton(
                povSupplier,
                getButtonID(povPort, direction),
                layer,
                command,
                runCondition);
    }

    /**
     * Creates a MultiButton from a POV button on a controller
     * This method does not bind any commands. It is just to instantiate a button.
     *
     * @param povPort   the POV's port number
     * @param direction the direction of the POV
     * @return instance of the MultiButton created. Does not have to be stored.
     */
    public MultiButton createPOVButton(
            int povPort,
            POVDirections direction) {
        BooleanSupplier povSupplier = () -> controller.getPOV(povPort) == direction.value;

        return createButton(
                povSupplier,
                getButtonID(povPort, direction));
    }

    public void setController(int controllerNumber) {
        controller = controllers[controllerNumber];
    }

    public void setAllLayers(int layer) {
        MultiButton.syncLayers(layer);
    }

    public int getAllLayers() {
        return MultiButton.getSyncLayer();
    }

    public void setButtonLayer(int controllerNumber, byte buttonID, int layer) {
        buttonMaps.get(controllers[controllerNumber]).get(buttonID).setButtonLayer(layer);
    }

    public int getButtonLayer(int controllerNumber, byte buttonID) {
        return buttonMaps.get(controllers[controllerNumber]).get(buttonID).getButtonLayer();
    }

    public HashMap<Byte, MultiButton> getButtonMap(int controllerNumber) {
        return buttonMaps.get(controllers[controllerNumber]);
    }

    public MultiButton getButton(int controllerNumber, byte buttonID) {
        return buttonMaps.get(controllers[controllerNumber]).get(buttonID);
    }

    /**
     * Returns a unique byte base on the type of button and its port number.
     * This is used to identify each button.
     * <p>
     * The first 6 bits represents the button's port number.
     * The 7th and 8th bits represents the button type.
     * If the 7th bit is true, then the button is an axis; 8th bit is true, then it is a POV.
     * If both bits are false, then it is a normal button; both are false, then it is an invalid button
     * <p>
     * In order to differentiate between the different directions on a POV or axis, a unique ID
     * must be created for each direction. This results in a limit of 32 axis buttons and 16 POV buttons.
     *
     * @param type the type of button
     * @param port the port number of the button
     * @return a byte that represents the unique ID of the button
     */
    private byte getButtonID(ButtonType type, Integer port) {
        if (port > 63) {
            return (byte) 0b11111111;
        }
        switch (type) {
            case BUTTON:
                return port.byteValue();
            case AXIS:
                return (byte) (0b01000000 | port.byteValue());
            case POV:
                return (byte) (0b10000000 | port.byteValue());
        }

        return (byte) 0b11111111;
    }

    /**
     * @param port the port number of the button
     * @return a byte that represents the unique ID of the button
     * @see ButtonHelper#getButtonID(ButtonType, Integer)
     */
    public byte getButtonID(Integer port) {
        return getButtonID(ButtonType.BUTTON, port);
    }

    /**
     * @param port       the port number of the axis
     * @param isNegative is the button using the positive or negative values of an axis
     * @return a byte that represents the unique ID of the button
     * @see ButtonHelper#getButtonID(ButtonType, Integer)
     */
    public byte getButtonID(Integer port, boolean isNegative) {
        int imaginaryAxisPort = isNegative ? port + maxAxis : port;

        return getButtonID(ButtonType.AXIS, imaginaryAxisPort);
    }

    /**
     * @param port      the port number of the axis
     * @param direction the associated direction of the POV for this button
     * @return a byte that represents the unique ID of the button
     * @see ButtonHelper#getButtonID(ButtonType, Integer)
     */
    public byte getButtonID(Integer port, POVDirections direction) {
        /*
         * Getting a unique port number based on the direction
         */
        int imaginaryPOVPort = (Math.max((direction.value / 90 * maxPOV - 1), 0)) + port;

        return getButtonID(ButtonType.POV, imaginaryPOVPort);
    }

    public enum ButtonType {
        BUTTON,
        AXIS,
        POV
    }
}