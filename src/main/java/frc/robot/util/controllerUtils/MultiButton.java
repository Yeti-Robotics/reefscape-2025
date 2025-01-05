package frc.robot.util.controllerUtils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.Arrays;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;


public class MultiButton {
    private static boolean isLayersSynced;
    private static int syncLayer;
    private final BooleanSupplier trigger;
    private final byte buttonID;
    private BiConsumer<Boolean, Boolean>[] buttonActions;
    private boolean pressed;
    private boolean pressedLast = false;
    private int layerCount;
    private int buttonLayer = 0;

    @SuppressWarnings("unchecked")
    public MultiButton(BooleanSupplier trigger, byte buttonID, int layer, Command command, RunCondition runCondition) {
        this.trigger = trigger;
        this.buttonID = buttonID;
        this.layerCount = layer + 1;
        buttonActions = new BiConsumer[layerCount];

        CommandScheduler.getInstance().getDefaultButtonLoop().bind(this::updateButton);

        addLayer(layer, command, runCondition);
    }

    public MultiButton(BooleanSupplier trigger, byte buttonID) {
        this(trigger, buttonID, 0, new InstantCommand(() -> {}), RunCondition.WHEN_PRESSED);
    }

    public static void syncLayers(int layer) {
        isLayersSynced = true;
        syncLayer = layer;
    }

    public static int getSyncLayer() {
        return syncLayer;
    }

    public void addLayer(int layer, Command command, RunCondition runCondition) {
        if (layerCount <= layer) {
            layerCount = layer + 1;
            buttonActions = Arrays.copyOf(buttonActions, layerCount);
        }

        buttonActions[layer] = defineButton(command, runCondition);

        int i = 0;
        for (Object o : buttonActions) {
            if (o == null) {
                buttonActions[i] = (aBoolean, aBoolean2) -> {
                };
            }
            i++;
        }
    }

    public void updateButton() {
        if (isLayersSynced) {
            setAllLayers();
        }
        pressed = trigger.getAsBoolean();
        buttonActions[buttonLayer].accept(pressed, pressedLast);
        pressedLast = pressed;
    }

    public int getButtonLayer() {
        return buttonLayer;
    }

    public void setButtonLayer(int layer) {
        isLayersSynced = false;
        buttonLayer = layer % layerCount;
    }

    public void setAllLayers() {
        buttonLayer = syncLayer % layerCount;
    }

    public byte getButtonID() {
        return buttonID;
    }

    public boolean isPressed() {
        return pressed;
    }

    @Override
    public String toString() {
        return ButtonHelper.buttonIDToString(buttonID);
    }

    public enum RunCondition {
        WHEN_PRESSED,
        WHEN_RELEASED,
        WHILE_HELD,
        TOGGLE_WHEN_PRESSED,
        CANCEL_WHEN_PRESSED
    }

    private static BiConsumer<Boolean, Boolean> defineButton(Command command, RunCondition runCondition) {
        BiConsumer<Boolean, Boolean> biConsumer;

        if (command == null || runCondition == null) {
            biConsumer = (aBoolean, bBoolean) -> {
            };
            return biConsumer;
        }

        switch (runCondition) {
            case WHEN_PRESSED:
                biConsumer = (pressed, pressedLast) -> {
                    if (!pressedLast && pressed) {
                        command.schedule();
                    }
                };
                break;
            case WHEN_RELEASED:
                biConsumer = (pressed, pressedLast) -> {
                    if (pressedLast && !pressed) {
                        command.schedule();
                    }
                };
                break;
            case WHILE_HELD:
                biConsumer = (pressed, pressedLast) -> {
                    if (pressed) {
                        command.schedule();
                    } else if (!pressedLast) {
                        command.cancel();
                    }
                };
                break;
            case TOGGLE_WHEN_PRESSED:
                biConsumer = (pressed, pressedLast) -> {
                    if (!pressedLast && pressed) {
                        if (command.isScheduled()) {
                            command.cancel();
                        } else {
                            command.schedule();
                        }
                    }
                };
                break;
            case CANCEL_WHEN_PRESSED:
                biConsumer = (pressed, pressedLast) -> {
                    if (!pressedLast && pressed) {
                        command.cancel();
                    }
                };
                break;
            default:
                biConsumer = null;
                break;
        }
        return biConsumer;
    }
}