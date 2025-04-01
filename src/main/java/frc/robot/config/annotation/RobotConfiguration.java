package frc.robot.config.annotation;

import static java.lang.annotation.RetentionPolicy.RUNTIME;

import frc.robot.config.RobotConfig;
import jakarta.inject.Qualifier;
import java.lang.annotation.Documented;
import java.lang.annotation.Retention;

@Qualifier
@Documented
@Retention(RUNTIME)
public @interface RobotConfiguration {
    RobotConfig value();
}
