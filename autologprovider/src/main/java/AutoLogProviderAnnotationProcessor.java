import annotation.AutoLogDataProvider;
import com.palantir.javapoet.ClassName;
import com.palantir.javapoet.TypeName;

import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.RoundEnvironment;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.PackageElement;
import javax.lang.model.element.TypeElement;
import java.util.Optional;
import java.util.Set;

public class AutoLogProviderAnnotationProcessor extends AbstractProcessor {
    private static final TypeName LOG_TABLE_TYPE = ClassName.get("org.littletonrobotics.junction", "LogTable");

    private static String getPackageName(Element e) {
        while (e != null) {
            if (e.getKind().equals(ElementKind.PACKAGE)) {
                return ((PackageElement) e).getQualifiedName().toString();
            }
            e = e.getEnclosingElement();
        }

        return null;
    }


    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        Optional<? extends TypeElement> autoLogDataProvider = annotations.stream()
                .filter(annotation -> annotation.getSimpleName().toString().equals(AutoLogDataProvider.class.getSimpleName()))
                .findFirst();

        if (autoLogDataProvider.isPresent()) {
            TypeElement autoLogDataProviderElement = autoLogDataProvider.get();
            String packageName = getPackageName(autoLogDataProviderElement);

        }
    }
}
