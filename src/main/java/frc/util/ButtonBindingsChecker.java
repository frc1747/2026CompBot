package frc.util;

import com.github.javaparser.*;
import com.github.javaparser.ast.*;
import com.github.javaparser.ast.body.VariableDeclarator;
import com.github.javaparser.ast.expr.*;
import java.io.File;
import java.util.*;

// Disclaimer: This file is almost entirely AI generated
public class ButtonBindingsChecker {

    static class Binding {
        String controller;
        Set<String> buttons;
        int line;

        Binding(String controller, Set<String> buttons, int line) {
            this.controller = controller;
            this.buttons = buttons;
            this.line = line;
        }

        @Override
        public String toString() {
            return controller + buttons + " @ line " + line;
        }
    }

    static class Declaration {
        String varName;
        String controller;
        String buttonId;
        int line;

        Declaration(String varName, String controller, String buttonId, int line) {
            this.varName = varName;
            this.controller = controller;
            this.buttonId = buttonId;
            this.line = line;
        }

        @Override
        public String toString() {
            return varName + "(" + controller + "," + buttonId + ") @ line " + line;
        }
    }

    public static void main(String[] args) throws Exception {
        String filePath = "src/main/java/frc/robot/RobotContainer.java";

        File file = new File(filePath);
        if (!file.exists()) {
            System.err.println("❌ File not found: " + filePath);
            System.exit(1);
        }

        CompilationUnit cu = StaticJavaParser.parse(file);

        List<Binding> bindings = new ArrayList<>();

        cu.findAll(MethodCallExpr.class).forEach(call -> {
            if (isBindingCall(call) && isRootBindingCall(call)) {
                Binding binding = extractBinding(call);
                if (binding != null) {
                    bindings.add(binding);
                }
            }
        });

        // Collect explicit Joystick/POV button object declarations so we
        // can detect multiple variables that wrap the same physical
        // controller/button pair (which is often accidental).
        List<Declaration> decls = extractButtonDeclarations(cu);

        boolean hasError = checkConflicts(bindings);

        // Also check for duplicate declarations; treat as an error too.
        hasError = checkDeclarationConflicts(decls) || hasError;

        if (hasError) {
            System.exit(1);
        } else {
            System.out.println("✅ No controller conflicts found");
        }
    }

    // Find variable declarations like:
    //   JoystickButton foo = new JoystickButton(operator, 2);
    //   POVButton pov = new POVButton(operator, 0);
    static List<Declaration> extractButtonDeclarations(CompilationUnit cu) {
        List<Declaration> decls = new ArrayList<>();

        cu.findAll(VariableDeclarator.class).forEach(var -> {
            // We only care about JoystickButton and POVButton types
            String typeName = var.getType().asString();
            if (!(typeName.equals("JoystickButton") || typeName.equals("POVButton"))) {
                return;
            }

            if (var.getInitializer().isPresent()) {
                Expression init = var.getInitializer().get();
                if (init instanceof ObjectCreationExpr oc) {
                    String created = oc.getType().getNameAsString();
                    if (created.equals("JoystickButton") || created.equals("POVButton")) {
                        // Expect two arguments: (controllerExpr, buttonId)
                        if (oc.getArguments().size() >= 2) {
                            Expression controllerExpr = oc.getArgument(0);
                            Expression idExpr = oc.getArgument(1);

                            String controller = null;
                            if (controllerExpr instanceof NameExpr ne) {
                                controller = ne.getNameAsString();
                            } else if (controllerExpr instanceof FieldAccessExpr fae) {
                                controller = fae.toString();
                            }

                            String buttonId = idExpr.toString();

                            if (controller != null) {
                                String varName = var.getNameAsString();
                                int line = var.getBegin().map(p -> p.line).orElse(-1);
                                decls.add(new Declaration(varName, controller, buttonId, line));
                            }
                        }
                    }
                }
            }
        });

        return decls;
    }

    static boolean checkDeclarationConflicts(List<Declaration> decls) {
        Map<String, List<Declaration>> byKey = new HashMap<>();

        for (Declaration d : decls) {
            String key = d.controller + ":" + d.buttonId;
            byKey.computeIfAbsent(key, k -> new ArrayList<>()).add(d);
        }

        boolean error = false;
        for (var entry : byKey.entrySet()) {
            List<Declaration> list = entry.getValue();
            if (list.size() > 1) {
                // Report all declarations that collide on same controller/button
                System.out.println("❌ Duplicate Joystick/POV declaration for " + entry.getKey());
                for (Declaration d : list) {
                    System.out.println("    " + d.varName + " at line " + d.line);
                }
                error = true;
            }
        }

        return error;
    }

    // Only treat THESE as binding entry points
    static boolean isBindingCall(MethodCallExpr call) {
        String name = call.getNameAsString();
        return name.equals("whileTrue")
            || name.equals("onTrue")
            || name.equals("toggleOnTrue");
    }

    // Ensure we only process ONE call per chain
    static boolean isRootBindingCall(MethodCallExpr call) {
        // If parent is also a binding call, skip this one
        if (call.getParentNode().isPresent() &&
            call.getParentNode().get() instanceof MethodCallExpr parent) {

            return !isBindingCall(parent);
        }
        return true;
    }

    static Binding extractBinding(MethodCallExpr call) {
        // Start from the expression that the binding is called on (the scope
        // of the binding call). For example, in
        //   driver.back().and(driver.y()).whileTrue(...)
        // the scope of the binding call (whileTrue) is the whole combo
        // expression (back().and(...)). We must parse that entire scope so
        // combos are collected correctly. The previous implementation walked
        // down to the left-most method call which missed the `.and(...)`
        // argument.
        Expression expr = call.getScope().orElse(null);

        Set<String> buttons = new HashSet<>();
        String controller = extractButtons(expr, buttons);

        if (controller == null || buttons.isEmpty()) return null;

        int line = call.getBegin().map(p -> p.line).orElse(-1);

        return new Binding(controller, buttons, line);
    }

    // Recursively extract buttons and combos
    static String extractButtons(Expression expr, Set<String> buttons) {
        if (expr instanceof MethodCallExpr m) {
            String name = m.getNameAsString();

            // Handle .and(...)
            if (name.equals("and")) {
                extractButtons(m.getScope().orElse(null), buttons);
                extractButtons(m.getArgument(0), buttons);
                return null;
            }

            // Handle driver.a()
            if (m.getScope().isPresent()) {
                Expression scope = m.getScope().get();

                if (scope instanceof NameExpr controller) {
                    buttons.add(name);
                    return controller.getNameAsString();
                } else {
                    return extractButtons(scope, buttons);
                }
            }
        }

        return null;
    }

    static boolean checkConflicts(List<Binding> bindings) {
        Map<String, List<Binding>> byController = new HashMap<>();

        for (Binding b : bindings) {
            byController.computeIfAbsent(b.controller, k -> new ArrayList<>()).add(b);
        }

        boolean error = false;

        for (var entry : byController.entrySet()) {
            String controller = entry.getKey();
            List<Binding> list = entry.getValue();

            for (int i = 0; i < list.size(); i++) {
                for (int j = i + 1; j < list.size(); j++) {
                    Binding a = list.get(i);
                    Binding b = list.get(j);

                    // Exact duplicate
                    if (a.buttons.equals(b.buttons)) {
                        System.out.println("❌ Duplicate binding on " + controller +
                                " buttons " + a.buttons +
                                " at lines " + a.line + " and " + b.line);
                        error = true;
                    }

                    // Single vs combo conflict
                    if (a.buttons.size() == 1 && b.buttons.containsAll(a.buttons)) {
                        System.out.println("❌ Conflict: " + controller + " " + a.buttons +
                                " used alone (line " + a.line + ") AND in combo " +
                                b.buttons + " (line " + b.line + ")");
                        error = true;
                    }

                    if (b.buttons.size() == 1 && a.buttons.containsAll(b.buttons)) {
                        System.out.println("❌ Conflict: " + controller + " " + b.buttons +
                                " used alone (line " + b.line + ") AND in combo " +
                                a.buttons + " (line " + a.line + ")");
                        error = true;
                    }
                }
            }
        }

        return error;
    }
}
