/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *              Copyright (C) 2023.
 *
 * See the NOTICE file distributed with this work for additional information
 * regarding copyright ownership.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *             http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package stark.speclang.parsing;

import stark.speclang.StarkSpecificationLanguageBaseVisitor;
import stark.speclang.StarkSpecificationLanguageParser;
import stark.speclang.types.*;
import stark.speclang.types.LocalTypeContext.*;
import java.util.*;
import java.util.stream.*;

public class SpecificationLanguageValidator extends StarkSpecificationLanguageBaseVisitor<Boolean> {

    /*
    private final ParseErrorCollector errors;
    private final SymbolTable symbols = new SymbolTable();

    private final Set<String> environmentVariables = new HashSet<>();

    private final GlobalTypeContext globalTypeContext = new GlobalTypeContext();

    public SpecificationLanguageValidator(ParseErrorCollector errors) {
        this.errors = errors;
    }

    @Override
    public Boolean visitStarkSpecificationModel(StarkSpecificationLanguageParser.StarkSpecificationModelContext ctx) {
        boolean flag = true;
        for (StarkSpecificationLanguageParser.ElementContext element: ctx.element()) {
            flag &= element.accept(this);
        }
        return flag;
    }

    @Override
    protected Boolean defaultResult() {
        return true;
    }

    @Override
    protected Boolean aggregateResult(Boolean aggregate, Boolean nextResult) {
        return aggregate&nextResult;
    }

    @Override
    public Boolean visitDeclarationFunction(StarkSpecificationLanguageParser.DeclarationFunctionContext ctx) {
        if (checkIfNotDuplicated(ctx.name.getText(), ctx)) {
            StarkType[] argumentType = ctx.arguments.stream().map(a -> typeOf(a.type())).toArray(StarkType[]::new);
            TypeEvaluationContext context = new NestedTypeContext(
                    new LocalTypeContext(getLocalDeclarations(ctx.arguments)),
                    this.globalTypeContext);
            StarkType returnType = ctx.accept(new StarkFunctionStatementTypeInference(context, this.errors));
            if (!returnType.isError()) {
                this.symbols.recordFunction(ctx.name.getText(), argumentType, returnType, ctx);
                return true;
            }
        }
        return false;
    }


    private Map<String, StarkType> getLocalDeclarations(List<StarkSpecificationLanguageParser.FunctionArgumentContext> arguments) {
        return arguments.stream().collect(Collectors.toMap(a -> a.name.getText(), a -> typeOf(a.type())));
    }

    private StarkType typeOf(StarkSpecificationLanguageParser.TypeContext type) {
        if (type instanceof StarkSpecificationLanguageParser.BooleanTypeContext) {
            return StarkType.BOOLEAN_TYPE;
        }
        if (type instanceof StarkSpecificationLanguageParser.IntegerTypeContext) {
            return StarkType.INTEGER_TYPE;
        }
        if (type instanceof StarkSpecificationLanguageParser.RealTypeContext) {
            return StarkType.REAL_TYPE;
        }
        if (type instanceof StarkSpecificationLanguageParser.CustomTypeContext) {
            String typeName = ((StarkSpecificationLanguageParser.CustomTypeContext) type).name.getText();
            if (!symbols.isACustomType(typeName)) {
                this.errors.record(ParseUtil.unknownType(((StarkSpecificationLanguageParser.CustomTypeContext) type).name));
                return StarkType.ERROR_TYPE;
            }
            return this.symbols.getCustomType(typeName);
        }
        return StarkType.ERROR_TYPE;
    }


    @Override
    public Boolean visitDeclarationEnvironmnet(StarkSpecificationLanguageParser.DeclarationEnvironmnetContext ctx) {
        TypeEvaluationContext localContext = getLocalContext(ctx.localVariables);
        boolean flag = true;
        ExpressionTypeInference inference = new ExpressionTypeInference(new NestedTypeContext(localContext, symbols), errors, true);
        for(StarkSpecificationLanguageParser.VariableAssignmentContext varAssignment: ctx.assignments) {
            flag &= checkEnvironmentVariableUpdate(inference, varAssignment);
        }
        return flag;
    }

    private boolean checkEnvironmentVariableUpdate(ExpressionTypeInference inference, StarkSpecificationLanguageParser.VariableAssignmentContext varAssignment) {
        boolean flag = (varAssignment.guard == null || inference.checkType(StarkType.BOOLEAN_TYPE, varAssignment.guard));
        Optional<StarkType> expectedType = retrieveAndCheckVariableExpression(inference, varAssignment.target);
        return flag&expectedType.map(t -> inference.checkType(t, varAssignment.value)).orElse(false);
    }

    @Override
    public Boolean visitDeclarationConstant(StarkSpecificationLanguageParser.DeclarationConstantContext ctx) {
        if (checkIfNotDuplicated(ctx.name.getText(), ctx)) {
            StarkType type = ctx.expression().accept(new ExpressionTypeInference(symbols, errors));
            if (!type.isError()) {
                symbols.recordConstant(ctx.name.getText(), type, ctx);
                return true;
            } else {
                return false;
            }
        }
        return false;
    }

    @Override
    public Boolean visitDeclarationType(StarkSpecificationLanguageParser.DeclarationTypeContext ctx) {
        String customTypeName = ctx.name.getText();
        if (checkIfNotDuplicated(customTypeName, ctx)
            &&ctx.elements.stream().allMatch(e -> checkCustomTypeElement(ctx, e))) {
            symbols.recordCustomType(ctx);
            return true;
        }
        return false;
    }

    private boolean checkCustomTypeElement(StarkSpecificationLanguageParser.DeclarationTypeContext ctx, StarkSpecificationLanguageParser.TypeElementDeclarationContext e) {
        if (checkIfNotDuplicated(e.name.getText(), e)) {
            if (ctx.name.getText().equals(e.name.getText())) {
                this.errors.record(ParseUtil.duplicatedSymbol(ctx.name.getText(), ctx, e));
            } else {
                return true;
            }
        }
        return false;
    }

    @Override
    public Boolean visitDeclarationComponent(StarkSpecificationLanguageParser.DeclarationComponentContext ctx) {
        return super.visitDeclarationComponent(ctx);
    }

    @Override
    public Boolean visitDeclarationPenalty(StarkSpecificationLanguageParser.DeclarationPenaltyContext ctx) {
        return super.visitDeclarationPenalty(ctx);
    }

    @Override
    public Boolean visitDeclarationVariables(StarkSpecificationLanguageParser.DeclarationVariablesContext ctx) {
        return super.visitDeclarationVariables(ctx);
    }

     */
}
