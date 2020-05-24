/*****************************************************************************/
/*                                                                           */
/* Copyright (c) 2020 Seoul National University.                             */
/* All rights reserved.                                                      */
/*                                                                           */
/* Redistribution and use in source and binary forms, with or without        */
/* modification, are permitted provided that the following conditions        */
/* are met:                                                                  */
/*   1. Redistributions of source code must retain the above copyright       */
/*      notice, this list of conditions and the following disclaimer.        */
/*   2. Redistributions in binary form must reproduce the above copyright    */
/*      notice, this list of conditions and the following disclaimer in the  */
/*      documentation and/or other materials provided with the distribution. */
/*   3. Neither the name of Seoul National University nor the names of its   */
/*      contributors may be used to endorse or promote products derived      */
/*      from this software without specific prior written permission.        */
/*                                                                           */
/* THIS SOFTWARE IS PROVIDED BY SEOUL NATIONAL UNIVERSITY "AS IS" AND ANY    */
/* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED */
/* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE    */
/* DISCLAIMED. IN NO EVENT SHALL SEOUL NATIONAL UNIVERSITY BE LIABLE FOR ANY */
/* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL        */
/* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS   */
/* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)     */
/* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       */
/* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  */
/* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           */
/* POSSIBILITY OF SUCH DAMAGE.                                               */
/*                                                                           */
/* Contact information:                                                      */
/*   Center for Manycore Programming                                         */
/*   Department of Computer Science and Engineering                          */
/*   Seoul National University, Seoul 08826, Korea                           */
/*   http://aces.snu.ac.kr                                                   */
/*                                                                           */
/* Contributors:                                                             */
/*   Gangwon Jo, Heehoon Kim, Jeesoo Lee, and Jaejin Lee                     */
/*                                                                           */
/*****************************************************************************/

#include "clang/SnuPreprocess/FunctionNormalizer.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/Attr.h"
#include "clang/AST/Decl.h"
#include "clang/AST/DeclCXX.h"
#include "clang/AST/Expr.h"
#include "clang/AST/ExprCXX.h"
#include "clang/AST/ExprObjC.h"
#include "clang/AST/Stmt.h"
#include "clang/AST/StmtCXX.h"
#include "clang/AST/StmtObjC.h"
#include "clang/AST/StmtOpenMP.h"
#include "clang/AST/StmtVisitor.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuPreprocess/ASTBuilder.h"
#include "clang/SnuPreprocess/ASTDuplicator.h"
#include "llvm/ADT/SmallVector.h"

namespace clang {

namespace snu {

void FunctionNormalizer::NormalizeAll(TranslationUnitDecl *TU) {
  for (DeclContext::decl_iterator D = TU->decls_begin(), DEnd = TU->decls_end();
       D != DEnd; ++D) {
    if (FunctionDecl *FD = dyn_cast<FunctionDecl>(*D)) {
      if (FD->isThisDeclarationADefinition()) {
        NormalizeFunction(FD);
      }
    }
  }
}

void FunctionNormalizer::NormalizeFunction(FunctionDecl *FD) {
  if (FD->hasBody() &&
      (!KernelOnly || FD->hasAttr<OpenCLKernelAttr>() ||
       FD->hasAttr<CUDAGlobalAttr>())) {
    DeclCtx = FD;
    Builder.setDeclContext(FD);

    FD->setBody(Visit(FD->getBody()));

    if (!TopLevelDeclStmts.empty()) {
      SmallVector<Stmt *, 64> NewStmts;

      NewStmts.append(TopLevelDeclStmts.begin(), TopLevelDeclStmts.end());
      TopLevelDeclStmts.clear();

      if (CompoundStmt *CS = dyn_cast<CompoundStmt>(FD->getBody())) {
        NewStmts.append(CS->body_begin(), CS->body_end());
      } else {
        NewStmts.push_back(FD->getBody());
      }

      FD->setBody(Builder.GetCompoundStmt(NewStmts));
    }

    DeclCtx = NULL;
    Builder.setDeclContext(NULL);
  }
}

// Statements

Stmt *FunctionNormalizer::VisitNullStmt(NullStmt *Node) {
  return Node;
}

Stmt *FunctionNormalizer::VisitCompoundStmt(CompoundStmt *Node) {
  SmallVector<Stmt *, 16> NewStmts;
  bool StmtInserted = false;

  SmallVector<Stmt *, 4> PreStmts;
  SmallVectorImpl<Stmt *> *PreviousPreStmts = CurrentPreStmts;
  CurrentPreStmts = &PreStmts;

  for (CompoundStmt::body_iterator S = Node->body_begin(), SEnd = Node->body_end();
       S != SEnd; ++S) {
    Stmt *NewStmt = Visit(*S);
    if (!PreStmts.empty()) {
      NewStmts.append(PreStmts.begin(), PreStmts.end());
      PreStmts.clear();
      StmtInserted = true;
    }
    if (NewStmt) {
      *S = NewStmt;
      NewStmts.push_back(NewStmt);
    }
  }
  if (StmtInserted) {
    Node->setStmts(ASTCtx, NewStmts.begin(), NewStmts.size());
  }

  CurrentPreStmts = PreviousPreStmts;
  return Node;
}

Stmt *FunctionNormalizer::VisitLabelStmt(LabelStmt *Node) {
  Stmt *NewSubStmt = Visit(Node->getSubStmt());
  if (!CurrentPreStmts->empty()) {
    Node->setSubStmt(CurrentPreStmts->front());
    CurrentPreStmts->front() = Node;
    return NewSubStmt;
  }
  if (NewSubStmt) {
    Node->setSubStmt(NewSubStmt);
    return Node;
  } else {
    return NULL;
  }
}

Stmt *FunctionNormalizer::VisitAttributedStmt(AttributedStmt *Node) {
  Stmt *NewSubStmt = VisitBlock(Node->getSubStmt());
  if (NewSubStmt) {
    Node = Builder.SetSubStmtOfAttributedStmt(Node, NewSubStmt);
  } else {
    Node = NULL;
  }
  return Node;
}

Stmt *FunctionNormalizer::VisitIfStmt(IfStmt *Node) {
  if (VarDecl *CondVar = Node->getConditionVariable()) {
    VarDecl *NewCondVar = VisitConditionVariable(CondVar);
    if (CondVar != NewCondVar) {
      Node->setConditionVariable(ASTCtx, NewCondVar);
    }
  }
  Node->setCond(VisitExpr(Node->getCond()));
  Node->setThen(VisitBlock(Node->getThen()));
  if (Node->getElse()) {
    Node->setElse(VisitBlock(Node->getElse()));
  }
  return Node;
}

Stmt *FunctionNormalizer::VisitSwitchStmt(SwitchStmt *Node) {
  if (VarDecl *CondVar = Node->getConditionVariable()) {
    VarDecl *NewCondVar = VisitConditionVariable(CondVar);
    if (CondVar != NewCondVar) {
      Node->setConditionVariable(ASTCtx, NewCondVar);
    }
  }
  Node->setCond(VisitExpr(Node->getCond()));
  Node->setBody(VisitBlock(Node->getBody()));
  return Node;
}

Stmt *FunctionNormalizer::VisitWhileStmt(WhileStmt *Node) {
  SmallVector<Stmt *, 4> PreStmts;
  SmallVectorImpl<Stmt *> *PreviousPreStmts = CurrentPreStmts;

  CurrentPreStmts = &PreStmts;
  VarDecl *CondVar = Node->getConditionVariable();
  VarDecl *NewCondVar = NULL;
  if (CondVar) {
    NewCondVar = VisitConditionVariable(CondVar);
  }
  Expr *NewCond = VisitExpr(Node->getCond());
  CurrentPreStmts = PreviousPreStmts;

  Stmt *NewBody = VisitBlock(Node->getBody());

  if (!PreStmts.empty()) {
    // while (pre - cond) body
    // =>
    // while (1) {
    //   pre
    //   if (!(cond))
    //     break;
    //   body
    // }
    SmallVector<Stmt *, 16> NewStmts;
    NewStmts.append(PreStmts.begin(), PreStmts.end());
    if (NewCondVar) {
      NewStmts.push_back(Builder.GetDeclStmt(NewCondVar));
    }
    NewStmts.push_back(Builder.GetBreakIfNotStmt(NewCond));
    if (CompoundStmt *CS = dyn_cast<CompoundStmt>(NewBody)) {
      NewStmts.append(CS->body_begin(), CS->body_end());
    } else {
      NewStmts.push_back(NewBody);
    }

    NewCondVar = NULL;
    NewCond = Builder.GetTrue();
    NewBody = Builder.GetCompoundStmt(NewStmts);
  }

  if (CondVar != NewCondVar) {
    Node->setConditionVariable(ASTCtx, NewCondVar);
  }
  Node->setCond(NewCond);
  Node->setBody(NewBody);
  return Node;
}

Stmt *FunctionNormalizer::VisitDoStmt(DoStmt *Node) {
  Stmt *NewBody = VisitBlock(Node->getBody());

  SmallVector<Stmt *, 4> PreStmts;
  SmallVectorImpl<Stmt *> *PreviousPreStmts = CurrentPreStmts;

  CurrentPreStmts = &PreStmts;
  Expr *NewCond = VisitExpr(Node->getCond());
  CurrentPreStmts = PreviousPreStmts;

  if (!PreStmts.empty()) {
    // do body while (pre - cond);
    // =>
    // do {
    //   { body }
    //   pre
    // } while (cond);
    SmallVector<Stmt *, 16> NewStmts;
    NewStmts.push_back(Builder.GetCompoundStmt(NewBody));
    NewStmts.append(PreStmts.begin(), PreStmts.end());

    NewBody = Builder.GetCompoundStmt(NewStmts);
  }

  Node->setBody(NewBody);
  Node->setCond(NewCond);
  return Node;
}

Stmt *FunctionNormalizer::VisitForStmt(ForStmt *Node) {
  if (Node->getInit()) {
    Node->setInit(Visit(Node->getInit()));
  }

  SmallVector<Stmt *, 4> PreStmtsForCond;
  SmallVector<Stmt *, 4> PreStmtsForInc;
  SmallVectorImpl<Stmt *> *PreviousPreStmts = CurrentPreStmts;

  CurrentPreStmts = &PreStmtsForCond;
  VarDecl *CondVar = Node->getConditionVariable();
  VarDecl *NewCondVar = NULL;
  if (CondVar) {
    NewCondVar = VisitConditionVariable(CondVar);
  }
  Expr *NewCond = NULL;
  if (Node->getCond()) {
    NewCond = VisitExpr(Node->getCond());
  }
  // CurrentPreStmts = PreviousPreStmts;

  CurrentPreStmts = &PreStmtsForInc;
  Expr *NewInc = NULL;
  if (Node->getInc()) {
    NewInc = VisitExpr(Node->getInc());
  }
  CurrentPreStmts = PreviousPreStmts;

  Stmt *NewBody = VisitBlock(Node->getBody());

  if (!PreStmtsForCond.empty() || !PreStmtsForInc.empty()) {
    // for (init; precond - cond; preinc - inc) body
    // =>
    // for (init; ; ) {
    //   {
    //     precond
    //     if (!(cond))
    //       break;
    //     body
    //   }
    //   preinc
    //   inc
    // }
    SmallVector<Stmt *, 16> NewStmts;
    NewStmts.append(PreStmtsForCond.begin(), PreStmtsForCond.end());
    if (NewCondVar) {
      NewStmts.push_back(Builder.GetDeclStmt(NewCondVar));
    }
    if (NewCond) {
      NewStmts.push_back(Builder.GetBreakIfNotStmt(NewCond));
    }
    if (CompoundStmt *CS = dyn_cast<CompoundStmt>(NewBody)) {
      NewStmts.append(CS->body_begin(), CS->body_end());
    } else {
      NewStmts.push_back(NewBody);
    }

    CompoundStmt *CS = Builder.GetCompoundStmt(NewStmts);
    NewStmts.clear();
    NewStmts.push_back(CS);
    NewStmts.append(PreStmtsForInc.begin(), PreStmtsForInc.end());
    if (NewInc) {
      NewStmts.push_back(NewInc);
    }

    NewCondVar = NULL;
    NewCond = NULL;
    NewInc = NULL;
    NewBody = Builder.GetCompoundStmt(NewStmts);
  }

  if (CondVar != NewCondVar) {
    Node->setConditionVariable(ASTCtx, NewCondVar);
  }
  Node->setCond(NewCond);
  Node->setInc(NewInc);
  Node->setBody(NewBody);
  return Node;
}

Stmt *FunctionNormalizer::VisitGotoStmt(GotoStmt *Node) {
  return Node;
}

Stmt *FunctionNormalizer::VisitIndirectGotoStmt(IndirectGotoStmt *Node) {
  Node->setTarget(VisitExpr(Node->getTarget()));
  return Node;
}

Stmt *FunctionNormalizer::VisitContinueStmt(ContinueStmt *Node) {
  return Node;
}

Stmt *FunctionNormalizer::VisitBreakStmt(BreakStmt *Node) {
  return Node;
}

Stmt *FunctionNormalizer::VisitReturnStmt(ReturnStmt *Node) {
  if (Node->getRetValue()) {
    Node->setRetValue(VisitExpr(Node->getRetValue()));
  }
  return Node;
}

Stmt *FunctionNormalizer::VisitDeclStmt(DeclStmt *Node) {
  if (NormalizeDecl && !Node->isSingleDecl()) {
    for (DeclStmt::decl_iterator D = Node->decl_begin(),
                                 DEnd = Node->decl_end();
         D != DEnd; ++D) {
      DeclStmt *SingleDeclStmt =
        static_cast<DeclStmt*>(VisitDeclStmt(Builder.GetDeclStmt(*D)));
      if (SingleDeclStmt) {
        CurrentPreStmts->push_back(SingleDeclStmt);
      }
    }
    return NULL;
  }

  if (NormalizeVarDeclInit) {
    CurrentPreStmts->push_back(Node);
    for (DeclStmt::decl_iterator D = Node->decl_begin(),
                                 DEnd = Node->decl_end();
         D != DEnd; ++D) {
      if (VarDecl *VD = dyn_cast<VarDecl>(*D)) {
        if (VD->hasInit()) {
          Expr *NewInit = VisitExpr(VD->getInit());
          assert(NewInit);
          CurrentPreStmts->push_back(Builder.GetAssignToVarExpr(VD, NewInit));
        }
      }
    }
    return NULL;
  }

  for (DeclStmt::decl_iterator D = Node->decl_begin(), DEnd = Node->decl_end();
       D != DEnd; ++D) {
    if (VarDecl *VD = dyn_cast<VarDecl>(*D)) {
      if (VD->hasInit()) {
        VD->setInit(VisitExpr(VD->getInit()));
      }
    } 
  }
  return Node;
}

Stmt *FunctionNormalizer::VisitCaseStmt(CaseStmt *Node) {
  Node->setLHS(VisitExpr(Node->getLHS()));
  if (Node->getRHS()) {
    Node->setRHS(VisitExpr(Node->getRHS()));
  }

  Stmt *NewSubStmt = Visit(Node->getSubStmt());
  if (!CurrentPreStmts->empty()) {
    Node->setSubStmt(CurrentPreStmts->front());
    CurrentPreStmts->front() = Node;
    return NewSubStmt;
  }
  if (NewSubStmt) {
    Node->setSubStmt(NewSubStmt);
    return Node;
  } else {
    return NULL;
  }
}

Stmt *FunctionNormalizer::VisitDefaultStmt(DefaultStmt *Node) {
  Stmt *NewSubStmt = Visit(Node->getSubStmt());
  if (!CurrentPreStmts->empty()) {
    Node->setSubStmt(CurrentPreStmts->front());
    CurrentPreStmts->front() = Node;
    return NewSubStmt;
  }
  if (NewSubStmt) {
    Node->setSubStmt(NewSubStmt);
    return Node;
  } else {
    return NULL;
  }
}

Stmt *FunctionNormalizer::VisitCapturedStmt(CapturedStmt *Node) {
  for (CapturedStmt::capture_init_iterator E = Node->capture_init_begin(), EEnd = Node->capture_init_end();
       E != EEnd; ++E) {
    *E = VisitExpr(*E);
  }
  Stmt *NewCapturedStmt = VisitBlock(Node->getCapturedStmt());
  Node = Builder.SetSubStmtOfCapturedStmt(Node, NewCapturedStmt);
  return Node;
}

Stmt *FunctionNormalizer::VisitBlock(Stmt *Node) {
  if (CompoundStmt *CS = dyn_cast<CompoundStmt>(Node)) {
    return VisitCompoundStmt(CS);
  }

  SmallVector<Stmt *, 4> PreStmts;
  SmallVectorImpl<Stmt *> *PreviousPreStmts = CurrentPreStmts;
  CurrentPreStmts = &PreStmts;

  Stmt *NewNode = Visit(Node);
  if (!PreStmts.empty()) {
    SmallVector<Stmt *, 16> NewStmts;
    NewStmts.append(PreStmts.begin(), PreStmts.end());
    if (NewNode) {
      NewStmts.push_back(NewNode);
    }
    NewNode = Builder.GetCompoundStmt(NewStmts);
  }
  if (!NewNode) {
    NewNode = Builder.GetNullStmt();
  }

  CurrentPreStmts = PreviousPreStmts;
  return NewNode;
}

// Asm statements

Stmt *FunctionNormalizer::VisitGCCAsmStmt(GCCAsmStmt *Node) { return Node; }
Stmt *FunctionNormalizer::VisitMSAsmStmt(MSAsmStmt *Node) { return Node; }

// Obj-C statements

Stmt *FunctionNormalizer::VisitObjCAtTryStmt(ObjCAtTryStmt *Node) { return Node;}
Stmt *FunctionNormalizer::VisitObjCAtCatchStmt(ObjCAtCatchStmt *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCAtFinallyStmt(ObjCAtFinallyStmt *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCAtThrowStmt(ObjCAtThrowStmt *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCAtSynchronizedStmt(ObjCAtSynchronizedStmt *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCForCollectionStmt(ObjCForCollectionStmt *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCAutoreleasePoolStmt(ObjCAutoreleasePoolStmt *Node) { return Node; }

// C++ statements

Stmt *FunctionNormalizer::VisitCXXCatchStmt(CXXCatchStmt *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXTryStmt(CXXTryStmt *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXForRangeStmt(CXXForRangeStmt *Node) { return Node; }

// Expressions

Stmt *FunctionNormalizer::VisitPredefinedExpr(PredefinedExpr *Node) {
  return Node;
}

Stmt *FunctionNormalizer::VisitDeclRefExpr(DeclRefExpr *Node) {
  return Node;
}

Stmt *FunctionNormalizer::VisitIntegerLiteral(IntegerLiteral *Node) {
  return Node;
}

Stmt *FunctionNormalizer::VisitFloatingLiteral(FloatingLiteral *Node) {
  return Node;
}

Stmt *FunctionNormalizer::VisitImaginaryLiteral(ImaginaryLiteral *Node) {
  Node->setSubExpr(VisitExpr(Node->getSubExpr()));
  return Node;
}

Stmt *FunctionNormalizer::VisitStringLiteral(StringLiteral *Node) {
  return Node;
}

Stmt *FunctionNormalizer::VisitCharacterLiteral(CharacterLiteral *Node) {
  return Node;
}

Stmt *FunctionNormalizer::VisitParenExpr(ParenExpr *Node) {
  Expr *NewSubExpr = VisitExpr(Node->getSubExpr());
  if (NewSubExpr) {
    Node->setSubExpr(NewSubExpr);
    return Node;
  } else {
    return NULL;
  }
}

Stmt *FunctionNormalizer::VisitUnaryOperator(UnaryOperator *Node) {
  Node->setSubExpr(VisitExpr(Node->getSubExpr()));
  return Node;
}

Stmt *FunctionNormalizer::VisitOffsetOfExpr(OffsetOfExpr *Node) {
  for (unsigned Index = 0, NumExprs = Node->getNumExpressions();
       Index != NumExprs; ++Index) {
    Node->setIndexExpr(Index, VisitExpr(Node->getIndexExpr(Index)));
  }
  return Node;
}

Stmt *FunctionNormalizer::VisitUnaryExprOrTypeTraitExpr(UnaryExprOrTypeTraitExpr *Node) {
  if (!Node->isArgumentType()) {
    Node->setArgument(VisitExpr(Node->getArgumentExpr()));
  }
  return Node;
}

Stmt *FunctionNormalizer::VisitArraySubscriptExpr(ArraySubscriptExpr *Node) {
  Node->setLHS(VisitExpr(Node->getLHS()));
  Node->setRHS(VisitExpr(Node->getRHS()));
  return Node;
}

static FunctionDecl *GetDefinition(FunctionDecl *FD) {
  const FunctionDecl *CalleeDef = NULL;
  if (!FD || !FD->isDefined(CalleeDef))
    return NULL;
  return const_cast<FunctionDecl *>(CalleeDef);
}

Stmt *FunctionNormalizer::VisitCallExpr(CallExpr *Node) {
  Node->setCallee(VisitExpr(Node->getCallee()));
  for (unsigned Index = 0, NumArgs = Node->getNumArgs();
       Index != NumArgs; ++Index) {
    Node->setArg(Index, VisitExpr(Node->getArg(Index)));
  }

  if (!InlineAll) {
    return Node;
  }

  if (FunctionDecl *CalleeFunc = GetDefinition(Node->getDirectCallee())) {
    SmallVector<Stmt *, 16> CalleeStmts;
    ASTDuplicator Duplicator(ASTCtx, *DeclCtx);

    // Before inlining:
    //   int add(int x, int y) { return x + y; }
    //   ...
    //   c = add(a, b + c) * add(d, e) * d;
    //   ...
    //
    // After inlining:
    //   int t1;
    //   int t4;
    //   ...
    //   {
    //     int t2 = a;
    //     int t3 = b + c;
    //     {
    //       t1 = t2 + t3;
    //     }
    //   }
    //   {
    //     int t5 = d;
    //     int t6 = e;
    //     {
    //       t4 = t5 + t6;
    //     }
    //   }
    //   c = t1 * t4 * d;

    for (unsigned Index = 0, NumParams = CalleeFunc->getNumParams();
         Index != NumParams; ++Index) {
      ParmVarDecl *ParamVar = CalleeFunc->getParamDecl(Index);
      VarDecl *NewParamVar = Builder.CreateTempVar(ParamVar);
      if (Index < Node->getNumArgs()) {
        NewParamVar->setInit(Node->getArg(Index));
      }
      Duplicator.ReplaceDecl(ParamVar, NewParamVar);
      CalleeStmts.push_back(Builder.GetDeclStmt(NewParamVar));
    }

    VarDecl *NewResultVar = NULL;
    QualType ResultType = CalleeFunc->getResultType();
    if (!ResultType->isVoidType()) {
      NewResultVar = Builder.CreateTempVar(ResultType);
      TopLevelDeclStmts.push_back(Builder.GetDeclStmt(NewResultVar));
    }
    Stmt *NewBody = Duplicator.VisitFunctionBody(CalleeFunc->getBody(), NewResultVar);
    NewBody = Visit(NewBody);
    CalleeStmts.push_back(NewBody);

    CurrentPreStmts->push_back(Builder.GetCompoundStmt(CalleeStmts));

    if (ResultType->isVoidType()) {
      return NULL;
    } else {
      return Builder.GetVarRValueExpr(NewResultVar);
    }
  }
  return Node;
}

Stmt *FunctionNormalizer::VisitMemberExpr(MemberExpr *Node) {
  Node->setBase(VisitExpr(Node->getBase()));
  return Node;
}

Stmt *FunctionNormalizer::VisitBinaryOperator(BinaryOperator *Node) {
  BinaryOperator::Opcode Op = Node->getOpcode();
  Expr *NewLHS = VisitExpr(Node->getLHS());
  Expr *NewRHS = NULL;

  if (((Op == BO_LAnd || Op == BO_LOr) && !Node->getType()->isExtVectorType()) ||
      (Op == BO_Comma && NewLHS)) {
    SmallVector<Stmt *, 4> PreStmtsForRHS;
    SmallVectorImpl<Stmt *> *PreviousPreStmts = CurrentPreStmts;
    CurrentPreStmts = &PreStmtsForRHS;
    NewRHS = VisitExpr(Node->getRHS());
    CurrentPreStmts = PreviousPreStmts;

    if (!PreStmtsForRHS.empty()) {
      if (Op == BO_LAnd || Op == BO_LOr) {
        // (prelhs - lhs) && (prerhs - rhs)
        // =>
        // prelhs
        // temp1 = lhs;
        // temp2 = 0;
        // if (temp1) {
        //   prerhs
        //   temp2 = rhs;
        // }
        // temp1 && temp2
        VarDecl *LHSTempVar = Builder.CreateTempVar(NewLHS->getType());
        VarDecl *RHSTempVar = Builder.CreateTempVar(NewRHS->getType());
        TopLevelDeclStmts.push_back(Builder.GetDeclStmt(LHSTempVar));
        TopLevelDeclStmts.push_back(Builder.GetDeclStmt(RHSTempVar));

        Expr *RHSDefault = Builder.GetFalse();
        Expr *RHSCond = Builder.GetVarRValueExpr(LHSTempVar);
        if (Op == BO_LOr) {
          RHSCond = Builder.GetNotOfExpr(RHSCond);
        }

        CurrentPreStmts->push_back(
          Builder.GetAssignToVarExpr(LHSTempVar, NewLHS));
        CurrentPreStmts->push_back(
          Builder.GetAssignToVarExpr(RHSTempVar, RHSDefault));

        SmallVector<Stmt *, 4> RHSStmts;
        RHSStmts.append(PreStmtsForRHS.begin(), PreStmtsForRHS.end());
        RHSStmts.push_back(Builder.GetAssignToVarExpr(RHSTempVar, NewRHS));
        CurrentPreStmts->push_back(
          Builder.GetIfStmt(RHSCond, Builder.GetCompoundStmt(RHSStmts)));

        NewLHS = Builder.GetVarRValueExpr(LHSTempVar);
        NewRHS = Builder.GetVarRValueExpr(RHSTempVar);

      } else { // Op == BO_Comma
        CurrentPreStmts->push_back(NewLHS);
        CurrentPreStmts->append(PreStmtsForRHS.begin(), PreStmtsForRHS.end());
        NewLHS = NULL;
      }
    }
  } else {
    NewRHS = VisitExpr(Node->getRHS());
  }

  if (NewLHS) {
    Node->setLHS(NewLHS);
    Node->setRHS(NewRHS);
    return Node;
  } else {
    // void function call, expr
    return NewRHS;
  }
}

Stmt *FunctionNormalizer::VisitCompoundAssignOperator(CompoundAssignOperator *Node) {
  Node->setLHS(VisitExpr(Node->getLHS()));
  Node->setRHS(VisitExpr(Node->getRHS()));
  return Node;
}

Stmt *FunctionNormalizer::VisitConditionalOperator(ConditionalOperator *Node) {
  if (Node->getType()->isExtVectorType()) {
    Expr *NewCond = VisitExpr(Node->getCond());
    Expr *NewTrue = VisitExpr(Node->getTrueExpr());
    Expr *NewFalse = VisitExpr(Node->getFalseExpr());
    Node = Builder.SetChildsOfConditionalOperator(
      Node, NewCond, NewTrue, NewFalse);
    return Node;
  }

  Expr *NewCond = VisitExpr(Node->getCond());

  SmallVector<Stmt *, 4> PreStmtsForTrue;
  SmallVector<Stmt *, 4> PreStmtsForFalse;
  SmallVectorImpl<Stmt *> *PreviousPreStmts = CurrentPreStmts;

  CurrentPreStmts = &PreStmtsForTrue;
  Expr *NewTrue = VisitExpr(Node->getTrueExpr());
  // CurrentPreStmts = PreviousPreStmts;

  CurrentPreStmts = &PreStmtsForFalse;
  Expr *NewFalse = VisitExpr(Node->getFalseExpr());
  CurrentPreStmts = PreviousPreStmts;

  if (!PreStmtsForTrue.empty() || !PreStmtsForFalse.empty()) {
    VarDecl *TempVar = NULL;
    if (NewTrue || NewFalse) {
      TempVar = Builder.CreateTempVar(Node->getType());
      TopLevelDeclStmts.push_back(Builder.GetDeclStmt(TempVar));
    }
    if (NewTrue) {
      PreStmtsForTrue.push_back(Builder.GetAssignToVarExpr(TempVar, NewTrue));
    }
    if (NewFalse) {
      PreStmtsForFalse.push_back(Builder.GetAssignToVarExpr(TempVar, NewFalse));
    }
    CurrentPreStmts->push_back(
      Builder.GetIfStmt(NewCond,
                        Builder.GetCompoundStmt(PreStmtsForTrue),
                        Builder.GetCompoundStmt(PreStmtsForFalse)));
    if (TempVar) {
      return Builder.GetVarRValueExpr(TempVar);
    } else {
      return NULL;
    }
  }

  Node = Builder.SetChildsOfConditionalOperator(
    Node, NewCond, NewTrue, NewFalse);
  return Node;
}

Stmt *FunctionNormalizer::VisitBinaryConditionalOperator(BinaryConditionalOperator *Node) {
  Expr *NewCommon = VisitExpr(Node->getCommon());
  Expr *NewFalse = VisitExpr(Node->getFalseExpr());
  Node = Builder.SetChildsOfBinaryConditionalOperator(
    Node, NewCommon, NewFalse);
  return Node;
}

Stmt *FunctionNormalizer::VisitImplicitCastExpr(ImplicitCastExpr *Node) {
  Node->setSubExpr(VisitExpr(Node->getSubExpr()));
  return Node;
}

Stmt *FunctionNormalizer::VisitCStyleCastExpr(CStyleCastExpr *Node) {
  Node->setSubExpr(VisitExpr(Node->getSubExpr()));
  return Node;
}

Stmt *FunctionNormalizer::VisitCompoundLiteralExpr(CompoundLiteralExpr *Node) {
  Node->setInitializer(VisitExpr(Node->getInitializer()));
  return Node;
}

Stmt *FunctionNormalizer::VisitExtVectorElementExpr(ExtVectorElementExpr *Node) {
  Node->setBase(VisitExpr(Node->getBase()));
  return Node;
}

Stmt *FunctionNormalizer::VisitInitListExpr(InitListExpr *Node) {
  for (unsigned Index = 0, NumInits = Node->getNumInits();
       Index != NumInits; ++Index) {
    if (Node->getInit(Index)) {
      Node->setInit(Index, VisitExpr(Node->getInit(Index)));
    }
  }
  // Discard the alternative form
  return Node;
}

Stmt *FunctionNormalizer::VisitDesignatedInitExpr(DesignatedInitExpr *Node) {
  for (unsigned Index = 0, NumSubExprs = Node->getNumSubExprs();
       Index != NumSubExprs; ++Index) {
    Node->setSubExpr(Index, VisitExpr(Node->getSubExpr(Index)));
  }
  Node->setInit(VisitExpr(Node->getInit()));
  return Node;
}

Stmt *FunctionNormalizer::VisitImplicitValueInitExpr(ImplicitValueInitExpr *Node) {
  return Node;
}

Stmt *FunctionNormalizer::VisitParenListExpr(ParenListExpr *Node) {
  // NOTE: ParenListExpr does not have a setExpr(unsigned, Expr*) method.
  Expr **Exprs = Node->getExprs();
  for (unsigned Index = 0, NumExprs = Node->getNumExprs();
       Index != NumExprs; ++Index) {
    Exprs[Index] = VisitExpr(Exprs[Index]);
  }
  return Node;
}

Stmt *FunctionNormalizer::VisitVAArgExpr(VAArgExpr *Node) {
  Node->setSubExpr(VisitExpr(Node->getSubExpr()));
  return Node;
}

Stmt *FunctionNormalizer::VisitGenericSelectionExpr(GenericSelectionExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitPseudoObjectExpr(PseudoObjectExpr *Node) { return Node; }

// Atomic expressions

Stmt *FunctionNormalizer::VisitAtomicExpr(AtomicExpr *Node) { return Node; }

// GNU extensions

Stmt *FunctionNormalizer::VisitAddrLabelExpr(AddrLabelExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitStmtExpr(StmtExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitChooseExpr(ChooseExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitGNUNullExpr(GNUNullExpr *Node) { return Node; }

// C++ expressions

Stmt *FunctionNormalizer::VisitCXXOperatorCallExpr(CXXOperatorCallExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXMemberCallExpr(CXXMemberCallExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXStaticCastExpr(CXXStaticCastExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXDynamicCastExpr(CXXDynamicCastExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXReinterpretCastExpr(CXXReinterpretCastExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXConstCastExpr(CXXConstCastExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXFunctionalCastExpr(CXXFunctionalCastExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXTypeidExpr(CXXTypeidExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitUserDefinedLiteral(UserDefinedLiteral *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXBoolLiteralExpr(CXXBoolLiteralExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXNullPtrLiteralExpr(CXXNullPtrLiteralExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXThisExpr(CXXThisExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXThrowExpr(CXXThrowExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXDefaultArgExpr(CXXDefaultArgExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXDefaultInitExpr(CXXDefaultInitExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXScalarValueInitExpr(CXXScalarValueInitExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXStdInitializerListExpr(CXXStdInitializerListExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXNewExpr(CXXNewExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXDeleteExpr(CXXDeleteExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXPseudoDestructorExpr(CXXPseudoDestructorExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitTypeTraitExpr(TypeTraitExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitUnaryTypeTraitExpr(UnaryTypeTraitExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitBinaryTypeTraitExpr(BinaryTypeTraitExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitArrayTypeTraitExpr(ArrayTypeTraitExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitExpressionTraitExpr(ExpressionTraitExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitDependentScopeDeclRefExpr(DependentScopeDeclRefExpr *Node) { return Node; }

Stmt *FunctionNormalizer::VisitCXXConstructExpr(CXXConstructExpr *Node) {
  assert(Node->getConstructor()->isDefaultConstructor() &&
         "not implemented yet");
  return Node;
}

Stmt *FunctionNormalizer::VisitCXXBindTemporaryExpr(CXXBindTemporaryExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitExprWithCleanups(ExprWithCleanups *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXTemporaryObjectExpr(CXXTemporaryObjectExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXUnresolvedConstructExpr(CXXUnresolvedConstructExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXDependentScopeMemberExpr(CXXDependentScopeMemberExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitUnresolvedLookupExpr(UnresolvedLookupExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitUnresolvedMemberExpr(UnresolvedMemberExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXNoexceptExpr(CXXNoexceptExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitPackExpansionExpr(PackExpansionExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitSizeOfPackExpr(SizeOfPackExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitSubstNonTypeTemplateParmExpr(SubstNonTypeTemplateParmExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitSubstNonTypeTemplateParmPackExpr(SubstNonTypeTemplateParmPackExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitFunctionParmPackExpr(FunctionParmPackExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitMaterializeTemporaryExpr(MaterializeTemporaryExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitLambdaExpr(LambdaExpr *Node) { return Node; }

// Obj-C expressions

Stmt *FunctionNormalizer::VisitObjCStringLiteral(ObjCStringLiteral *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCBoxedExpr(ObjCBoxedExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCArrayLiteral(ObjCArrayLiteral *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCDictionaryLiteral(ObjCDictionaryLiteral *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCEncodeExpr(ObjCEncodeExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCMessageExpr(ObjCMessageExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCSelectorExpr(ObjCSelectorExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCProtocolExpr(ObjCProtocolExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCIvarRefExpr(ObjCIvarRefExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCPropertyRefExpr(ObjCPropertyRefExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCIsaExpr(ObjCIsaExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCIndirectCopyRestoreExpr(ObjCIndirectCopyRestoreExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCBoolLiteralExpr(ObjCBoolLiteralExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitObjCSubscriptRefExpr(ObjCSubscriptRefExpr *Node) { return Node; }

// Obj-C ARC expressions

Stmt *FunctionNormalizer::VisitObjCBridgedCastExpr(ObjCBridgedCastExpr *Node) { return Node; }

// CUDA expressions

Stmt *FunctionNormalizer::VisitCUDAKernelCallExpr(CUDAKernelCallExpr *Node) { return Node; }

// Clang extensions

Stmt *FunctionNormalizer::VisitShuffleVectorExpr(ShuffleVectorExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitConvertVectorExpr(ConvertVectorExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitBlockExpr(BlockExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitOpaqueValueExpr(OpaqueValueExpr *Node) { return Node; }

// Microsoft extensions

Stmt *FunctionNormalizer::VisitMSPropertyRefExpr(MSPropertyRefExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitCXXUuidofExpr(CXXUuidofExpr *Node) { return Node; }
Stmt *FunctionNormalizer::VisitSEHTryStmt(SEHTryStmt *Node) { return Node; }
Stmt *FunctionNormalizer::VisitSEHExceptStmt(SEHExceptStmt *Node) { return Node; }
Stmt *FunctionNormalizer::VisitSEHFinallyStmt(SEHFinallyStmt *Node) { return Node; }
Stmt *FunctionNormalizer::VisitMSDependentExistsStmt(MSDependentExistsStmt *Node) { return Node; }

// OpenCL extensions

Stmt *FunctionNormalizer::VisitAsTypeExpr(AsTypeExpr *Node) {
  Node = Builder.SetSrcExprOfAsTypeExpr(Node, VisitExpr(Node->getSrcExpr()));
  return Node;
}

// OpenMP directives

Stmt *FunctionNormalizer::VisitOMPParallelDirective(OMPParallelDirective *Node) { return Node; }

// Declarations

VarDecl *FunctionNormalizer::VisitConditionVariable(VarDecl *Var) {
  if (NormalizeDecl || NormalizeVarDeclInit) {
    Var->setDeclName(Builder.GetUniqueDeclName(Var));
    CurrentPreStmts->push_back(Builder.GetDeclStmt(Var));

    assert(Var->getInit());
    Expr *NewInit = VisitExpr(Var->getInit());
    if (NormalizeVarDeclInit) {
      CurrentPreStmts->push_back(Builder.GetAssignToVarExpr(Var, NewInit));
      Var->setInit(NULL);
    } else {
      Var->setInit(NewInit);
    }

    return NULL;
  }

  return Var;
}

} // namespace snu

} // namespace clang
