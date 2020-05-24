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

#include "clang/SnuPreprocess/ASTBuilder.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/Attr.h"
#include "clang/AST/Decl.h"
#include "clang/AST/Expr.h"
#include "clang/AST/ExprCXX.h"
#include "clang/AST/ExprObjC.h"
#include "clang/AST/OperationKinds.h"
#include "clang/AST/Stmt.h"
#include "clang/AST/StmtCXX.h"
#include "clang/AST/StmtObjC.h"
#include "clang/AST/StmtOpenMP.h"
#include "clang/AST/StmtVisitor.h"
#include "clang/AST/Type.h"
#include "clang/Basic/IdentifierTable.h"
#include "clang/Basic/LLVM.h"
#include "clang/Basic/Specifiers.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/raw_ostream.h"
#include <sstream>

namespace clang {

namespace snu {

static bool LookupNameInDeclContext(DeclContext *DeclCtx, StringRef Name) {
  for (DeclContext::decl_iterator D = DeclCtx->decls_begin(),
                                  DEnd = DeclCtx->decls_end();
       D != DEnd; ++D) {
    if (NamedDecl *ND = dyn_cast<NamedDecl>(*D)) {
      if (ND->getName() == Name)
        return true;
    }
  }
  return false;
}

static bool LookupNameInDeclContext(DeclContext *DeclCtx, IdentifierInfo *II,
                                    NamedDecl *Exclude = NULL) {
  for (DeclContext::decl_iterator D = DeclCtx->decls_begin(),
                                  DEnd = DeclCtx->decls_end();
       D != DEnd; ++D) {
    if (NamedDecl *ND = dyn_cast<NamedDecl>(*D)) {
      if (ND != Exclude && ND->getIdentifier() == II)
        return true;
    }
  }
  return false;
}

IdentifierInfo *ASTBuilder::GetUniqueDeclName(IdentifierInfo *II) {
  if (!DeclCtx) return NULL;
  if (II && LookupNameInDeclContext(DeclCtx, II)) {
    return GetUniqueDeclName(II->getName());
  }
  return II;
}

IdentifierInfo *ASTBuilder::GetUniqueDeclName(NamedDecl *D) {
  if (!DeclCtx) return NULL;
  IdentifierInfo *II = D->getIdentifier();
  if (II && LookupNameInDeclContext(DeclCtx, II, D)) {
    return GetUniqueDeclName(II->getName());
  }
  return II;
}

IdentifierInfo *ASTBuilder::GetUniqueDeclName(StringRef Prefix) {
  if (!DeclCtx) return NULL;
  for (unsigned Index = 1; Index != 0; ++Index) {
    std::ostringstream Name;
    Name << Prefix.str() << "_" << Index;
    if (!LookupNameInDeclContext(DeclCtx, Name.str())) {
      return &ASTCtx.Idents.get(Name.str());
    }
  }
  llvm_unreachable("failed to get a unique name");
  return NULL;
}

VarDecl *ASTBuilder::CreateTempVar(QualType Ty, StringRef Prefix) {
  if (!DeclCtx) return NULL;
  if (Prefix.empty()) {
    Prefix = "__temp";
  }
  IdentifierInfo *II = GetUniqueDeclName(Prefix);
  Ty = Ty.getUnqualifiedType();

  VarDecl *VD = VarDecl::Create(
    ASTCtx, DeclCtx, DummyLoc, DummyLoc, II,
    Ty, ASTCtx.CreateTypeSourceInfo(Ty), SC_None);
  DeclCtx->addDecl(VD);
  return VD;
}

VarDecl *ASTBuilder::CreateTempVar(VarDecl *Var) {
  if (!DeclCtx) return NULL;
  IdentifierInfo *II = GetUniqueDeclName(Var->getIdentifier());
  QualType Ty = Var->getType().getUnqualifiedType();

  VarDecl *VD = VarDecl::Create(
    ASTCtx, DeclCtx, DummyLoc, DummyLoc, II,
    Ty, ASTCtx.CreateTypeSourceInfo(Ty), SC_None);
  DeclCtx->addDecl(VD);
  return VD;
}

NullStmt *ASTBuilder::GetNullStmt() {
  return new (ASTCtx) NullStmt(DummyLoc);
}

DeclStmt *ASTBuilder::GetDeclStmt(Decl *D) {
  return new (ASTCtx) DeclStmt(DeclGroupRef(D), DummyLoc, DummyLoc);
}

CompoundStmt *ASTBuilder::GetCompoundStmt(Stmt *S) {
  if (CompoundStmt *CS = dyn_cast<CompoundStmt>(S)) {
    return CS;
  } else {
    return new (ASTCtx) CompoundStmt(ASTCtx, S, DummyLoc, DummyLoc);
  }
}

CompoundStmt *ASTBuilder::GetCompoundStmt(SmallVectorImpl<Stmt *> &Stmts) {
  if (Stmts.size() == 1) {
    if (CompoundStmt *CS = dyn_cast<CompoundStmt>(Stmts.front())) {
      return CS;
    }
  }
  return new (ASTCtx) CompoundStmt(ASTCtx, Stmts, DummyLoc, DummyLoc);
}

CompoundStmt *ASTBuilder::GetCompoundStmt(Stmt **Begin, Stmt **End) {
  if (Begin + 1 == End) {
    if (CompoundStmt *CS = dyn_cast<CompoundStmt>(*Begin)) {
      return CS;
    }
  }
  return new (ASTCtx) CompoundStmt(
    ASTCtx, ArrayRef<Stmt*>(Begin, End), DummyLoc, DummyLoc);
}

IfStmt *ASTBuilder::GetIfStmt(Expr *Cond, Stmt *True, Stmt *Else) {
  return new (ASTCtx) IfStmt(
    ASTCtx, DummyLoc, NULL, Cond, True, DummyLoc, Else);
}

// if (!E) break;
IfStmt *ASTBuilder::GetBreakIfNotStmt(Expr *E) {
  return new (ASTCtx) IfStmt(
    ASTCtx, DummyLoc, NULL, GetNotOfExpr(E),
    new (ASTCtx) BreakStmt(DummyLoc), DummyLoc, NULL);
}

// __attribute__((annotate("str"))) S;
AttributedStmt *ASTBuilder::GetAnnotatedStmt(Stmt *S, StringRef Annotation) {
  AnnotateAttr *NewAttr = new (ASTCtx) AnnotateAttr(
    DummyLoc, ASTCtx, Annotation);
  return AttributedStmt::Create(ASTCtx, DummyLoc, NewAttr, S);
}

// 1
IntegerLiteral *ASTBuilder::GetTrue() {
  return new (ASTCtx) IntegerLiteral(
    ASTCtx, llvm::APInt(32, 1), ASTCtx.IntTy, DummyLoc);
}

// 0
IntegerLiteral *ASTBuilder::GetFalse() {
  return new (ASTCtx) IntegerLiteral(
    ASTCtx, llvm::APInt(32, 0), ASTCtx.IntTy, DummyLoc);
}

// Var
Expr *ASTBuilder::GetVarRValueExpr(VarDecl *Var) {
  QualType Ty = Var->getType();
  Expr *DeclRef = new (ASTCtx) DeclRefExpr(Var, false, Ty, VK_LValue, DummyLoc);
  if (Ty.hasQualifiers()) {
    Ty = Ty.getUnqualifiedType();
  }
  return ImplicitCastExpr::Create(
    ASTCtx, Ty, CK_LValueToRValue, DeclRef, NULL, VK_RValue);
}

// Var = E
Expr *ASTBuilder::GetAssignToVarExpr(VarDecl *Var, Expr *E) {
  Expr *LHS = new (ASTCtx) DeclRefExpr(
    Var, false, Var->getType(), VK_LValue, DummyLoc);
  if (BinaryOperator *BO = dyn_cast<BinaryOperator>(E->IgnoreImplicit())) {
    if (BO->getOpcode() == BO_Comma) {
      E = new (ASTCtx) ParenExpr(DummyLoc, DummyLoc, E);
    }
  }
  return new (ASTCtx) BinaryOperator(
    LHS, E, BO_Assign, LHS->getType(), VK_RValue, OK_Ordinary, DummyLoc, false);
}

// !E
Expr *ASTBuilder::GetNotOfExpr(Expr *E) {
  if (!(isa<ParenExpr>(E) || isa<DeclRefExpr>(E))) {
    E = new (ASTCtx) ParenExpr(DummyLoc, DummyLoc, E);
  }
  return new (ASTCtx) UnaryOperator(
    E, UO_LNot, ASTCtx.IntTy, VK_RValue, OK_Ordinary, DummyLoc);
}

AttributedStmt *ASTBuilder::SetSubStmtOfAttributedStmt(AttributedStmt *Node,
                                                       Stmt *NewSubStmt) {
  if (Node->getSubStmt() != NewSubStmt) {
    Node = AttributedStmt::Create(
      ASTCtx, Node->getAttrLoc(), Node->getAttrs(), NewSubStmt);
  }
  return Node;
}

CapturedStmt *ASTBuilder::SetSubStmtOfCapturedStmt(CapturedStmt *Node,
                                                   Stmt *NewSubStmt) {
  if (Node->getCapturedStmt() != NewSubStmt) {
    Node = CapturedStmt::Create(
      ASTCtx, NewSubStmt, Node->getCapturedRegionKind(),
      ArrayRef<CapturedStmt::Capture>(Node->capture_begin(), Node->capture_end()),
      ArrayRef<Expr *>(Node->capture_init_begin(), Node->capture_init_end()),
      Node->getCapturedDecl(),
      const_cast<RecordDecl *>(Node->getCapturedRecordDecl()));
  }
  return Node;
}

ConditionalOperator *ASTBuilder::SetChildsOfConditionalOperator(
  ConditionalOperator *Node, Expr *NewCond, Expr *NewTrue, Expr *NewFalse) {
  if (Node->getCond() != NewCond || Node->getTrueExpr() != NewTrue ||
      Node->getFalseExpr() != NewFalse) {
    Node = new (ASTCtx) ConditionalOperator(
      NewCond, Node->getQuestionLoc(), NewTrue, Node->getColonLoc(), NewFalse,
      Node->getType(), Node->getValueKind(), Node->getObjectKind());
  }
  return Node;
}

BinaryConditionalOperator *ASTBuilder::SetChildsOfBinaryConditionalOperator(
  BinaryConditionalOperator *Node, Expr *NewCommon, Expr *NewFalse) {
  if (Node->getCommon() != NewCommon || Node->getFalseExpr() != NewFalse) {
    OpaqueValueExpr *NewOpaqueValue = new (ASTCtx) OpaqueValueExpr(
      NewCommon->getExprLoc(), NewCommon->getType(),
      NewCommon->getValueKind(), NewCommon->getObjectKind(), NewCommon);
    Node = new (ASTCtx) BinaryConditionalOperator(
      NewCommon, NewOpaqueValue, NewOpaqueValue, NewOpaqueValue, NewFalse,
      Node->getQuestionLoc(), Node->getColonLoc(), Node->getType(),
      Node->getValueKind(), Node->getObjectKind());
  }
  return Node;
}

AsTypeExpr *ASTBuilder::SetSrcExprOfAsTypeExpr(AsTypeExpr *Node, Expr *NewSrcExpr) {
  if (Node->getSrcExpr() != NewSrcExpr) {
    Node = new (ASTCtx) AsTypeExpr(
      NewSrcExpr, Node->getType(), Node->getValueKind(), Node->getObjectKind(),
      Node->getBuiltinLoc(), Node->getRParenLoc());
  }
  return Node;
}

} // namespace snu

} // namespace clang
