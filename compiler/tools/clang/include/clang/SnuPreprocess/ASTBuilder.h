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

#ifndef LLVM_CLANG_SNU_PREPROCESS_ASTBUILDER_H
#define LLVM_CLANG_SNU_PREPROCESS_ASTBUILDER_H

#include "clang/AST/ASTContext.h"
#include "clang/AST/Decl.h" 
#include "clang/AST/Expr.h"
#include "clang/AST/ExprCXX.h"
#include "clang/AST/ExprObjC.h"
#include "clang/AST/Stmt.h"
#include "clang/AST/StmtCXX.h"
#include "clang/AST/StmtObjC.h"
#include "clang/AST/StmtOpenMP.h"
#include "clang/AST/Type.h"
#include "clang/Basic/IdentifierTable.h"
#include "clang/Basic/LLVM.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"

namespace clang {

namespace snu {

class ASTBuilder {
  ASTContext &ASTCtx;
  DeclContext *DeclCtx;
  SourceLocation DummyLoc;

public:
  explicit ASTBuilder(ASTContext &Ctx) :
    ASTCtx(Ctx), DeclCtx(NULL) {}

  ASTBuilder(ASTContext &Ctx, DeclContext *DCtx) :
    ASTCtx(Ctx), DeclCtx(DCtx) {}

  void setDeclContext(DeclContext *DCtx) {
    DeclCtx = DCtx;
  }

  // Declarations
  IdentifierInfo *GetUniqueDeclName(IdentifierInfo *II);
  IdentifierInfo *GetUniqueDeclName(NamedDecl *D);
  IdentifierInfo *GetUniqueDeclName(StringRef Prefix);
  VarDecl *CreateTempVar(QualType Ty, StringRef Prefix = StringRef());
  VarDecl *CreateTempVar(VarDecl *Var);

  // Statements
  NullStmt *GetNullStmt();
  DeclStmt *GetDeclStmt(Decl *D);
  CompoundStmt *GetCompoundStmt(Stmt *S);
  CompoundStmt *GetCompoundStmt(SmallVectorImpl<Stmt *> &Stmts);
  CompoundStmt *GetCompoundStmt(Stmt **Begin, Stmt **End);
  IfStmt *GetIfStmt(Expr *Cond, Stmt *True, Stmt *Else = NULL);
  // if (!E) break;
  IfStmt *GetBreakIfNotStmt(Expr *E);
  // __attribute__((annotate("str"))) S;
  AttributedStmt *GetAnnotatedStmt(Stmt *S, StringRef Annotation);

  // Expressions
  // 1, 0
  IntegerLiteral *GetTrue();
  IntegerLiteral *GetFalse();
  // Var
  Expr *GetVarRValueExpr(VarDecl *Var);
  // Var = E
  Expr *GetAssignToVarExpr(VarDecl *Var, Expr *E);
  // !E
  Expr *GetNotOfExpr(Expr *E);

  // Set childs
  AttributedStmt *SetSubStmtOfAttributedStmt(AttributedStmt *Node,
                                             Stmt *NewSubStmt);
  CapturedStmt *SetSubStmtOfCapturedStmt(CapturedStmt *Node, Stmt *NewSubStmt);
  ConditionalOperator *SetChildsOfConditionalOperator(
    ConditionalOperator *Node, Expr *NewCond, Expr *NewTrue,Expr *NewFalse);
  BinaryConditionalOperator *SetChildsOfBinaryConditionalOperator(
    BinaryConditionalOperator *Node, Expr *NewCommon, Expr *NewFalse);
  AsTypeExpr *SetSrcExprOfAsTypeExpr(AsTypeExpr *Node, Expr *NewSrcExpr);
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_PREPROCESS_ASTBUILDER_H
