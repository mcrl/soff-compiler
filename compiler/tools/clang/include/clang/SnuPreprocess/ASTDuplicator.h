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

#ifndef LLVM_CLANG_SNU_PREPROCESS_ASTDUPLICATOR_H
#define LLVM_CLANG_SNU_PREPROCESS_ASTDUPLICATOR_H

#include "clang/AST/ASTContext.h"
#include "clang/AST/Decl.h" 
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
#include "llvm/ADT/SmallVector.h"

namespace clang {

namespace snu {

class ASTDuplicator : public StmtVisitor<ASTDuplicator, Stmt *> {
  ASTContext &ASTCtx;
  DeclContext &DeclCtx;
  ASTBuilder Builder;
  std::map<NamedDecl*, NamedDecl*> DupDecl;
  bool ConvertReturnToExpr;
  VarDecl *ReturnVar;
  LabelDecl *ReturnLabel;
  SwitchStmt *CurrentSwitchStmt;

public:
  ASTDuplicator(ASTContext &Ctx, DeclContext &DCtx) :
    ASTCtx(Ctx), DeclCtx(DCtx), Builder(Ctx, &DCtx),
    ConvertReturnToExpr(false), ReturnVar(NULL), ReturnLabel(NULL),
    CurrentSwitchStmt(NULL) {}

  Stmt *VisitFunctionBody(Stmt *Node, VarDecl *Ret);

#define ABSTRACT_STMT(STMT)
#define STMT(CLASS, PARENT) Stmt *Visit ## CLASS(CLASS *S);
#include "clang/AST/StmtNodes.inc"
#undef ABSTRACT_STMT
#undef STMT

  Stmt *VisitBlock(Stmt *Node);

  Expr *VisitExpr(Expr *E) {
    return reinterpret_cast<Expr*>(Visit(E));
  }

private:
  void VisitCompoundStmtBody(CompoundStmt *CS, bool Outermost,
                             SmallVectorImpl<Stmt *> &NewStmts);

public:
  void ReplaceDecl(NamedDecl *From, NamedDecl *To);

private:
  NamedDecl *LookupDecl(NamedDecl *D);
  VarDecl *LookupVarDecl(VarDecl *VD);
  VarDecl *DuplicateVarDecl(VarDecl *VD);
  LabelDecl *LookupOrDuplicateLabelDecl(LabelDecl *LD, SourceLocation Loc);
  LabelDecl *LookupOrCreateReturnLabelDecl();
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_PREPROCESS_ASTDUPLICATOR_H
