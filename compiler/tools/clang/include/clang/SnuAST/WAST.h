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

#ifndef LLVM_CLANG_SNU_AST_WAST_H
#define LLVM_CLANG_SNU_AST_WAST_H

#include "clang/AST/ASTContext.h"
#include "clang/AST/Attr.h"
#include "clang/AST/CharUnits.h"
#include "clang/AST/Decl.h"
#include "clang/AST/DeclBase.h"
#include "clang/AST/Expr.h"
#include "clang/AST/ExprCXX.h"
#include "clang/AST/PrettyPrinter.h"
#include "clang/AST/Stmt.h"
#include "clang/AST/Type.h"
#include "clang/Basic/LLVM.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/APSInt.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/Support/Allocator.h"
#include <utility>

namespace clang {

namespace snu {

class WDeclContext;
class WExpr;
class WMuFunction;
class WParmVarDecl;
class WStmt;
class WSubVarDecl;

// Wrapped AST nodes

#define CLANG_WSTMTS() \
  STMT(DeclStmt) \
  STMT(NullStmt) \
  STMT(CompoundStmt) \
  STMT(CaseStmt) \
  STMT(DefaultStmt) \
  STMT(LabelStmt) \
  STMT(AttributedStmt) \
  STMT(IfStmt) \
  STMT(SwitchStmt) \
  STMT(WhileStmt) \
  STMT(DoStmt) \
  STMT(ForStmt) \
  STMT(GotoStmt) \
  STMT(ContinueStmt) \
  STMT(BreakStmt) \
  STMT(ReturnStmt) \
  STMT(DeclRefExpr) \
  STMT(PredefinedExpr) \
  STMT(IntegerLiteral) \
  STMT(CharacterLiteral) \
  STMT(FloatingLiteral) \
  STMT(StringLiteral) \
  STMT(ParenExpr) \
  STMT(UnaryOperator) \
  STMT(UnaryExprOrTypeTraitExpr) \
  STMT(ArraySubscriptExpr) \
  STMT(CallExpr) \
  STMT(MemberExpr) \
  STMT(CompoundLiteralExpr) \
  STMT(ImplicitCastExpr) \
  STMT(CStyleCastExpr) \
  STMT(BinaryOperator) \
  STMT(CompoundAssignOperator) \
  STMT(ConditionalOperator) \
  STMT(InitListExpr) \
  STMT(DesignatedInitExpr) \
  STMT(ImplicitValueInitExpr) \
  STMT(ParenListExpr) \
  STMT(ExtVectorElementExpr) \
  STMT(CXXBoolLiteralExpr) \
  STMT(CXXDefaultArgExpr) \
  STMT(CXXConstructExpr) \
  STMT(BlockExpr) \
  STMT(AsTypeExpr) \

#define EXTENDED_WSTMTS() \
  WSTMT(PhiFunction) \
  WSTMT(WorkItemFunction) \
  WSTMT(BaseMuFunction) \
  WSTMT(AuxiliaryMuFunction) \


// Declarations

class WVarDecl {
public:
  enum WVarDeclKind {
    // 0 to XXX are reserved for the original DeclKind
    WVarDeclKindBegin = 1000,
    WSubVarKind,
    WVirtualVarKind,
    WTemporaryVectorVarKind,
    WVarDeclKindEnd
  };

  typedef union {
    FieldDecl *Field;
    unsigned Index;
  } SubVarFieldTy;

protected:
  VarDecl *Original;
  union {
    Decl::Kind Kind;
    WVarDeclKind WKind;
  };

  QualType DeclType;
  WExpr *Init;

  // Compound variables
  unsigned NumSubVars;
  SubVarFieldTy *SubFields;
  WSubVarDecl **SubVars;

  WVarDecl(WVarDeclKind kind, VarDecl *D, QualType T)
    : Original(D), DeclType(T), Init(NULL), NumSubVars(0), SubFields(NULL),
      SubVars(NULL) {
    WKind = kind;
  }

public:
  WVarDecl(VarDecl *D, const ASTContext &Ctx);

  VarDecl *getOriginal() const { return Original; }
  bool isClangDecl() const { return WKind < WVarDeclKindBegin; }
  Decl::Kind getKind() const { return Kind; }
  WVarDeclKind getWKind() const { return WKind; }

  QualType getType() const { return DeclType; }
  bool hasInit() const { return Init != NULL; }
  WExpr *getInit() const { return Init; }
  void setInit(WExpr *init) { Init = init; }

  bool isCompound() const { return NumSubVars > 0; }
  unsigned getNumSubVars() const { return NumSubVars; }
  WSubVarDecl *getSubVar(unsigned Index) const {
    assert(Index < NumSubVars && "Sub variable access out of range!");
    return SubVars[Index];
  }
  WSubVarDecl *getSubVarOfStructure(FieldDecl *Field) const;

  StringRef getName() const;

  bool isParameter() const;
  WParmVarDecl *getAsParameter();
  bool hasAddressSpace() const;
  unsigned getAddressSpace() const;

  void printName(raw_ostream &OS) const;

protected:
  void CreateSubVarsOfStructure(const ASTContext &Ctx, QualType Ty);
  void CreateSubVarsOfVector(const ASTContext &Ctx, QualType Ty);

public:
  static WVarDecl *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                VarDecl *Node);
};

class WParmVarDecl : public WVarDecl {
public:
  WParmVarDecl(ParmVarDecl *D, const ASTContext &Ctx)
    : WVarDecl(D, Ctx) {}

  static bool classof(const WVarDecl *D) {
    return D->getKind() == Decl::ParmVar;
  }
};

class WSubVarDecl : public WVarDecl {
  WVarDecl *Parent;
  WVarDecl::SubVarFieldTy Field;
  char *ReadableName;

public:
  WSubVarDecl(WVarDecl *parent, WVarDecl::SubVarFieldTy field, StringRef name,
              QualType T, const ASTContext &Ctx);

  WVarDecl *getParent() const { return Parent; }
  WVarDecl::SubVarFieldTy getField() const { return Field; }
  StringRef getName() const { return StringRef(ReadableName); }
  uint64_t getOffset(const ASTContext &Ctx) const;

  static bool classof(const WVarDecl *D) {
    return D->getWKind() == WVarDecl::WSubVarKind;
  }
};

class WVirtualVarDecl : public WVarDecl {
  char *ReadableName;

public:
  WVirtualVarDecl(StringRef name, QualType T, const ASTContext &Ctx);

  StringRef getName() const { return StringRef(ReadableName); }

  static bool classof(const WVarDecl *D) {
    return D->getWKind() == WVarDecl::WVirtualVarKind;
  }
};

class WTemporaryVectorVarDecl : public WVarDecl {
public:
  WTemporaryVectorVarDecl(WVarDecl *base, ArrayRef<unsigned> elements,
                          QualType T, const ASTContext &Ctx);

  StringRef getName() const { return StringRef("<vector>"); }

  static bool classof(const WVarDecl *D) {
    return D->getWKind() == WVarDecl::WTemporaryVectorVarKind;
  }
};

class WDeclContext {
  typedef llvm::DenseMap<const VarDecl*, WVarDecl*> DeclWDeclMap;

  DeclWDeclMap WrappingMap;

public:
  WDeclContext() {}

  WVarDecl *Lookup(const VarDecl *Decl) const;
  void Register(const VarDecl *Decl, WVarDecl *WrappedDecl);
};


// Indexed Declarations

class IndexedVarDecl {
  WVarDecl *Decl;
  unsigned Index;
  WStmt *DefinedStmt;
  WMuFunction *DefinedMu;

public:
  IndexedVarDecl(WVarDecl *D, unsigned index)
    : Decl(D), Index(index), DefinedStmt(NULL), DefinedMu(NULL) {}

  WVarDecl *getDecl() const { return Decl; }
  QualType getType() const { return Decl->getType(); }
  unsigned getIndex() const { return Index; }

  bool isParameter() const;

  // DefinedStmt is one of DeclStmt w/ SingleInit,
  //                       UnaryOperator w/ IncrementDecrementOp,
  //                       BinaryOperator w/ AssignmentOp
  //                       PhiFunction
  WStmt *getDefinedStmt() const {
    if ((uintptr_t)DefinedStmt & 0x1) {
      return NULL;
    } else {
      return DefinedStmt;
    }
  }
  WStmt *getCompoundDefinedStmt() const {
    if ((uintptr_t)DefinedStmt & 0x1) {
      return (WStmt*)((uintptr_t)DefinedStmt & ~0x1);
    } else {
      return NULL;
    }
  }
  WStmt *getAnyDefinedStmt() const {
    return (WStmt*)((uintptr_t)DefinedStmt & ~0x1);
  }
  void setDefinedStmt(WStmt *S) { DefinedStmt = S; }
  void setCompoundDefinedStmt(WStmt *S) {
    DefinedStmt = (WStmt*)((uintptr_t)S | 0x1);
  }

  WMuFunction *getDefinedMu() const {
    return DefinedMu;
  }
  void setDefinedMu(WMuFunction *mu) {
    DefinedMu = mu;
  }

  void printName(raw_ostream &OS) const;
};

class IndexedVarDeclRef {
  IndexedVarDecl *SingleVar;
  IndexedVarDecl **SubVars;
  unsigned NumSubVars;

public:
  IndexedVarDeclRef()
    : SingleVar(NULL), SubVars(NULL), NumSubVars(0) {}
  IndexedVarDeclRef(IndexedVarDecl *D)
    : SingleVar(D), SubVars(NULL), NumSubVars(0) {
    assert(D != NULL);
  }
  IndexedVarDeclRef(llvm::BumpPtrAllocator &Allocator,
                    ArrayRef<IndexedVarDecl*> vars);

  bool isNull() const { return (SingleVar == NULL && NumSubVars == 0); }
  operator bool() const { return !isNull(); }

  bool isSingleVar() const { return SingleVar != NULL; }
  IndexedVarDecl *getSingleVar() const { return SingleVar; }
  IndexedVarDecl *operator*() const {
    assert(isSingleVar());
    return getSingleVar();
  }
  IndexedVarDecl *operator->() const {
    assert(isSingleVar());
    return getSingleVar();
  }

  bool isCompound() const { return NumSubVars > 0; }
  unsigned getNumSubVars() const { return NumSubVars; }
  IndexedVarDecl *getSubVar(unsigned Index) const {
    assert(isCompound());
    assert(Index < NumSubVars && "access out of range");
    return SubVars[Index];
  }
  IndexedVarDecl *operator[](unsigned Index) const {
    return getSubVar(Index);
  }

  bool operator==(const IndexedVarDeclRef &RHS) const;
  bool operator!=(const IndexedVarDeclRef &RHS) const {
    return !operator==(RHS);
  }

  void setDefinedStmt(WStmt *S);
  void printName(raw_ostream &OS) const;
};


// Statements

class WStmt {
public:
  enum WStmtClass {
    // 0 to XXX are reserved for the original StmtClass
    WStmtClassBegin = 1000,
#define WSTMT(type) type##Class,
    EXTENDED_WSTMTS()
#undef WSTMT
    firstWExprConstant = WorkItemFunctionClass,
    lastWExprConstant = AuxiliaryMuFunctionClass,
    WStmtClassEnd
  };

protected:
  // Only allow allocation of WStmts using the allocator in ASTContext
  // as the original Stmts do
  void* operator new(size_t bytes) throw() {
    llvm_unreachable("WStmts cannot be allocated with regular 'new'.");
  }
  void operator delete(void* data) throw() {
    llvm_unreachable("WStmts cannot be released with regular 'delete'.");
  }

public:
  void* operator new(size_t bytes, const ASTContext& C,
                     unsigned alignment = 8) {
    return Stmt::operator new(bytes, C, alignment);
  }

  template <class SomeContext>
  void* operator new(size_t bytes, const SomeContext& C,
                     unsigned alignment = 8) {
    return C.getAllocator().Allocate(bytes, alignment);
  }

  void* operator new(size_t bytes, void* mem) throw() {
    return mem;
  }

  void operator delete(void*, const ASTContext&, unsigned) throw() { }
  void operator delete(void*, size_t) throw() { }
  void operator delete(void*, void*) throw() { }

protected:
  Stmt *Original;
  union {
    Stmt::StmtClass SClass;
    WStmtClass WClass;
  };

protected:
  WStmt(Stmt::StmtClass SC) : Original(NULL) {
    SClass = SC;
  }
  WStmt(WStmtClass SC) : Original(NULL) {
    WClass = SC;
  }
  WStmt(WStmtClass SC, Stmt *S) : Original(S) {
    WClass = SC;
  }
  explicit WStmt(Stmt *S) : Original(S) {
    SClass = Original->getStmtClass();
  }

public:
  Stmt *getOriginal() const { return Original; }
  bool isClangStmt() const { return WClass < WStmtClassBegin; }
  Stmt::StmtClass getStmtClass() const { return SClass; }
  WStmt::WStmtClass getWStmtClass() const { return WClass; }

  const char *getStmtClassName() const;

  typedef WStmt** child_iterator;
  typedef WStmt* const* const_child_iterator;

  typedef std::pair<child_iterator, child_iterator> child_range;
  typedef std::pair<const_child_iterator, const_child_iterator>
      const_child_range;

  child_range children();
  const_child_range children() const {
    child_range range = const_cast<WStmt*>(this)->children();
    return const_child_range(range.first, range.second);
  }

  child_iterator child_begin() { return children().first; }
  child_iterator child_end() { return children().second; }

  const_child_iterator child_begin() const { return children().first; }
  const_child_iterator child_end() const { return children().second; }

  bool contains(WStmt *S) const;
  void replace(WStmt *From, WStmt *To);

  void print(raw_ostream &OS, const PrintingPolicy &Policy) const;

  static WStmt *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                             Stmt *Node);
};

class WDeclStmt : public WStmt {
public:
  typedef WVarDecl WDeclTy;

private:
  WDeclTy *SingleDecl;
  WDeclTy **Decls;
  unsigned NumDecls;
  // SSA form
  IndexedVarDeclRef IndexedDecl;

public:
  WDeclStmt(DeclStmt *S, const ASTContext &Ctx, ArrayRef<WDeclTy*> decls);
  WDeclStmt(DeclStmt *S, WDeclTy *decl);

  DeclStmt *getOriginal() const {
    return static_cast<DeclStmt*>(Original);
  }

  unsigned getNumDecls() const { return NumDecls; }
  bool isSingleDecl() const { return SingleDecl != NULL; }

  WVarDecl *getVarDecl(unsigned Index) const {
    assert(Index < NumDecls && "Decl access out of range!");
    if (NumDecls == 1) {
      return SingleDecl;
    } else {
      return Decls[Index];
    }
  }
  WVarDecl *getVarDecl(DeclStmt::decl_iterator D) const {
    return getVarDecl(D - decl_begin());
  }
  WVarDecl *getSingleVarDecl() const { return SingleDecl; }

  bool hasSingleInit() const {
    return isSingleDecl() && SingleDecl->hasInit();
  }
  WExpr *getSingleInit() const {
    assert(hasSingleInit());
    return SingleDecl->getInit();
  }

  IndexedVarDeclRef getIndexedDecl() const { return IndexedDecl; }
  void setIndexedDecl(IndexedVarDeclRef IDecl) {
    assert(isSingleDecl());
    IndexedDecl = IDecl;
  }

  child_range children() { return child_range(NULL, NULL); }

  DeclStmt::decl_iterator decl_begin() {
    return getOriginal()->decl_begin();
  }
  DeclStmt::decl_iterator decl_end() {
    return getOriginal()->decl_end();
  }
  DeclStmt::const_decl_iterator decl_begin() const {
    return getOriginal()->decl_begin();
  }
  DeclStmt::const_decl_iterator decl_end() const {
    return getOriginal()->decl_end();
  }

  bool contains(WStmt *S) const;
  void replace(WStmt *From, WStmt *To);

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::DeclStmtClass;
  }
  static WDeclStmt *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                 DeclStmt *Node);
};

class WNullStmt : public WStmt {
public:
  explicit WNullStmt(NullStmt *S) : WStmt(S) {}

  NullStmt *getOriginal() const {
    return static_cast<NullStmt*>(Original);
  }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::NullStmtClass;
  }
  static WNullStmt *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                 NullStmt *Node);
};

class WCompoundStmt : public WStmt {
  WStmt **Body;
  size_t BodySize;

public:
  WCompoundStmt(CompoundStmt *S, const ASTContext &Ctx,
                ArrayRef<WStmt*> stmts);

  CompoundStmt *getOriginal() const {
    return static_cast<CompoundStmt*>(Original);
  }

  child_range children() { return child_range(Body, Body + BodySize); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::CompoundStmtClass;
  }
  static WCompoundStmt *WrapClangAST(const ASTContext &Ctx,
                                     WDeclContext &DeclCtx, CompoundStmt *Node);
};

class WSwitchCase : public WStmt {
protected:
  explicit WSwitchCase(SwitchCase *S) : WStmt(S) {}

public:
  SwitchCase *getOriginal() const {
    return static_cast<SwitchCase*>(Original);
  }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::CaseStmtClass ||
           T->getStmtClass() == Stmt::DefaultStmtClass;
  }
};

class WCaseStmt : public WSwitchCase {
  enum { LHS, SUBSTMT, END_STMT };
  WStmt *SubStmts[END_STMT];

public:
  WCaseStmt(CaseStmt *S, WExpr *lhs, WStmt *substmt)
    : WSwitchCase(S) {
    SubStmts[LHS] = reinterpret_cast<WStmt*>(lhs);
    SubStmts[SUBSTMT] = substmt;
  }

  CaseStmt *getOriginal() const {
    return static_cast<CaseStmt*>(Original);
  }

  WExpr *getLHS() const { return reinterpret_cast<WExpr*>(SubStmts[LHS]); }
  WStmt *getSubStmt() const { return SubStmts[SUBSTMT]; }

  child_range children() { return child_range(SubStmts, SubStmts + END_STMT); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::CaseStmtClass;
  }
  static WCaseStmt *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                 CaseStmt *Node);
};

class WDefaultStmt : public WSwitchCase {
  WStmt *SubStmt;

public:
  WDefaultStmt(DefaultStmt *S, WStmt *substmt)
    : WSwitchCase(S), SubStmt(substmt) {}

  DefaultStmt *getOriginal() const {
    return static_cast<DefaultStmt*>(Original);
  }

  WStmt *getSubStmt() const { return SubStmt; }

  child_range children() { return child_range(&SubStmt, &SubStmt + 1); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::DefaultStmtClass;
  }
  static WDefaultStmt *WrapClangAST(const ASTContext &Ctx,
                                    WDeclContext &DeclCtx, DefaultStmt *Node);
};

class WLabelStmt : public WStmt {
  WStmt *SubStmt;

public:
  WLabelStmt(LabelStmt *S, WStmt *substmt) : WStmt(S), SubStmt(substmt) {}

  LabelStmt *getOriginal() const {
    return static_cast<LabelStmt*>(Original);
  }

  WStmt *getSubStmt() const { return SubStmt; }
  const char *getName() const { return getOriginal()->getName(); }

  child_range children() { return child_range(&SubStmt, &SubStmt + 1); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::LabelStmtClass;
  }
  static WLabelStmt *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                  LabelStmt *Node);
};

class WAttributedStmt : public WStmt {
  WStmt *SubStmt;

public:
  WAttributedStmt(AttributedStmt *S, WStmt *substmt)
    : WStmt(S), SubStmt(substmt) {}

  AttributedStmt *getOriginal() const {
    return static_cast<AttributedStmt*>(Original);
  }

  WStmt *getSubStmt() const { return SubStmt; }
  ArrayRef<const Attr*> getAttrs() const { return getOriginal()->getAttrs(); }

  child_range children() { return child_range(&SubStmt, &SubStmt + 1); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::AttributedStmtClass;
  }
  static WAttributedStmt *WrapClangAST(const ASTContext &Ctx,
                                       WDeclContext &DeclCtx,
                                       AttributedStmt *Node);
};

class WIfStmt : public WStmt {
  enum { COND, THEN, ELSE, END_STMT };
  WStmt *SubStmts[END_STMT];

public:
  WIfStmt(IfStmt *S, WExpr *cond, WStmt *then, WStmt *elsev = 0)
    : WStmt(S) {
    SubStmts[COND] = reinterpret_cast<WStmt*>(cond);
    SubStmts[THEN] = then;
    SubStmts[ELSE] = elsev;
  }

  IfStmt *getOriginal() const {
    return static_cast<IfStmt*>(Original);
  }

  WExpr *getCond() const { return reinterpret_cast<WExpr*>(SubStmts[COND]); }
  WStmt *getThen() const { return SubStmts[THEN]; }
  WStmt *getElse() const { return SubStmts[ELSE]; }

  child_range children() { return child_range(SubStmts, SubStmts + END_STMT); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::IfStmtClass;
  }
  static WIfStmt *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                               IfStmt *Node);
};

class WSwitchStmt : public WStmt {
  enum { COND, BODY, END_STMT };
  WStmt *SubStmts[END_STMT];

public:
  WSwitchStmt(SwitchStmt *S, WExpr *cond, WStmt *body)
    : WStmt(S) {
    SubStmts[COND] = reinterpret_cast<WStmt*>(cond);
    SubStmts[BODY] = body;
  }

  SwitchStmt *getOriginal() const {
    return static_cast<SwitchStmt*>(Original);
  }

  WExpr *getCond() const { return reinterpret_cast<WExpr*>(SubStmts[COND]); }
  WStmt *getBody() const { return SubStmts[BODY]; }

  child_range children() { return child_range(SubStmts, SubStmts + END_STMT); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::SwitchStmtClass;
  }
  static WSwitchStmt *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                   SwitchStmt *Node);
};

class WWhileStmt : public WStmt {
  enum { COND, BODY, END_STMT };
  WStmt *SubStmts[END_STMT];

public:
  WWhileStmt(WhileStmt *S, WExpr *cond, WStmt *body)
    : WStmt(S) {
    SubStmts[COND] = reinterpret_cast<WStmt*>(cond);
    SubStmts[BODY] = body;
  }

  WhileStmt *getOriginal() const {
    return static_cast<WhileStmt*>(Original);
  }

  WExpr *getCond() const { return reinterpret_cast<WExpr*>(SubStmts[COND]); }
  WStmt *getBody() const { return SubStmts[BODY]; }

  child_range children() { return child_range(SubStmts, SubStmts + END_STMT); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::WhileStmtClass;
  }
  static WWhileStmt *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                  WhileStmt *Node);
};

class WDoStmt : public WStmt {
  enum { BODY, COND, END_STMT };
  WStmt *SubStmts[END_STMT];

public:
  WDoStmt(DoStmt *S, WStmt *body, WExpr *cond)
    : WStmt(S) {
    SubStmts[BODY] = body;
    SubStmts[COND] = reinterpret_cast<WStmt*>(cond);
  }

  DoStmt *getOriginal() const {
    return static_cast<DoStmt*>(Original);
  }

  WExpr *getCond() const { return reinterpret_cast<WExpr*>(SubStmts[COND]); }
  WStmt *getBody() const { return SubStmts[BODY]; }

  child_range children() { return child_range(SubStmts, SubStmts + END_STMT); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::DoStmtClass;
  }
  static WDoStmt *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                               DoStmt *Node);
};

class WForStmt : public WStmt {
  enum { INIT, COND, INC, BODY, END_STMT };
  WStmt *SubStmts[END_STMT];

public:
  WForStmt(ForStmt *S, WStmt *init, WExpr *cond, WExpr *inc, WStmt *body)
    : WStmt(S) {
    SubStmts[INIT] = init;
    SubStmts[COND] = reinterpret_cast<WStmt*>(cond);
    SubStmts[INC] = reinterpret_cast<WStmt*>(inc);
    SubStmts[BODY] = body;
  }

  ForStmt *getOriginal() const {
    return static_cast<ForStmt*>(Original);
  }

  WStmt *getInit() const { return SubStmts[INIT]; }
  WExpr *getCond() const { return reinterpret_cast<WExpr*>(SubStmts[COND]); }
  WExpr *getInc() const { return reinterpret_cast<WExpr*>(SubStmts[INC]); }
  WStmt *getBody() const { return SubStmts[BODY]; }

  child_range children() { return child_range(SubStmts, SubStmts + END_STMT); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::ForStmtClass;
  }
  static WForStmt *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                ForStmt *Node);
};

class WGotoStmt : public WStmt {
public:
  explicit WGotoStmt(GotoStmt *S) : WStmt(S) {}

  GotoStmt *getOriginal() const {
    return static_cast<GotoStmt*>(Original);
  }

  LabelDecl *getLabel() const { return getOriginal()->getLabel(); }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::GotoStmtClass;
  }
  static WGotoStmt *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                 GotoStmt *Node);
};

class WContinueStmt : public WStmt {
public:
  explicit WContinueStmt(ContinueStmt *S) : WStmt(S) {}

  ContinueStmt *getOriginal() const {
    return static_cast<ContinueStmt*>(Original);
  }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::ContinueStmtClass;
  }
  static WContinueStmt *WrapClangAST(const ASTContext &Ctx,
                                     WDeclContext &DeclCtx, ContinueStmt *Node);
};

class WBreakStmt : public WStmt {
public:
  explicit WBreakStmt(BreakStmt *S) : WStmt(S) {}

  BreakStmt *getOriginal() const {
    return static_cast<BreakStmt*>(Original);
  }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::BreakStmtClass;
  }
  static WBreakStmt *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                  BreakStmt *Node);
};

class WReturnStmt : public WStmt {
  WStmt *RetValue;

public:
  WReturnStmt(ReturnStmt *S, WExpr *ret)
    : WStmt(S), RetValue(reinterpret_cast<WStmt*>(ret)) {}

  ReturnStmt *getOriginal() const {
    return static_cast<ReturnStmt*>(Original);
  }

  WExpr *getRetExpr() const { return reinterpret_cast<WExpr*>(RetValue); }

  child_range children() { return child_range(&RetValue, &RetValue + 1); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::ReturnStmtClass;
  }
  static WReturnStmt *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                   ReturnStmt *Node);
};


// Expressions

class WExpr : public WStmt {
  QualType Type;

protected:
  WExpr(Stmt::StmtClass SC, QualType Ty)
    : WStmt(SC), Type(Ty) {}
  WExpr(WStmtClass SC, QualType Ty)
    : WStmt(SC), Type(Ty) {}
  WExpr(WStmtClass SC, Expr *E)
    : WStmt(SC, E), Type(E->getType()) {}
  explicit WExpr(Expr *E)
    : WStmt(E), Type(E->getType()) {}

public:
  Expr *getOriginal() const {
    return static_cast<Expr*>(Original);
  }

  QualType getType() const { return Type; }

  WExpr *IgnoreParens();
  WExpr *IgnoreParenCasts();
  WExpr *IgnoreParenImpCasts();
  const WExpr *IgnoreParens() const;
  const WExpr *IgnoreParenCasts() const;
  const WExpr *IgnoreParenImpCasts() const;

  bool EvaluateAsInt(llvm::APSInt &Result, const ASTContext &Ctx) const {
    if (Expr *Original = getOriginal()) {
      return Original->EvaluateAsInt(Result, Ctx);
    } else {
      return false;
    }
  }

  static bool classof(const WStmt *T) {
    return (T->getStmtClass() >= Stmt::firstExprConstant &&
            T->getStmtClass() <= Stmt::lastExprConstant) ||
           (T->getWStmtClass() >= WStmt::firstWExprConstant &&
            T->getWStmtClass() <= WStmt::lastWExprConstant);
  }
  static WExpr *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                             Expr *Node);
};

class WDeclRefExpr : public WExpr {
public:
  typedef WVarDecl WDeclTy;

private:
  WDeclTy *Decl;
  // SSA form
  IndexedVarDeclRef IndexedDefDecl;
  IndexedVarDeclRef IndexedUseDecl;

public:
  explicit WDeclRefExpr(DeclRefExpr *E)
    : WExpr(E), Decl(NULL) {}
  WDeclRefExpr(DeclRefExpr *E, WDeclTy *decl)
    : WExpr(E), Decl(decl) {}
  explicit WDeclRefExpr(WDeclTy *decl)
    : WExpr(Stmt::DeclRefExprClass, decl->getType()), Decl(decl) {}

  DeclRefExpr *getOriginal() const {
    return static_cast<DeclRefExpr*>(Original);
  }

  WVarDecl *getVarDecl() const { return Decl; }
  StringRef getName() const {
    if (Decl) {
      return Decl->getName();
    } else {
      assert(getOriginal() != NULL);
      return getOriginal()->getDecl()->getName();
    }
  }

  IndexedVarDeclRef getIndexedDefDecl() const { return IndexedDefDecl; }
  IndexedVarDeclRef getIndexedUseDecl() const { return IndexedUseDecl; }
  void setIndexedDefDecl(IndexedVarDeclRef IDecl) { IndexedDefDecl = IDecl; }
  void setIndexedUseDecl(IndexedVarDeclRef IDecl) { IndexedUseDecl = IDecl; }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::DeclRefExprClass;
  }
  static WExpr *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                             DeclRefExpr *Node);
};

class WPredefinedExpr : public WExpr {
public:
  explicit WPredefinedExpr(PredefinedExpr *E) : WExpr(E) {}

  PredefinedExpr *getOriginal() const {
    return static_cast<PredefinedExpr*>(Original);
  }

  PredefinedExpr::IdentType getIdentType() const {
    return getOriginal()->getIdentType();
  }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::PredefinedExprClass;
  }
  static WPredefinedExpr *WrapClangAST(const ASTContext &Ctx,
                                       WDeclContext &DeclCtx,
                                       PredefinedExpr *Node);
};

class WIntegerLiteral : public WExpr {
  llvm::APInt Value;

public:
  explicit WIntegerLiteral(IntegerLiteral *E)
    : WExpr(E), Value(E->getValue()) {}
  WIntegerLiteral(llvm::APInt V, QualType Ty)
    : WExpr(Stmt::IntegerLiteralClass, Ty), Value(V) {}
  WIntegerLiteral(const ASTContext &Ctx, uint64_t V, QualType Ty);

  IntegerLiteral *getOriginal() const {
    return static_cast<IntegerLiteral*>(Original);
  }

  llvm::APInt getValue() const { return Value; }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::IntegerLiteralClass;
  }
  static WIntegerLiteral *WrapClangAST(const ASTContext &Ctx,
                                       WDeclContext &DeclCtx,
                                       IntegerLiteral *Node);
};

class WCharacterLiteral : public WExpr {
  unsigned Value;
public:
  explicit WCharacterLiteral(CharacterLiteral *E)
    : WExpr(E), Value(E->getValue()) {}
  WCharacterLiteral(unsigned V, QualType Ty)
    : WExpr(Stmt::CharacterLiteralClass, Ty), Value(V) {}

  CharacterLiteral *getOriginal() const {
    return static_cast<CharacterLiteral*>(Original);
  }

  unsigned getValue() const { return Value; }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::CharacterLiteralClass;
  }
  static WCharacterLiteral *WrapClangAST(const ASTContext &Ctx,
                                         WDeclContext &DeclCtx,
                                         CharacterLiteral *Node);
};

class WFloatingLiteral : public WExpr {
public:
  explicit WFloatingLiteral(FloatingLiteral *E) : WExpr(E) {}

  FloatingLiteral *getOriginal() const {
    return static_cast<FloatingLiteral*>(Original);
  }

  llvm::APFloat getValue() const { return getOriginal()->getValue(); }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::FloatingLiteralClass;
  }
  static WFloatingLiteral *WrapClangAST(const ASTContext &Ctx,
                                        WDeclContext &DeclCtx,
                                        FloatingLiteral *Node);
};

class WStringLiteral : public WExpr {
public:
  explicit WStringLiteral(StringLiteral *E) : WExpr(E) {}

  StringLiteral *getOriginal() const {
    return static_cast<StringLiteral*>(Original);
  }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::StringLiteralClass;
  }
  static WStringLiteral *WrapClangAST(const ASTContext &Ctx,
                                      WDeclContext &DeclCtx,
                                      StringLiteral *Node);
};

class WParenExpr : public WExpr {
  WStmt *SubExpr;

public:
  WParenExpr(ParenExpr *E, WExpr *subexpr) : WExpr(E), SubExpr(subexpr) {}
  explicit WParenExpr(WExpr *subexpr)
    : WExpr(Stmt::ParenExprClass, subexpr->getType()), SubExpr(subexpr) {}

  ParenExpr *getOriginal() const {
    return static_cast<ParenExpr*>(Original);
  }

  WExpr *getSubExpr() const { return static_cast<WExpr*>(SubExpr); }

  child_range children() { return child_range(&SubExpr, &SubExpr + 1); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::ParenExprClass;
  }
  static WParenExpr *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                  ParenExpr *Node);
};

class WUnaryOperator : public WExpr {
  WStmt *SubExpr;
  UnaryOperator::Opcode Opc;

public:
  WUnaryOperator(UnaryOperator *E, WExpr *subexpr)
    : WExpr(E), SubExpr(subexpr), Opc(E->getOpcode()) {}
  WUnaryOperator(WExpr *subexpr, UnaryOperator::Opcode opc, QualType Ty)
    : WExpr(Stmt::UnaryOperatorClass, Ty), SubExpr(subexpr), Opc(opc) {}

  UnaryOperator *getOriginal() const {
    return static_cast<UnaryOperator*>(Original);
  }

  WExpr *getSubExpr() const { return static_cast<WExpr*>(SubExpr); }
  UnaryOperator::Opcode getOpcode() const { return Opc; }
  bool isPrefix() const { return UnaryOperator::isPrefix(Opc); }
  bool isPostfix() const { return UnaryOperator::isPostfix(Opc); }
  bool isIncrementOp() const { return UnaryOperator::isIncrementOp(Opc); }
  bool isDecrementOp() const { return UnaryOperator::isDecrementOp(Opc); }
  bool isIncrementDecrementOp() const {
    return UnaryOperator::isIncrementDecrementOp(Opc);
  }

  child_range children() { return child_range(&SubExpr, &SubExpr + 1); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::UnaryOperatorClass;
  }
  static WUnaryOperator *WrapClangAST(const ASTContext &Ctx,
                                      WDeclContext &DeclCtx,
                                      UnaryOperator *Node);
};

class WUnaryExprOrTypeTraitExpr : public WExpr {
  WStmt *ArgExpr;
  QualType ArgType;
  UnaryExprOrTypeTrait Kind;

public:
  WUnaryExprOrTypeTraitExpr(UnaryExprOrTypeTraitExpr *E, WExpr *argexpr)
    : WExpr(E), ArgExpr(argexpr), ArgType(), Kind(E->getKind()) {}
  explicit WUnaryExprOrTypeTraitExpr(UnaryExprOrTypeTraitExpr *E)
    : WExpr(E), ArgExpr(NULL), ArgType(E->getArgumentType()),
      Kind(E->getKind()) {}
  WUnaryExprOrTypeTraitExpr(WExpr *argexpr, UnaryExprOrTypeTrait kind,
                            QualType Ty)
    : WExpr(Stmt::UnaryExprOrTypeTraitExprClass, Ty), ArgExpr(argexpr),
      ArgType(), Kind(kind) {}
  WUnaryExprOrTypeTraitExpr(QualType argtype, UnaryExprOrTypeTrait kind,
                            QualType Ty)
    : WExpr(Stmt::UnaryExprOrTypeTraitExprClass, Ty), ArgExpr(NULL),
      ArgType(argtype), Kind(kind) {}

  UnaryExprOrTypeTraitExpr *getOriginal() const {
    return static_cast<UnaryExprOrTypeTraitExpr*>(Original);
  }

  bool isArgumentType() const { return (ArgExpr == NULL); }
  WExpr *getArgumentExpr() const { return static_cast<WExpr*>(ArgExpr); }
  QualType getArgumentType() const { return ArgType; }
  UnaryExprOrTypeTrait getKind() const { return Kind; }

  child_range children() {
    if (ArgExpr)
      return child_range(&ArgExpr, &ArgExpr + 1);
    return child_range(NULL, NULL);
  }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::UnaryExprOrTypeTraitExprClass;
  }
  static WUnaryExprOrTypeTraitExpr *WrapClangAST(
      const ASTContext &Ctx, WDeclContext &DeclCtx,
      UnaryExprOrTypeTraitExpr *Node);
};

class WArraySubscriptExpr : public WExpr {
  enum { LHS, RHS, END_EXPR };
  WStmt *SubExpr[END_EXPR];

public:
  WArraySubscriptExpr(ArraySubscriptExpr *E, WExpr *lhs, WExpr *rhs)
    : WExpr(E) {
    SubExpr[LHS] = lhs;
    SubExpr[RHS] = rhs;
  }
  WArraySubscriptExpr(WExpr *lhs, WExpr *rhs, QualType Ty)
    : WExpr(Stmt::ArraySubscriptExprClass, Ty) {
    SubExpr[LHS] = lhs;
    SubExpr[RHS] = rhs;
  }

  ArraySubscriptExpr *getOriginal() const {
    return static_cast<ArraySubscriptExpr*>(Original);
  }

  WExpr *getLHS() const { return static_cast<WExpr*>(SubExpr[LHS]); }
  WExpr *getRHS() const { return static_cast<WExpr*>(SubExpr[RHS]); }
  WExpr *getBase() const {
    return getRHS()->getType()->isIntegerType() ? getLHS() : getRHS();
  }
  WExpr *getIdx() const {
    return getRHS()->getType()->isIntegerType() ? getRHS() : getLHS();
  }

  child_range children() { return child_range(SubExpr, SubExpr + END_EXPR); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::ArraySubscriptExprClass;
  }
  static WArraySubscriptExpr *WrapClangAST(const ASTContext &Ctx,
                                           WDeclContext &DeclCtx,
                                           ArraySubscriptExpr *Node);
};

class WCallExpr : public WExpr {
public:
  enum BuiltinFunctionKind {
    BF_None,
    // Math functions
    BF_acos,
    BF_asin,
    BF_atan,
    BF_ceil,
    BF_cos,
    BF_exp,
    BF_fabs,
    BF_floor,
    BF_fmod,
    BF_log,
    BF_log2,
    BF_log10,
    BF_pow,
    BF_round,
    BF_rsqrt,
    BF_sin,
    BF_sqrt,
    BF_tan,
    BF_trunc,
    // Integer functions
    BF_abs,
    BF_max, BF_min,
    BF_mul24,
    // Vector data load and store functions
    BF_vload2, BF_vload3, BF_vload4, BF_vload8, BF_vload16,
    BF_vstore2, BF_vstore3, BF_vstore4, BF_vstore8, BF_vstore16,
    // Barrier function
    BF_barrier,
    // Atomic functions
    BF_atomic_add,
    BF_atomic_sub,
    BF_atomic_xchg,
    BF_atomic_inc,
    BF_atomic_dec,
    BF_atomic_cmpxchg,
    BF_atomic_min,
    BF_atomic_max,
    BF_atomic_and,
    BF_atomic_or,
    BF_atomic_xor,
    // Ranges
    BF_first_barrier = BF_barrier,
    BF_last_barrier = BF_barrier,
    BF_first_atomics = BF_atomic_add,
    BF_last_atomics = BF_atomic_xor,
    BF_first_side_effect = BF_vload2,
    BF_last_side_effect = BF_atomic_xor
  };

private:
  enum { FN, ARGS_START };
  WStmt **SubExprs;
  unsigned NumArgs;
  BuiltinFunctionKind BuiltinKind;

public:
  WCallExpr(CallExpr *E, const ASTContext &Ctx, WExpr *fn,
            ArrayRef<WExpr*> args);

  CallExpr *getOriginal() const {
    return static_cast<CallExpr*>(Original);
  }

  WExpr *getCallee() const { return static_cast<WExpr*>(SubExprs[FN]); }
  unsigned getNumArgs() const { return NumArgs; }
  WExpr *getArg(unsigned Arg) const {
    assert(Arg < NumArgs && "Arg access out of range!");
    return static_cast<WExpr*>(SubExprs[Arg + ARGS_START]);
  }
  FunctionDecl *getDirectCallee() const {
    return getOriginal()->getDirectCallee();
  }
  BuiltinFunctionKind getBuiltinKind() const { return BuiltinKind; }

  child_range children() {
    return child_range(SubExprs, SubExprs + NumArgs + ARGS_START);
  }

  bool isBuiltin() const { return BuiltinKind != BF_None; }

  bool isVectorLoad() const {
    return BuiltinKind >= BF_vload2 && BuiltinKind <= BF_vload16;
  }
  unsigned getVectorLoadWidth() const;
  bool isVectorStore() const {
    return BuiltinKind >= BF_vstore2 && BuiltinKind <= BF_vstore16;
  }
  unsigned getVectorStoreWidth() const;

  bool isBarrier() const {
    return BuiltinKind >= BF_first_barrier && BuiltinKind <= BF_last_barrier;
  }
  bool isAtomic() const {
    return BuiltinKind >= BF_first_atomics && BuiltinKind <= BF_last_atomics;
  }
  bool hasSideEffect() const {
    return BuiltinKind >= BF_first_side_effect &&
           BuiltinKind <= BF_last_side_effect;
  }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::CallExprClass;
  }
  static WExpr *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                             CallExpr *Node);

private:
  static unsigned GetWorkItemFunctionArg(CallExpr *E);
};

class WMemberExpr : public WExpr {
  WStmt *Base;
  FieldDecl *MemberDecl;
  bool IsArrow;
  WSubVarDecl *Var;
  // SSA form
  IndexedVarDeclRef IndexedDefDecl;
  IndexedVarDeclRef IndexedUseDecl;

public:
  WMemberExpr(MemberExpr *E, WExpr *base, WSubVarDecl *var = NULL);
  WMemberExpr(WExpr *base, FieldDecl *memberdecl, bool isarrow, QualType Ty,
              WSubVarDecl *var = NULL)
    : WExpr(Stmt::MemberExprClass, Ty), Base(base), MemberDecl(memberdecl),
      IsArrow(isarrow), Var(var) {
  }

  MemberExpr *getOriginal() const {
    return static_cast<MemberExpr*>(Original);
  }

  WExpr *getBase() const { return static_cast<WExpr*>(Base); }
  FieldDecl *getMemberDecl() const { return MemberDecl; }
  bool isArrow() const { return IsArrow; }
  WVarDecl *getVarDecl() const { return Var; }

  IndexedVarDeclRef getIndexedDefDecl() const { return IndexedDefDecl; }
  IndexedVarDeclRef getIndexedUseDecl() const { return IndexedUseDecl; }
  void setIndexedDefDecl(IndexedVarDeclRef IDecl) { IndexedDefDecl = IDecl; }
  void setIndexedUseDecl(IndexedVarDeclRef IDecl) { IndexedUseDecl = IDecl; }

  child_range children() { return child_range(&Base, &Base + 1); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::MemberExprClass;
  }
  static WExpr *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                             MemberExpr *Node);
};

class WCompoundLiteralExpr : public WExpr {
  WStmt *Init;

public:
  WCompoundLiteralExpr(CompoundLiteralExpr *E, WExpr *init)
    : WExpr(E), Init(init) {}

  CompoundLiteralExpr *getOriginal() const {
    return static_cast<CompoundLiteralExpr*>(Original);
  }

  WExpr *getInitializer() const { return static_cast<WExpr*>(Init); }

  child_range children() { return child_range(&Init, &Init + 1); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::CompoundLiteralExprClass;
  }
  static WCompoundLiteralExpr *WrapClangAST(const ASTContext &Ctx,
                                            WDeclContext &DeclCtx,
                                            CompoundLiteralExpr *Node);
};

class WCastExpr : public WExpr {
protected:
  WStmt *SubExpr;
  CastKind Kind;

  WCastExpr(CastExpr *E, WExpr *subexpr)
    : WExpr(E), SubExpr(subexpr), Kind(E->getCastKind()) {}
  WCastExpr(Stmt::StmtClass SC, WExpr *subexpr, CastKind kind, QualType Ty)
    : WExpr(SC, Ty), SubExpr(subexpr), Kind(kind) {}

public:
  CastExpr *getOriginal() const {
    return static_cast<CastExpr*>(Original);
  }

  WExpr *getSubExpr() const { return static_cast<WExpr*>(SubExpr); }
  CastKind getCastKind() const { return Kind; }

  child_range children() { return child_range(&SubExpr, &SubExpr + 1); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() >= Stmt::firstCastExprConstant &&
           T->getStmtClass() <= Stmt::lastCastExprConstant;
  }
};

class WImplicitCastExpr : public WCastExpr {
public:
  WImplicitCastExpr(ImplicitCastExpr *E, WExpr *subexpr)
    : WCastExpr(E, subexpr) {}
  WImplicitCastExpr(WExpr *subexpr, CastKind kind, QualType Ty)
    : WCastExpr(Stmt::ImplicitCastExprClass, subexpr, kind, Ty) {}

  ImplicitCastExpr *getOriginal() const {
    return static_cast<ImplicitCastExpr*>(Original);
  }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::ImplicitCastExprClass;
  }
  static WImplicitCastExpr *WrapClangAST(const ASTContext &Ctx,
                                         WDeclContext &DeclCtx,
                                         ImplicitCastExpr *Node);
};

class WExplicitCastExpr : public WCastExpr {
protected:
  WExplicitCastExpr(ExplicitCastExpr *E, WExpr *subexpr)
    : WCastExpr(E, subexpr) {}
  WExplicitCastExpr(Stmt::StmtClass SC, WExpr *subexpr, CastKind kind,
                    QualType Ty)
    : WCastExpr(SC, subexpr, kind, Ty) {}

public:
  ExplicitCastExpr *getOriginal() const {
    return static_cast<ExplicitCastExpr*>(Original);
  }

  QualType getTypeAsWritten() const {
    return getOriginal()->getTypeAsWritten();
  }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() >= Stmt::firstExplicitCastExprConstant &&
           T->getStmtClass() <= Stmt::lastExplicitCastExprConstant;
  }
};

class WCStyleCastExpr : public WExplicitCastExpr {
public:
  WCStyleCastExpr(CStyleCastExpr *E, WExpr *subexpr)
    : WExplicitCastExpr(E, subexpr) {}
  WCStyleCastExpr(WExpr *subexpr, CastKind kind, QualType Ty)
    : WExplicitCastExpr(Stmt::CStyleCastExprClass, subexpr, kind, Ty) {}

  CStyleCastExpr *getOriginal() const {
    return static_cast<CStyleCastExpr*>(Original);
  }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::CStyleCastExprClass;
  }
  static WCStyleCastExpr *WrapClangAST(const ASTContext &Ctx,
                                       WDeclContext &DeclCtx,
                                       CStyleCastExpr *Node);
};

class WBinaryOperator : public WExpr {
  enum { LHS, RHS, END_EXPR };
  WStmt *SubExprs[END_EXPR];
  BinaryOperator::Opcode Opc;

public:
  WBinaryOperator(BinaryOperator *E, WExpr *lhs, WExpr *rhs)
    : WExpr(E), Opc(E->getOpcode()) {
    SubExprs[LHS] = lhs;
    SubExprs[RHS] = rhs;
  }
  WBinaryOperator(WExpr *lhs, WExpr *rhs, BinaryOperator::Opcode opc,
                  QualType Ty)
    : WExpr(Stmt::BinaryOperatorClass, Ty), Opc(opc) {
    SubExprs[LHS] = lhs;
    SubExprs[RHS] = rhs;
  }

  BinaryOperator *getOriginal() const {
    return static_cast<BinaryOperator*>(Original);
  }

  WExpr *getLHS() const { return static_cast<WExpr*>(SubExprs[LHS]); }
  WExpr *getRHS() const { return static_cast<WExpr*>(SubExprs[RHS]); }
  BinaryOperator::Opcode getOpcode() const { return Opc; }
  bool isLogicalOp() const { return BinaryOperator::isLogicalOp(Opc); }
  bool isAssignmentOp() const { return BinaryOperator::isAssignmentOp(Opc); }
  bool isCompoundAssignmentOp() const {
    return BinaryOperator::isCompoundAssignmentOp(Opc);
  }

  child_range children() { return child_range(SubExprs, SubExprs + END_EXPR); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() >= Stmt::firstBinaryOperatorConstant &&
           T->getStmtClass() <= Stmt::lastBinaryOperatorConstant;
  }
  static WBinaryOperator *WrapClangAST(const ASTContext &Ctx,
                                       WDeclContext &DeclCtx,
                                       BinaryOperator *Node);
};

class WCompoundAssignOperator : public WBinaryOperator {
public:
  WCompoundAssignOperator(CompoundAssignOperator *E, WExpr *lhs, WExpr *rhs)
    : WBinaryOperator(E, lhs, rhs) {}
  WCompoundAssignOperator(WExpr *lhs, WExpr *rhs, BinaryOperator::Opcode opc,
                          QualType Ty)
    : WBinaryOperator(lhs, rhs, opc, Ty) {}

  CompoundAssignOperator *getOriginal() const {
    return static_cast<CompoundAssignOperator*>(Original);
  }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::CompoundAssignOperatorClass;
  }
  static WCompoundAssignOperator *WrapClangAST(const ASTContext &Ctx,
                                               WDeclContext &DeclCtx,
                                               CompoundAssignOperator *Node);
};

class WConditionalOperator : public WExpr {
  enum { COND, LHS, RHS, END_EXPR };
  WStmt *SubExprs[END_EXPR];

public:
  WConditionalOperator(ConditionalOperator *E, WExpr *cond, WExpr *lhs,
                       WExpr *rhs)
    : WExpr(E) {
    SubExprs[COND] = cond;
    SubExprs[LHS] = lhs;
    SubExprs[RHS] = rhs;
  }
  WConditionalOperator(WExpr *cond, WExpr *lhs, WExpr *rhs, QualType Ty)
    : WExpr(Stmt::ConditionalOperatorClass, Ty) {
    SubExprs[COND] = cond;
    SubExprs[LHS] = lhs;
    SubExprs[RHS] = rhs;
  }

  ConditionalOperator *getOriginal() const {
    return static_cast<ConditionalOperator*>(Original);
  }

  WExpr *getCond() const { return static_cast<WExpr*>(SubExprs[COND]); }
  WExpr *getLHS() const { return static_cast<WExpr*>(SubExprs[LHS]); }
  WExpr *getRHS() const { return static_cast<WExpr*>(SubExprs[RHS]); }

  child_range children() { return child_range(SubExprs, SubExprs + END_EXPR); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::ConditionalOperatorClass;
  }
  static WConditionalOperator *WrapClangAST(const ASTContext &Ctx,
                                            WDeclContext &DeclCtx,
                                            ConditionalOperator *Node);
};

class WInitListExpr : public WExpr {
  WStmt **InitExprs;
  unsigned NumInitExprs;

public:
  WInitListExpr(InitListExpr *E, const ASTContext &Ctx,
                ArrayRef<WExpr*> initExprs);

  InitListExpr *getOriginal() const {
    return static_cast<InitListExpr*>(Original);
  }

  unsigned getNumInits() const { return NumInitExprs; }
  WExpr *getInit(unsigned Init) const {
    assert(Init < NumInitExprs && "Initializer access out of range!");
    return static_cast<WExpr*>(InitExprs[Init]);
  }

  child_range children() {
    if (NumInitExprs == 0)
      return child_range(NULL, NULL);
    return child_range(InitExprs, InitExprs + NumInitExprs);
  }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::InitListExprClass;
  }
  static WInitListExpr *WrapClangAST(const ASTContext &Ctx,
                                     WDeclContext &DeclCtx, InitListExpr *Node);
};

class WDesignatedInitExpr : public WExpr {
  enum { INIT, INDEX_START };
  WStmt **SubExprs;
  unsigned NumSubExprs;

public:
  WDesignatedInitExpr(DesignatedInitExpr *E, const ASTContext &Ctx,
                      ArrayRef<WExpr*> indexExprs, WExpr *init);

  DesignatedInitExpr *getOriginal() const {
    return static_cast<DesignatedInitExpr*>(Original);
  }

  WExpr *getInit() const { return static_cast<WExpr*>(SubExprs[INIT]); }
  unsigned getNumSubExprs() const { return NumSubExprs; }
  WExpr *getSubExpr(unsigned Idx) const {
    assert(Idx < NumSubExprs && "Subscript out of range!");
    return static_cast<WExpr*>(SubExprs[Idx]);
  }
  WExpr *getArrayIndex(const DesignatedInitExpr::Designator &D) const;

  child_range children() {
    return child_range(SubExprs, SubExprs + NumSubExprs);
  }

  DesignatedInitExpr::designators_iterator designators_begin() {
    return getOriginal()->designators_begin();
  }
  DesignatedInitExpr::designators_iterator designators_end() {
    return getOriginal()->designators_end();
  }
  DesignatedInitExpr::const_designators_iterator designators_begin() const {
    return getOriginal()->designators_begin();
  }
  DesignatedInitExpr::const_designators_iterator designators_end() const {
    return getOriginal()->designators_end();
  }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::DesignatedInitExprClass;
  }
  static WDesignatedInitExpr *WrapClangAST(const ASTContext &Ctx,
                                           WDeclContext &DeclCtx,
                                           DesignatedInitExpr *Node);
};

class WImplicitValueInitExpr : public WExpr {
public:
  explicit WImplicitValueInitExpr(ImplicitValueInitExpr *E) : WExpr(E) {}

  ImplicitValueInitExpr *getOriginal() const {
    return static_cast<ImplicitValueInitExpr*>(Original);
  }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::ImplicitValueInitExprClass;
  }
  static WImplicitValueInitExpr *WrapClangAST(const ASTContext &Ctx,
                                              WDeclContext &DeclCtx,
                                              ImplicitValueInitExpr *Node);
};

class WParenListExpr : public WExpr {
  WStmt **Exprs;
  unsigned NumExprs;

public:
  WParenListExpr(ParenListExpr *E, const ASTContext &Ctx,
                 ArrayRef<WExpr*> exprs);

  ParenListExpr *getOriginal() const {
    return static_cast<ParenListExpr*>(Original);
  }

  unsigned getNumExprs() const { return NumExprs; }
  WExpr *getExpr(unsigned Idx) const {
    assert(Idx < NumExprs && "Initializer access out of range!");
    return static_cast<WExpr*>(Exprs[Idx]);
  }

  child_range children() { return child_range(Exprs, Exprs + NumExprs); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::ParenListExprClass;
  }
  static WParenListExpr *WrapClangAST(const ASTContext &Ctx,
                                      WDeclContext &DeclCtx,
                                      ParenListExpr *Node);
};

class WExtVectorElementExpr : public WExpr {
  WStmt *Base;
  IdentifierInfo &Accessor;
  WVarDecl *Var;
  // SSA form
  IndexedVarDeclRef IndexedDefDecl;
  IndexedVarDeclRef IndexedUseDecl;

public:
  WExtVectorElementExpr(ExtVectorElementExpr *E, WExpr *base,
                        WVarDecl *var = NULL)
    : WExpr(E), Base(base), Accessor(E->getAccessor()), Var(var) {}
  WExtVectorElementExpr(WExpr *base, IdentifierInfo &accessor, QualType Ty,
                        WVarDecl *var = NULL)
    : WExpr(Stmt::ExtVectorElementExprClass, Ty), Base(base),
      Accessor(accessor), Var(var) {}

  ExtVectorElementExpr *getOriginal() const {
    return static_cast<ExtVectorElementExpr*>(Original);
  }

  WExpr *getBase() const { return static_cast<WExpr*>(Base); }
  IdentifierInfo &getAccessor() const { return Accessor; }
  unsigned getNumElements() const;
  void getEncodedElementAccess(SmallVectorImpl<unsigned> &Elts) const;
  WVarDecl *getVarDecl() const { return Var; }

  IndexedVarDeclRef getIndexedDefDecl() const { return IndexedDefDecl; }
  IndexedVarDeclRef getIndexedUseDecl() const { return IndexedUseDecl; }
  void setIndexedDefDecl(IndexedVarDeclRef IDecl) { IndexedDefDecl = IDecl; }
  void setIndexedUseDecl(IndexedVarDeclRef IDecl) { IndexedUseDecl = IDecl; }

  child_range children() { return child_range(&Base, &Base + 1); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::ExtVectorElementExprClass;
  }
  static WExtVectorElementExpr *WrapClangAST(const ASTContext &Ctx,
                                             WDeclContext &DeclCtx,
                                             ExtVectorElementExpr *Node);
};

class WCXXBoolLiteralExpr : public WExpr {
  bool Value;

public:
  explicit WCXXBoolLiteralExpr(CXXBoolLiteralExpr *E)
    : WExpr(E), Value(E->getValue()) {}
  WCXXBoolLiteralExpr(bool V, QualType Ty)
    : WExpr(Stmt::CXXBoolLiteralExprClass, Ty), Value(V) {}

  CXXBoolLiteralExpr *getOriginal() const {
    return static_cast<CXXBoolLiteralExpr*>(Original);
  }

  bool getValue() const { return Value; }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::CXXBoolLiteralExprClass;
  }
  static WCXXBoolLiteralExpr *WrapClangAST(const ASTContext &Ctx,
                                           WDeclContext &DeclCtx,
                                           CXXBoolLiteralExpr *Node);
};

class WCXXDefaultArgExpr : public WExpr {
public:
  explicit WCXXDefaultArgExpr(CXXDefaultArgExpr *E) : WExpr(E) {}

  CXXDefaultArgExpr *getOriginal() const {
    return static_cast<CXXDefaultArgExpr*>(Original);
  }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::CXXDefaultArgExprClass;
  }
  static WCXXDefaultArgExpr *WrapClangAST(const ASTContext &Ctx,
                                          WDeclContext &DeclCtx,
                                          CXXDefaultArgExpr *Node);

};

class WCXXConstructExpr : public WExpr {
  WStmt **SubExprs;
  unsigned NumArgs;

public:
  WCXXConstructExpr(CXXConstructExpr *E, const ASTContext &Ctx,
                    ArrayRef<WExpr*> args);

  CXXConstructExpr *getOriginal() const {
    return static_cast<CXXConstructExpr*>(Original);
  }

  unsigned getNumArgs() const { return NumArgs; }
  WExpr *getArg(unsigned Arg) const {
    assert(Arg < NumArgs && "Arg access out of range!");
    return static_cast<WExpr*>(SubExprs[Arg]);
  }

  CXXConstructorDecl *getConstructor() const {
    return getOriginal()->getConstructor();
  }

  child_range children() {
    return child_range(SubExprs, SubExprs + NumArgs);
  }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::CXXConstructExprClass;
  }
  static WCXXConstructExpr *WrapClangAST(const ASTContext &Ctx,
                                         WDeclContext &DeclCtx,
                                         CXXConstructExpr *Node);
};

class WBlockExpr : public WExpr {
public:
  explicit WBlockExpr(BlockExpr *E) : WExpr(E) {}

  BlockExpr *getOriginal() const {
    return static_cast<BlockExpr*>(Original);
  }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::BlockExprClass;
  }
  static WBlockExpr *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                  BlockExpr *Node);
};

class WAsTypeExpr : public WExpr {
  WStmt *SrcExpr;

public:
  WAsTypeExpr(AsTypeExpr *E, WExpr *srcExpr) : WExpr(E), SrcExpr(srcExpr) {}
  WAsTypeExpr(WExpr *srcExpr, QualType Ty)
    : WExpr(Stmt::AsTypeExprClass, Ty), SrcExpr(srcExpr) {}

  AsTypeExpr *getOriginal() const {
    return static_cast<AsTypeExpr*>(Original);
  }

  WExpr *getSrcExpr() const { return static_cast<WExpr*>(SrcExpr); }

  child_range children() { return child_range(&SrcExpr, &SrcExpr + 1); }

  static bool classof(const WStmt *T) {
    return T->getStmtClass() == Stmt::AsTypeExprClass;
  }
  static WAsTypeExpr *WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                   AsTypeExpr *Node);
};

class WCFGBlock;

class WPhiFunction : public WStmt {
  enum { LHS, ARGS_START };
  WVarDecl *Var;
  unsigned NumArgs;
  // WPhiFunction refers IndexedVarDecl directly instead of
  // using IndexedVarDeclRef
  IndexedVarDecl **IndexedDecls;

  WPhiFunction(WVarDecl *var, unsigned numArgs)
    : WStmt(PhiFunctionClass), Var(var), NumArgs(numArgs), IndexedDecls(NULL) {}

public:
  WVarDecl *getVarDecl() const { return Var; }
  unsigned getNumArgs() const { return NumArgs; }

  IndexedVarDecl *getIndexedLHSDecl() const { return IndexedDecls[LHS]; }
  IndexedVarDecl *getIndexedArgDecl(unsigned Arg) const {
    assert(Arg < NumArgs && "Arg access out of range!");
    return IndexedDecls[Arg + ARGS_START];
  }
  void setIndexedLHSDecl(IndexedVarDecl *IDecl);
  void setIndexedArgDecl(unsigned Arg, IndexedVarDecl *IDecl);

  bool containArg(IndexedVarDecl *IDecl) const;

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getWStmtClass() == WStmt::PhiFunctionClass;
  }
  static WPhiFunction *Create(WCFGBlock *Block, WVarDecl *Var);
};

class WWorkItemFunction : public WExpr {
public:
  enum WorkItemFunctionKind {
    WIF_get_global_size,
    WIF_get_global_id,
    WIF_get_local_size,
    WIF_get_local_id,
    WIF_get_num_groups,
    WIF_get_group_id
  };

private:
  WorkItemFunctionKind FunctionKind;
  unsigned Arg;

public:
  WWorkItemFunction(Expr *original, WorkItemFunctionKind Kind, unsigned arg)
    : WExpr(WorkItemFunctionClass, original), FunctionKind(Kind), Arg(arg) {}
  WWorkItemFunction(WorkItemFunctionKind Kind, unsigned arg, QualType Ty)
    : WExpr(WorkItemFunctionClass, Ty), FunctionKind(Kind), Arg(arg) {}

  WorkItemFunctionKind getFunctionKind() const { return FunctionKind; }
  unsigned getArg() const { return Arg; }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getWStmtClass() == WStmt::WorkItemFunctionClass;
  }
};

class WMuFunction : public WExpr {
protected:
  WStmt *Loop;
  IndexedVarDecl *InitVar;
  WExpr *Step;
  bool IsIncrement; // true: InitVar+(n*Step), false: InitVar-(n*Step)

  WMuFunction(WStmtClass SC, WStmt *loop, IndexedVarDecl *init,
              WExpr *step, bool increment)
    : WExpr(SC, init->getType()), Loop(loop), InitVar(init), Step(step),
      IsIncrement(increment) {}

public:
  WStmt *getLoop() const { return Loop; }
  IndexedVarDecl *getInitVar() const { return InitVar; }
  WExpr *getStep() const { return Step; }
  bool isIncrement() const { return IsIncrement; }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const WStmt *T) {
    return T->getWStmtClass() == WStmt::BaseMuFunctionClass ||
           T->getWStmtClass() == WStmt::AuxiliaryMuFunctionClass;
  }
};

class WBaseMuFunction : public WMuFunction {
  WExpr *Bound;
  bool IsBoundInclusive;

public:
  WBaseMuFunction(WStmt *loop, IndexedVarDecl *init, WExpr *step,
                  bool increment, WExpr *bound, bool inclusive)
    : WMuFunction(BaseMuFunctionClass, loop, init, step, increment),
      Bound(bound), IsBoundInclusive(inclusive) {}

  WExpr *getBound() const { return Bound; }
  bool isBoundInclusive() const { return IsBoundInclusive; }

  static bool classof(const WStmt *T) {
    return T->getWStmtClass() == WStmt::BaseMuFunctionClass;
  }
};

class WAuxiliaryMuFunction : public WMuFunction {
  WBaseMuFunction *Base;

public:
  WAuxiliaryMuFunction(WStmt *loop, IndexedVarDecl *init, WExpr *step,
                       bool increment, WBaseMuFunction *base)
    : WMuFunction(AuxiliaryMuFunctionClass, loop, init, step, increment),
      Base(base) {}

  WBaseMuFunction *getBase() const { return Base; }

  static bool classof(const WStmt *T) {
    return T->getWStmtClass() == WStmt::AuxiliaryMuFunctionClass;
  }
};


// Statement Visitor

template<typename ImplClass, typename RetTy=void>
class WStmtVisitor {
public:
#define DISPATCH(type) \
  return static_cast<ImplClass*>(this)->Visit ## type(static_cast<W##type*>(S))

  RetTy Visit(WStmt* S) {
    if (isa<WCompoundAssignOperator>(S)) {
      DISPATCH(CompoundAssignOperator);
    }

    if (S->isClangStmt()) {
      switch (S->getStmtClass()) {
#define STMT(type) \
      case Stmt::type##Class: DISPATCH(type);
      CLANG_WSTMTS()
#undef STMT
      default:
        llvm_unreachable("invalid statement class");
      }
    } else {
      switch (S->getWStmtClass()) {
#define WSTMT(type) \
      case WStmt::type##Class: DISPATCH(type);
      EXTENDED_WSTMTS()
#undef WSTMT
      default:
        llvm_unreachable("invalid statement class");
      }
    }
  }

#undef DISPATCH
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_AST_WAST_H
