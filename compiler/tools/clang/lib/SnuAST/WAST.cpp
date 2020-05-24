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

#include "clang/SnuAST/WAST.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/CharUnits.h"
#include "clang/AST/Attr.h"
#include "clang/AST/Decl.h"
#include "clang/AST/Expr.h"
#include "clang/AST/ExprCXX.h"
#include "clang/AST/RecordLayout.h"
#include "clang/AST/Stmt.h"
#include "clang/AST/Type.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WCFG.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/Allocator.h"
#include <utility>

namespace clang {

namespace snu {

// Declarations

WVarDecl::WVarDecl(VarDecl *D, const ASTContext &Ctx)
  : Original(D), DeclType(D->getType()), Init(NULL), NumSubVars(0),
    SubFields(NULL), SubVars(NULL) {
  Kind = D->getKind();
  if (DeclType->isStructureType()) {
    CreateSubVarsOfStructure(Ctx, DeclType);
  } else if (DeclType->isVectorType()) {
    CreateSubVarsOfVector(Ctx, DeclType);
  } else {
    // Do nothing
  }
}

void WVarDecl::CreateSubVarsOfStructure(const ASTContext &Ctx, QualType Ty) {
  assert(Ty->isStructureType());
  const RecordType *StructTy = Ty->getAsStructureType();
  assert(StructTy != NULL);
  RecordDecl *Struct = StructTy->getDecl();
  assert(Struct != NULL);
  Struct = Struct->getDefinition();
  assert(Struct != NULL && Struct->isCompleteDefinition());

  unsigned NumFields = 0;
  for (RecordDecl::field_iterator F = Struct->field_begin(),
                                  FEnd = Struct->field_end();
       F != FEnd; ++F) {
    assert((*F) != NULL);
    assert((*F)->getType()->isScalarType() && "not implemented yet");
    NumFields++;
  }
  assert(NumFields > 0);

  NumSubVars = NumFields;
  SubFields = new (Ctx) SubVarFieldTy[NumSubVars];
  SubVars = new (Ctx) WSubVarDecl*[NumSubVars];
  unsigned AddressSpace = Ty.getAddressSpace();
  unsigned Index = 0;
  for (RecordDecl::field_iterator F = Struct->field_begin(),
                                  FEnd = Struct->field_end();
       F != FEnd; ++F, ++Index) {
    QualType FieldType = Ctx.getAddrSpaceQualType((*F)->getType(),
                                                  AddressSpace);
    SubFields[Index].Field = *F;
    SubVars[Index] = new (Ctx) WSubVarDecl(this, SubFields[Index],
                                           (*F)->getName(), FieldType, Ctx);
  }
}

void WVarDecl::CreateSubVarsOfVector(const ASTContext &Ctx, QualType Ty) {
  assert(Ty->isVectorType());
  const VectorType *VectorTy = Ty->getAs<VectorType>();
  assert(VectorTy != NULL);

  NumSubVars = VectorTy->getNumElements();
  SubFields = new (Ctx) SubVarFieldTy[NumSubVars];
  SubVars = new (Ctx) WSubVarDecl*[NumSubVars];
  unsigned AddressSpace = Ty.getAddressSpace();
  QualType ElementType = Ctx.getAddrSpaceQualType(VectorTy->getElementType(),
                                                  AddressSpace);
  for (unsigned Index = 0; Index != NumSubVars; ++Index) {
    SubFields[Index].Index = Index;
    assert(Index < 100);
    char Name[4] = {'s', (char)((Index / 10) + '0'), (char)((Index % 10) + '0'),
                    '\0'};
    SubVars[Index] = new (Ctx) WSubVarDecl(this, SubFields[Index], Name,
                                           ElementType, Ctx);
  }
}

WSubVarDecl *WVarDecl::getSubVarOfStructure(FieldDecl *Field) const {
  if (isCompound() && DeclType->isStructureType()) {
    for (unsigned Index = 0; Index < NumSubVars; ++Index) {
      if (SubFields[Index].Field == Field) {
        return SubVars[Index];
      }
    }
  }
  return NULL;
}

StringRef WVarDecl::getName() const {
  if (isClangDecl()) {
    assert(getOriginal() != NULL);
    return getOriginal()->getName();
  } else {
    switch (getWKind()) {
    case WSubVarKind:
      return static_cast<const WSubVarDecl*>(this)->getName();
    case WVirtualVarKind:
      return static_cast<const WVirtualVarDecl*>(this)->getName();
    case WTemporaryVectorVarKind:
      return static_cast<const WTemporaryVectorVarDecl*>(this)->getName();
    default:
      llvm_unreachable("invalid decl kind");
    }
  }
}

bool WVarDecl::isParameter() const {
  if (const WSubVarDecl *SubVar = dyn_cast<WSubVarDecl>(this)) {
    return SubVar->getParent()->isParameter();
  }
  return isa<WParmVarDecl>(this);
}

WParmVarDecl *WVarDecl::getAsParameter() {
  if (WSubVarDecl *SubVar = dyn_cast<WSubVarDecl>(this)) {
    return SubVar->getParent()->getAsParameter();
  }
  return dyn_cast<WParmVarDecl>(this);
}

bool WVarDecl::hasAddressSpace() const {
  return getType().hasAddressSpace();
}

unsigned WVarDecl::getAddressSpace() const {
  return getType().getAddressSpace();
}

WVarDecl *WVarDecl::WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                 VarDecl *Node) {
  if (WVarDecl *L = DeclCtx.Lookup(Node)) {
    return L;
  }

  WVarDecl *WrappedNode = NULL;
  if (ParmVarDecl *PVD = dyn_cast<ParmVarDecl>(Node)) {
    WrappedNode = new (Ctx) WParmVarDecl(PVD, Ctx);
  } else {
    WrappedNode = new (Ctx) WVarDecl(Node, Ctx);
  }
  DeclCtx.Register(Node, WrappedNode);
  if (Node->hasInit()) {
    WExpr *Init = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getInit());
    WrappedNode->setInit(Init);
  }
  return WrappedNode;
}

WSubVarDecl::WSubVarDecl(WVarDecl *parent, WVarDecl::SubVarFieldTy field,
                         StringRef name, QualType T, const ASTContext &Ctx)
  : WVarDecl(WSubVarKind, NULL, T), Parent(parent), Field(field) {
  std::string readable_name = parent->getName().str() + "." + name.str();
  ReadableName = new (Ctx) char[readable_name.size() + 1];
  ::strcpy(ReadableName, readable_name.c_str());
}

uint64_t WSubVarDecl::getOffset(const ASTContext &Ctx) const {
  assert(Parent != NULL);
  QualType ParentTy = Parent->getType();
  if (ParentTy->isStructureType()) {
    FieldDecl *Decl = Field.Field;
    const ASTRecordLayout &Layout = Ctx.getASTRecordLayout(Decl->getParent());
    return Layout.getFieldOffset(Decl->getFieldIndex());
  } else if (ParentTy->isVectorType()) {
    QualType ElementTy = ParentTy->getAs<VectorType>()->getElementType();
    unsigned Index = Field.Index;
    assert(Index < 16);
    return Ctx.getTypeSize(ElementTy) * Index;
  } else {
    llvm_unreachable("impossible case");
  }
}

WVirtualVarDecl::WVirtualVarDecl(StringRef name, QualType T,
                                 const ASTContext &Ctx)
  : WVarDecl(WVirtualVarKind, NULL, T) {
  ReadableName = new (Ctx) char[name.size() + 1];
  ::strcpy(ReadableName, name.data());
  if (T->isScalarType() || T->isConstantArrayType()) {
    // Do nothing
  } else if (T->isVectorType()) {
    CreateSubVarsOfVector(Ctx, T);
  } else if (T->isStructureType()) {
    CreateSubVarsOfStructure(Ctx, T);
  } else {
    llvm_unreachable("not supported");
  }
}

WTemporaryVectorVarDecl::WTemporaryVectorVarDecl(WVarDecl *base,
                                                 ArrayRef<unsigned> elements,
                                                 QualType T,
                                                 const ASTContext &Ctx)
  : WVarDecl(WTemporaryVectorVarKind, NULL, T) {
  NumSubVars = elements.size();
  SubFields = new (Ctx) SubVarFieldTy[NumSubVars];
  SubVars = new (Ctx) WSubVarDecl*[NumSubVars];
  for (unsigned Index = 0; Index != NumSubVars; ++Index) {
    SubFields[Index].Index = Index;
    SubVars[Index] = base->getSubVar(elements[Index]);
  }
}

WVarDecl *WDeclContext::Lookup(const VarDecl *Decl) const {
  DeclWDeclMap::const_iterator I = WrappingMap.find(Decl);
  if (I != WrappingMap.end()) {
    return I->second;
  }
  return NULL;
}

void WDeclContext::Register(const VarDecl *Decl, WVarDecl *WrappedDecl) {
  WrappingMap[Decl] = WrappedDecl;
}


// Indexed Declarations

bool IndexedVarDecl::isParameter() const {
  return Decl->isParameter() && Index == 1;
}

IndexedVarDeclRef::IndexedVarDeclRef(llvm::BumpPtrAllocator &Allocator,
                                     ArrayRef<IndexedVarDecl*> vars)
  : SingleVar(NULL) {
  NumSubVars = vars.size();
  assert(NumSubVars > 0);
  void *Mem = Allocator.Allocate<IndexedVarDecl*>(NumSubVars);
  SubVars = new (Mem) IndexedVarDecl*[NumSubVars];
  std::copy(vars.begin(), vars.end(), SubVars);
  for (unsigned Index = 0; Index < NumSubVars; ++Index) {
    assert(SubVars[Index] != NULL);
  }
}

bool IndexedVarDeclRef::operator==(const IndexedVarDeclRef &RHS) const {
  if (isNull()) {
    return RHS.isNull();
  } else if (isSingleVar()) {
    return RHS.isSingleVar() && getSingleVar() == RHS.getSingleVar();
  } else {
    if (getNumSubVars() != RHS.getNumSubVars()) {
      return false;
    }
    for (unsigned Index = 0; Index < NumSubVars; ++Index) {
      if (getSubVar(Index) != RHS.getSubVar(Index)) {
        return false;
      }
    }
    return true;
  }
}

void IndexedVarDeclRef::setDefinedStmt(WStmt *S) {
  assert(!isNull());
  if (isSingleVar()) {
    getSingleVar()->setDefinedStmt(S);
  } else {
    assert(isCompound());
    for (unsigned Index = 0; Index < NumSubVars; ++Index) {
      getSubVar(Index)->setCompoundDefinedStmt(S);
    }
  }
}


// Statements

const char *WStmt::getStmtClassName() const {
  if (isClangStmt()) {
    assert(Original);
    return Original->getStmtClassName();
  } else {
    switch (getWStmtClass()) {
#define WSTMT(type) \
    case WStmt::type##Class: \
      return #type;
    EXTENDED_WSTMTS()
#undef WSTMT
    default:
      llvm_unreachable("invalid statement class");
    }
  }
}

WStmt::child_range WStmt::children() {
  if (isClangStmt()) {
    switch (getStmtClass()) {
#define STMT(type) \
    case Stmt::type##Class: \
      return static_cast<W##type*>(this)->children();
    CLANG_WSTMTS()
#undef STMT
    default:
      llvm_unreachable("invalid statement class");
    }
  } else {
    switch (getWStmtClass()) {
#define WSTMT(type) \
    case WStmt::type##Class: \
      return static_cast<W##type*>(this)->children();
    EXTENDED_WSTMTS()
#undef WSTMT
    default:
      llvm_unreachable("invalid statement class");
    }
  }
}

bool WStmt::contains(WStmt *S) const {
  if (this == S) return true;
  if (const WDeclStmt *DS = dyn_cast<WDeclStmt>(this)) {
    return DS->contains(S);
  } else {
    for (const_child_iterator C = child_begin(), CEnd = child_end();
         C != CEnd; ++C) {
      if ((*C) != NULL && (*C)->contains(S)) {
        return true;
      }
    }
  }
  return false;
}

void WStmt::replace(WStmt *From, WStmt *To) {
  if (WDeclStmt *DS = dyn_cast<WDeclStmt>(this)) {
    DS->replace(From, To);
  } else {
    for (child_iterator C = child_begin(), CEnd = child_end();
         C != CEnd; ++C) {
      if (*C == From) {
        *C = To;
      } else if (*C != NULL) {
        (*C)->replace(From, To);
      }
    }
  }
}

WDeclStmt::WDeclStmt(DeclStmt *S, const ASTContext &Ctx,
                     ArrayRef<WDeclStmt::WDeclTy*> decls)
  : WStmt(S) {
  NumDecls = decls.size();
  assert(NumDecls > 0);
  if (NumDecls == 1) {
    SingleDecl = decls.front();
    Decls = NULL;
  } else {
    SingleDecl = NULL;
    Decls = new (Ctx) WDeclStmt::WDeclTy*[NumDecls];
    std::copy(decls.begin(), decls.end(), Decls);
  }
}

WDeclStmt::WDeclStmt(DeclStmt *S, WDeclStmt::WDeclTy *decl)
  : WStmt(S) {
  NumDecls = 1;
  SingleDecl = decl;
  Decls = NULL;
}

bool WDeclStmt::contains(WStmt *S) const {
  return hasSingleInit() ? getSingleInit()->contains(S) : false;
}

void WDeclStmt::replace(WStmt *From, WStmt *To) {
  if (hasSingleInit()) {
    if (getSingleInit() == From) {
      WExpr *ToExpr = dyn_cast<WExpr>(To);
      assert(ToExpr);
      SingleDecl->setInit(ToExpr);
    } else {
      getSingleInit()->replace(From, To);
    }
  }
}

WCompoundStmt::WCompoundStmt(CompoundStmt *S, const ASTContext &Ctx,
                             ArrayRef<WStmt*> stmts)
  : WStmt(S) {
  BodySize = stmts.size();
  if (BodySize == 0) {
    Body = NULL;
    return;
  }
  Body = new (Ctx) WStmt*[BodySize];
  std::copy(stmts.begin(), stmts.end(), Body);
}


// Expressions

WExpr *WExpr::IgnoreParens() {
  WExpr *E = this;
  while (E) {
    if (WParenExpr *P = dyn_cast<WParenExpr>(E)) {
      E = P->getSubExpr();
      continue;
    }
    break;
  }
  return E;
}

WExpr *WExpr::IgnoreParenCasts() {
  WExpr *E = this;
  while (E) {
    E = E->IgnoreParens();
    if (WCastExpr *P = dyn_cast<WCastExpr>(E)) {
      E = P->getSubExpr();
      continue;
    }
    break;
  }
  return E;
}

WExpr *WExpr::IgnoreParenImpCasts() {
  WExpr *E = this;
  while (E) {
    E = E->IgnoreParens();
    if (WImplicitCastExpr *P = dyn_cast<WImplicitCastExpr>(E)) {
      E = P->getSubExpr();
      continue;
    }
    break;
  }
  return E;
}

const WExpr *WExpr::IgnoreParens() const {
  return (const_cast<WExpr*>(this))->IgnoreParens();
}

const WExpr *WExpr::IgnoreParenCasts() const {
  return (const_cast<WExpr*>(this))->IgnoreParenCasts();
}

const WExpr *WExpr::IgnoreParenImpCasts() const {
  return (const_cast<WExpr*>(this))->IgnoreParenImpCasts();
}

WIntegerLiteral::WIntegerLiteral(const ASTContext &Ctx, uint64_t V, QualType Ty)
  : WExpr(Stmt::IntegerLiteralClass, Ty),
    Value(Ctx.getTypeSize(Ty), V, Ty->isSignedIntegerType()) {
}

WCallExpr::WCallExpr(CallExpr *E, const ASTContext &Ctx, WExpr *fn,
                     ArrayRef<WExpr*> args)
  : WExpr(E) {
  NumArgs = args.size();
  SubExprs = new (Ctx) WStmt*[NumArgs + ARGS_START];
  SubExprs[FN] = fn;
  std::copy(args.begin(), args.end(), &SubExprs[ARGS_START]);

  BuiltinKind = BF_None;
  if (FunctionDecl *CalleeFunc = E->getDirectCallee()) {
    StringRef CalleeFuncName = CalleeFunc->getName();

#define CHECK_BUILTIN(name) \
    else if (CalleeFuncName == #name) \
      BuiltinKind = BF_ ## name;

    if (0) ;
    CHECK_BUILTIN(acos)
    CHECK_BUILTIN(asin)
    CHECK_BUILTIN(atan)
    CHECK_BUILTIN(atan)
    CHECK_BUILTIN(cos)
    CHECK_BUILTIN(exp)
    CHECK_BUILTIN(fabs)
    CHECK_BUILTIN(floor)
    CHECK_BUILTIN(fmod)
    CHECK_BUILTIN(log)
    CHECK_BUILTIN(log2)
    CHECK_BUILTIN(log10)
    CHECK_BUILTIN(pow)
    CHECK_BUILTIN(round)
    CHECK_BUILTIN(rsqrt)
    CHECK_BUILTIN(sin)
    CHECK_BUILTIN(sqrt)
    CHECK_BUILTIN(tan)
    CHECK_BUILTIN(trunc)
    CHECK_BUILTIN(abs)
    CHECK_BUILTIN(max)
    CHECK_BUILTIN(min)
    CHECK_BUILTIN(mul24)
    CHECK_BUILTIN(vload2)
    CHECK_BUILTIN(vload3)
    CHECK_BUILTIN(vload4)
    CHECK_BUILTIN(vload8)
    CHECK_BUILTIN(vload16)
    CHECK_BUILTIN(vstore2)
    CHECK_BUILTIN(vstore3)
    CHECK_BUILTIN(vstore4)
    CHECK_BUILTIN(vstore8)
    CHECK_BUILTIN(vstore16)
    CHECK_BUILTIN(barrier)
    CHECK_BUILTIN(atomic_add)
    CHECK_BUILTIN(atomic_sub)
    CHECK_BUILTIN(atomic_xchg)
    CHECK_BUILTIN(atomic_inc)
    CHECK_BUILTIN(atomic_dec)
    CHECK_BUILTIN(atomic_cmpxchg)
    CHECK_BUILTIN(atomic_min)
    CHECK_BUILTIN(atomic_max)
    CHECK_BUILTIN(atomic_and)
    CHECK_BUILTIN(atomic_or)
    CHECK_BUILTIN(atomic_xor)
    else if (CalleeFuncName == "atom_add")
      BuiltinKind = BF_atomic_add;
    else if (CalleeFuncName == "atom_sub")
      BuiltinKind = BF_atomic_sub;
    else if (CalleeFuncName == "atom_xchg")
      BuiltinKind = BF_atomic_xchg;
    else if (CalleeFuncName == "atom_inc")
      BuiltinKind = BF_atomic_inc;
    else if (CalleeFuncName == "atom_dec")
      BuiltinKind = BF_atomic_dec;
    else if (CalleeFuncName == "atom_cmpxchg")
      BuiltinKind = BF_atomic_cmpxchg;
    else if (CalleeFuncName == "atom_min")
      BuiltinKind = BF_atomic_min;
    else if (CalleeFuncName == "atom_max")
      BuiltinKind = BF_atomic_max;
    else if (CalleeFuncName == "atom_and")
      BuiltinKind = BF_atomic_and;
    else if (CalleeFuncName == "atom_or")
      BuiltinKind = BF_atomic_or;
    else if (CalleeFuncName == "atom_xor")
     BuiltinKind = BF_atomic_xor;

#undef CHECK_BUILTIN
  }
}

unsigned WCallExpr::getVectorLoadWidth() const {
  switch (BuiltinKind) {
    case BF_vload2: return 2;
    case BF_vload3: return 3;
    case BF_vload4: return 4;
    case BF_vload8: return 8;
    case BF_vload16: return 16;
    default: return 0;
  }
}

unsigned WCallExpr::getVectorStoreWidth() const {
  switch (BuiltinKind) {
    case BF_vstore2: return 2;
    case BF_vstore3: return 3;
    case BF_vstore4: return 4;
    case BF_vstore8: return 8;
    case BF_vstore16: return 16;
    default: return 0;
  }
}

WMemberExpr::WMemberExpr(MemberExpr *E, WExpr *base, WSubVarDecl *var)
  : WExpr(E), Base(base), MemberDecl(dyn_cast<FieldDecl>(E->getMemberDecl())),
    IsArrow(E->isArrow()), Var(var) {
  assert(MemberDecl != NULL);
}

WInitListExpr::WInitListExpr(InitListExpr *E, const ASTContext &Ctx,
                             ArrayRef<WExpr*> initExprs)
  : WExpr(E) {
  NumInitExprs = initExprs.size();
  if (NumInitExprs == 0) {
    InitExprs = NULL;
    return;
  }
  InitExprs = new (Ctx) WStmt*[NumInitExprs];
  std::copy(initExprs.begin(), initExprs.end(), InitExprs);
}

WDesignatedInitExpr::WDesignatedInitExpr(DesignatedInitExpr *E,
                                         const ASTContext& Ctx,
                                         ArrayRef<WExpr*> indexExprs,
                                         WExpr *init)
  : WExpr(E) {
  NumSubExprs = indexExprs.size() + 1;
  SubExprs = new (Ctx) WStmt*[NumSubExprs];
  SubExprs[INIT] = init;
  std::copy(indexExprs.begin(), indexExprs.end(), SubExprs + INDEX_START);
}

WExpr *WDesignatedInitExpr::getArrayIndex(
    const DesignatedInitExpr::Designator &D) const {
  // return getSubExpr(D.ArrayOrRange.Index + 1);
  // ... is impossible because D.ArrayOrRange is a private field.

  Expr *ArrayIndex = getOriginal()->getArrayIndex(D);
  for (unsigned Index = 1; Index < NumSubExprs; Index++) {
    if (SubExprs[Index]->getOriginal() == ArrayIndex)
      return static_cast<WExpr*>(SubExprs[Index]);
  }
  llvm_unreachable("invalid designator");
}

WParenListExpr::WParenListExpr(ParenListExpr *E, const ASTContext &Ctx,
                               ArrayRef<WExpr*> exprs)
  : WExpr(E) {
  NumExprs = exprs.size();
  Exprs = new (Ctx) WStmt*[NumExprs];
  std::copy(exprs.begin(), exprs.end(), Exprs);
}

unsigned WExtVectorElementExpr::getNumElements() const {
  if (const VectorType *VT = getType()->getAs<VectorType>())
    return VT->getNumElements();
  return 1;
}

void WExtVectorElementExpr::getEncodedElementAccess(
    SmallVectorImpl<unsigned> &Elts) const {
  // From ExtVectorElementExpr::getEncodedElementAccess()
  StringRef Comp = Accessor.getName();
  if (Comp[0] == 's' || Comp[0] == 'S')
    Comp = Comp.substr(1);

  bool isHi =   Comp == "hi";
  bool isLo =   Comp == "lo";
  bool isEven = Comp == "even";
  bool isOdd  = Comp == "odd";

  for (unsigned i = 0, e = getNumElements(); i != e; ++i) {
    uint64_t Index;

    if (isHi)
      Index = e + i;
    else if (isLo)
      Index = i;
    else if (isEven)
      Index = 2 * i;
    else if (isOdd)
      Index = 2 * i + 1;
    else
      Index = ExtVectorType::getAccessorIdx(Comp[i]);

    Elts.push_back(Index);
  }
}

WCXXConstructExpr::WCXXConstructExpr(CXXConstructExpr *E, const ASTContext &Ctx,
                                     ArrayRef<WExpr*> args)
  : WExpr(E) {
  NumArgs = args.size();
  SubExprs = new (Ctx) WStmt*[NumArgs];
  std::copy(args.begin(), args.end(), SubExprs);
}

void WPhiFunction::setIndexedLHSDecl(IndexedVarDecl *IDecl) {
  assert(IDecl->getDecl() == Var);
  IndexedDecls[LHS] = IDecl;
}

void WPhiFunction::setIndexedArgDecl(unsigned Arg, IndexedVarDecl *IDecl) {
  assert(IDecl->getDecl() == Var);
  IndexedDecls[Arg + ARGS_START] = IDecl;
}

bool WPhiFunction::containArg(IndexedVarDecl *IDecl) const {
  for (unsigned i = 0; i < NumArgs; ++i) {
    if (IndexedDecls[i + ARGS_START] == IDecl) {
      return true;
    }
  }
  return false;
}

// WrapClangAST

WStmt *WStmt::WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                           Stmt *Node) {
  switch (Node->getStmtClass()) {
#define STMT(type) \
  case Stmt::type##Class: \
    return W##type::WrapClangAST(Ctx, DeclCtx, static_cast<type*>(Node));
  CLANG_WSTMTS()
#undef STMT
  default:
    llvm_unreachable("invalid statement class");
  }
  return NULL;
}

WExpr *WExpr::WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                           Expr *Node) {
  return static_cast<WExpr*>(WStmt::WrapClangAST(Ctx, DeclCtx, Node));
}

WDeclStmt *WDeclStmt::WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                   DeclStmt *Node) {
  SmallVector<WDeclStmt::WDeclTy*, 2> Decls;
  for (DeclStmt::decl_iterator D = Node->decl_begin(), DEnd = Node->decl_end();
       D != DEnd; ++D) {
    if (VarDecl *VD = dyn_cast<VarDecl>(*D)) {
      Decls.push_back(WVarDecl::WrapClangAST(Ctx, DeclCtx, VD));
    } else {
      Decls.push_back(NULL);
    }
  }
  return new (Ctx) WDeclStmt(Node, Ctx, Decls);
}

WNullStmt *WNullStmt::WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                   NullStmt *Node) {
  return new (Ctx) WNullStmt(Node);
}

WCompoundStmt *WCompoundStmt::WrapClangAST(const ASTContext &Ctx,
                                           WDeclContext &DeclCtx,
                                           CompoundStmt *Node) {
  SmallVector<WStmt *, 16> Body;
  for (CompoundStmt::body_iterator S = Node->body_begin(),
                                   SEnd = Node->body_end();
       S != SEnd; ++S) {
    Body.push_back(WStmt::WrapClangAST(Ctx, DeclCtx, *S));
  }
  return new (Ctx) WCompoundStmt(Node, Ctx, Body);
}

WCaseStmt *WCaseStmt::WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                   CaseStmt *Node) {
  WExpr *LHS = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getLHS());
  WStmt *SubStmt = WStmt::WrapClangAST(Ctx, DeclCtx, Node->getSubStmt());
  return new (Ctx) WCaseStmt(Node, LHS, SubStmt);
}

WDefaultStmt *WDefaultStmt::WrapClangAST(const ASTContext &Ctx,
                                         WDeclContext &DeclCtx,
                                         DefaultStmt *Node) {
  WStmt *SubStmt = WStmt::WrapClangAST(Ctx, DeclCtx, Node->getSubStmt());
  return new (Ctx) WDefaultStmt(Node, SubStmt);
}

WLabelStmt *WLabelStmt::WrapClangAST(const ASTContext &Ctx,
                                     WDeclContext &DeclCtx, LabelStmt *Node) {
  WStmt *SubStmt = WStmt::WrapClangAST(Ctx, DeclCtx, Node->getSubStmt());
  return new (Ctx) WLabelStmt(Node, SubStmt);
}

WAttributedStmt *WAttributedStmt::WrapClangAST(const ASTContext &Ctx,
                                               WDeclContext &DeclCtx,
                                               AttributedStmt *Node) {
  WStmt *SubStmt = WStmt::WrapClangAST(Ctx, DeclCtx, Node->getSubStmt());
  return new (Ctx) WAttributedStmt(Node, SubStmt);
}

WIfStmt *WIfStmt::WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                               IfStmt *Node) {
  WExpr *Cond = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getCond());
  WStmt *Then = WStmt::WrapClangAST(Ctx, DeclCtx, Node->getThen());
  WStmt *Else = NULL;
  if (Node->getElse()) {
    Else = WStmt::WrapClangAST(Ctx, DeclCtx, Node->getElse());
  }
  return new (Ctx) WIfStmt(Node, Cond, Then, Else);
}

WSwitchStmt *WSwitchStmt::WrapClangAST(const ASTContext &Ctx,
                                       WDeclContext &DeclCtx,
                                       SwitchStmt *Node) {
  WExpr *Cond = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getCond());
  WStmt *Body = WStmt::WrapClangAST(Ctx, DeclCtx, Node->getBody());
  return new (Ctx) WSwitchStmt(Node, Cond, Body);
}

WWhileStmt *WWhileStmt::WrapClangAST(const ASTContext &Ctx,
                                     WDeclContext &DeclCtx, WhileStmt *Node) {
  WExpr *Cond = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getCond());
  WStmt *Body = WStmt::WrapClangAST(Ctx, DeclCtx, Node->getBody());
  return new (Ctx) WWhileStmt(Node, Cond, Body);
}

WDoStmt *WDoStmt::WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                               DoStmt *Node) {
  WStmt *Body = WStmt::WrapClangAST(Ctx, DeclCtx, Node->getBody());
  WExpr *Cond = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getCond());
  return new (Ctx) WDoStmt(Node, Body, Cond);
}

WForStmt *WForStmt::WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                 ForStmt *Node) {
  WStmt *Init = NULL;
  if (Node->getInit()) {
    Init = WStmt::WrapClangAST(Ctx, DeclCtx, Node->getInit());
  }
  WExpr *Cond = NULL;
  if (Node->getCond()) {
    Cond = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getCond());
  }
  WExpr *Inc = NULL;
  if (Node->getInc()) {
    Inc = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getInc());
  }
  WStmt *Body = WStmt::WrapClangAST(Ctx, DeclCtx, Node->getBody());
  return new (Ctx) WForStmt(Node, Init, Cond, Inc, Body);
}

WGotoStmt *WGotoStmt::WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                   GotoStmt *Node) {
  return new (Ctx) WGotoStmt(Node);
}

WContinueStmt *WContinueStmt::WrapClangAST(const ASTContext &Ctx,
                                           WDeclContext &DeclCtx,
                                           ContinueStmt *Node) {
  return new (Ctx) WContinueStmt(Node);
}

WBreakStmt *WBreakStmt::WrapClangAST(const ASTContext &Ctx,
                                     WDeclContext &DeclCtx, BreakStmt *Node) {
  return new (Ctx) WBreakStmt(Node);
}

WReturnStmt *WReturnStmt::WrapClangAST(const ASTContext &Ctx,
                                       WDeclContext &DeclCtx,
                                       ReturnStmt *Node) {
  WExpr *RetValue = NULL;
  if (Node->getRetValue()) {
    RetValue = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getRetValue());
  }
  return new (Ctx) WReturnStmt(Node, RetValue);
}

WExpr *WDeclRefExpr::WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                  DeclRefExpr *Node) {
  if (VarDecl *VD = dyn_cast<VarDecl>(Node->getDecl())) {
    WDeclRefExpr::WDeclTy *Decl = WVarDecl::WrapClangAST(Ctx, DeclCtx, VD);
    return new (Ctx) WDeclRefExpr(Node, Decl);
  } else if (EnumConstantDecl *Enum =
      dyn_cast<EnumConstantDecl>(Node->getDecl())) {
    return new (Ctx) WIntegerLiteral(Enum->getInitVal(), Node->getType());
  } else {
    return new (Ctx) WDeclRefExpr(Node);
  }
}

WPredefinedExpr *WPredefinedExpr::WrapClangAST(const ASTContext &Ctx,
                                               WDeclContext &DeclCtx,
                                               PredefinedExpr *Node) {
  return new (Ctx) WPredefinedExpr(Node);
}

WIntegerLiteral *WIntegerLiteral::WrapClangAST(const ASTContext &Ctx,
                                               WDeclContext &DeclCtx,
                                               IntegerLiteral *Node) {
  return new (Ctx) WIntegerLiteral(Node);
}

WCharacterLiteral *WCharacterLiteral::WrapClangAST(const ASTContext &Ctx,
                                                   WDeclContext &DeclCtx,
                                                   CharacterLiteral *Node) {
  return new (Ctx) WCharacterLiteral(Node);
}

WFloatingLiteral *WFloatingLiteral::WrapClangAST(const ASTContext &Ctx,
                                                 WDeclContext &DeclCtx,
                                                 FloatingLiteral *Node) {
  return new (Ctx) WFloatingLiteral(Node);
}

WStringLiteral *WStringLiteral::WrapClangAST(const ASTContext &Ctx,
                                             WDeclContext &DeclCtx,
                                             StringLiteral *Node) {
  return new (Ctx) WStringLiteral(Node);
}

WParenExpr *WParenExpr::WrapClangAST(const ASTContext &Ctx,
                                     WDeclContext &DeclCtx, ParenExpr *Node) {
  WExpr *SubExpr = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getSubExpr());
  return new (Ctx) WParenExpr(Node, SubExpr);
}

WUnaryOperator *WUnaryOperator::WrapClangAST(const ASTContext &Ctx,
                                             WDeclContext &DeclCtx,
                                             UnaryOperator *Node) {
  WExpr *SubExpr = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getSubExpr());
  return new (Ctx) WUnaryOperator(Node, SubExpr);
}

WUnaryExprOrTypeTraitExpr *WUnaryExprOrTypeTraitExpr::WrapClangAST(
    const ASTContext &Ctx, WDeclContext &DeclCtx,
    UnaryExprOrTypeTraitExpr *Node) {
  if (Node->isArgumentType()) {
    return new (Ctx) WUnaryExprOrTypeTraitExpr(Node);
  } else {
    WExpr *ArgumentExpr = WExpr::WrapClangAST(Ctx, DeclCtx,
                                              Node->getArgumentExpr());
    return new (Ctx) WUnaryExprOrTypeTraitExpr(Node, ArgumentExpr);
  }
}

WArraySubscriptExpr *WArraySubscriptExpr::WrapClangAST(
    const ASTContext &Ctx, WDeclContext &DeclCtx, ArraySubscriptExpr *Node) {
  WExpr *LHS = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getLHS());
  WExpr *RHS = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getRHS());
  return new (Ctx) WArraySubscriptExpr(Node, LHS, RHS);
}

WExpr *WCallExpr::WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                               CallExpr *Node) {
  if (FunctionDecl *CalleeFunc = Node->getDirectCallee()) {
    StringRef CalleeFuncName = CalleeFunc->getName();

    if (CalleeFuncName == "get_global_size") {
      return new (Ctx) WWorkItemFunction(
          Node, WWorkItemFunction::WIF_get_global_size,
          GetWorkItemFunctionArg(Node));
    } else if (CalleeFuncName == "get_global_id") {
      return new (Ctx) WWorkItemFunction(
          Node, WWorkItemFunction::WIF_get_global_id,
          GetWorkItemFunctionArg(Node));
    } else if (CalleeFuncName == "get_local_size") {
      return new (Ctx) WWorkItemFunction(
          Node, WWorkItemFunction::WIF_get_local_size,
          GetWorkItemFunctionArg(Node));
    } else if (CalleeFuncName == "get_local_id") {
      return new (Ctx) WWorkItemFunction(
          Node, WWorkItemFunction::WIF_get_local_id,
          GetWorkItemFunctionArg(Node));
    } else if (CalleeFuncName == "get_num_groups") {
      return new (Ctx) WWorkItemFunction(
          Node, WWorkItemFunction::WIF_get_num_groups,
          GetWorkItemFunctionArg(Node));
    } else if (CalleeFuncName == "get_group_id") {
      return new (Ctx) WWorkItemFunction(
          Node, WWorkItemFunction::WIF_get_group_id,
          GetWorkItemFunctionArg(Node));
    }
  }

  WExpr *Callee = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getCallee());
  SmallVector<WExpr *, 16> Args;
  for (CallExpr::arg_iterator A = Node->arg_begin(), AEnd = Node->arg_end();
       A != AEnd; ++A) {
    Args.push_back(WExpr::WrapClangAST(Ctx, DeclCtx, *A));
  }
  return new (Ctx) WCallExpr(Node, Ctx, Callee, Args);
}

unsigned WCallExpr::GetWorkItemFunctionArg(CallExpr *E) {
  assert(E->getNumArgs() == 1);
  Expr *Arg = E->getArg(0)->IgnoreParenCasts();
  IntegerLiteral *ArgLiteral = dyn_cast<IntegerLiteral>(Arg);
  assert(ArgLiteral != NULL);

  llvm::APInt ArgValue = ArgLiteral->getValue();
  if (ArgValue == 0) return 0;
  else if (ArgValue == 1) return 1;
  else if (ArgValue == 2) return 2;
  else llvm_unreachable("invalid work-item function argument");
}

WExpr *WMemberExpr::WrapClangAST(const ASTContext &Ctx, WDeclContext &DeclCtx,
                                 MemberExpr *Node) {
  if (isa<DeclRefExpr>(Node->getBase()) && !Node->isArrow()) {
    DeclRefExpr *Base = dyn_cast<DeclRefExpr>(Node->getBase());
    assert(Base != NULL);
    NamedDecl *BaseDecl = Base->getFoundDecl();
    NamedDecl *FieldDecl = dyn_cast<NamedDecl>(Node->getMemberDecl());
    if (BaseDecl != NULL && FieldDecl != NULL) {
      if (BaseDecl->getName() == "threadIdx") {
        if (FieldDecl->getName() == "x") {
          return new (Ctx) WWorkItemFunction(
              Node, WWorkItemFunction::WIF_get_local_id, 0);
        } else if (FieldDecl->getName() == "y") {
          return new (Ctx) WWorkItemFunction(
              Node, WWorkItemFunction::WIF_get_local_id, 1);
        } else if (FieldDecl->getName() == "z") {
          return new (Ctx) WWorkItemFunction(
              Node, WWorkItemFunction::WIF_get_local_id, 2);
        }
      } else if (BaseDecl->getName() == "blockIdx") {
        if (FieldDecl->getName() == "x") {
          return new (Ctx) WWorkItemFunction(
              Node, WWorkItemFunction::WIF_get_group_id, 0);
        } else if (FieldDecl->getName() == "y") {
          return new (Ctx) WWorkItemFunction(
              Node, WWorkItemFunction::WIF_get_group_id, 1);
        } else if (FieldDecl->getName() == "z") {
          return new (Ctx) WWorkItemFunction(
              Node, WWorkItemFunction::WIF_get_group_id, 2);
        }
      } else if (BaseDecl->getName() == "blockDim") {
        if (FieldDecl->getName() == "x") {
          return new (Ctx) WWorkItemFunction(
              Node, WWorkItemFunction::WIF_get_local_size, 0);
        } else if (FieldDecl->getName() == "y") {
          return new (Ctx) WWorkItemFunction(
              Node, WWorkItemFunction::WIF_get_local_size, 1);
        } else if (FieldDecl->getName() == "z") {
          return new (Ctx) WWorkItemFunction(
              Node, WWorkItemFunction::WIF_get_local_size, 2);
        }
      } else if (BaseDecl->getName() == "gridDim") {
        if (FieldDecl->getName() == "x") {
          return new (Ctx) WWorkItemFunction(
              Node, WWorkItemFunction::WIF_get_num_groups, 0);
        } else if (FieldDecl->getName() == "y") {
          return new (Ctx) WWorkItemFunction(
              Node, WWorkItemFunction::WIF_get_num_groups, 1);
        } else if (FieldDecl->getName() == "z") {
          return new (Ctx) WWorkItemFunction(
              Node, WWorkItemFunction::WIF_get_num_groups, 2);
        }
      }
    }
  }

  WExpr *Base = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getBase());
  if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(Base->IgnoreParenImpCasts())) {
    if (WVarDecl *Var = DRE->getVarDecl()) {
      FieldDecl *Field = dyn_cast<FieldDecl>(Node->getMemberDecl());
      assert(Field != NULL);
      if (WSubVarDecl *SubVar = Var->getSubVarOfStructure(Field)) {
        return new (Ctx) WMemberExpr(Node, Base, SubVar);
      }
    }
  }
  return new (Ctx) WMemberExpr(Node, Base);
}

WCompoundLiteralExpr *WCompoundLiteralExpr::WrapClangAST(
    const ASTContext &Ctx, WDeclContext &DeclCtx, CompoundLiteralExpr *Node) {
  WExpr *Init = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getInitializer());
  return new (Ctx) WCompoundLiteralExpr(Node, Init);
}

WImplicitCastExpr *WImplicitCastExpr::WrapClangAST(const ASTContext &Ctx,
                                                   WDeclContext &DeclCtx,
                                                   ImplicitCastExpr *Node) {
  WExpr *SubExpr = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getSubExpr());
  return new (Ctx) WImplicitCastExpr(Node, SubExpr);
}

WCStyleCastExpr *WCStyleCastExpr::WrapClangAST(const ASTContext &Ctx,
                                               WDeclContext &DeclCtx,
                                               CStyleCastExpr *Node) {
  WExpr *SubExpr = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getSubExpr());
  return new (Ctx) WCStyleCastExpr(Node, SubExpr);
}

WBinaryOperator *WBinaryOperator::WrapClangAST(const ASTContext &Ctx,
                                               WDeclContext &DeclCtx,
                                               BinaryOperator *Node) {
  WExpr *LHS = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getLHS());
  WExpr *RHS = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getRHS());
  return new (Ctx) WBinaryOperator(Node, LHS, RHS);
}

WCompoundAssignOperator *WCompoundAssignOperator::WrapClangAST(
    const ASTContext &Ctx, WDeclContext &DeclCtx,
    CompoundAssignOperator *Node) {
  WExpr *LHS = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getLHS());
  WExpr *RHS = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getRHS());
  return new (Ctx) WCompoundAssignOperator(Node, LHS, RHS);
}

WConditionalOperator *WConditionalOperator::WrapClangAST(
    const ASTContext &Ctx, WDeclContext &DeclCtx, ConditionalOperator *Node) {
  WExpr *Cond = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getCond());
  WExpr *LHS = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getLHS());
  WExpr *RHS = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getRHS());
  return new (Ctx) WConditionalOperator(Node, Cond, LHS, RHS);
}

WInitListExpr *WInitListExpr::WrapClangAST(const ASTContext &Ctx,
                                           WDeclContext &DeclCtx,
                                           InitListExpr *Node) {
  SmallVector<WExpr *, 16> Inits;
  for (unsigned Index = 0, NumInits = Node->getNumInits();
       Index != NumInits; ++Index) {
    if (Node->getInit(Index)) {
      Inits.push_back(WExpr::WrapClangAST(Ctx, DeclCtx, Node->getInit(Index)));
    } else {
      Inits.push_back(NULL);
    }
  }
  return new (Ctx) WInitListExpr(Node, Ctx, Inits);
}

WDesignatedInitExpr *WDesignatedInitExpr::WrapClangAST(
    const ASTContext &Ctx, WDeclContext &DeclCtx, DesignatedInitExpr *Node) {
  SmallVector<WExpr *, 16> Indices;
  for (unsigned Index = 1, NumSubExprs = Node->getNumSubExprs();
       Index != NumSubExprs; ++Index) {
    Indices.push_back(WExpr::WrapClangAST(Ctx, DeclCtx,
                                          Node->getSubExpr(Index)));
  }
  WExpr *Init = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getInit());
  return new (Ctx) WDesignatedInitExpr(Node, Ctx, Indices, Init);
}

WImplicitValueInitExpr *WImplicitValueInitExpr::WrapClangAST(
    const ASTContext &Ctx, WDeclContext &DeclCtx, ImplicitValueInitExpr *Node) {
  return new (Ctx) WImplicitValueInitExpr(Node);
}

WParenListExpr *WParenListExpr::WrapClangAST(const ASTContext &Ctx,
                                             WDeclContext &DeclCtx,
                                             ParenListExpr *Node) {
  SmallVector<WExpr *, 16> Exprs;
  for (unsigned Index = 0, NumExprs = Node->getNumExprs();
       Index != NumExprs; ++Index) {
    if (Node->getExpr(Index)) {
      Exprs.push_back(WExpr::WrapClangAST(Ctx, DeclCtx, Node->getExpr(Index)));
    } else {
      Exprs.push_back(NULL);
    }
  }
  return new (Ctx) WParenListExpr(Node, Ctx, Exprs);
}

WExtVectorElementExpr *WExtVectorElementExpr::WrapClangAST(
    const ASTContext &Ctx, WDeclContext &DeclCtx, ExtVectorElementExpr *Node) {
  WExpr *Base = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getBase());
  WExpr *RealBase = Base->IgnoreParenImpCasts();
  WVarDecl *BaseVar = NULL;
  if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(RealBase)) {
    BaseVar = DRE->getVarDecl();
  } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(RealBase)) {
    BaseVar = ME->getVarDecl();
  }
  if (BaseVar) {
    SmallVector<unsigned, 16> Elements;
    Node->getEncodedElementAccess(Elements);
    WVarDecl *Var = NULL;
    if (Node->getNumElements() == 1) {
      Var = BaseVar->getSubVar(Elements[0]);
    } else {
      Var = new (Ctx) WTemporaryVectorVarDecl(BaseVar, Elements,
                                              Node->getType(), Ctx);
    }
    return new (Ctx) WExtVectorElementExpr(Node, Base, Var);
  } else {
    return new (Ctx) WExtVectorElementExpr(Node, Base);
  }
}

WCXXBoolLiteralExpr *WCXXBoolLiteralExpr::WrapClangAST(
    const ASTContext &Ctx, WDeclContext &DeclCtx, CXXBoolLiteralExpr *Node) {
  return new (Ctx) WCXXBoolLiteralExpr(Node);
}

WCXXDefaultArgExpr *WCXXDefaultArgExpr::WrapClangAST(const ASTContext &Ctx,
                                                     WDeclContext &DeclCtx,
                                                     CXXDefaultArgExpr *Node) {
  return new (Ctx) WCXXDefaultArgExpr(Node);
}

WCXXConstructExpr *WCXXConstructExpr::WrapClangAST(const ASTContext &Ctx,
                                                   WDeclContext &DeclCtx,
                                                   CXXConstructExpr *Node) {
  SmallVector<WExpr *, 16> Args;
  for (CXXConstructExpr::arg_iterator A = Node->arg_begin(),
                                      AEnd = Node->arg_end();
       A != AEnd; ++A) {
    Args.push_back(WExpr::WrapClangAST(Ctx, DeclCtx, *A));
  }
  return new (Ctx) WCXXConstructExpr(Node, Ctx, Args);
}

WBlockExpr *WBlockExpr::WrapClangAST(const ASTContext &Ctx,
                                     WDeclContext &DeclCtx, BlockExpr *Node) {
  return new (Ctx) WBlockExpr(Node);
}

WAsTypeExpr *WAsTypeExpr::WrapClangAST(const ASTContext &Ctx,
                                       WDeclContext &DeclCtx,
                                       AsTypeExpr *Node) {
  WExpr *SrcExpr = WExpr::WrapClangAST(Ctx, DeclCtx, Node->getSrcExpr());
  return new (Ctx) WAsTypeExpr(Node, SrcExpr);
}

WPhiFunction *WPhiFunction::Create(WCFGBlock *Block, WVarDecl *Var) {
  unsigned NumArgs = Block->pred_size();
  void *Mem = Block->getAllocator().Allocate<WPhiFunction>();
  WPhiFunction *PF = new (Mem) WPhiFunction(Var, NumArgs);
  PF->IndexedDecls =
    Block->getAllocator().Allocate<IndexedVarDecl*>(NumArgs + ARGS_START);
  PF->IndexedDecls[LHS] = NULL;
  for (unsigned Index = 0; Index < NumArgs; Index++) {
    PF->IndexedDecls[Index + ARGS_START] = NULL;
  }
  return PF;
}

} // namespace snu

} // namespace clang
