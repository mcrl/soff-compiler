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

#ifndef LLVM_CLANG_SNU_ANALYSIS_MEMORYACCESS_H
#define LLVM_CLANG_SNU_ANALYSIS_MEMORYACCESS_H

#include "clang/AST/ASTContext.h"
#include "clang/AST/Type.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuAST/WCFG.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/SmallVector.h"
#include <utility>

namespace clang {

namespace snu {

class Loop;

class MemoryAccess {
public:
  enum MemoryAccessKind {
    AK_READ,
    AK_WRITE,
    AK_READWRITE,
    AK_ATOMIC
  };

private:
  WExpr *AccessExpr;
  QualType AccessType;
  MemoryAccessKind AccessKind;
  WCFGBlock *BasicBlock;

  WVarDecl *Variable;
  WExpr *Pointer;
  WExpr *Index;
  MemoryAccess *Parent;
  MemoryAccess *Child;

public:
  MemoryAccess(WExpr *expr, QualType type, MemoryAccessKind kind, WCFGBlock *bb,
               WVarDecl *variable);
  MemoryAccess(WExpr *expr, QualType type, MemoryAccessKind kind, WCFGBlock *bb,
               WExpr *pointer);
  MemoryAccess(WExpr *expr, QualType type, MemoryAccessKind kind, WCFGBlock *bb,
               WExpr *pointer, WExpr *index);
  MemoryAccess(WExpr *expr, QualType type, MemoryAccessKind kind, WCFGBlock *bb,
               MemoryAccess *parent);

  WExpr *getAccessExpr() const { return AccessExpr; }
  QualType getAccessType() const { return AccessType; }
  QualType getUnqualifiedAccessType() const {
    return AccessType.getUnqualifiedType();
  }
  MemoryAccessKind getAccessKind() const { return AccessKind; }
  bool isRead() const {
    return AccessKind == AK_READ || AccessKind == AK_READWRITE ||
           AccessKind == AK_ATOMIC;
  }
  bool isWrite() const {
    return AccessKind == AK_WRITE || AccessKind == AK_READWRITE ||
           AccessKind == AK_ATOMIC;
  }
  WCFGBlock *getBasicBlock() const { return BasicBlock; }

  bool isAccessFromVariable() const { return Variable != NULL; }
  bool isAccessFromPointer() const { return Pointer != NULL; }
  bool isInheritedAccess() const { return Parent != NULL; }
  WVarDecl *getVariable() const { return Variable; }
  WExpr *getPointer() const { return Pointer; }
  WExpr *getIndex() const { return Index; }
  MemoryAccess *getParent() const { return Parent; }
  bool hasChild() const { return Child != NULL; }
  MemoryAccess *getChild() const { return Child; }
};

class MemoryAccessTrace {
  SmallVector<MemoryAccess*, 64> Trace;
  llvm::DenseMap<WExpr*, MemoryAccess*> AccessMap;
  bool IsFullTrace;

public:
  typedef bool (*FilterTy)(QualType PointeeTy);

private:
  FilterTy Filter;
  WCFGBlock *CurrentBlock;

public:
  // Commonly-used filters
  static bool AlwaysTrue(QualType PointeeTy);
  static bool GlobalAddressSpaceOnly(QualType PointeeTy);
  static bool ZeroAddressSpaceOnly(QualType PointeeTy);

  static FilterTy GetGlobalAddressSpaceFilter(const ASTContext &Ctx) {
    if (Ctx.getLangOpts().OpenCL) {
      return GlobalAddressSpaceOnly;
    } else {
      return AlwaysTrue;
    }
  }

  explicit MemoryAccessTrace(const WCFG *Program, FilterTy filter = AlwaysTrue);
  explicit MemoryAccessTrace(const WCFGBlock *Block, FilterTy filter = AlwaysTrue);
  explicit MemoryAccessTrace(const Loop &L, FilterTy filter = AlwaysTrue);
  ~MemoryAccessTrace();

  typedef SmallVectorImpl<MemoryAccess*>::const_iterator iterator;
  iterator begin() const { return Trace.begin(); }
  iterator end() const { return Trace.end(); }

  bool isFullTrace() const { return IsFullTrace; }

private:
  typedef MemoryAccess::MemoryAccessKind MemoryAccessKind;

  void FindMemoryAccesses(WCFGBlock *Block);
  void FindMemoryAccesses(WStmt *S, bool Outermost = false);
  void FindMemoryAccessesInLHS(WExpr *E, bool Compound = false);
  void FindMemoryAccessesInRHS(WExpr *E, bool Innermost = false);
  void AddMemoryAccess(WExpr *Access, QualType AccessType,
                       MemoryAccessKind AccessKind, WVarDecl *Variable);
  void AddMemoryAccess(WExpr *Access, QualType AccessType,
                       MemoryAccessKind AccessKind, WExpr *Pointer);
  void AddMemoryAccess(WExpr *Access, QualType AccessType,
                       MemoryAccessKind AccessKind, WExpr *Pointer,
                       WExpr *Index);
  void InheritMemoryAccess(WExpr *Access, QualType AccessType,
                           MemoryAccessKind AccessKind, WExpr *Parent);

  bool IsValidVariableType(QualType Ty) {
    return Filter(Ty);
  }
  bool IsValidPointerType(QualType Ty) {
    return Ty->isPointerType() && Filter(Ty->getPointeeType());
  }
};

// Binary relation between memory accesses
class MemoryAccessRelation {
  typedef std::pair<MemoryAccess*, MemoryAccess*> Pair;
  typedef llvm::DenseSet<Pair> PairSetTy;

  const MemoryAccessTrace &Source;
  PairSetTy Elements;

public:
  explicit MemoryAccessRelation(const MemoryAccessTrace &Trace)
    : Source(Trace) {
  }

  bool contains(MemoryAccess *First, MemoryAccess *Second) const {
    return Elements.count(Pair(First, Second));
  }
  bool containsAtFirst(MemoryAccess *First) const;
  bool containsAtSecond(MemoryAccess *Second) const;
  void insert(MemoryAccess *First, MemoryAccess *Second) {
    Elements.insert(Pair(First, Second));
  }

  // Commonly-used filters
  typedef bool (*FilterTy)(MemoryAccess *First, MemoryAccess *Second);
  static bool ReadRead(MemoryAccess *First, MemoryAccess *Second);
  static bool ReadWrite(MemoryAccess *First, MemoryAccess *Second);
  static bool WriteRead(MemoryAccess *First, MemoryAccess *Second);
  static bool WriteWrite(MemoryAccess *First, MemoryAccess *Second);
  void filter(FilterTy Filter);

  void join(const MemoryAccessRelation &RHS);

  static MemoryAccessRelation *CreateReachability(
      const MemoryAccessTrace &Trace, const WCFG *Program);
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_ANALYSIS_MEMORYACCESS_H
