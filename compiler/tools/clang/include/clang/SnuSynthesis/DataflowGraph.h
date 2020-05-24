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

#ifndef LLVM_CLANG_SNU_SYNTHESIS_DATAFLOWGRAPH_H
#define LLVM_CLANG_SNU_SYNTHESIS_DATAFLOWGRAPH_H

#include "clang/AST/ASTContext.h"
#include "clang/AST/Decl.h"
#include "clang/AST/Type.h"
#include "clang/Analysis/Support/BumpVector.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuSupport/OrderedDenseADT.h"
#include "clang/SnuSynthesis/VirtualVariables.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/Allocator.h"
#include <utility>

namespace clang {

namespace snu {

class AliasSet;
class ControlDataflowGraph;
class DataflowGraph;
class IndexedVarDecl;
class SnuCLOptions;
class WCFG;
class WCFGBlock;
class WVarDecl;

class DFGNode {
public:
  enum DFGNodeClass {
    DFGSourceClass = 1,
    DFGSinkClass,
    DFGIntConstClass,
    DFGFloatConstClass,
    DFGUndefinedConstClass,
    DFGPlatformConstClass,
    DFGAddrOfClass,
    DFGUnaryOpClass,
    DFGBinaryOpClass,
    DFGTernaryOpClass,
    DFGRangeOpClass,
    DFGVariableRangeOpClass,
    DFGConcatOpClass,
    DFGSubstituteOpClass,
    DFGLoadClass,
    DFGStoreClass,
    DFGNullaryAtomicClass,
    DFGUnaryAtomicClass,
    DFGBinaryAtomicClass,
    DFGFunctionCallClass,
    DFGShiftRegisterClass,
    DFGQueueClass,
    DFGBarrierClass,
    DFGScatterClass,
    DFGCompoundClass
  };

private:
  DFGNodeClass Class;
  QualType ValueType;

  typedef BumpVector<DFGNode*> SuccessorListTy;
  SuccessorListTy Succs;

protected:
  void* operator new(size_t bytes) throw() {
    llvm_unreachable("DFGNodes cannot be allocated with regular 'new'.");
  }
  void operator delete(void* data) throw() {
    llvm_unreachable("DFGNodes cannot be released with regular 'delete'.");
  }

public:
  inline void* operator new(size_t bytes, const DataflowGraph& G);
  inline void* operator new(size_t bytes, const DataflowGraph* G);
  void* operator new(size_t bytes, void* mem) throw() {
    return mem;
  }

  void operator delete(void*, const DataflowGraph&) throw() { }
  void operator delete(void*, const DataflowGraph*) throw() { }
  void operator delete(void*, size_t) throw() { }
  void operator delete(void*, void*) throw() { }

protected:
  DFGNode(DataflowGraph *P, DFGNodeClass C, bool Standalone = false);
  DFGNode(DataflowGraph *P, DFGNodeClass C, QualType Ty);

public:
  DFGNodeClass getClass() const { return Class; }
  const char *getClassName() const;
  QualType getType() const { return ValueType; }
  bool isVirtualKind() const;

  typedef DFGNode** pred_iterator;
  typedef DFGNode* const* const_pred_iterator;
  typedef std::pair<pred_iterator, pred_iterator> pred_range;
  typedef std::pair<const_pred_iterator, const_pred_iterator> const_pred_range;

  pred_range predecessors();
  const_pred_range predecessors() const {
    pred_range range = const_cast<DFGNode*>(this)->predecessors();
    return const_pred_range(range.first, range.second);
  }
  pred_iterator pred_begin() { return predecessors().first; }
  pred_iterator pred_end() { return predecessors().second; }
  const_pred_iterator pred_begin() const { return predecessors().first; }
  const_pred_iterator pred_end() const { return predecessors().second; }

  typedef SuccessorListTy::iterator succ_iterator;
  typedef SuccessorListTy::const_iterator const_succ_iterator;

  succ_iterator succ_begin() { return Succs.begin(); }
  succ_iterator succ_end() { return Succs.end(); }
  const_succ_iterator succ_begin() const { return Succs.begin(); }
  const_succ_iterator succ_end() const { return Succs.end(); }

  unsigned pred_size() const {
    const_pred_range range = predecessors();
    return range.second - range.first;
  }
  unsigned succ_size() const { return Succs.size(); }

  void addSuccessor(DFGNode *succ, DataflowGraph *P);
  void removeSuccessor(DFGNode *succ, DataflowGraph *P);
  void removeAllSuccessors();
  unsigned replacePredecessor(DFGNode *old_pred, DFGNode *new_pred);
  bool replacePredecessorOnce(DFGNode *old_pred, DFGNode *new_pred);

  bool isReachableFrom(const DFGNode *from) const;

private:
  void getReachableSet(llvm::DenseSet<const DFGNode*>& set) const;
};

class DFGScatterNode;

class DFGSourceNode : public DFGNode {
public:
  explicit DFGSourceNode(DataflowGraph *P);

  IndexedVarDecl *getVariableOf(DFGScatterNode *LiveIn) const;
  DFGScatterNode *addLiveInVariable(IndexedVarDecl *var, DataflowGraph *P);

  pred_range predecessors() { return pred_range(NULL, NULL); }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGSourceClass;
  }
};

class DFGSinkNode : public DFGNode {
  typedef BumpVector<DFGNode*> PredecessorListTy;
  PredecessorListTy Preds;
  // live-out values = live-out variables (+ a condition value)
  BumpVector<unsigned> LiveOutVariableIndices;
  BumpVector<IndexedVarDecl*> LiveOutVariableDecls;
  unsigned LiveOutConditionIndex;

public:
  explicit DFGSinkNode(DataflowGraph *P);

  unsigned getNumLiveOuts() const;
  unsigned getNumLiveOutVariables() const {
    return LiveOutVariableIndices.size();
  }
  bool hasLiveOutCondition() const {
    return LiveOutConditionIndex != (unsigned)-1;
  }

  DFGNode *getLiveOut(unsigned Index) const;
  IndexedVarDecl *getLiveOutDecl(unsigned Index) const;
  DFGNode *getLiveOutVariable(unsigned Index) const;
  IndexedVarDecl *getLiveOutVariableDecl(unsigned Index) const;
  DFGNode *getLiveOutCondition() const;

  void addPredecessor(DFGNode *pred, DataflowGraph *P);
  void addLiveOutVariable(IndexedVarDecl *var, DFGNode *pred, DataflowGraph *P);
  void addLiveOutCondition(DFGNode *pred, DataflowGraph *P);

  pred_range predecessors() { return pred_range(Preds.begin(), Preds.end()); }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGSinkClass;
  }
};

class DFGConstNode : public DFGNode {
  DFGNode *Source;

protected:
  DFGConstNode(DataflowGraph *P, DFGNodeClass C, QualType Ty);

public:
  uint64_t getZExtValue() const;

  pred_range predecessors() { return pred_range(&Source, &Source + 1); }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGIntConstClass ||
           T->getClass() == DFGNode::DFGFloatConstClass ||
           T->getClass() == DFGNode::DFGUndefinedConstClass ||
           T->getClass() == DFGNode::DFGPlatformConstClass ||
           T->getClass() == DFGNode::DFGAddrOfClass;
  }
};

class DFGIntConstNode : public DFGConstNode {
  llvm::APInt Value;

public:
  DFGIntConstNode(DataflowGraph *P, const llvm::APInt &value, QualType Ty);

  llvm::APInt getValue() const { return Value; }
  uint64_t getZExtValue() const { return Value.getZExtValue(); }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGIntConstClass;
  }
};

class DFGFloatConstNode : public DFGConstNode {
  llvm::APFloat Value;

public:
  DFGFloatConstNode(DataflowGraph *P, const llvm::APFloat &value, QualType Ty);

  llvm::APFloat getValue() const { return Value; }
  uint64_t getZExtValue() const {
    return Value.bitcastToAPInt().getZExtValue();
  }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGFloatConstClass;
  }
};

class DFGUndefinedConstNode : public DFGConstNode {
public:
  DFGUndefinedConstNode(DataflowGraph *P, QualType Ty);

  uint64_t getZExtValue() const { return 0; }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGUndefinedConstClass;
  }
};

class DFGResolvableConstNode : public DFGConstNode {
  bool IsResolved;
  uint64_t Value;

protected:
  DFGResolvableConstNode(DataflowGraph *P, DFGNodeClass C, QualType Ty);

public:
  bool isResolved() const { return IsResolved; }
  uint64_t getZExtValue() const;
  void Resolve(uint64_t V);

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGPlatformConstClass ||
           T->getClass() == DFGNode::DFGAddrOfClass;
  }
};

class DFGPlatformConstNode : public DFGResolvableConstNode {
  char Name[128];

public:
  DFGPlatformConstNode(DataflowGraph *P, StringRef name, QualType Ty);

  StringRef getName() const { return Name; }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGPlatformConstClass;
  }
};

class DFGAddrOfNode : public DFGResolvableConstNode {
  WVarDecl *Var;
  unsigned AddressSpace;

public:
  DFGAddrOfNode(DataflowGraph *P, WVarDecl *V, QualType Ty);

  WVarDecl *getVariable() const { return Var; }
  unsigned getAddressSpace() const { return AddressSpace; }

  void setAddressSpace(unsigned AS) {
    AddressSpace = AS;
  }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGAddrOfClass;
  }
};

class DFGUnaryOpNode : public DFGNode {
public:
  enum OperatorKind {
    DUO_Minus,
    DUO_Not,
    DUO_LNot,
    DUO_LNot_V,
    DUO_ReinterpretCast,
    DUO_IntCast,
    DUO_IntToFloatCast,
    DUO_FloatToIntCast,
    DUO_FloatCast,
    DUO_Cos, DUO_Sin, DUO_Tan,
    DUO_Acos, DUO_Asin, DUO_Atan,
    DUO_Exp,
    DUO_Log, DUO_Log2, DUO_Log10,
    DUO_Sqrt, DUO_RSqrt,
    DUO_Ceil, DUO_Floor, DUO_Round, DUO_Trunc,
    DUO_Abs
  };

private:
  OperatorKind Opcode;
  DFGNode *Operand;

public:
  DFGUnaryOpNode(DataflowGraph *P, OperatorKind op, DFGNode *operand,
                 QualType Ty);

  OperatorKind getOpcode() const { return Opcode; }
  DFGNode *getOperand() const { return Operand; }

  pred_range predecessors() { return pred_range(&Operand, &Operand + 1); }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGUnaryOpClass;
  }
};

class DFGBinaryOpNode : public DFGNode {
public:
  enum OperatorKind {
    DBO_Mul, DBO_Div, DBO_Rem,
    DBO_Add, DBO_Sub,
    DBO_Shl, DBO_Shr,
    DBO_LT, DBO_GT, DBO_LE, DBO_GE,
    DBO_LT_V, DBO_GT_V, DBO_LE_V, DBO_GE_V,
    DBO_EQ, DBO_NE,
    DBO_EQ_V, DBO_NE_V,
    DBO_And,
    DBO_Xor,
    DBO_Or,
    DBO_LAnd,
    DBO_LAnd_V,
    DBO_LOr,
    DBO_LOr_V,
    DBO_Pow,
    DBO_Fmod,
    DBO_Max, DBO_Min,
    DBO_Mul24
  };

private:
  OperatorKind Opcode;
  DFGNode *Operands[2];

public:
  DFGBinaryOpNode(DataflowGraph *P, OperatorKind op, DFGNode *lhs, DFGNode *rhs,
                  QualType Ty);

  OperatorKind getOpcode() const { return Opcode; }
  DFGNode *getLHS() const { return Operands[0]; }
  DFGNode *getRHS() const { return Operands[1]; }

  pred_range predecessors() { return pred_range(Operands, Operands + 2); }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGBinaryOpClass;
  }
};

class DFGTernaryOpNode : public DFGNode {
public:
  enum OperatorKind {
    DTO_Conditional
  };

private:
  OperatorKind Opcode;
  DFGNode *Operands[3];

public:
  DFGTernaryOpNode(DataflowGraph *P, OperatorKind op, DFGNode *operand0,
                   DFGNode *operand1, DFGNode *operand2, QualType Ty);

  OperatorKind getOpcode() const { return Opcode; }
  DFGNode *getOperand0() const { return Operands[0]; }
  DFGNode *getOperand1() const { return Operands[1]; }
  DFGNode *getOperand2() const { return Operands[2]; }

  pred_range predecessors() { return pred_range(Operands, Operands + 3); }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGTernaryOpClass;
  }
};

class DFGRangeOpNode : public DFGNode {
  DFGNode *Operand;
  unsigned Offset;
  unsigned Width;

public:
  DFGRangeOpNode(DataflowGraph *P, DFGNode *operand, unsigned offset,
                 unsigned width, QualType Ty);

  DFGNode *getOperand() const { return Operand; }
  unsigned getOffset() const { return Offset; }
  unsigned getWidth() const { return Width; }
  unsigned getLowerIndex() const { return Offset; }
  unsigned getUpperIndex() const { return Offset + Width - 1; }

  pred_range predecessors() { return pred_range(&Operand, &Operand + 1); }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGRangeOpClass;
  }
};

class DFGVariableRangeOpNode : public DFGNode {
  enum { VALUE, OFFSET, END_PREDS };
  DFGNode *Operands[END_PREDS];
  unsigned Width;

public:
  DFGVariableRangeOpNode(DataflowGraph *P, DFGNode *value, DFGNode *offset,
                         unsigned width, QualType Ty);

  DFGNode *getValue() const { return Operands[VALUE]; }
  DFGNode *getOffset() const { return Operands[OFFSET]; }
  unsigned getWidth() const { return Width; }

  pred_range predecessors() {
    return pred_range(Operands, Operands + END_PREDS);
  }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGVariableRangeOpClass;
  }
};

class DFGConcatOpNode : public DFGNode {
  typedef BumpVector<DFGNode*> OperandListTy;
  OperandListTy Operands;

public:
  DFGConcatOpNode(DataflowGraph *P, ArrayRef<DFGNode*> args, QualType Ty);

  unsigned getNumOperands() const { return Operands.size(); }
  DFGNode *getOperand(unsigned Index) const { return Operands[Index]; }

  pred_range predecessors() {
    return pred_range(Operands.begin(), Operands.end());
  }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGConcatOpClass;
  }
};

class DFGSubstituteOpNode : public DFGNode {
  enum { VALUE, OFFSET, NEW_VALUE, END_PREDS };
  DFGNode *Operands[END_PREDS];
  unsigned Width;

public:
  DFGSubstituteOpNode(DataflowGraph *P, DFGNode *value, DFGNode *offset,
                      unsigned width, DFGNode *new_value, QualType Ty);

  DFGNode *getValue() const { return Operands[VALUE]; }
  DFGNode *getOffset() const { return Operands[OFFSET]; }
  unsigned getWidth() const { return Width; }
  DFGNode *getNewValue() const { return Operands[NEW_VALUE]; }

  pred_range predecessors() {
    return pred_range(Operands, Operands + END_PREDS);
  }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGSubstituteOpClass;
  }
};

class DFGMemoryAccessNode : public DFGNode {
protected:
  // Preds = {Ordered, Address, Operand, ..., Operand(, Additional)}
  enum { ORDERED, ADDR, END_COMMON_PREDS, MAX_NUM_PREDS = END_COMMON_PREDS + 3};
  DFGNode *Preds[MAX_NUM_PREDS];

private:
  unsigned NumPreds;
  bool HasAdditionalPred;
  QualType AccessType;
  unsigned AddressSpace;
  unsigned AliasGroup;

protected:
  DFGMemoryAccessNode(DataflowGraph *P, DFGNodeClass C, unsigned NPreds,
                      DFGNode *A, QualType AccessTy, QualType ResultTy);

public:
  bool isLoadKind() const;
  bool isStoreKind() const;

  DFGNode *getAddress() const { return Preds[ADDR]; }
  DFGNode *getOrderedPredecessor() const { return Preds[ORDERED]; }
  bool hasAdditionalPredecessor() const { return HasAdditionalPred; }
  DFGNode *getAdditionalPredecessor() const;
  QualType getAccessType() const { return AccessType; }
  unsigned getAddressSpace() const { return AddressSpace; }
  unsigned getAliasGroup() const { return AliasGroup; }

  void setOrderedPredecessor(DataflowGraph *P, DFGNode *pred);
  void setAdditionalPredecessor(DataflowGraph *P, DFGNode *pred);
  void setAddressSpace(unsigned AS) {
    AddressSpace = AS;
  }
  void setAliasGroup(unsigned AG) {
    AliasGroup = AG;
  }

  pred_range predecessors();

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGLoadClass ||
           T->getClass() == DFGNode::DFGStoreClass ||
           T->getClass() == DFGNode::DFGNullaryAtomicClass ||
           T->getClass() == DFGNode::DFGUnaryAtomicClass ||
           T->getClass() == DFGNode::DFGBinaryAtomicClass;
  }
};

class DFGLoadNode : public DFGMemoryAccessNode {
  enum { END_PREDS = END_COMMON_PREDS };

public:
  DFGLoadNode(DataflowGraph *P, DFGNode *A, QualType Ty);

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGLoadClass;
  }
};

class DFGStoreNode : public DFGMemoryAccessNode {
  enum { VALUE = END_COMMON_PREDS, END_PREDS };

public:
  DFGStoreNode(DataflowGraph *P, DFGNode *A, DFGNode *value, QualType Ty);

  DFGNode *getValue() const { return Preds[VALUE]; }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGStoreClass;
  }
};

class DFGAtomicNode : public DFGMemoryAccessNode {
protected:
  DFGAtomicNode(DataflowGraph *P, DFGNodeClass C, unsigned NPreds, DFGNode *A,
                QualType Ty);

public:
  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGNullaryAtomicClass ||
           T->getClass() == DFGNode::DFGUnaryAtomicClass ||
           T->getClass() == DFGNode::DFGBinaryAtomicClass;
  }
};

class DFGNullaryAtomicNode : public DFGAtomicNode {
public:
  enum OperatorKind {
    DNA_Inc, DNA_Dec
  };

private:
  enum { END_PREDS = END_COMMON_PREDS };
  OperatorKind Opcode;

public:
  DFGNullaryAtomicNode(DataflowGraph *P, OperatorKind op, DFGNode *A,
                       QualType Ty);

  OperatorKind getOpcode() const { return Opcode; }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGNullaryAtomicClass;
  }
};

class DFGUnaryAtomicNode : public DFGAtomicNode {
public:
  enum OperatorKind {
    DUA_Add, DUA_Sub,
    DUA_Xchg,
    DUA_Min, DUA_Max,
    DUA_And, DUA_Or, DUA_Xor
  };

private:
  enum { OPERAND = END_COMMON_PREDS, END_PREDS };
  OperatorKind Opcode;

public:
  DFGUnaryAtomicNode(DataflowGraph *P, OperatorKind op, DFGNode *A,
                     DFGNode *operand, QualType Ty);

  OperatorKind getOpcode() const { return Opcode; }
  DFGNode *getOperand() const { return Preds[OPERAND]; }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGUnaryAtomicClass;
  }
};

class DFGBinaryAtomicNode : public DFGAtomicNode {
public:
  enum OperatorKind {
    DBA_CmpXchg
  };

private:
  enum { LHS = END_COMMON_PREDS, RHS, END_PREDS };
  OperatorKind Opcode;

public:
  DFGBinaryAtomicNode(DataflowGraph *P, OperatorKind op, DFGNode *A,
                      DFGNode *lhs, DFGNode *rhs, QualType Ty);

  OperatorKind getOpcode() const { return Opcode; }
  DFGNode *getLHS() const { return Preds[LHS]; }
  DFGNode *getRHS() const { return Preds[RHS]; }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGBinaryAtomicClass;
  }
};

class DFGFunctionCallNode : public DFGNode {
  FunctionDecl *Func;
  typedef BumpVector<DFGNode*> ArgListTy;
  ArgListTy Args;

public:
  DFGFunctionCallNode(DataflowGraph *P, FunctionDecl *func,
                      ArrayRef<DFGNode*> args, QualType Ty);

  FunctionDecl *getFunction() const { return Func; }
  unsigned getNumArgs() const { return Args.size(); }
  DFGNode *getArg(unsigned Index) const { return Args[Index]; }

  pred_range predecessors() { return pred_range(Args.begin(), Args.end()); }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGFunctionCallClass;
  }
};

class DFGShiftRegisterNode : public DFGNode {
  DFGNode *Pred;
  unsigned Size;

public:
  DFGShiftRegisterNode(DataflowGraph *P, DFGNode *pred, unsigned size);

  DFGNode *getPredecessor() const { return Pred; }
  unsigned getSize() const { return Size; }

  pred_range predecessors() { return pred_range(&Pred, &Pred + 1); }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGShiftRegisterClass;
  }
};

class DFGQueueNode : public DFGNode {
  DFGNode *Pred;
  unsigned Size;

public:
  DFGQueueNode(DataflowGraph *P, DFGNode *pred, unsigned size);

  DFGNode *getPredecessor() const { return Pred; }
  unsigned getSize() const { return Size; }

  pred_range predecessors() { return pred_range(&Pred, &Pred + 1); }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGQueueClass;
  }
};

class DFGBarrierNode : public DFGNode {
  typedef BumpVector<DFGNode*> PredecessorListTy;
  PredecessorListTy Preds;

public:
  explicit DFGBarrierNode(DataflowGraph *P);

  DFGNode *getPredOf(DFGScatterNode *Succ) const;

  void addPredecessor(DFGNode *pred, DataflowGraph *P);
  DFGScatterNode *addLiveVariable(DFGNode *pred, DataflowGraph *P);

  pred_range predecessors() { return pred_range(Preds.begin(), Preds.end()); }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGBarrierClass;
  }
};

class DFGScatterNode : public DFGNode {
  DFGNode *Origin;
  uint64_t Key;

protected:
  template <typename KeyTy>
  DFGScatterNode(DataflowGraph *P, DFGNode *origin, const KeyTy &key,
                 QualType Ty)
    : DFGNode(P, DFGScatterClass, Ty), Origin(origin), Key((uint64_t)key) {
    assert(Origin != NULL);
    Origin->addSuccessor(this, P);
  }

  template <typename KeyTy>
  KeyTy getKey() const { return (KeyTy)Key; }

public:
  DFGNode *getOrigin() const { return Origin; }

  pred_range predecessors() { return pred_range(&Origin, &Origin + 1); }

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGScatterClass;
  }

  friend class DFGSourceNode;
  friend class DFGBarrierNode;
};

class DFGCompoundNode : public DFGNode {
public:
  typedef OrderedDenseSet<DFGNode*> NodeSetTy;

private:
  NodeSetTy Nodes;
  BumpVector<DFGNode*> Preds;

public:
  DFGCompoundNode(DataflowGraph *P, const NodeSetTy &N);

  typedef NodeSetTy::const_iterator iterator;
  iterator begin() const { return Nodes.begin(); }
  iterator end() const { return Nodes.end(); }

  bool count(DFGNode *Node) const { return Nodes.count(Node); }

  pred_range predecessors() { return pred_range(Preds.begin(), Preds.end()); }

  void ReloadNeighbors(DataflowGraph *P);

  static bool classof(const DFGNode *T) {
    return T->getClass() == DFGNode::DFGCompoundClass;
  }
};

template<typename ImplClass, typename RetTy=void>
class DFGNodeVisitor {
public:
#define DISPATCH(type) \
  return static_cast<ImplClass*>(this)->Visit ## type(static_cast<DFG##type*>(N))

  RetTy Visit(DFGNode *N) {
    switch (N->getClass()) {
#define NODE(type) \
    case DFGNode::DFG##type##Class: DISPATCH(type##Node);
    NODE(Source)
    NODE(Sink)
    NODE(IntConst)
    NODE(FloatConst)
    NODE(UndefinedConst)
    NODE(AddrOf)
    NODE(UnaryOp)
    NODE(BinaryOp)
    NODE(TernaryOp)
    NODE(RangeOp)
    NODE(VariableRangeOp)
    NODE(ConcatOp)
    NODE(SubstituteOp)
    NODE(Load)
    NODE(Store)
    NODE(NullaryAtomic)
    NODE(UnaryAtomic)
    NODE(BinaryAtomic)
    NODE(FunctionCall)
    NODE(ShiftRegister)
    NODE(Queue)
    NODE(Barrier)
    NODE(Scatter)
    NODE(Compound)
#undef NODE
    default:
      llvm_unreachable("invalid node class");
    }
  }

#undef DISPATCH
};


class DataflowGraph {
  DFGSourceNode *Source;
  DFGSinkNode *Sink;

  mutable BumpVectorContext BVCtx;
  typedef BumpVector<DFGNode*> DFGNodeListTy;
  DFGNodeListTy Nodes;
  bool HasTopologicalOrder;

  typedef BumpVector<IndexedVarDecl*> SSAVarListTy;
  SSAVarListTy LiveIns;
  SSAVarListTy LiveOuts;

  typedef BumpVector<DFGCompoundNode*> CompoundListTy;
  typedef llvm::DenseSet<DFGNode*> DFGNodeSetTy;
  CompoundListTy Compounds;
  DFGNodeSetTy NodesInAnyCompound;

  WCFGBlock *EntryBlock;
  WCFGBlock *ExitBlock;

public:
  DataflowGraph()
    : Source(NULL), Sink(NULL), Nodes(BVCtx, 16), HasTopologicalOrder(false),
      LiveIns(BVCtx, 16), LiveOuts(BVCtx, 16), Compounds(BVCtx, 16),
      EntryBlock(NULL), ExitBlock(NULL) {}

  typedef DFGNodeListTy::const_iterator iterator;
  typedef DFGNodeListTy::const_reverse_iterator reverse_iterator;

  iterator begin() const { return Nodes.begin(); }
  iterator end() const { return Nodes.end(); }
  reverse_iterator rbegin() const { return Nodes.rbegin(); }
  reverse_iterator rend() const { return Nodes.rend(); }

  iterator t_begin() const {
    assert(HasTopologicalOrder);
    return Nodes.begin();
  }
  iterator t_end() const {
    assert(HasTopologicalOrder);
    return Nodes.end();
  }
  reverse_iterator t_rbegin() const {
    assert(HasTopologicalOrder);
    return Nodes.rbegin();
  }
  reverse_iterator t_rend() const {
    assert(HasTopologicalOrder);
    return Nodes.rend();
  }

  unsigned size() const { return Nodes.size(); }

  DFGSourceNode *getSource() const { return Source; }
  DFGSinkNode *getSink() const { return Sink; }

  bool hasBarrier() const;

  typedef SSAVarListTy::iterator live_var_iterator;
  typedef SSAVarListTy::const_iterator const_live_var_iterator;

  live_var_iterator live_in_begin() { return LiveIns.begin(); }
  live_var_iterator live_in_end() { return LiveIns.end(); }
  const_live_var_iterator live_in_begin() const { return LiveIns.begin(); }
  const_live_var_iterator live_in_end() const { return LiveIns.end(); }
  live_var_iterator live_out_begin() { return LiveOuts.begin(); }
  live_var_iterator live_out_end() { return LiveOuts.end(); }
  const_live_var_iterator live_out_begin() const { return LiveOuts.begin(); }
  const_live_var_iterator live_out_end() const { return LiveOuts.end(); }

  void addNode(DFGNode *Node);
  void addLiveIn(IndexedVarDecl *Var);
  void addLiveOut(IndexedVarDecl *Var);
  void removeNode(DFGNode *Node);

  void addCompound(DFGCompoundNode *Compound);
  bool isNodeInAnyCompound(DFGNode *Node) const;
  DFGCompoundNode *getCompoundOf(DFGNode *Node) const;

  WCFGBlock *getEntryBlock() const { return EntryBlock; }
  WCFGBlock *getExitBlock() const { return ExitBlock; }
  void setEntryBlock(WCFGBlock *B) { EntryBlock = B; }
  void setExitBlock(WCFGBlock *B) { ExitBlock = B; }

  unsigned getNumPreds() const;
  unsigned getNumSuccs() const;
  DataflowGraph *getSucc(const ControlDataflowGraph *cdfg,
                         unsigned Index) const;

  llvm::BumpPtrAllocator &getAllocator() const {
    return BVCtx.getAllocator();
  }
  BumpVectorContext &getBumpVectorContext() const {
    return BVCtx;
  }

  void print(raw_ostream &OS) const;

private:
  void MakeTopologicalOrder();
  void PruneUnusedNodes();
  void ReloadAllCompoundNeighbors();

public:
  void Redefine() {
    MakeTopologicalOrder();
    PruneUnusedNodes();
    ReloadAllCompoundNeighbors();
  }
};

class ControlDataflowGraph {
  WCFG *cfg;
  SmallVector<DataflowGraph*, 16> DataflowGraphMap;
  DataflowGraph *Entry;
  DataflowGraph *Exit;
  VirtualVariablePool VVars;

  ControlDataflowGraph(WCFG *cfg_, const ASTContext &ASTCtx);

public:
  WCFG *getOriginal() const { return cfg; }

  typedef SmallVectorImpl<DataflowGraph*>::iterator iterator;
  typedef SmallVectorImpl<DataflowGraph*>::const_iterator const_iterator;

  iterator begin() { return DataflowGraphMap.begin(); }
  iterator end() { return DataflowGraphMap.end(); }
  const_iterator begin() const { return DataflowGraphMap.begin(); }
  const_iterator end() const { return DataflowGraphMap.end(); }

  DataflowGraph *getDFG(WCFGBlock *Block) const;
  void setDFG(WCFGBlock *Block, DataflowGraph *DFG);

  DataflowGraph *getEntry() const { return Entry; }
  DataflowGraph *getExit() const { return Exit; }

  const VirtualVariablePool &getVirtualVariablePool() const { return VVars; }

  void print(raw_ostream &OS) const;

private:
  void ConstructOnBlock(WCFGBlock *Block, const ASTContext &ASTCtx,
                        const SnuCLOptions &SnuCLOpts, const AliasSet &AS);

public:
  static ControlDataflowGraph *MakeCDFG(WCFG *cfg,
                                        const ASTContext &ASTCtx,
                                        const SnuCLOptions &SnuCLOpts,
                                        const AliasSet &AS);
};


void* DFGNode::operator new(size_t bytes, const DataflowGraph &G) {
  return ::operator new(bytes, G.getAllocator());
}

void* DFGNode::operator new(size_t bytes, const DataflowGraph *G) {
  return ::operator new(bytes, G->getAllocator());
}

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_SYNTHESIS_DATAFLOWGRAPH_H
