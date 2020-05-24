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

#include "clang/SnuSynthesis/DataflowGraph.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/CharUnits.h"
#include "clang/AST/Decl.h"
#include "clang/AST/Expr.h"
#include "clang/AST/OperationKinds.h"
#include "clang/AST/RecordLayout.h"
#include "clang/AST/Type.h"
#include "clang/Analysis/Support/BumpVector.h"
#include "clang/Basic/AddressSpaces.h"
#include "clang/Basic/LLVM.h"
#include "clang/Basic/LangOptions.h"
#include "clang/Basic/TypeTraits.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuAST/WCFG.h"
#include "clang/SnuAnalysis/LiveVariables.h"
#include "clang/SnuAnalysis/PointerAnalysis.h"
#include "clang/SnuFrontend/Options.h"
#include "clang/SnuSupport/OrderedDenseADT.h"
#include "clang/SnuSynthesis/VirtualVariables.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/APSInt.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/Allocator.h"
#include <cstring>
#include <map>
#include <utility>

namespace clang {

namespace snu {

// DFGNode

DFGNode::DFGNode(DataflowGraph *P, DFGNodeClass C, bool Standalone)
  : Class(C), ValueType(), Succs(P->getBumpVectorContext(), 1) {
  if (!Standalone) {
    P->addNode(this);
  }
}

DFGNode::DFGNode(DataflowGraph *P, DFGNodeClass C, QualType Ty)
  : Class(C), ValueType(Ty), Succs(P->getBumpVectorContext(), 1) {
  P->addNode(this);
}

const char *DFGNode::getClassName() const {
  switch (getClass()) {
#define NODE(type) \
  case DFGNode::DFG##type##Class: \
    return #type;
  NODE(Source)
  NODE(Sink)
  NODE(IntConst)
  NODE(FloatConst)
  NODE(UndefinedConst)
  NODE(PlatformConst)
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

bool DFGNode::isVirtualKind() const {
  switch (getClass()) {
    case DFGNode::DFGIntConstClass:
    case DFGNode::DFGFloatConstClass:
    case DFGNode::DFGUndefinedConstClass:
    case DFGNode::DFGPlatformConstClass:
    case DFGNode::DFGAddrOfClass:
    case DFGNode::DFGScatterClass:
      return true;
    default:
      return false;
  }
}

DFGNode::pred_range DFGNode::predecessors() {
  switch (getClass()) {
#define NODE(type) \
  case DFGNode::DFG##type##Class: \
    return static_cast<DFG##type##Node*>(this)->predecessors();
  NODE(Source)
  NODE(Sink)
  NODE(IntConst)
  NODE(FloatConst)
  NODE(UndefinedConst)
  NODE(PlatformConst)
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

void DFGNode::addSuccessor(DFGNode *succ, DataflowGraph *P) {
  assert(!(isa<DFGScatterNode>(succ) && !ValueType.isNull()));
  Succs.push_back(succ, P->getBumpVectorContext());
}

void DFGNode::removeSuccessor(DFGNode *succ, DataflowGraph *P) {
  assert(!isa<DFGScatterNode>(succ)); // cannot unlink a scatter
  SmallVector<DFGNode*, 16> NewSuccs;
  for (succ_iterator S = succ_begin(), SEnd = succ_end();
       S != SEnd; ++S) {
    if (*S != succ) {
      NewSuccs.push_back(*S);
    }
  }
  Succs.clear();
  for (SmallVectorImpl<DFGNode*>::iterator S = NewSuccs.begin(),
                                           SEnd = NewSuccs.end();
       S != SEnd; ++S) {
    Succs.push_back(*S, P->getBumpVectorContext());
  }
}

void DFGNode::removeAllSuccessors() {
  Succs.clear();
}

unsigned DFGNode::replacePredecessor(DFGNode *old_pred, DFGNode *new_pred) {
  unsigned count = 0;
  for (pred_iterator P = pred_begin(), PEnd = pred_end();
       P != PEnd; ++P) {
    if (*P == old_pred) {
      *P = new_pred;
      count++;
    }
  }
  return count;
}

bool DFGNode::replacePredecessorOnce(DFGNode *old_pred, DFGNode *new_pred) {
  bool count = false;
  for (pred_iterator P = pred_begin(), PEnd = pred_end();
       P != PEnd; ++P) {
    if (*P == old_pred) {
      *P = new_pred;
      count = true;
      break;
    }
  }
  return count;

}

bool DFGNode::isReachableFrom(const DFGNode *from) const {
  llvm::DenseSet<const DFGNode*> RSet;
  getReachableSet(RSet);
  return RSet.count(from);
}

void DFGNode::getReachableSet(llvm::DenseSet<const DFGNode*>& set) const {
  if (!set.count(this)) {
    set.insert(this);
    for (const_pred_iterator P = pred_begin(), PEnd = pred_end();
         P != PEnd; ++P) {
      (*P)->getReachableSet(set);
    }
  }
}

// DFGSourceNode
// (this) -> LiveIn (Scatter)
//        -> LiveIn (Scatter)
//        -> ...
//        -> Const
//        -> Const
//        -> ...

DFGSourceNode::DFGSourceNode(DataflowGraph *P)
  : DFGNode(P, DFGSourceClass) {
}

IndexedVarDecl *DFGSourceNode::getVariableOf(DFGScatterNode *LiveIn) const {
  return LiveIn->getKey<IndexedVarDecl*>();
}

DFGScatterNode *DFGSourceNode::addLiveInVariable(IndexedVarDecl *var,
                                                 DataflowGraph *P) {
  return new (P) DFGScatterNode(P, this, var, var->getType());
}

// DFGSinkNode
// LiveOut -> (this)
// LiveOut ->
// ...     ->

DFGSinkNode::DFGSinkNode(DataflowGraph *P)
  : DFGNode(P, DFGSinkClass), Preds(P->getBumpVectorContext(), 16),
    LiveOutVariableIndices(P->getBumpVectorContext(), 16),
    LiveOutVariableDecls(P->getBumpVectorContext(), 16),
    LiveOutConditionIndex(-1) {
}

unsigned DFGSinkNode::getNumLiveOuts() const {
  return getNumLiveOutVariables() + (hasLiveOutCondition() ? 1 : 0);
}

DFGNode *DFGSinkNode::getLiveOut(unsigned Index) const {
  assert(Index < getNumLiveOuts());
  if (Index < LiveOutVariableIndices.size()) {
    return getLiveOutVariable(Index);
  } else {
    return getLiveOutCondition();
  }
}

IndexedVarDecl *DFGSinkNode::getLiveOutDecl(unsigned Index) const {
  assert(Index < getNumLiveOuts());
  if (Index < LiveOutVariableIndices.size()) {
    return getLiveOutVariableDecl(Index);
  } else {
    assert(hasLiveOutCondition());
    return NULL;
  }
}

DFGNode *DFGSinkNode::getLiveOutVariable(unsigned Index) const {
  assert(Index < LiveOutVariableIndices.size());
  unsigned LiveOutVariableIndex = LiveOutVariableIndices[Index];
  assert(LiveOutVariableIndex < Preds.size());
  return Preds[LiveOutVariableIndex];
}

IndexedVarDecl *DFGSinkNode::getLiveOutVariableDecl(unsigned Index) const {
  assert(Index < LiveOutVariableDecls.size());
  return LiveOutVariableDecls[Index];
}

DFGNode *DFGSinkNode::getLiveOutCondition() const {
  assert(hasLiveOutCondition());
  assert(LiveOutConditionIndex < Preds.size());
  return Preds[LiveOutConditionIndex];
}

void DFGSinkNode::addPredecessor(DFGNode *pred, DataflowGraph *P) {
  assert(pred != NULL);
  Preds.push_back(pred, P->getBumpVectorContext());
  pred->addSuccessor(this, P);
}

void DFGSinkNode::addLiveOutVariable(IndexedVarDecl *var, DFGNode *pred,
                                     DataflowGraph *P) {
  LiveOutVariableIndices.push_back(Preds.size(), P->getBumpVectorContext());
  assert(var != NULL);
  LiveOutVariableDecls.push_back(var, P->getBumpVectorContext());
  addPredecessor(pred, P);
}

void DFGSinkNode::addLiveOutCondition(DFGNode *pred, DataflowGraph *P) {
  assert(LiveOutConditionIndex == (unsigned)-1);
  LiveOutConditionIndex = Preds.size();
  addPredecessor(pred, P);
}

// DFGConstNode
// Source -> (this)

DFGConstNode::DFGConstNode(DataflowGraph *P, DFGNodeClass C, QualType Ty)
  : DFGNode(P, C, Ty), Source(P->getSource()) {
  assert(Source != NULL);
  Source->addSuccessor(this, P);
}

uint64_t DFGConstNode::getZExtValue() const {
  switch (getClass()) {
    case DFGNode::DFGIntConstClass:
      return static_cast<const DFGIntConstNode*>(this)->getZExtValue();
    case DFGNode::DFGFloatConstClass:
      return static_cast<const DFGFloatConstNode*>(this)->getZExtValue();
    case DFGNode::DFGUndefinedConstClass:
      return static_cast<const DFGUndefinedConstNode*>(this)->getZExtValue();
    case DFGNode::DFGPlatformConstClass:
      return static_cast<const DFGPlatformConstNode*>(this)->getZExtValue();
    case DFGNode::DFGAddrOfClass:
      return static_cast<const DFGAddrOfNode*>(this)->getZExtValue();
    default:
      llvm_unreachable("invalid node class");
  }
}

// DFGIntConstNode

DFGIntConstNode::DFGIntConstNode(DataflowGraph *P, const llvm::APInt &value,
                                 QualType Ty)
  : DFGConstNode(P, DFGIntConstClass, Ty), Value(value) {
}

// DFGFloatConstNode

DFGFloatConstNode::DFGFloatConstNode(DataflowGraph *P,
                                     const llvm::APFloat &value, QualType Ty)
  : DFGConstNode(P, DFGFloatConstClass, Ty), Value(value) {
}

// DFGUndefinedConstNode

DFGUndefinedConstNode::DFGUndefinedConstNode(DataflowGraph *P, QualType Ty)
  : DFGConstNode(P, DFGUndefinedConstClass, Ty) {
}

// DFGResolvableConstNode

DFGResolvableConstNode::DFGResolvableConstNode(DataflowGraph *P, DFGNodeClass C,
                                               QualType Ty)
  : DFGConstNode(P, C, Ty), IsResolved(false), Value(0) {
}

uint64_t DFGResolvableConstNode::getZExtValue() const {
  assert(IsResolved && "read the value of an unresolved node");
  return Value;
}

void DFGResolvableConstNode::Resolve(uint64_t V) {
  assert(!IsResolved);
  IsResolved = true;
  Value = V;
}

// DFGPlatformConstNode

DFGPlatformConstNode::DFGPlatformConstNode(DataflowGraph *P, StringRef name,
                                           QualType Ty)
  : DFGResolvableConstNode(P, DFGPlatformConstClass, Ty) {
  assert(name.size() < sizeof(Name));
  std::strcpy(Name, name.data());
}

// DFGAddrOfNode

DFGAddrOfNode::DFGAddrOfNode(DataflowGraph *P, WVarDecl *V, QualType Ty)
  : DFGResolvableConstNode(P, DFGAddrOfClass, Ty), Var(V), AddressSpace(0) {
}

// DFGUnaryOpNode
// Operand -> (this)

DFGUnaryOpNode::DFGUnaryOpNode(DataflowGraph *P, OperatorKind op,
                               DFGNode *operand, QualType Ty)
  : DFGNode(P, DFGUnaryOpClass, Ty), Opcode(op), Operand(operand) {
  assert(Operand != NULL);
  Operand->addSuccessor(this, P);
}

// DFGBinaryOpNode
// Operands[0] -> (this)
// Operands[1] ->

DFGBinaryOpNode::DFGBinaryOpNode(DataflowGraph *P, OperatorKind op,
                                 DFGNode *lhs, DFGNode *rhs, QualType Ty)
  : DFGNode(P, DFGBinaryOpClass, Ty), Opcode(op) {
  Operands[0] = lhs;
  assert(Operands[0] != NULL);
  Operands[0]->addSuccessor(this, P);

  Operands[1] = rhs;
  assert(Operands[1] != NULL);
  Operands[1]->addSuccessor(this, P);
}

// DFGTernaryOpNode
// Operands[0] -> (this)
// Operands[1] ->
// Operands[2] ->

DFGTernaryOpNode::DFGTernaryOpNode(DataflowGraph *P, OperatorKind op,
                                   DFGNode *operand0, DFGNode *operand1,
                                   DFGNode *operand2, QualType Ty)
  : DFGNode(P, DFGTernaryOpClass, Ty), Opcode(op) {
  Operands[0] = operand0;
  assert(Operands[0] != NULL);
  Operands[0]->addSuccessor(this, P);

  Operands[1] = operand1;
  assert(Operands[1] != NULL);
  Operands[1]->addSuccessor(this, P);

  Operands[2] = operand2;
  assert(Operands[2] != NULL);
  Operands[2]->addSuccessor(this, P);
}

// DFGRangeOpNode
// Operand -> (this)

DFGRangeOpNode::DFGRangeOpNode(DataflowGraph *P, DFGNode *operand,
                               unsigned offset, unsigned width, QualType Ty)
  : DFGNode(P, DFGRangeOpClass, Ty), Operand(operand), Offset(offset),
    Width(width) {
  assert(Offset < Offset + Width);
  assert(Operand != NULL);
  Operand->addSuccessor(this, P);
}

// DFGVariableRangeOpNode
// Value  -> (this)
// Offset ->

DFGVariableRangeOpNode::DFGVariableRangeOpNode(DataflowGraph *P, DFGNode *value,
                                               DFGNode *offset, unsigned width,
                                               QualType Ty)
  : DFGNode(P, DFGVariableRangeOpClass, Ty), Width(width) {
  assert(Width > 0);

  Operands[VALUE] = value;
  assert(Operands[VALUE] != NULL);
  Operands[VALUE]->addSuccessor(this, P);

  Operands[OFFSET] = offset;
  assert(Operands[OFFSET] != NULL);
  Operands[OFFSET]->addSuccessor(this, P);
}

// DFGConcatOpNode
// Operands[0]             -> (this)
// ...                     ->
// Operands[NumOperands-1] ->

DFGConcatOpNode::DFGConcatOpNode(DataflowGraph *P, ArrayRef<DFGNode*> operands,
                                 QualType Ty)
  : DFGNode(P, DFGConcatOpClass, Ty),
    Operands(P->getBumpVectorContext(), operands.size()) {
  for (unsigned Index = 0, NumOperands = operands.size();
       Index != NumOperands; ++Index) {
    Operands.push_back(operands[Index], P->getBumpVectorContext());
    assert(Operands[Index] != NULL);
    Operands[Index]->addSuccessor(this, P);
  }
}

// DFGSubstituteOpNode
// Value    -> (this)
// Offset   ->
// NewValue ->

DFGSubstituteOpNode::DFGSubstituteOpNode(DataflowGraph *P, DFGNode *value,
                                         DFGNode *offset, unsigned width,
                                         DFGNode *new_value, QualType Ty)
  : DFGNode(P, DFGSubstituteOpClass, Ty), Width(width) {
  assert(Width > 0);

  Operands[VALUE] = value;
  assert(Operands[VALUE] != NULL);
  Operands[VALUE]->addSuccessor(this, P);

  Operands[OFFSET] = offset;
  assert(Operands[OFFSET] != NULL);
  Operands[OFFSET]->addSuccessor(this, P);

  Operands[NEW_VALUE] = new_value;
  assert(Operands[NEW_VALUE] != NULL);
  Operands[NEW_VALUE]->addSuccessor(this, P);
}

// DFGMemoryAccessNode
// (Ordered) -> (this)
// Addr      ->
// ...       ->

DFGMemoryAccessNode::DFGMemoryAccessNode(DataflowGraph *P, DFGNodeClass C,
                                         unsigned NPreds, DFGNode *A,
                                         QualType AccessTy, QualType ResultTy)
  : DFGNode(P, C, ResultTy), NumPreds(NPreds), HasAdditionalPred(false),
    AccessType(AccessTy), AddressSpace(0), AliasGroup(0) {
  assert(NumPreds >= END_COMMON_PREDS && NumPreds <= MAX_NUM_PREDS);
  Preds[ORDERED] = NULL;
  Preds[ADDR] = A;
  assert(Preds[ADDR] != NULL);
  Preds[ADDR]->addSuccessor(this, P);
}

DFGNode *DFGMemoryAccessNode::getAdditionalPredecessor() const {
  assert(HasAdditionalPred);
  return Preds[NumPreds - 1];
}

bool DFGMemoryAccessNode::isLoadKind() const {
  switch (getClass()) {
    case DFGNode::DFGLoadClass:
    case DFGNode::DFGNullaryAtomicClass:
    case DFGNode::DFGUnaryAtomicClass:
    case DFGNode::DFGBinaryAtomicClass:
      return true;
    default:
      return false;
  }
}

bool DFGMemoryAccessNode::isStoreKind() const {
  switch (getClass()) {
    case DFGNode::DFGStoreClass:
    case DFGNode::DFGNullaryAtomicClass:
    case DFGNode::DFGUnaryAtomicClass:
    case DFGNode::DFGBinaryAtomicClass:
      return true;
    default:
      return false;
  }
}

void DFGMemoryAccessNode::setOrderedPredecessor(DataflowGraph *P,
                                                DFGNode *pred) {
  assert(Preds[ORDERED] == NULL);
  Preds[ORDERED] = pred;
  assert(Preds[ORDERED] != NULL);
  Preds[ORDERED]->addSuccessor(this, P);
}

void DFGMemoryAccessNode::setAdditionalPredecessor(DataflowGraph *P,
                                                   DFGNode *pred) {
  assert(NumPreds + 1 <= MAX_NUM_PREDS && !HasAdditionalPred);
  Preds[NumPreds] = pred;
  assert(Preds[NumPreds] != NULL);
  Preds[NumPreds]->addSuccessor(this, P);
  NumPreds++;
  HasAdditionalPred = true;
}

DFGNode::pred_range DFGMemoryAccessNode::predecessors() {
  return Preds[ORDERED] ? pred_range(Preds, Preds + NumPreds)
                        : pred_range(Preds + 1, Preds + NumPreds);
}

// DFGLoadNode

DFGLoadNode::DFGLoadNode(DataflowGraph *P, DFGNode *A, QualType Ty)
  : DFGMemoryAccessNode(P, DFGLoadClass, END_PREDS, A, Ty, Ty) {
}

// DFGStoreNode

DFGStoreNode::DFGStoreNode(DataflowGraph *P, DFGNode *A, DFGNode *value,
                           QualType Ty)
  : DFGMemoryAccessNode(P, DFGStoreClass, END_PREDS, A, Ty, QualType()) {
  Preds[VALUE] = value;
  assert(Preds[VALUE] != NULL);
  Preds[VALUE]->addSuccessor(this, P);
}

// DFGAtomicNode

DFGAtomicNode::DFGAtomicNode(DataflowGraph *P, DFGNodeClass C, unsigned NPreds,
                             DFGNode *A, QualType Ty)
  : DFGMemoryAccessNode(P, C, NPreds, A, Ty, Ty) {
}

// DFGNullaryAtomicNode

DFGNullaryAtomicNode::DFGNullaryAtomicNode(DataflowGraph *P, OperatorKind op,
                                           DFGNode *A, QualType Ty)
  : DFGAtomicNode(P, DFGNullaryAtomicClass, END_PREDS, A, Ty), Opcode(op) {
}

// DFGUnaryAtomicNode

DFGUnaryAtomicNode::DFGUnaryAtomicNode(DataflowGraph *P, OperatorKind op,
                                       DFGNode *A, DFGNode *operand,
                                       QualType Ty)
  : DFGAtomicNode(P, DFGUnaryAtomicClass, END_PREDS, A, Ty), Opcode(op) {
  Preds[OPERAND] = operand;
  assert(Preds[OPERAND] != NULL);
  Preds[OPERAND]->addSuccessor(this, P);
}

// DFGBinaryAtomicNode

DFGBinaryAtomicNode::DFGBinaryAtomicNode(DataflowGraph *P, OperatorKind op,
                                         DFGNode *A, DFGNode *lhs, DFGNode *rhs,
                                         QualType Ty)
  : DFGAtomicNode(P, DFGBinaryAtomicClass, END_PREDS, A, Ty), Opcode(op) {
  Preds[LHS] = lhs;
  assert(Preds[LHS] != NULL);
  Preds[LHS]->addSuccessor(this, P);

  Preds[RHS] = rhs;
  assert(Preds[RHS] != NULL);
  Preds[RHS]->addSuccessor(this, P);
}

// DFGFunctionCallNode
// Args[0]         -> (this)
// ...             ->
// Args[NumArgs-1] ->

DFGFunctionCallNode::DFGFunctionCallNode(DataflowGraph *P, FunctionDecl *func,
                                         ArrayRef<DFGNode*> args, QualType Ty)
  : DFGNode(P, DFGFunctionCallClass, Ty), Func(func),
    Args(P->getBumpVectorContext(), args.size()) {
  for (unsigned Index = 0, NumArgs = args.size();
       Index != NumArgs; ++Index) {
    Args.push_back(args[Index], P->getBumpVectorContext());
    assert(Args[Index] != NULL);
    Args[Index]->addSuccessor(this, P);
  }
}

// DFGShiftRegisterNode
// Pred -> (this)

DFGShiftRegisterNode::DFGShiftRegisterNode(DataflowGraph *P, DFGNode *pred,
                                           unsigned size)
  : DFGNode(P, DFGShiftRegisterClass, pred->getType()), Pred(pred), Size(size) {
  assert(Pred != NULL);
  Pred->addSuccessor(this, P);
}

// DFGQueueNode
// Pred -> (this)

DFGQueueNode::DFGQueueNode(DataflowGraph *P, DFGNode *pred, unsigned size)
  : DFGNode(P, DFGQueueClass, pred->getType()), Pred(pred), Size(size) {
  assert(Pred != NULL);
  Pred->addSuccessor(this, P);
}

// DFGBarrierNode
// Pred -> (this) -> Scatter ->
// Pred ->        -> Scatter ->
// Pred ->        -> Scatter ->
// ...               ...

DFGBarrierNode::DFGBarrierNode(DataflowGraph *P)
  : DFGNode(P, DFGBarrierClass), Preds(P->getBumpVectorContext(), 16) {
}

DFGNode *DFGBarrierNode::getPredOf(DFGScatterNode *Succ) const {
  unsigned Key = Succ->getKey<unsigned>();
  assert(Key < Preds.size());
  return Preds[Key];
}

void DFGBarrierNode::addPredecessor(DFGNode *pred, DataflowGraph *P) {
  assert(pred != NULL);
  Preds.push_back(pred, P->getBumpVectorContext());
  pred->addSuccessor(this, P);
}

DFGScatterNode *DFGBarrierNode::addLiveVariable(DFGNode *pred,
                                                DataflowGraph *P) {
  unsigned Index = Preds.size();
  DFGScatterNode *Out = new (P) DFGScatterNode(P, this, Index, pred->getType());
  addPredecessor(pred, P);
  return Out;
}

// DFGCompoundNode

DFGCompoundNode::DFGCompoundNode(DataflowGraph *P, const NodeSetTy &N)
  : DFGNode(P, DFGCompoundClass, true), Nodes(N),
    Preds(P->getBumpVectorContext(), 64) {
  ReloadNeighbors(P);
}

void DFGCompoundNode::ReloadNeighbors(DataflowGraph *P) {
  Preds.clear();
  removeAllSuccessors();
  for (NodeSetTy::const_iterator N = Nodes.begin(), NEnd = Nodes.end();
       N != NEnd; ++N) {
    DFGNode *Node = *N;
    for (DFGNode::pred_iterator PI = Node->pred_begin(), PE = Node->pred_end();
         PI != PE; ++PI) {
      if (!Nodes.count(*PI)) {
        Preds.push_back(*PI, P->getBumpVectorContext());
      }
    }
    for (DFGNode::succ_iterator SI = Node->succ_begin(), SE = Node->succ_end();
         SI != SE; ++SI) {
      if (!Nodes.count(*SI)) {
        addSuccessor(*SI, P);
      }
    }
  }
}

// DFGBuilder

namespace {

class LValue {
  IndexedVarDeclRef SSAVars;
  WVarDecl *MemVar;
  DFGNode *MemAddr;
  QualType ValueType;
  unsigned ValueSpace;

public:
  LValue()
    : SSAVars(), MemVar(NULL), MemAddr(NULL), ValueType(), ValueSpace(0) {}
  explicit LValue(IndexedVarDecl *V)
    : SSAVars(V), MemVar(NULL), MemAddr(NULL), ValueType(V->getType()),
      ValueSpace(0) {}
  LValue(IndexedVarDeclRef VRef, QualType Ty)
    : SSAVars(VRef), MemVar(NULL), MemAddr(NULL), ValueType(Ty),
      ValueSpace(0) {}
  explicit LValue(WVarDecl *V)
    : SSAVars(), MemVar(V), MemAddr(NULL), ValueType(V->getType()),
      ValueSpace(V->getType().getAddressSpace()) {}
  LValue(DFGNode *A, QualType Ty, unsigned S)
    : SSAVars(), MemVar(NULL), MemAddr(A), ValueType(Ty), ValueSpace(S) {}
  LValue(const LValue &LV)
    : SSAVars(LV.SSAVars), MemVar(LV.MemVar), MemAddr(LV.MemAddr),
      ValueType(LV.ValueType), ValueSpace(LV.ValueSpace) {}

  bool isValid() const { return SSAVars || MemVar || MemAddr; }
  IndexedVarDeclRef getSSAVars() const { return SSAVars; }
  WVarDecl *getMemVar() const { return MemVar; }
  DFGNode *getMemAddr() const { return MemAddr; }

  QualType getType() const { return ValueType; }
  unsigned getAddressSpace() const { return ValueSpace; }
};

class ExprValue {
  SmallVector<DFGNode*, 16> RV;
  LValue LV;
  DFGNode *MOffset;
  unsigned MWidth;
  QualType MType;

public:
  ExprValue() : RV(), LV(), MOffset(NULL), MWidth(0), MType() {}

  explicit ExprValue(DFGNode *rv)
    : RV(1, rv), LV(), MOffset(NULL), MWidth(0), MType() {}
  explicit ExprValue(ArrayRef<DFGNode*> rv)
    : RV(rv.begin(), rv.end()), LV(), MOffset(NULL), MWidth(0), MType() {}
  explicit ExprValue(LValue lv)
    : RV(), LV(lv), MOffset(NULL), MWidth(0), MType() {}
  ExprValue(DFGNode *rv, LValue lv)
    : RV(1, rv), LV(lv), MOffset(NULL), MWidth(0), MType() {}
  ExprValue(ArrayRef<DFGNode*> rv, LValue lv)
    : RV(rv.begin(), rv.end()), LV(lv), MOffset(NULL), MWidth(0), MType() {}
  ExprValue(const ExprValue &base, DFGNode *moffset, unsigned mwidth,
            QualType mtype)
    : RV(base.RV), LV(base.LV), MOffset(moffset), MWidth(mwidth), MType(mtype) {}

  ExprValue(const ExprValue &EV)
    : RV(EV.RV), LV(EV.LV), MOffset(EV.MOffset), MWidth(EV.MWidth),
      MType(EV.MType) {}

  bool hasRValue() const { return !RV.empty(); }
  unsigned size() const { return RV.size(); }
  DFGNode *operator*() const {
    assert(RV.size() == 1);
    return RV[0];
  }
  DFGNode *operator[](unsigned Index) const {
    assert(Index < RV.size());
    return RV[Index];
  }
  ArrayRef<DFGNode*> getRValueRef() const {
    return RV;
  }

  bool hasLValue() const { return LV.isValid(); }
  LValue getLValue() const { return LV; }

  bool hasMask() const { return MOffset != NULL; }
  DFGNode *getMaskOffset() const { return MOffset; }
  unsigned getMaskWidth() const { return MWidth; }
  QualType getMaskType() const { return MType; }

  ExprValue filterRValue() const {
    return ExprValue(RV);
  }
};

class DFGSpecificLiveness : public AdditionalLiveness {
  const VirtualVariablePool &VVars;
  const SnuCLOptions &SnuCLOpts;
  const AliasSet &Aliases;
  llvm::BumpPtrAllocator Allocator;

public:
  DFGSpecificLiveness(const VirtualVariablePool &VV, const SnuCLOptions &Opts,
                      const AliasSet &AS)
    : VVars(VV), SnuCLOpts(Opts), Aliases(AS), Allocator() {}

  IndexedVarDeclRef handleStmt(WStmt *S, bool &Halt);
  IndexedVarDeclRef handleBlock(WCFGBlock *B);

private:
  IndexedVarDeclRef handleCallExpr(WCallExpr *CE, bool &Halt);
  IndexedVarDeclRef handleWorkItemFunction(WWorkItemFunction *WIF, bool &Halt);
};

IndexedVarDeclRef DFGSpecificLiveness::handleStmt(WStmt *S, bool &Halt) {
  // The flat work-group ID goes to all basic blocks by default.
#if 0
  if (isa<WExpr>(S) && Aliases.isMemoryAccess(static_cast<WExpr*>(S))) {
    return VVars.getIndexedFlatWorkGroupID();
  }
#endif
  if (WCallExpr *CE = dyn_cast<WCallExpr>(S)) {
    return handleCallExpr(CE, Halt);
  }
  if (WWorkItemFunction *WIF = dyn_cast<WWorkItemFunction>(S)) {
    return handleWorkItemFunction(WIF, Halt);
  }
  return IndexedVarDeclRef();
}

IndexedVarDeclRef DFGSpecificLiveness::handleBlock(WCFGBlock *B) {
  IndexedVarDecl *LiveVars[2] = {VVars.getIndexedFlatLocalID(),
                                 VVars.getIndexedFlatWorkGroupID()};
  return IndexedVarDeclRef(Allocator, LiveVars);
}

IndexedVarDeclRef DFGSpecificLiveness::handleCallExpr(WCallExpr *CE,
                                                      bool &Halt) {
  return IndexedVarDeclRef();
}

IndexedVarDeclRef DFGSpecificLiveness::handleWorkItemFunction(
    WWorkItemFunction *WIF, bool &Halt) {
  switch (WIF->getFunctionKind()) {
    case WWorkItemFunction::WIF_get_global_size:
      Halt = true;
      return VVars.getIndexedGlobalSize(WIF->getArg());
    case WWorkItemFunction::WIF_get_local_size:
      Halt = true;
      return VVars.getIndexedLocalSize(WIF->getArg());
    case WWorkItemFunction::WIF_get_num_groups:
      Halt = true;
      return VVars.getIndexedNumWorkGroups(WIF->getArg());
    case WWorkItemFunction::WIF_get_global_id:
      Halt = true;
      return VVars.getIndexedGlobalID(WIF->getArg());
    case WWorkItemFunction::WIF_get_local_id:
      Halt = true;
      return VVars.getIndexedLocalID(WIF->getArg());
    case WWorkItemFunction::WIF_get_group_id:
      Halt = true;
      return VVars.getIndexedWorkGroupID(WIF->getArg());
    default:
      llvm_unreachable("impossible");
      return IndexedVarDeclRef();
  }
}

class DFGBuilder : public WCFGBlockVisitor<DFGBuilder, ExprValue> {
  const ASTContext &ASTCtx;
  const SnuCLOptions &SnuCLOpts;
  const AliasSet &Aliases;
  const VirtualVariablePool &VVars;

  DataflowGraph *Graph;
  DFGSourceNode *Source;
  DFGSinkNode *Sink;

  typedef OrderedDenseMap<IndexedVarDecl*, DFGNode*> VarValueMapTy;
  VarValueMapTy VarValueMap;
  llvm::DenseMap<WExpr*, ExprValue> TopLevelExprValueMap;

  typedef std::map<unsigned, DFGNode*> MemoryAccessMapTy;
  MemoryAccessMapTy GlobalAccessMap;
  MemoryAccessMapTy GlobalStoreMap;
  MemoryAccessMapTy LocalAccessMap;
  MemoryAccessMapTy LocalStoreMap;

  static LiveSSAVariables *LiveVars;

public:
  DFGBuilder(const ASTContext &Ctx, const SnuCLOptions &Opts,
             const AliasSet &AS, const VirtualVariablePool &VV)
    : ASTCtx(Ctx), SnuCLOpts(Opts), Aliases(AS), VVars(VV), Graph(NULL),
      Source(NULL), Sink(NULL) {}

  DataflowGraph *build(WCFGBlock *EntryBlock);

private:
  void VisitLiveIns(WCFGBlock *Block);
  void VisitLiveOuts(WCFGBlock *Block);
  WCFGBlock *VisitBlock(WCFGBlock *Block);
  WCFGBlock *VisitLogicalOpSuccessorBlock(WCFGBlock *Block,
                                          WCFGBlock *GTrueBlock,
                                          WCFGBlock *GFalseBlock);
  void VisitStmtsInBlock(WCFGBlock *Block);

  bool HasRValueOf(IndexedVarDecl *Var);
  DFGNode *GetRValueOf(IndexedVarDecl *Var);
  void SetRValueOf(IndexedVarDecl *Var, DFGNode *Value);
  ExprValue GetExprValueOf(WExpr *E);
  void SetExprValueOf(WExpr *E, ExprValue Value);

  ExprValue FinalizeToRValue(ExprValue V, WExpr *Origin);
  DFGNode *GetSingleValueOf(ExprValue V, WExpr *Origin);
  DFGNode *GetOrMakeSingleValueOf(ExprValue V, WExpr *Origin);
  LValue FinalizeToAddressAccess(ExprValue V);
  LValue FinalizeToAddressAccess(LValue V);

  uint64_t GetAddStep(QualType Ty);
  QualType GetAddressType(QualType Ty);
  unsigned GetPointeeAddressSpace(QualType Ty);
  unsigned GetSynthesizableAddressSpace(unsigned AS, WExpr *Origin);

  DFGIntConstNode *CreateIntConst(uint64_t Value, QualType Ty);
  DFGNode *CreateCastedInt(DFGNode *Node, QualType TargetTy);
  DFGNode *CreateMultipliedInt(DFGNode *Node, uint64_t MultiplyBy);
  DFGLoadNode *CreateLoad(LValue V, WExpr *Origin);
  DFGStoreNode *CreateStore(LValue V, DFGNode *Value, WExpr *Origin);

  RecordDecl *GetStructDefinition(QualType Ty);
  uint64_t GetStructFieldOffset(FieldDecl *Field, unsigned FieldIndex = -1);
  QualType GetVectorElementType(QualType Ty);
  unsigned GetVectorNumElements(QualType Ty);

  LValue GetFieldOf(DFGNode *BaseAddr, FieldDecl *Field, QualType FieldTy,
                    unsigned FieldSpace, unsigned FieldIndex = -1);
  ExprValue GetElementOf(ExprValue Base, DFGNode *Idx, QualType ElementTy);
  ExprValue GetElementOf(ExprValue Base, unsigned Idx, QualType ElementTy);
  LValue GetElementOf(DFGNode *BaseAddr, DFGNode *Idx, QualType ElementTy,
                      unsigned ElementSpace);
  LValue GetElementOf(DFGNode *BaseAddr, unsigned Idx, QualType ElementTy,
                      unsigned ElementSpace);
  ExprValue SplitRValue(ExprValue V);
  void SplitStructure(LValue V, SmallVectorImpl<LValue> &Fields);
  void SplitStructure(DFGNode *V, SmallVectorImpl<DFGNode*> &Elements);
  void SplitVector(LValue V, SmallVectorImpl<LValue> &Elements);
  void SplitVector(DFGNode *V, SmallVectorImpl<DFGNode*> &Elements);

  DFGBinaryOpNode::OperatorKind GetBinaryOpcode(BinaryOperator::Opcode Op,
                                                bool Vector = false);
  DFGUnaryOpNode::OperatorKind GetUnaryOpcodeForBuiltin(
      WCallExpr::BuiltinFunctionKind Func);
  DFGBinaryOpNode::OperatorKind GetBinaryOpcodeForBuiltin(
      WCallExpr::BuiltinFunctionKind Func);
  DFGNullaryAtomicNode::OperatorKind GetNullaryAtomicOpcodeForBuiltin(
      WCallExpr::BuiltinFunctionKind Func);
  DFGUnaryAtomicNode::OperatorKind GetUnaryAtomicOpcodeForBuiltin(
      WCallExpr::BuiltinFunctionKind Func);
  DFGBinaryAtomicNode::OperatorKind GetBinaryAtomicOpcodeForBuiltin(
      WCallExpr::BuiltinFunctionKind Func);

  void OrderMemoryAccess(MemoryAccessMapTy &AccessMap,
                         MemoryAccessMapTy &StoreMap,
                         DFGMemoryAccessNode *Access);
  void OrderMemoryAccesses(MemoryAccessMapTy &AccessMap, DFGSinkNode *Sink);
  void OrderMemoryAccesses(MemoryAccessMapTy &AccessMap,
                           DFGBarrierNode *Barrier);

  ExprValue VisitAssignment(ExprValue Target, ExprValue Value, WExpr *OriginLHS,
                            WExpr *OriginRHS);
  ExprValue VisitAssignment(ExprValue Target, DFGNode *Value, WExpr *OriginLHS,
                            WExpr *OriginRHS);
  ExprValue VisitAssignment(LValue Target, ExprValue Value, WExpr *OriginLHS,
                            WExpr *OriginRHS);
  ExprValue VisitAssignment(LValue Target, DFGNode *Value, WExpr *OriginLHS,
                            WExpr *OriginRHS);

  ExprValue VisitIndexedVarDeclRef(IndexedVarDeclRef UseDecl,
                                   IndexedVarDeclRef DefDecl, QualType Ty);

  ExprValue VisitDeclStmt(WDeclStmt *Node);
  ExprValue VisitReturnStmt(WReturnStmt *Node);
  ExprValue VisitDeclRefExpr(WDeclRefExpr *Node);
  ExprValue VisitPredefinedExpr(WPredefinedExpr *Node);
  ExprValue VisitIntegerLiteral(WIntegerLiteral *Node);
  ExprValue VisitCharacterLiteral(WCharacterLiteral *Node);
  ExprValue VisitFloatingLiteral(WFloatingLiteral *Node);
  ExprValue VisitStringLiteral(WStringLiteral *Node);
  ExprValue VisitParenExpr(WParenExpr *Node);
  ExprValue VisitUnaryOperator(WUnaryOperator *Node);
  ExprValue VisitUnaryExprOrTypeTraitExpr(WUnaryExprOrTypeTraitExpr *Node);
  ExprValue VisitArraySubscriptExpr(WArraySubscriptExpr *Node);
  ExprValue VisitCallExpr(WCallExpr *Node);
  ExprValue VisitVirtualCallExpr(WCallExpr *Node);
  ExprValue VisitMemberExpr(WMemberExpr *Node);
  ExprValue VisitCompoundLiteralExpr(WCompoundLiteralExpr *Node);
  ExprValue VisitCastExprInternal(WCastExpr *Node);
  ExprValue VisitImplicitCastExpr(WImplicitCastExpr *Node);
  ExprValue VisitCStyleCastExpr(WCStyleCastExpr *Node);
  ExprValue VisitBinaryOperator(WBinaryOperator *Node);
  ExprValue VisitVirtualBinaryOperator(WBinaryOperator *Node);
  ExprValue VisitCompoundAssignOperator(WCompoundAssignOperator *Node);
  ExprValue VisitVirtualConditionalOperator(WConditionalOperator *Node);
  ExprValue VisitInitListExpr(WInitListExpr *Node);
  ExprValue VisitDesignatedInitExpr(WDesignatedInitExpr *Node);
  ExprValue VisitImplicitValueInitExpr(WImplicitValueInitExpr *Node);
  ExprValue VisitParenListExpr(WParenListExpr *Node);
  ExprValue VisitExtVectorElementExpr(WExtVectorElementExpr *Node);
  ExprValue VisitCXXBoolLiteralExpr(WCXXBoolLiteralExpr *Node);
  ExprValue VisitBlockExpr(WBlockExpr *Node);
  ExprValue VisitAsTypeExpr(WAsTypeExpr *Node);
  ExprValue VisitPhiFunction(WPhiFunction *Node);
  ExprValue VisitWorkItemFunction(WWorkItemFunction *Node);
  ExprValue VisitBaseMuFunction(WBaseMuFunction *Node);
  ExprValue VisitAuxiliaryMuFunction(WAuxiliaryMuFunction *Node);

  void PruneUnusedNodes();

  friend class WCFGBlockVisitor;
};

LiveSSAVariables *DFGBuilder::LiveVars = NULL;

DataflowGraph *DFGBuilder::build(WCFGBlock *EntryBlock) {
  assert(Aliases.getProgram() == EntryBlock->getParent());

  // Live variable analysis
  if (LiveVars == NULL || LiveVars->getProgram() != EntryBlock->getParent()) {
    DFGSpecificLiveness DSLive(VVars, SnuCLOpts, Aliases);
    LiveVars = LiveSSAVariables::Create(EntryBlock->getParent(), &DSLive, NULL);
  }

  Graph = new DataflowGraph();
  Source = new (Graph) DFGSourceNode(Graph);
  Sink = new (Graph) DFGSinkNode(Graph);

  GlobalAccessMap.clear();
  GlobalStoreMap.clear();
  LocalAccessMap.clear();
  LocalStoreMap.clear();

  Graph->setEntryBlock(EntryBlock);
  VisitLiveIns(EntryBlock);
  WCFGBlock *ExitBlock = VisitBlock(EntryBlock);
  Graph->setExitBlock(ExitBlock);
  VisitLiveOuts(ExitBlock);

  OrderMemoryAccesses(GlobalAccessMap, Sink);
  OrderMemoryAccesses(LocalAccessMap, Sink);

  if (!Sink->isReachableFrom(Source)) {
    assert(Sink->pred_size() == 0);
    Sink->addPredecessor(Source, Graph);
  }

  Graph->Redefine();
  return Graph;
}

void DFGBuilder::VisitLiveIns(WCFGBlock *Block) {
  for (LiveSSAVariables::iterator L = LiveVars->live_in_begin(Block),
                                  LEnd = LiveVars->live_in_end(Block);
       L != LEnd; ++L) {
    IndexedVarDecl *Var = const_cast<IndexedVarDecl*>(*L);
    Graph->addLiveIn(Var);
    SetRValueOf(Var, Source->addLiveInVariable(Var, Graph));
  }
}

void DFGBuilder::VisitLiveOuts(WCFGBlock *Block) {
  if (WExpr *Cond = Block->getTerminatorCondition()) {
    DFGNode *CondValue = GetSingleValueOf(GetExprValueOf(Cond), Cond);
    Sink->addLiveOutCondition(CondValue, Graph);
  }
  for (LiveSSAVariables::iterator L = LiveVars->live_out_begin(Block),
                                  LEnd = LiveVars->live_out_end(Block);
       L != LEnd; ++L) {
    IndexedVarDecl *Var = const_cast<IndexedVarDecl*>(*L);
    Graph->addLiveOut(Var);
    Sink->addLiveOutVariable(Var, GetRValueOf(Var), Graph);
  }
}

WCFGBlock *DFGBuilder::VisitBlock(WCFGBlock *Block) {
  if (!Block)
    return NULL;

  VisitStmtsInBlock(Block);

  // Concatenate next blocks if possible
  // TODO: short-circuit evaluation
  if (Block->real_succ_size() == 1) {
    WCFGBlock *Succ = *(Block->real_succ_begin());
    if (Succ->pred_size() == 1 && !Succ->hasBarrier()) {
      return VisitBlock(Succ);
    }
  }
  WStmt *T = Block->getTerminator();
  if (WBinaryOperator *BO = dyn_cast_or_null<WBinaryOperator>(T)) {
    assert(BO->isLogicalOp());
    assert(Block->real_succ_size() == 2);
    WCFGBlock *TrueBlock = *(Block->real_succ_begin() + 0);
    WCFGBlock *FalseBlock = *(Block->real_succ_begin() + 1);
    if (BO->getOpcode() == BO_LAnd) {
      return VisitLogicalOpSuccessorBlock(TrueBlock, NULL, FalseBlock);
    } else { // BO->getOpcode() == BO_LOr
      return VisitLogicalOpSuccessorBlock(FalseBlock, TrueBlock, NULL);
    }
  }
  if (dyn_cast_or_null<WConditionalOperator>(T)) {
    assert(Block->real_succ_size() == 2);
    WCFGBlock *TrueBlock = *(Block->real_succ_begin() + 0);
    WCFGBlock *FalseBlock = *(Block->real_succ_begin() + 1);
    TrueBlock = VisitBlock(TrueBlock);
    FalseBlock = VisitBlock(FalseBlock);
    assert(TrueBlock->real_succ_size() == 1 &&
           FalseBlock->real_succ_size() == 1 &&
           *(TrueBlock->real_succ_begin()) == *(FalseBlock->real_succ_begin()));
    return VisitBlock(*(TrueBlock->real_succ_begin()));
  }

  return Block;
}

WCFGBlock *DFGBuilder::VisitLogicalOpSuccessorBlock(WCFGBlock *Block,
                                                    WCFGBlock *GTrueBlock,
                                                    WCFGBlock *GFalseBlock) {
  VisitStmtsInBlock(Block);

  WStmt *T = Block->getTerminator();
  if (WBinaryOperator *BO = dyn_cast_or_null<WBinaryOperator>(T)) {
    assert(BO->isLogicalOp());
    assert(Block->real_succ_size() == 2);
    WCFGBlock *TrueBlock = *(Block->real_succ_begin() + 0);
    WCFGBlock *FalseBlock = *(Block->real_succ_begin() + 1);
    if (BO->getOpcode() == BO_LAnd) {
      assert(GFalseBlock == NULL || GFalseBlock == FalseBlock);
      if (GTrueBlock == TrueBlock) {
        return VisitLogicalOpSuccessorBlock(TrueBlock, NULL, FalseBlock);
      } else {
        return VisitLogicalOpSuccessorBlock(TrueBlock, GTrueBlock, FalseBlock);
      }
    } else { // BO->getOpcode() == BO_LOr
      assert(GTrueBlock == NULL || GTrueBlock == TrueBlock);
      if (GFalseBlock == FalseBlock) {
        return VisitLogicalOpSuccessorBlock(FalseBlock, TrueBlock, NULL);
      } else {
        return VisitLogicalOpSuccessorBlock(FalseBlock, TrueBlock, GFalseBlock);
      }
    }
  } else if (dyn_cast_or_null<WConditionalOperator>(T)) {
    assert(Block->real_succ_size() == 2);
    WCFGBlock *TrueBlock = *(Block->real_succ_begin() + 0);
    WCFGBlock *FalseBlock = *(Block->real_succ_begin() + 1);
    TrueBlock = VisitBlock(TrueBlock);
    FalseBlock = VisitBlock(FalseBlock);
    assert(TrueBlock->real_succ_size() == 1 &&
           FalseBlock->real_succ_size() == 1 &&
           *(TrueBlock->real_succ_begin()) == *(FalseBlock->real_succ_begin()));
    return VisitLogicalOpSuccessorBlock(*(TrueBlock->real_succ_begin()),
                                        GTrueBlock, GFalseBlock);
  } else {
    if (Block->real_succ_size() == 1) {
      WCFGBlock *SuccBlock = *(Block->real_succ_begin());
      assert(GTrueBlock == NULL || GTrueBlock == SuccBlock);
      assert(GFalseBlock == NULL || GFalseBlock == SuccBlock);
      return VisitBlock(SuccBlock);
    } else if (Block->real_succ_size() == 2) {
      WCFGBlock *TrueBlock = *(Block->real_succ_begin() + 0);
      WCFGBlock *FalseBlock = *(Block->real_succ_begin() + 1);
      assert(GTrueBlock == NULL || GTrueBlock == TrueBlock);
      assert(GFalseBlock == NULL || GFalseBlock == FalseBlock);
      return Block;
    } else {
      llvm_unreachable("impossible");
    }
  }
}

void DFGBuilder::VisitStmtsInBlock(WCFGBlock *Block) {
   for (WCFGBlock::iterator S = Block->begin(), SEnd = Block->end();
       S != SEnd; ++S) {
    ExprValue EV = VisitStmt(*S, true);
    if (WExpr *E = dyn_cast<WExpr>(*S)) {
      SetExprValueOf(E, EV);
    }
  }
}

bool DFGBuilder::HasRValueOf(IndexedVarDecl *Var) {
  return Var->getIndex() == 0 || VarValueMap.count(Var);
}

DFGNode *DFGBuilder::GetRValueOf(IndexedVarDecl *Var) {
  if (Var->getIndex() == 0) {
    return new (Graph) DFGUndefinedConstNode(Graph, Var->getType());
  } else {
    assert(VarValueMap.count(Var));
    return VarValueMap[Var];
  }
}

void DFGBuilder::SetRValueOf(IndexedVarDecl *Var, DFGNode *Value) {
  assert(!VarValueMap.count(Var));
  VarValueMap[Var] = Value;
}

ExprValue DFGBuilder::GetExprValueOf(WExpr *E) {
  E = E->IgnoreParens();
  if (TopLevelExprValueMap.count(E)) {
    return TopLevelExprValueMap[E];
  } else if (WBinaryOperator *BO = dyn_cast<WBinaryOperator>(E)) {
    if (BO->isLogicalOp() || BO->getOpcode() == BO_Comma) {
      return VisitVirtualBinaryOperator(BO);
    } else {
      llvm_unreachable("invalid virtual expression");
    }
  } else if (WConditionalOperator *CO = dyn_cast<WConditionalOperator>(E)) {
    return VisitVirtualConditionalOperator(CO);
  } else {
    llvm_unreachable("invalid virtual expression");
  }
}

void DFGBuilder::SetExprValueOf(WExpr *E, ExprValue Value) {
  assert(!TopLevelExprValueMap.count(E));
  TopLevelExprValueMap[E] = Value;
}

ExprValue DFGBuilder::FinalizeToRValue(ExprValue V, WExpr *Origin) {
  if (V.hasRValue()) {
    if (V.hasMask()) {
      assert(V.hasLValue());
      QualType Ty = V.getMaskType();
      DFGNode *MaskedValue = new (Graph) DFGVariableRangeOpNode(
          Graph, *V, V.getMaskOffset(), V.getMaskWidth(), Ty);
      return SplitRValue(ExprValue(MaskedValue));
    } else {
      return V.filterRValue();
    }
  }

  assert(V.hasLValue());
  assert(!V.hasMask() && "Masked ExprValue should have a valid r-value");
  QualType Ty = V.getLValue().getType();
  if (Ty->isStructureType()) {
    uint64_t StructSize = ASTCtx.getTypeSize(Ty);
    if (StructSize <= 64 * 8 && (StructSize & (StructSize - 1)) == 0) {
      LValue LV = FinalizeToAddressAccess(V.getLValue());
      SmallVector<DFGNode*, 16> RVList;
      SplitStructure(CreateLoad(LV, Origin), RVList);
      return ExprValue(RVList);
    } else {
      SmallVector<LValue, 16> LVList;
      SplitStructure(V.getLValue(), LVList);
      SmallVector<DFGNode*, 16> RVList;
      for (unsigned Index = 0, NumValues = LVList.size();
           Index != NumValues; ++Index) {
        DFGNode *V = GetSingleValueOf(ExprValue(LVList[Index]), Origin);
        RVList.push_back(V);
      }
      return ExprValue(RVList);
    }

  } else if (Ty->isVectorType()) {
    uint64_t VectorSize = ASTCtx.getTypeSize(Ty);
    if (VectorSize <= 64 * 8) {
      LValue LV = FinalizeToAddressAccess(V.getLValue());
      SmallVector<DFGNode*, 16> RVList;
      SplitVector(CreateLoad(LV, Origin), RVList);
      return ExprValue(RVList);
    } else {
      // double16
      llvm_unreachable("not implemented yet");
    }

  } else {
    LValue LV = FinalizeToAddressAccess(V.getLValue());
    return ExprValue(CreateLoad(LV, Origin));
  }
}

DFGNode *DFGBuilder::GetSingleValueOf(ExprValue V, WExpr *Origin) {
  V = FinalizeToRValue(V, Origin);
  assert(V.size() == 1);
  return *V;
}

DFGNode *DFGBuilder::GetOrMakeSingleValueOf(ExprValue V, WExpr *Origin) {
  V = FinalizeToRValue(V, Origin);
  if (V.size() == 1) {
    return *V;
  } else {
    assert(V.size() > 1);
    return new (Graph) DFGConcatOpNode(Graph, V.getRValueRef(),
                                       Origin->getType());
  }
}

LValue DFGBuilder::FinalizeToAddressAccess(ExprValue V) {
  assert(V.hasLValue());
  assert(!V.hasMask() && "Masked ExprValue should refer an SSA variable");
  return FinalizeToAddressAccess(V.getLValue());
}

LValue DFGBuilder::FinalizeToAddressAccess(LValue V) {
  assert(V.isValid() && !V.getSSAVars());
  if (V.getMemAddr()) {
    return V;
  }
  WVarDecl *MemVar = V.getMemVar();
  assert(MemVar != NULL);
  if (WSubVarDecl *SubVar = dyn_cast<WSubVarDecl>(MemVar)) {
    WVarDecl *BaseVar = SubVar->getParent();
    LValue Base = FinalizeToAddressAccess(LValue(BaseVar));
    if (BaseVar->getType()->isStructureType()) {
      return GetFieldOf(Base.getMemAddr(), SubVar->getField().Field,
                        SubVar->getType(), Base.getAddressSpace());
    } else { // BaseVar->getType()->isVectorType()
      return GetElementOf(Base.getMemAddr(), SubVar->getField().Index,
                          SubVar->getType(), Base.getAddressSpace());
    }
  } else {
    QualType AddrTy = GetAddressType(V.getType());
    DFGAddrOfNode *Addr = new (Graph) DFGAddrOfNode(Graph, MemVar, AddrTy);
    assert(V.getAddressSpace() == MemVar->getType().getAddressSpace());
    Addr->setAddressSpace(V.getAddressSpace());
    if (V.getAddressSpace() == 0) {
      assert(HasRValueOf(VVars.getIndexedFlatLocalID()));
      uint64_t WorkItemSlice = ASTCtx.getTypeSize(MemVar->getType()) /
                                ASTCtx.getCharWidth();
      DFGNode *Base = CreateMultipliedInt(
          GetRValueOf(VVars.getIndexedFlatLocalID()), WorkItemSlice);
      DFGNode *BasePlusAddr = new (Graph) DFGBinaryOpNode(
          Graph, DFGBinaryOpNode::DBO_Add, Base, Addr, AddrTy);
      return LValue(BasePlusAddr, V.getType(), V.getAddressSpace());
    } else {
      return LValue(Addr, V.getType(), V.getAddressSpace());
    }
  }
}

uint64_t DFGBuilder::GetAddStep(QualType Ty) {
  if (Ty->isPointerType()) {
    return ASTCtx.getTypeSize(Ty->getPointeeType()) / ASTCtx.getCharWidth();
  } else {
    return 1;
  }
}

QualType DFGBuilder::GetAddressType(QualType Ty) {
  if (Ty->isArrayType()) {
    return ASTCtx.getDecayedType(Ty);
  } else {
    return ASTCtx.getPointerType(Ty);
  }
}

unsigned DFGBuilder::GetPointeeAddressSpace(QualType Ty) {
  assert(Ty->isPointerType());
  return Ty->getPointeeType().getAddressSpace();
}

unsigned DFGBuilder::GetSynthesizableAddressSpace(unsigned AS, WExpr *Origin) {
  if (AS == 0 && ASTCtx.getLangOpts().CUDA) {
    Origin = Origin->IgnoreParenCasts();
    if (Aliases.isMemoryAccess(Origin) &&
        Aliases.isPointsToParameter(Origin)) {
      return LangAS::opencl_global;
    }
  }
  switch (AS) {
    case LangAS::opencl_global:
    case LangAS::opencl_constant:
      return LangAS::opencl_global;
    case LangAS::opencl_local:
    case LangAS::cuda_shared:
    case 0: // opencl_private
      return LangAS::opencl_local;
    default:
      return AS;
  }
}

DFGIntConstNode *DFGBuilder::CreateIntConst(uint64_t Value, QualType Ty) {
  return new (Graph) DFGIntConstNode(Graph, ASTCtx.MakeIntValue(Value, Ty), Ty);
}

DFGNode *DFGBuilder::CreateCastedInt(DFGNode *Node, QualType TargetTy) {
  if (ASTCtx.getTypeSize(Node->getType()) == ASTCtx.getTypeSize(TargetTy)) {
    return Node;
  } else {
    return new (Graph) DFGUnaryOpNode(Graph, DFGUnaryOpNode::DUO_IntCast,
                                      Node, TargetTy);
  }
}

DFGNode *DFGBuilder::CreateMultipliedInt(DFGNode *Node, uint64_t MultiplyBy) {
  assert(MultiplyBy > 0);
  if (MultiplyBy == 1) {
    return Node;
  } else if ((MultiplyBy & (MultiplyBy - 1)) == 0) { // MultiplyBy is 2^k
    unsigned Log2 = 0;
    while (MultiplyBy > 1) {
      MultiplyBy >>= 1;
      Log2++;
    }
    assert(Log2 > 0);
    DFGNode *RHS = CreateIntConst(Log2, Node->getType());
    return new (Graph) DFGBinaryOpNode(Graph, DFGBinaryOpNode::DBO_Shl, Node,
                                       RHS, Node->getType());
  } else {
    DFGNode *RHS = CreateIntConst(MultiplyBy, Node->getType());
    return new (Graph) DFGBinaryOpNode(Graph, DFGBinaryOpNode::DBO_Mul, Node,
                                       RHS, Node->getType());
  }
}

DFGLoadNode *DFGBuilder::CreateLoad(LValue V, WExpr *Origin) {
  Origin = Origin->IgnoreParenCasts();
  assert(V.getMemAddr());
  DFGLoadNode *Load = new (Graph) DFGLoadNode(Graph, V.getMemAddr(),
                                              V.getType());
  unsigned AddressSpace = GetSynthesizableAddressSpace(V.getAddressSpace(),
                                                       Origin);
  Load->setAddressSpace(AddressSpace);
  assert(Aliases.isMemoryAccess(Origin));
  Load->setAliasGroup(Aliases.getAlias(Origin));
  if (AddressSpace == LangAS::opencl_global) {
    OrderMemoryAccess(GlobalAccessMap, GlobalStoreMap, Load);
  } else if (AddressSpace == LangAS::opencl_local) {
    OrderMemoryAccess(LocalAccessMap, LocalStoreMap, Load);
  }
  if (AddressSpace == LangAS::opencl_local) {
    assert(HasRValueOf(VVars.getIndexedFlatWorkGroupID()));
    DFGNode *WorkGroupID = GetRValueOf(VVars.getIndexedFlatWorkGroupID());
    Load->setAdditionalPredecessor(Graph, WorkGroupID);
  }
  return Load;
}

DFGStoreNode *DFGBuilder::CreateStore(LValue V, DFGNode *Value, WExpr *Origin) {
  Origin = Origin->IgnoreParenCasts();
  assert(V.getMemAddr());
  DFGStoreNode *Store = new (Graph) DFGStoreNode(Graph, V.getMemAddr(), Value,
                                                 V.getType());
  unsigned AddressSpace = GetSynthesizableAddressSpace(V.getAddressSpace(),
                                                       Origin);
  Store->setAddressSpace(AddressSpace);
  assert(Aliases.isMemoryAccess(Origin));
  Store->setAliasGroup(Aliases.getAlias(Origin));
  if (AddressSpace == LangAS::opencl_global) {
    OrderMemoryAccess(GlobalAccessMap, GlobalStoreMap, Store);
  } else if (AddressSpace == LangAS::opencl_local) {
    OrderMemoryAccess(LocalAccessMap, LocalStoreMap, Store);
  }
  if (AddressSpace == LangAS::opencl_local) {
    assert(HasRValueOf(VVars.getIndexedFlatWorkGroupID()));
    DFGNode *WorkGroupID = GetRValueOf(VVars.getIndexedFlatWorkGroupID());
    Store->setAdditionalPredecessor(Graph, WorkGroupID);
  }
  return Store;
}

// Struct and vector support

RecordDecl *DFGBuilder::GetStructDefinition(QualType Ty) {
  assert(Ty->isStructureType());
  const RecordType *StructTy = Ty->getAsStructureType();
  assert(StructTy != NULL);
  RecordDecl *Struct = StructTy->getDecl();
  assert(Struct != NULL);
  Struct = Struct->getDefinition();
  assert(Struct != NULL && Struct->isCompleteDefinition());
  return Struct;
}

uint64_t DFGBuilder::GetStructFieldOffset(FieldDecl *Field,
                                          unsigned FieldIndex) {
  if (FieldIndex == (unsigned)-1) { // no hint
    FieldIndex = Field->getFieldIndex();
  }
  const ASTRecordLayout &Layout = ASTCtx.getASTRecordLayout(Field->getParent());
  return Layout.getFieldOffset(FieldIndex);
}

QualType DFGBuilder::GetVectorElementType(QualType Ty) {
  const VectorType *VectorTy = Ty->getAs<VectorType>();
  assert(VectorTy != NULL);
  return VectorTy->getElementType();
}

unsigned DFGBuilder::GetVectorNumElements(QualType Ty) {
  const VectorType *VectorTy = Ty->getAs<VectorType>();
  assert(VectorTy != NULL);
  return VectorTy->getNumElements();
}

LValue DFGBuilder::GetFieldOf(DFGNode *BaseAddr, FieldDecl *Field,
                              QualType FieldTy, unsigned FieldSpace,
                              unsigned FieldIndex) {
  uint64_t OffsetValue = GetStructFieldOffset(Field, FieldIndex) /
                         ASTCtx.getCharWidth();
  if (OffsetValue == 0) {
    return LValue(BaseAddr, FieldTy, FieldSpace);
  } else {
    DFGNode *Offset = CreateIntConst(OffsetValue, ASTCtx.getSizeType());
    DFGNode *Addr = new (Graph) DFGBinaryOpNode(Graph, DFGBinaryOpNode::DBO_Add,
                                                BaseAddr, Offset,
                                                GetAddressType(FieldTy));
    return LValue(Addr, FieldTy, FieldSpace);
  }
}

ExprValue DFGBuilder::GetElementOf(ExprValue Base, DFGNode *Idx,
                                   QualType ElementTy) {
  unsigned Width = ASTCtx.getTypeSize(ElementTy);
  if (Base.hasLValue()) {
    if (IndexedVarDeclRef SSAVars = Base.getLValue().getSSAVars()) {
      if (SSAVars.isSingleVar()) {
        assert(Base.hasRValue() && Base.size() == 1);
        DFGNode *Offset = CreateMultipliedInt(Idx, Width);
        if (Base.hasMask()) {
          assert(Width <= Base.getMaskWidth());
          Offset = new (Graph) DFGBinaryOpNode(Graph, DFGBinaryOpNode::DBO_Add,
                                               Base.getMaskOffset(), Offset,
                                               Offset->getType());
        }
        return ExprValue(Base, Offset, Width, ElementTy);
      }
    }
  }

  DFGNode *ResultRV = NULL;
  if (Base.hasRValue()) {
    if (Base.size() == 1) {
      DFGNode *Offset = CreateMultipliedInt(Idx, Width);
      ResultRV = new (Graph) DFGVariableRangeOpNode(Graph, *Base, Offset, Width,
                                                    ElementTy);
    } else {
      llvm_unreachable("A compound value cannot be accessed through "
                       "a variable index");
    }
  }
  LValue ResultLV;
  if (Base.hasLValue()) {
    LValue BaseLV = Base.getLValue();
    if (IndexedVarDeclRef SSAVars = BaseLV.getSSAVars()) {
      assert(SSAVars.isCompound());
      llvm_unreachable("A compound value cannot be accessed through "
                       "a variable index");
    } else {
      BaseLV = FinalizeToAddressAccess(BaseLV);
      ResultLV = GetElementOf(BaseLV.getMemAddr(), Idx, ElementTy,
                              BaseLV.getAddressSpace());
    }
  }
  if (ResultRV) {
    return SplitRValue(ExprValue(ResultRV, ResultLV));
  } else {
    return ExprValue(ResultLV);
  }
}

ExprValue DFGBuilder::GetElementOf(ExprValue Base, unsigned Idx,
                                   QualType ElementTy) {
  unsigned Width = ASTCtx.getTypeSize(ElementTy);
  if (Base.hasLValue()) {
    if (IndexedVarDeclRef SSAVars = Base.getLValue().getSSAVars()) {
      if (SSAVars.isSingleVar()) {
        assert(Base.hasRValue() && Base.size() == 1);
        DFGNode *Offset = CreateIntConst(Idx * Width, ASTCtx.getSizeType());
        if (Base.hasMask()) {
          assert(Width <= Base.getMaskWidth());
          Offset = new (Graph) DFGBinaryOpNode(Graph, DFGBinaryOpNode::DBO_Add,
                                               Base.getMaskOffset(), Offset,
                                               Offset->getType());
        }
        return ExprValue(Base, Offset, Width, ElementTy);
      }
    }
  }

  DFGNode *ResultRV = NULL;
  if (Base.hasRValue()) {
    if (Base.size() == 1) {
      ResultRV = new (Graph) DFGRangeOpNode(Graph, *Base, Idx * Width, Width,
                                            ElementTy);
    } else {
      assert(Idx < Base.size());
      ResultRV = Base[Idx];
    }
  }
  LValue ResultLV;
  if (Base.hasLValue()) {
    LValue BaseLV = Base.getLValue();
    if (IndexedVarDeclRef SSAVars = BaseLV.getSSAVars()) {
      assert(SSAVars.isCompound());
      ResultLV = LValue(SSAVars[Idx]);
    } else if (WVarDecl *MemVar = BaseLV.getMemVar()) {
      assert(MemVar->isCompound() && Idx < MemVar->getNumSubVars());
      ResultLV = LValue(MemVar->getSubVar(Idx));
    } else {
      BaseLV = FinalizeToAddressAccess(BaseLV);
      ResultLV = GetElementOf(BaseLV.getMemAddr(), Idx, ElementTy,
                              BaseLV.getAddressSpace());
    }
  }
  if (ResultRV) {
    return SplitRValue(ExprValue(ResultRV, ResultLV));
  } else {
    return ExprValue(ResultLV);
  }
}

LValue DFGBuilder::GetElementOf(DFGNode *BaseAddr, DFGNode *Idx,
                                QualType ElementTy, unsigned ElementSpace) {
  uint64_t Width = ASTCtx.getTypeSize(ElementTy) / ASTCtx.getCharWidth();
  DFGNode *Offset = CreateMultipliedInt(Idx, Width);
  Offset = CreateCastedInt(Offset, ASTCtx.getSizeType());
  DFGNode *Addr = new (Graph) DFGBinaryOpNode(Graph, DFGBinaryOpNode::DBO_Add,
                                              BaseAddr, Offset,
                                              GetAddressType(ElementTy));
  return LValue(Addr, ElementTy, ElementSpace);
}

LValue DFGBuilder::GetElementOf(DFGNode *BaseAddr, unsigned Idx,
                                QualType ElementTy, unsigned ElementSpace) {
  if (Idx == 0) {
    return LValue(BaseAddr, ElementTy, ElementSpace);
  } else {
    uint64_t Width = ASTCtx.getTypeSize(ElementTy) / ASTCtx.getCharWidth();
    DFGNode *Offset = CreateIntConst(Idx * Width, ASTCtx.getSizeType());

    DFGNode *Addr = new (Graph) DFGBinaryOpNode(Graph, DFGBinaryOpNode::DBO_Add,
                                                BaseAddr, Offset,
                                                GetAddressType(ElementTy));
    return LValue(Addr, ElementTy, ElementSpace);
  }
}

void DFGBuilder::SplitStructure(LValue V, SmallVectorImpl<LValue> &Fields) {
  assert(V.getType()->isStructureType());
  if (IndexedVarDeclRef SSAVars = V.getSSAVars()) {
    assert(SSAVars.isCompound());
    for (unsigned Index = 0, NumSubVars = SSAVars.getNumSubVars();
         Index != NumSubVars; ++Index) {
      Fields.push_back(LValue(SSAVars[Index]));
    }
  } else if (WVarDecl *MemVar = V.getMemVar()) {
    assert(MemVar->isCompound());
    for (unsigned Index = 0, NumSubVars = MemVar->getNumSubVars();
         Index != NumSubVars; ++Index) {
      Fields.push_back(LValue(MemVar->getSubVar(Index)));
    }
  } else if (DFGNode *Addr = V.getMemAddr()) {
    RecordDecl *Struct = GetStructDefinition(V.getType());
    unsigned Index = 0;
    for (RecordDecl::field_iterator F = Struct->field_begin(),
                                    FEnd = Struct->field_end();
         F != FEnd; ++F, ++Index) {
      FieldDecl *Field = *F;
      assert(Field != NULL);
      assert(Field->getType()->isScalarType() && "not implemented yet");
      Fields.push_back(GetFieldOf(Addr, Field, Field->getType(),
                                  V.getAddressSpace(), Index));
    }

  } else {
    llvm_unreachable("invalid l-value");
  }
}

ExprValue DFGBuilder::SplitRValue(ExprValue V) {
  if (V.hasRValue() && V.size() == 1 && !V.hasMask()) {
    DFGNode *RV = *V;
    if (RV->getType()->isStructureType()) {
      SmallVector<DFGNode*, 16> RVList;
      SplitStructure(RV, RVList);
      return ExprValue(RVList, V.getLValue());
    } else if (RV->getType()->isVectorType()) {
      SmallVector<DFGNode*, 16> RVList;
      SplitVector(RV, RVList);
      return ExprValue(RVList, V.getLValue());
    }
  }
  return V;
}

void DFGBuilder::SplitStructure(DFGNode *V, SmallVectorImpl<DFGNode*> &Fields) {
  assert(V->getType()->isStructureType());
  RecordDecl *Struct = GetStructDefinition(V->getType());
  unsigned Index = 0;
  for (RecordDecl::field_iterator F = Struct->field_begin(),
                                  FEnd = Struct->field_end();
       F != FEnd; ++F, ++Index) {
    FieldDecl *Field = *F;
    assert(Field != NULL);
    assert(Field->getType()->isScalarType() && "not implemented yet");
    unsigned Offset = GetStructFieldOffset(Field, Index);
    unsigned Width = ASTCtx.getTypeSize(Field->getType());
    Fields.push_back(new (Graph) DFGRangeOpNode(Graph, V, Offset, Width,
                                                Field->getType()));
  }
}

void DFGBuilder::SplitVector(LValue V, SmallVectorImpl<LValue> &Elements) {
  assert(V.getType()->isVectorType());
  if (IndexedVarDeclRef SSAVars = V.getSSAVars()) {
    assert(SSAVars.isCompound());
    for (unsigned Index = 0, NumSubVars = SSAVars.getNumSubVars();
         Index != NumSubVars; ++Index) {
      Elements.push_back(LValue(SSAVars[Index]));
    }
  } else if (WVarDecl *MemVar = V.getMemVar()) {
    assert(MemVar->isCompound());
    for (unsigned Index = 0, NumSubVars = MemVar->getNumSubVars();
         Index != NumSubVars; ++Index) {
      Elements.push_back(LValue(MemVar->getSubVar(Index)));
    }
  } else if (DFGNode *Addr = V.getMemAddr()) {
    QualType ElementTy = GetVectorElementType(V.getType());
    for (unsigned Index = 0, NumElements = GetVectorNumElements(V.getType());
         Index != NumElements; ++Index) {
      Elements.push_back(GetElementOf(Addr, Index, ElementTy,
                                      V.getAddressSpace()));
    }

  } else {
    llvm_unreachable("invalid l-value");
  }
}

void DFGBuilder::SplitVector(DFGNode *V, SmallVectorImpl<DFGNode*> &Elements) {
  assert(V->getType()->isVectorType());
  QualType ElementTy = GetVectorElementType(V->getType());
  unsigned Width = ASTCtx.getTypeSize(ElementTy);
  for (unsigned Index = 0, NumElements = GetVectorNumElements(V->getType());
       Index != NumElements; ++Index) {
    unsigned Offset = Width * Index;
    DFGNode *Element = new (Graph) DFGRangeOpNode(Graph, V, Offset, Width,
                                                  ElementTy);
    Elements.push_back(Element);
  }
}

DFGBinaryOpNode::OperatorKind DFGBuilder::GetBinaryOpcode(
    BinaryOperator::Opcode Op, bool Vector) {
  switch (Op) {
    case BO_Mul:  return DFGBinaryOpNode::DBO_Mul;
    case BO_Div:  return DFGBinaryOpNode::DBO_Div;
    case BO_Rem:  return DFGBinaryOpNode::DBO_Rem;
    case BO_Add:  return DFGBinaryOpNode::DBO_Add;
    case BO_Sub:  return DFGBinaryOpNode::DBO_Sub;
    case BO_Shl:  return DFGBinaryOpNode::DBO_Shl;
    case BO_Shr:  return DFGBinaryOpNode::DBO_Shr;
    case BO_LT:   return Vector ? DFGBinaryOpNode::DBO_LT_V
                                : DFGBinaryOpNode::DBO_LT;
    case BO_GT:   return Vector ? DFGBinaryOpNode::DBO_GT_V
                                : DFGBinaryOpNode::DBO_GT;
    case BO_LE:   return Vector ? DFGBinaryOpNode::DBO_LE_V
                                : DFGBinaryOpNode::DBO_LE;
    case BO_GE:   return Vector ? DFGBinaryOpNode::DBO_GE_V
                                : DFGBinaryOpNode::DBO_GE;
    case BO_EQ:   return Vector ? DFGBinaryOpNode::DBO_EQ_V
                                : DFGBinaryOpNode::DBO_EQ;
    case BO_NE:   return Vector ? DFGBinaryOpNode::DBO_NE_V
                                : DFGBinaryOpNode::DBO_NE;
    case BO_And:  return DFGBinaryOpNode::DBO_And;
    case BO_Xor:  return DFGBinaryOpNode::DBO_Xor;
    case BO_Or:   return DFGBinaryOpNode::DBO_Or;
    case BO_LAnd: return Vector ? DFGBinaryOpNode::DBO_LAnd_V
                                : DFGBinaryOpNode::DBO_LAnd;
    case BO_LOr:  return Vector ? DFGBinaryOpNode::DBO_LOr_V
                                : DFGBinaryOpNode::DBO_LOr;
    case BO_MulAssign: return DFGBinaryOpNode::DBO_Mul;
    case BO_DivAssign: return DFGBinaryOpNode::DBO_Div;
    case BO_RemAssign: return DFGBinaryOpNode::DBO_Rem;
    case BO_AddAssign: return DFGBinaryOpNode::DBO_Add;
    case BO_SubAssign: return DFGBinaryOpNode::DBO_Sub;
    case BO_ShlAssign: return DFGBinaryOpNode::DBO_Shl;
    case BO_ShrAssign: return DFGBinaryOpNode::DBO_Shr;
    case BO_AndAssign: return DFGBinaryOpNode::DBO_And;
    case BO_XorAssign: return DFGBinaryOpNode::DBO_Xor;
    case BO_OrAssign:  return DFGBinaryOpNode::DBO_Or;
    default: llvm_unreachable("impossible");
  }
}

DFGUnaryOpNode::OperatorKind DFGBuilder::GetUnaryOpcodeForBuiltin(
    WCallExpr::BuiltinFunctionKind Func) {
  switch (Func) {
    case WCallExpr::BF_acos:  return DFGUnaryOpNode::DUO_Acos;
    case WCallExpr::BF_asin:  return DFGUnaryOpNode::DUO_Asin;
    case WCallExpr::BF_atan:  return DFGUnaryOpNode::DUO_Atan;
    case WCallExpr::BF_cos:   return DFGUnaryOpNode::DUO_Cos;
    case WCallExpr::BF_ceil:  return DFGUnaryOpNode::DUO_Ceil;
    case WCallExpr::BF_exp:   return DFGUnaryOpNode::DUO_Exp;
    case WCallExpr::BF_fabs:
    case WCallExpr::BF_abs:   return DFGUnaryOpNode::DUO_Abs;
    case WCallExpr::BF_floor: return DFGUnaryOpNode::DUO_Floor;
    case WCallExpr::BF_log:   return DFGUnaryOpNode::DUO_Log;
    case WCallExpr::BF_log2:  return DFGUnaryOpNode::DUO_Log2;
    case WCallExpr::BF_log10: return DFGUnaryOpNode::DUO_Log10;
    case WCallExpr::BF_round: return DFGUnaryOpNode::DUO_Round;
    case WCallExpr::BF_rsqrt: return DFGUnaryOpNode::DUO_RSqrt;
    case WCallExpr::BF_sin:   return DFGUnaryOpNode::DUO_Sin;
    case WCallExpr::BF_sqrt:  return DFGUnaryOpNode::DUO_Sqrt;
    case WCallExpr::BF_tan:   return DFGUnaryOpNode::DUO_Tan;
    case WCallExpr::BF_trunc: return DFGUnaryOpNode::DUO_Trunc;
    default: llvm_unreachable("impossible");
  }
}

DFGBinaryOpNode::OperatorKind DFGBuilder::GetBinaryOpcodeForBuiltin(
    WCallExpr::BuiltinFunctionKind Func) {
  switch (Func) {
    case WCallExpr::BF_fmod:  return DFGBinaryOpNode::DBO_Fmod;
    case WCallExpr::BF_max:   return DFGBinaryOpNode::DBO_Max;
    case WCallExpr::BF_min:   return DFGBinaryOpNode::DBO_Min;
    case WCallExpr::BF_mul24: return DFGBinaryOpNode::DBO_Mul24;
    case WCallExpr::BF_pow:   return DFGBinaryOpNode::DBO_Pow;
    default: llvm_unreachable("impossible");
  }
}

DFGNullaryAtomicNode::OperatorKind DFGBuilder::GetNullaryAtomicOpcodeForBuiltin(
    WCallExpr::BuiltinFunctionKind Func) {
  switch (Func) {
    case WCallExpr::BF_atomic_inc: return DFGNullaryAtomicNode::DNA_Inc;
    case WCallExpr::BF_atomic_dec: return DFGNullaryAtomicNode::DNA_Dec;
    default: llvm_unreachable("impossible");
  }
}

DFGUnaryAtomicNode::OperatorKind DFGBuilder::GetUnaryAtomicOpcodeForBuiltin(
    WCallExpr::BuiltinFunctionKind Func) {
  switch (Func) {
    case WCallExpr::BF_atomic_add:  return DFGUnaryAtomicNode::DUA_Add;
    case WCallExpr::BF_atomic_sub:  return DFGUnaryAtomicNode::DUA_Sub;
    case WCallExpr::BF_atomic_xchg: return DFGUnaryAtomicNode::DUA_Xchg;
    case WCallExpr::BF_atomic_min:  return DFGUnaryAtomicNode::DUA_Min;
    case WCallExpr::BF_atomic_max:  return DFGUnaryAtomicNode::DUA_Max;
    case WCallExpr::BF_atomic_and:  return DFGUnaryAtomicNode::DUA_And;
    case WCallExpr::BF_atomic_or:   return DFGUnaryAtomicNode::DUA_Or;
    case WCallExpr::BF_atomic_xor:  return DFGUnaryAtomicNode::DUA_Xor;
    default: llvm_unreachable("impossible");
  }
}

DFGBinaryAtomicNode::OperatorKind DFGBuilder::GetBinaryAtomicOpcodeForBuiltin(
    WCallExpr::BuiltinFunctionKind Func) {
  switch (Func) {
    case WCallExpr::BF_atomic_cmpxchg: return DFGBinaryAtomicNode::DBA_CmpXchg;
    default: llvm_unreachable("impossible");
  }
}

void DFGBuilder::OrderMemoryAccess(MemoryAccessMapTy &AccessMap,
                                   MemoryAccessMapTy &StoreMap,
                                   DFGMemoryAccessNode *Access) {
  unsigned Alias = Access->getAliasGroup();
  if (Access->isStoreKind()) {
    if (AccessMap.count(Alias)) {
      DFGNode *LastAccess = AccessMap[Alias];
      if (!Access->isReachableFrom(LastAccess)) {
        Access->setOrderedPredecessor(Graph, LastAccess);
      }
    }
    AccessMap[Alias] = Access;
    StoreMap[Alias] = Access;
  } else if (SnuCLOpts.LoadOrdering) {
    if (AccessMap.count(Alias)) {
      DFGNode *LastAccess = AccessMap[Alias];
      if (!Access->isReachableFrom(LastAccess)) {
        Access->setOrderedPredecessor(Graph, LastAccess);
      }
    }
    AccessMap[Alias] = Access;
  } else {
    if (StoreMap.count(Alias)) {
      DFGNode *LastStore = StoreMap[Alias];
      if (!Access->isReachableFrom(LastStore)) {
        Access->setOrderedPredecessor(Graph, LastStore);
      }
    }
    AccessMap[Alias] = Access;
  }
}

void DFGBuilder::OrderMemoryAccesses(MemoryAccessMapTy &AccessMap,
                                     DFGSinkNode *Sink) {
  for (MemoryAccessMapTy::iterator I = AccessMap.begin(), E = AccessMap.end();
       I != E; ++I) {
    if (!Sink->isReachableFrom(I->second)) {
      Sink->addPredecessor(I->second, Graph);
    }
  }
  AccessMap.clear();
}

void DFGBuilder::OrderMemoryAccesses(MemoryAccessMapTy &AccessMap,
                                     DFGBarrierNode *Barrier) {
  for (MemoryAccessMapTy::iterator I = AccessMap.begin(), E = AccessMap.end();
       I != E; ++I) {
    if (!Barrier->isReachableFrom(I->second)) {
      Barrier->addPredecessor(I->second, Graph);
    }
    I->second = Barrier;
  }
}

/* VisitAssignment
 * Version 1: ExprValue = ExprValue
 * Version 2: ExprValue = DFGNode*
 * Version 3: LValue = ExprValue
 * Version 4: LValue = DFGNode*, actually performs the assignment
 *
 *                           ---    For every    -->
 * DeclStmt --> [Version 3]  --- struct & vector -->  [Version 4]
 *                   ^       ---     element     -->       ^
 *                   |                                     |
 *                   |                           Append other elements
 *              (otherwise)                      to the assigned value
 *                   |                                     |
 *              [Version 1]  --(masked SSAVar) ---->  [Version 2]
 *                   ^                                     ^
 *                   |                                     |
 *             UnaryOperator                         UnaryOperator
 *           AssignmentOperator                CompoundAssignmentOperator
 *       CompoundAssignmentOperator
 */

ExprValue DFGBuilder::VisitAssignment(ExprValue Target, ExprValue Value,
                                      WExpr *OriginLHS, WExpr *OriginRHS) {
  assert(Target.hasLValue());
  if (Target.hasMask()) {
    return VisitAssignment(Target, GetOrMakeSingleValueOf(Value, OriginRHS),
                           OriginLHS, OriginRHS);
  } else {
    return VisitAssignment(Target.getLValue(), Value, OriginLHS, OriginRHS);
  }
}

ExprValue DFGBuilder::VisitAssignment(ExprValue Target, DFGNode *Value,
                                      WExpr *OriginLHS, WExpr *OriginRHS) {
  assert(Target.hasLValue());
  if (Target.hasMask()) {
    assert(Target.hasRValue() && Target.size() == 1);
    DFGNode *NewTarget = new (Graph) DFGSubstituteOpNode(
        Graph, *Target, Target.getMaskOffset(), Target.getMaskWidth(), Value,
        (*Target)->getType());
    VisitAssignment(Target.getLValue(), NewTarget, OriginLHS, OriginLHS);
    return ExprValue(Value);
  } else {
    return VisitAssignment(Target.getLValue(), Value, OriginLHS, OriginRHS);
  }
}

ExprValue DFGBuilder::VisitAssignment(LValue Target, ExprValue Value,
                                      WExpr *OriginLHS, WExpr *OriginRHS) {
  QualType TargetTy = Target.getType();
  if (TargetTy->isStructureType()) {
    SmallVector<LValue, 16> TargetList;
    SplitStructure(Target, TargetList);
    Value = FinalizeToRValue(Value, OriginRHS);
    assert(Value.size() == TargetList.size());
    for (unsigned Index = 0, NumTargets = TargetList.size();
         Index != NumTargets; ++Index) {
      VisitAssignment(TargetList[Index], Value[Index], OriginLHS, OriginRHS);
    }
    return Value;

  } else if (TargetTy->isVectorType()) {
    if (Target.getSSAVars()) {
      SmallVector<LValue, 16> TargetList;
      SplitVector(Target, TargetList);
      Value = FinalizeToRValue(Value, OriginRHS);
      assert(Value.size() == TargetList.size());
      for (unsigned Index = 0, NumTargets = TargetList.size();
           Index != NumTargets; ++Index) {
        VisitAssignment(TargetList[Index], Value[Index], OriginLHS, OriginRHS);
      }
      return Value;

    } else {
      DFGNode *SingleValue = GetOrMakeSingleValueOf(Value, OriginRHS);
      return VisitAssignment(Target, SingleValue, OriginLHS, OriginRHS);
    }

  } else {
    DFGNode *SingleValue = GetSingleValueOf(Value, OriginRHS);
    return VisitAssignment(Target, SingleValue, OriginLHS, OriginRHS);
  }
}

ExprValue DFGBuilder::VisitAssignment(LValue Target, DFGNode *Value,
                                      WExpr *OriginLHS, WExpr *OriginRHS) {
  if (IndexedVarDeclRef SSAVars = Target.getSSAVars()) {
    assert(SSAVars.isSingleVar());
    SetRValueOf(*SSAVars, Value);
  } else {
    Target = FinalizeToAddressAccess(Target);
    CreateStore(Target, Value, OriginLHS);
  }
  return ExprValue(Value);
}

ExprValue DFGBuilder::VisitIndexedVarDeclRef(IndexedVarDeclRef UseDecl,
                                             IndexedVarDeclRef DefDecl,
                                             QualType Ty) {
  if (UseDecl.isSingleVar()) {
    if (DefDecl) {
      if (HasRValueOf(*UseDecl)) {
        return ExprValue(GetRValueOf(*UseDecl), LValue(DefDecl, Ty));
      } else {
        return ExprValue(LValue(DefDecl, Ty));
      }
    } else {
      return ExprValue(GetRValueOf(*UseDecl));
    }
  } else {
    assert(UseDecl.isCompound());
    SmallVector<DFGNode*, 16> RValues;
    if (HasRValueOf(UseDecl[0])) {
      for (unsigned Index = 0, NumSubVars = UseDecl.getNumSubVars();
           Index != NumSubVars; ++Index) {
        RValues.push_back(GetRValueOf(UseDecl[Index]));
      }
    } else {
      for (unsigned Index = 0, NumSubVars = UseDecl.getNumSubVars();
           Index != NumSubVars; ++Index) {
        assert(!HasRValueOf(UseDecl[Index]));
      }
    }
    if (DefDecl) {
      if (RValues.empty()) {
        return ExprValue(LValue(DefDecl, Ty));
      } else {
        return ExprValue(RValues, LValue(DefDecl, Ty));
      }
    } else {
      return ExprValue(RValues);
    }
  }
}

ExprValue DFGBuilder::VisitDeclStmt(WDeclStmt *Node) {
  assert(Node->isSingleDecl());
  if (Node->hasSingleInit()) {
    WExpr *InitExpr = Node->getSingleInit();
    ExprValue Value = VisitStmt(InitExpr);
    LValue Target;
    if (IndexedVarDeclRef SSAVars = Node->getIndexedDecl()) {
      Target = LValue(SSAVars, Node->getSingleVarDecl()->getType());
    } else if (WVarDecl *MemVar = Node->getSingleVarDecl()) {
      Target = LValue(MemVar);
    } else {
      llvm_unreachable("not implemented yet");
    }
    VisitAssignment(Target, Value, NULL, InitExpr);
  }
  return ExprValue();
}

ExprValue DFGBuilder::VisitReturnStmt(WReturnStmt *Node) {
  // Since all user-defined functions are already inlined, we simply ignore all
  // return statements.
  return ExprValue();
}

ExprValue DFGBuilder::VisitDeclRefExpr(WDeclRefExpr *Node) {
  if (IndexedVarDeclRef SSAVars = Node->getIndexedUseDecl()) {
    return VisitIndexedVarDeclRef(SSAVars, Node->getIndexedDefDecl(),
                                  Node->getType());
  } else if (WVarDecl *MemVar = Node->getVarDecl()) {
    return ExprValue(LValue(MemVar));
  } else {
    llvm_unreachable("not implemented yet");
  }
}

ExprValue DFGBuilder::VisitPredefinedExpr(WPredefinedExpr *Node) {
  llvm_unreachable("not implemented yet");
}

ExprValue DFGBuilder::VisitIntegerLiteral(WIntegerLiteral *Node) {
  return ExprValue(new (Graph) DFGIntConstNode(Graph, Node->getValue(),
                                               Node->getType()));
}

ExprValue DFGBuilder::VisitCharacterLiteral(WCharacterLiteral *Node) {
  return ExprValue(CreateIntConst(Node->getValue(), Node->getType()));
}

ExprValue DFGBuilder::VisitFloatingLiteral(WFloatingLiteral *Node) {
  return ExprValue(new (Graph) DFGFloatConstNode(Graph, Node->getValue(),
                                                 Node->getType()));
}

ExprValue DFGBuilder::VisitStringLiteral(WStringLiteral *Node) {
  llvm_unreachable("not implemented yet");
}

ExprValue DFGBuilder::VisitParenExpr(WParenExpr *Node) {
  return VisitStmt(Node->getSubExpr());
}

ExprValue DFGBuilder::VisitUnaryOperator(WUnaryOperator *Node) {
  switch (Node->getOpcode()) {
    case UO_PostInc:
    case UO_PostDec:
    case UO_PreInc:
    case UO_PreDec: {
      WExpr *SubExpr = Node->getSubExpr();
      ExprValue SubValue = VisitStmt(SubExpr);
      ExprValue Target = SubValue;

      if (Node->getType()->isVectorType()) {
        SubValue = FinalizeToRValue(SubValue, SubExpr);
        assert(SubValue.size() > 1);
        DFGBinaryOpNode::OperatorKind Op = (Node->isIncrementOp() ?
                                            DFGBinaryOpNode::DBO_Add :
                                            DFGBinaryOpNode::DBO_Sub);
        QualType ElementTy = GetVectorElementType(Node->getType());
        DFGNode *One = CreateIntConst(1, ElementTy);
        SmallVector<DFGNode*, 16> Result;
        for (unsigned Index = 0, NumValues = SubValue.size();
             Index != NumValues; ++Index) {
          DFGNode *E = new (Graph) DFGBinaryOpNode(Graph, Op, SubValue[Index],
                                                   One, ElementTy);
          Result.push_back(E);
        }
        ExprValue ResultValue = ExprValue(Result);
        VisitAssignment(Target, ResultValue, SubExpr, SubExpr);
        return (Node->isPostfix() ? SubValue : ResultValue);

      } else {
        DFGNode *Operand = GetSingleValueOf(SubValue, SubExpr);
        DFGBinaryOpNode::OperatorKind Op = (Node->isIncrementOp() ?
                                            DFGBinaryOpNode::DBO_Add :
                                            DFGBinaryOpNode::DBO_Sub);
        DFGNode *Step = CreateIntConst(GetAddStep(Node->getType()),
                                       Node->getType());
        DFGNode *Result = new (Graph) DFGBinaryOpNode(Graph, Op, Operand, Step,
                                                      Node->getType());
        VisitAssignment(Target, Result, SubExpr, SubExpr);
        return (Node->isPostfix() ? ExprValue(Operand) : ExprValue(Result));
      }
    }

    case UO_AddrOf: {
      WExpr *SubExpr = Node->getSubExpr();
      ExprValue SubValue = VisitStmt(SubExpr);
      assert(SubValue.hasLValue());
      LValue SubLV = FinalizeToAddressAccess(SubValue);
      return ExprValue(SubLV.getMemAddr());
    }

    case UO_Deref: {
      WExpr *SubExpr = Node->getSubExpr();
      ExprValue SubValue = VisitStmt(SubExpr);
      DFGNode *Operand = GetSingleValueOf(SubValue, SubExpr);
      return ExprValue(LValue(Operand, Node->getType(),
                              GetPointeeAddressSpace(SubExpr->getType())));
    }

    case UO_Plus: {
      WExpr *SubExpr = Node->getSubExpr();
      ExprValue SubValue = VisitStmt(SubExpr);
      return FinalizeToRValue(SubValue, SubExpr);
    }

    case UO_Minus:
    case UO_Not:
    case UO_LNot: {
      WExpr *SubExpr = Node->getSubExpr();
      ExprValue SubValue = VisitStmt(SubExpr);

      if (Node->getType()->isVectorType()) {
        SubValue = FinalizeToRValue(SubValue, SubExpr);
        assert(SubValue.size() > 1);
        DFGUnaryOpNode::OperatorKind Op;
        switch (Node->getOpcode()) {
          case UO_Minus: Op = DFGUnaryOpNode::DUO_Minus;  break;
          case UO_Not:   Op = DFGUnaryOpNode::DUO_Not;    break;
          case UO_LNot:  Op = DFGUnaryOpNode::DUO_LNot_V; break;
          default:       llvm_unreachable("impossible");
        }
        QualType ElementTy = GetVectorElementType(Node->getType());
        SmallVector<DFGNode*, 16> Result;
        for (unsigned Index = 0, NumValues = SubValue.size();
             Index != NumValues; ++Index) {
          DFGNode *E = new (Graph) DFGUnaryOpNode(Graph, Op, SubValue[Index],
                                                  ElementTy);
          Result.push_back(E);
        }
        return ExprValue(Result);

      } else {
        DFGNode *Operand = GetSingleValueOf(SubValue, SubExpr);
        DFGUnaryOpNode::OperatorKind Op;
        switch (Node->getOpcode()) {
          case UO_Minus: Op = DFGUnaryOpNode::DUO_Minus; break;
          case UO_Not:   Op = DFGUnaryOpNode::DUO_Not;   break;
          case UO_LNot:  Op = DFGUnaryOpNode::DUO_LNot;  break;
          default:       llvm_unreachable("impossible");
        }
        DFGNode *Result = new (Graph) DFGUnaryOpNode(Graph, Op, Operand,
                                                     Node->getType());
        return ExprValue(Result);
      }
    }

    default:
      llvm_unreachable("unsupported unary operator");
  }
}

ExprValue DFGBuilder::VisitUnaryExprOrTypeTraitExpr(
    WUnaryExprOrTypeTraitExpr *Node) {
  switch (Node->getKind()) {
    case UETT_SizeOf: {
      QualType Ty = (Node->isArgumentType() ?
                     Node->getArgumentType() :
                     Node->getArgumentExpr()->getType());
      uint64_t Value = ASTCtx.getTypeSize(Ty) / ASTCtx.getCharWidth();
      return ExprValue(CreateIntConst(Value, Node->getType()));
    }

    case UETT_AlignOf: {
      llvm_unreachable("not implemented yet");
    }

    case UETT_VecStep: {
      llvm_unreachable("not implemented yet");
    }

    default:
      llvm_unreachable("impossible");
  }
}

ExprValue DFGBuilder::VisitArraySubscriptExpr(WArraySubscriptExpr *Node) {
  WExpr *BaseExpr = Node->getBase();
  WExpr *IdxExpr = Node->getIdx();
  ExprValue BaseValue = VisitStmt(BaseExpr);
  ExprValue IdxValue = VisitStmt(IdxExpr);
  QualType BaseTy = BaseExpr->getType();
  DFGNode *Base;
  unsigned BaseSpace;
  DFGNode *Idx = GetSingleValueOf(IdxValue, IdxExpr);
  if (BaseTy->isPointerType()) {
    Base = GetSingleValueOf(BaseValue, BaseExpr);
    BaseSpace = GetPointeeAddressSpace(BaseTy);
    return ExprValue(GetElementOf(Base, Idx, Node->getType(), BaseSpace));
  } else {
    assert(BaseTy->isArrayType());
    return GetElementOf(BaseValue, Idx, Node->getType());
  }
}

ExprValue DFGBuilder::VisitCallExpr(WCallExpr *Node) {
  if (Node->isBarrier()) {
    SmallVector<IndexedVarDecl*, 64> LVars;
    SmallVector<DFGNode*, 64> LValues;
    for (VarValueMapTy::const_iterator I = VarValueMap.begin(),
                                       E = VarValueMap.end();
         I != E; ++I) {
      LVars.push_back(I->first);
      LValues.push_back(I->second);
    }
    DFGBarrierNode *Barrier = new (Graph) DFGBarrierNode(Graph);
    for (unsigned Index = 0, NumLives = LValues.size();
         Index != NumLives; ++Index) {
      LValues[Index] = Barrier->addLiveVariable(LValues[Index], Graph);
      VarValueMap[LVars[Index]] = LValues[Index];
    }
    OrderMemoryAccesses(GlobalAccessMap, Barrier);
    OrderMemoryAccesses(GlobalStoreMap, Barrier);
    OrderMemoryAccesses(LocalAccessMap, Barrier);
    OrderMemoryAccesses(LocalStoreMap, Barrier);
    return ExprValue();
  }

  if (Node->isAtomic()) {
    WExpr *AddrExpr = Node->getArg(0);
    ExprValue AddrValue = VisitStmt(AddrExpr);
    DFGNode *Addr = GetSingleValueOf(AddrValue, AddrExpr);
    DFGAtomicNode *Atomic = NULL;

    if (Node->getNumArgs() == 1) {
      DFGNullaryAtomicNode::OperatorKind Op = GetNullaryAtomicOpcodeForBuiltin(
          Node->getBuiltinKind());
      Atomic = new (Graph) DFGNullaryAtomicNode(Graph, Op, Addr,
                                                Node->getType());

    } else if (Node->getNumArgs() == 2) {
      WExpr *SubExpr = Node->getArg(1);
      ExprValue SubValue = VisitStmt(SubExpr);
      DFGNode *Operand = GetSingleValueOf(SubValue, SubExpr);
      DFGUnaryAtomicNode::OperatorKind Op = GetUnaryAtomicOpcodeForBuiltin(
          Node->getBuiltinKind());
      Atomic = new (Graph) DFGUnaryAtomicNode(Graph, Op, Addr, Operand,
                                              Node->getType());

    } else if (Node->getNumArgs() == 3) {
      WExpr *LHSExpr = Node->getArg(1);
      WExpr *RHSExpr = Node->getArg(2);
      ExprValue LHSValue = VisitStmt(LHSExpr);
      ExprValue RHSValue = VisitStmt(RHSExpr);
      DFGNode *LHS = GetSingleValueOf(LHSValue, LHSExpr);
      DFGNode *RHS = GetSingleValueOf(RHSValue, RHSExpr);
      DFGBinaryAtomicNode::OperatorKind Op = GetBinaryAtomicOpcodeForBuiltin(
          Node->getBuiltinKind());
      Atomic = new (Graph) DFGBinaryAtomicNode(Graph, Op, Addr, LHS, RHS,
                                               Node->getType());
    }

    if (Atomic != NULL) {
      unsigned AddressSpace = GetPointeeAddressSpace(AddrExpr->getType());
      AddressSpace = GetSynthesizableAddressSpace(AddressSpace, Node);
      Atomic->setAddressSpace(AddressSpace);
      assert(Aliases.isMemoryAccess(Node));
      Atomic->setAliasGroup(Aliases.getAlias(Node));
      if (AddressSpace == LangAS::opencl_global) {
        OrderMemoryAccess(GlobalAccessMap, GlobalStoreMap, Atomic);
      } else if (AddressSpace == LangAS::opencl_local) {
        OrderMemoryAccess(LocalAccessMap, LocalStoreMap, Atomic);
      }
      if (AddressSpace == LangAS::opencl_local) {
        // An atomic operation does not need to provide the work-group ID
        // to the global memory subsystem because it assumes a single cache
        assert(HasRValueOf(VVars.getIndexedFlatWorkGroupID()));
        DFGNode *WorkGroupID = GetRValueOf(VVars.getIndexedFlatWorkGroupID());
        Atomic->setAdditionalPredecessor(Graph, WorkGroupID);
      }
      return ExprValue(Atomic);
    }
  }

  if (Node->isBuiltin()) {
    if (Node->getNumArgs() == 1) {
      WExpr *SubExpr = Node->getArg(0);
      ExprValue SubValue = VisitStmt(SubExpr);
      DFGUnaryOpNode::OperatorKind Op = GetUnaryOpcodeForBuiltin(
          Node->getBuiltinKind());

      if (Node->getType()->isVectorType()) {
        SubValue = FinalizeToRValue(SubValue, SubExpr);
        assert(SubValue.size() > 1);
        QualType ElementTy = GetVectorElementType(Node->getType());
        SmallVector<DFGNode*, 16> Result;
        for (unsigned Index = 0, NumValues = SubValue.size();
             Index != NumValues; ++Index) {
          DFGNode *E = new (Graph) DFGUnaryOpNode(Graph, Op, SubValue[Index],
                                                  ElementTy);
          Result.push_back(E);
        }
        return ExprValue(Result);

      } else {
        DFGNode *Operand = GetSingleValueOf(SubValue, SubExpr);
        DFGNode *Result = new (Graph) DFGUnaryOpNode(Graph, Op, Operand,
                                                     Node->getType());
        return ExprValue(Result);
      }

    } else if (Node->getNumArgs() == 2) {
      WExpr *LHSExpr = Node->getArg(0);
      WExpr *RHSExpr = Node->getArg(1);
      ExprValue LHSValue = VisitStmt(LHSExpr);
      ExprValue RHSValue = VisitStmt(RHSExpr);
      DFGBinaryOpNode::OperatorKind Op = GetBinaryOpcodeForBuiltin(
          Node->getBuiltinKind());

      if (Node->getType()->isVectorType()) {
        LHSValue = FinalizeToRValue(LHSValue, LHSExpr);
        RHSValue = FinalizeToRValue(RHSValue, RHSExpr);
        assert(LHSValue.size() == RHSValue.size());
        QualType ElementTy = GetVectorElementType(Node->getType());
        SmallVector<DFGNode*, 16> Result;
        for (unsigned Index = 0, NumValues = LHSValue.size();
             Index != NumValues; ++Index) {
          DFGNode *E = new (Graph) DFGBinaryOpNode(Graph, Op, LHSValue[Index],
                                                   RHSValue[Index], ElementTy);
          Result.push_back(E);
        }
        return ExprValue(Result);

      } else {
        DFGNode *LHS = GetSingleValueOf(LHSValue, LHSExpr);
        DFGNode *RHS = GetSingleValueOf(RHSValue, RHSExpr);
        DFGNode *Result = new (Graph) DFGBinaryOpNode(Graph, Op, LHS, RHS,
                                                      Node->getType());
        return ExprValue(Result);
      }
    }
  }

  FunctionDecl *Func = Node->getDirectCallee();
  assert(Func != NULL);

  SmallVector<DFGNode*, 16> Args;
  for (unsigned Index = 0, NumArgs = Node->getNumArgs();
       Index != NumArgs; ++Index) {
    WExpr *ArgExpr = Node->getArg(Index);
    ExprValue ArgValue = VisitStmt(ArgExpr);
    Args.push_back(GetSingleValueOf(ArgValue, ArgExpr));
  }
  DFGNode *Result = new (Graph) DFGFunctionCallNode(Graph, Func, Args,
                                                    Node->getType());
  return ExprValue(Result);
}

ExprValue DFGBuilder::VisitVirtualCallExpr(WCallExpr *Node) {
  return GetExprValueOf(Node);
}

ExprValue DFGBuilder::VisitMemberExpr(WMemberExpr *Node) {
  if (IndexedVarDeclRef SSAVars = Node->getIndexedUseDecl()) {
    return VisitIndexedVarDeclRef(SSAVars, Node->getIndexedDefDecl(),
                                  Node->getType());
  } else if (WVarDecl *MemVar = Node->getVarDecl()) {
    return ExprValue(LValue(MemVar));
  }

  WExpr *BaseExpr = Node->getBase();
  ExprValue BaseValue = VisitStmt(BaseExpr);
  QualType BaseTy = BaseExpr->getType();
  DFGNode *Base;
  unsigned BaseSpace;
  if (Node->isArrow()) {
    assert(BaseTy->isPointerType());
    Base = GetSingleValueOf(BaseValue, BaseExpr);
    BaseSpace = GetPointeeAddressSpace(BaseTy);
  } else {
    assert(BaseTy->isStructureType());
    assert(BaseValue.hasLValue());
    LValue BaseLV = FinalizeToAddressAccess(BaseValue);
    Base = BaseLV.getMemAddr();
    BaseSpace = BaseLV.getAddressSpace();
  }
  return ExprValue(GetFieldOf(Base, Node->getMemberDecl(), Node->getType(),
                              BaseSpace));
}

ExprValue DFGBuilder::VisitCompoundLiteralExpr(WCompoundLiteralExpr *Node) {
  return VisitStmt(Node->getInitializer());
}

ExprValue DFGBuilder::VisitCastExprInternal(WCastExpr *Node) {
  switch (Node->getCastKind()) {
    case CK_BitCast: {
      WExpr *SubExpr = Node->getSubExpr();
      assert(ASTCtx.getTypeSize(SubExpr->getType()) ==
             ASTCtx.getTypeSize(Node->getType()));
      assert(!SubExpr->getType()->isVectorType() &&
             !Node->getType()->isVectorType());
      ExprValue SubValue = VisitStmt(SubExpr);
      DFGNode *Operand = GetOrMakeSingleValueOf(SubValue, SubExpr);
      DFGNode *Result = new (Graph) DFGUnaryOpNode(
          Graph, DFGUnaryOpNode::DUO_ReinterpretCast, Operand, Node->getType());
      return ExprValue(Result);
    }

    case CK_LValueToRValue:
    case CK_NoOp: {
      return VisitStmt(Node->getSubExpr());
    }

    case CK_ArrayToPointerDecay: {
      WExpr *SubExpr = Node->getSubExpr();
      ExprValue SubValue = VisitStmt(SubExpr);
      assert(SubValue.hasLValue());
      LValue SubLV = FinalizeToAddressAccess(SubValue);
      return ExprValue(SubLV.getMemAddr());
    }

    case CK_NullToPointer:
    case CK_IntegralToPointer:
    case CK_PointerToIntegral:
    case CK_IntegralCast:
    case CK_IntegralToBoolean:
    case CK_IntToOCLSampler: {
      WExpr *SubExpr = Node->getSubExpr();
      ExprValue SubValue = VisitStmt(SubExpr);
      DFGNode *Operand = GetSingleValueOf(SubValue, SubExpr);
      DFGUnaryOpNode::OperatorKind Op =
          (ASTCtx.getTypeSize(SubExpr->getType()) ==
           ASTCtx.getTypeSize(Node->getType()) ?
           DFGUnaryOpNode::DUO_ReinterpretCast :
           DFGUnaryOpNode::DUO_IntCast);
      DFGNode *Result = new (Graph) DFGUnaryOpNode(Graph, Op, Operand,
                                                   Node->getType());
      return ExprValue(Result);
    }

    case CK_VectorSplat: {
      WExpr *SubExpr = Node->getSubExpr();
      ExprValue SubValue = VisitStmt(SubExpr);
      DFGNode *Operand = GetSingleValueOf(SubValue, SubExpr);
      SmallVector<DFGNode*, 16> Result;
      for (unsigned Index = 0,
                    NumElements = GetVectorNumElements(Node->getType());
           Index != NumElements; ++Index) {
        Result.push_back(Operand);
      }
      return ExprValue(Result);
    }

    case CK_IntegralToFloating:
    case CK_FloatingToIntegral:
    case CK_FloatingToBoolean:
    case CK_FloatingCast: {
      WExpr *SubExpr = Node->getSubExpr();
      ExprValue SubValue = VisitStmt(SubExpr);
      DFGNode *Operand = GetSingleValueOf(SubValue, SubExpr);
      DFGUnaryOpNode::OperatorKind Op;
      switch (Node->getCastKind()) {
        case CK_IntegralToFloating:
          Op = DFGUnaryOpNode::DUO_IntToFloatCast;
          break;
        case CK_FloatingToIntegral:
        case CK_FloatingToBoolean:
          Op = DFGUnaryOpNode::DUO_FloatToIntCast;
          break;
        case CK_FloatingCast:
          Op = DFGUnaryOpNode::DUO_FloatCast;
          break;
        default: llvm_unreachable("impossible");
      }
      DFGNode *Result = new (Graph) DFGUnaryOpNode(Graph, Op, Operand,
                                                   Node->getType());
      return ExprValue(Result);
    }

    default:
      llvm_unreachable("unsupported cast kind");
  }
}

ExprValue DFGBuilder::VisitImplicitCastExpr(WImplicitCastExpr *Node) {
  return VisitCastExprInternal(Node);
}

ExprValue DFGBuilder::VisitCStyleCastExpr(WCStyleCastExpr *Node) {
  return VisitCastExprInternal(Node);
}

ExprValue DFGBuilder::VisitBinaryOperator(WBinaryOperator *Node) {
  if (Node->isAssignmentOp()) {
    WExpr *LHSExpr = Node->getLHS();
    WExpr *RHSExpr = Node->getRHS();
    ExprValue RHSValue = VisitStmt(RHSExpr);
    ExprValue LHSValue = VisitStmt(LHSExpr);
    return VisitAssignment(LHSValue, RHSValue, LHSExpr, RHSExpr);

  } else {
    WExpr *LHSExpr = Node->getLHS();
    WExpr *RHSExpr = Node->getRHS();

    llvm::APSInt RHSConst;
    // LHS * 16 => LHS << 4; LHS / 16 => LHS >> 4
    if ((Node->getOpcode() == BO_Mul || Node->getOpcode() == BO_Div) &&
        LHSExpr->getType()->isIntegerType() &&
        RHSExpr->EvaluateAsInt(RHSConst, ASTCtx) &&
        RHSConst.exactLogBase2() > 0) {
      ExprValue LHSValue = VisitStmt(LHSExpr);
      DFGNode *LHS = GetSingleValueOf(LHSValue, LHSExpr);
      DFGNode *RHS = CreateIntConst(RHSConst.exactLogBase2(), ASTCtx.IntTy);
      DFGBinaryOpNode::OperatorKind Op;
      if (Node->getOpcode() == BO_Mul) {
        Op = DFGBinaryOpNode::DBO_Shl;
      } else {
        Op = DFGBinaryOpNode::DBO_Shr;
      }
      DFGNode *Result = new (Graph) DFGBinaryOpNode(Graph, Op, LHS, RHS,
                                                    Node->getType());
      return ExprValue(Result);
    }
    // LHS % 16 => LHS & 15
    if (Node->getOpcode() == BO_Rem &&
        LHSExpr->getType()->isUnsignedIntegerType() &&
        RHSExpr->EvaluateAsInt(RHSConst, ASTCtx) &&
        RHSConst.exactLogBase2() > 0) {
      ExprValue LHSValue = VisitStmt(LHSExpr);
      DFGNode *LHS = GetSingleValueOf(LHSValue, LHSExpr);
      DFGNode *RHS = CreateIntConst(RHSConst.getZExtValue() - 1, LHS->getType());
      DFGBinaryOpNode::OperatorKind Op = DFGBinaryOpNode::DBO_And;
      DFGNode *Result = new (Graph) DFGBinaryOpNode(Graph, Op, LHS, RHS,
                                                    Node->getType());
      return ExprValue(Result);
    }

    ExprValue LHSValue = VisitStmt(LHSExpr);
    ExprValue RHSValue = VisitStmt(RHSExpr);
    if (Node->getType()->isVectorType()) {
      LHSValue = FinalizeToRValue(LHSValue, LHSExpr);
      RHSValue = FinalizeToRValue(RHSValue, RHSExpr);
      assert(LHSValue.size() == RHSValue.size());
      DFGBinaryOpNode::OperatorKind Op = GetBinaryOpcode(Node->getOpcode(),
                                                         true);
      QualType ElementTy = GetVectorElementType(Node->getType());
      SmallVector<DFGNode*, 16> Result;
      for (unsigned Index = 0, NumValues = LHSValue.size();
           Index != NumValues; ++Index) {
        DFGNode *E = new (Graph) DFGBinaryOpNode(Graph, Op, LHSValue[Index],
                                                 RHSValue[Index], ElementTy);
        Result.push_back(E);
      }
      return ExprValue(Result);

    } else {
      DFGNode *LHS = GetSingleValueOf(LHSValue, LHSExpr);
      DFGNode *RHS = GetSingleValueOf(RHSValue, RHSExpr);
      DFGBinaryOpNode::OperatorKind Op = GetBinaryOpcode(Node->getOpcode());

      // Pointer + Integer, Pointer - Integer
      if ((Node->getOpcode() == BO_Add || Node->getOpcode() == BO_Sub) &&
          LHS->getType()->isPointerType() && RHS->getType()->isIntegerType()) {
        RHS = CreateMultipliedInt(RHS, GetAddStep(LHS->getType()));
        RHS = CreateCastedInt(RHS, ASTCtx.getSizeType());
      }
      // Integer + Pointer
      if (Node->getOpcode() == BO_Add && LHS->getType()->isIntegerType() &&
          RHS->getType()->isPointerType()) {
        LHS = CreateMultipliedInt(LHS, GetAddStep(RHS->getType()));
        LHS = CreateCastedInt(LHS, ASTCtx.getSizeType());
      }

      DFGNode *Result = new (Graph) DFGBinaryOpNode(Graph, Op, LHS, RHS,
                                                    Node->getType());

      // Pointer - Pointer
      if (Node->getOpcode() == BO_Sub && LHS->getType()->isPointerType() &&
          RHS->getType()->isPointerType()) {
        uint64_t StepValue = GetAddStep(LHS->getType());
        if (StepValue != 1) {
          DFGNode *Step = CreateIntConst(StepValue, Result->getType());
          Result = new (Graph) DFGBinaryOpNode(Graph, DFGBinaryOpNode::DBO_Div,
                                               Result, Step, Result->getType());
        }
      }

      return ExprValue(Result);
    }
  }
}

ExprValue DFGBuilder::VisitVirtualBinaryOperator(WBinaryOperator *Node) {
  if (Node->isLogicalOp()) {
    WExpr *LHSExpr = Node->getLHS();
    WExpr *RHSExpr = Node->getRHS();
    ExprValue LHSValue = GetExprValueOf(LHSExpr);
    ExprValue RHSValue = GetExprValueOf(RHSExpr);
    if (Node->getType()->isVectorType()) {
      LHSValue = FinalizeToRValue(LHSValue, LHSExpr);
      RHSValue = FinalizeToRValue(RHSValue, RHSExpr);
      assert(LHSValue.size() == RHSValue.size());
      DFGBinaryOpNode::OperatorKind Op = GetBinaryOpcode(Node->getOpcode(),
                                                         true);
      QualType ElementTy = GetVectorElementType(Node->getType());
      SmallVector<DFGNode*, 16> Result;
      for (unsigned Index = 0, NumValues = LHSValue.size();
           Index != NumValues; ++Index) {
        DFGNode *E = new (Graph) DFGBinaryOpNode(Graph, Op, LHSValue[Index],
                                                 RHSValue[Index], ElementTy);
        Result.push_back(E);
      }
      return ExprValue(Result);

    } else {
      DFGNode *LHS = GetSingleValueOf(LHSValue, LHSExpr);
      DFGNode *RHS = GetSingleValueOf(RHSValue, RHSExpr);
      DFGBinaryOpNode::OperatorKind Op = GetBinaryOpcode(Node->getOpcode());
      DFGNode *Result = new (Graph) DFGBinaryOpNode(Graph, Op, LHS, RHS,
                                                    Node->getType());
      return ExprValue(Result);
    }

  } else if (Node->getOpcode() == BO_Comma) {
    return GetExprValueOf(Node->getRHS());

  } else {
    llvm_unreachable("impossible");
  }
}

ExprValue DFGBuilder::VisitCompoundAssignOperator(WCompoundAssignOperator *Node) {
  WExpr *LHSExpr = Node->getLHS();
  WExpr *RHSExpr = Node->getRHS();
  ExprValue RHSValue = VisitStmt(RHSExpr);
  ExprValue LHSValue = VisitStmt(LHSExpr);
  ExprValue Target = LHSValue;

  if (Node->getType()->isVectorType()) {
    LHSValue = FinalizeToRValue(LHSValue, LHSExpr);
    RHSValue = FinalizeToRValue(RHSValue, RHSExpr);
    assert(LHSValue.size() == RHSValue.size());
    DFGBinaryOpNode::OperatorKind Op = GetBinaryOpcode(Node->getOpcode(), true);
    QualType ElementTy = GetVectorElementType(Node->getType());
    SmallVector<DFGNode*, 16> Result;
    for (unsigned Index = 0, NumValues = LHSValue.size();
         Index != NumValues; ++Index) {
      DFGNode *E = new (Graph) DFGBinaryOpNode(Graph, Op, LHSValue[Index],
                                               RHSValue[Index], ElementTy);
      Result.push_back(E);
    }
    return VisitAssignment(Target, ExprValue(Result), LHSExpr, Node);

  } else {
    DFGNode *LHS = GetSingleValueOf(LHSValue, LHSExpr);
    DFGNode *RHS = GetSingleValueOf(RHSValue, RHSExpr);
    DFGBinaryOpNode::OperatorKind Op = GetBinaryOpcode(Node->getOpcode());

    // Pointer += Integer, Pointer -= Integer
    if ((Node->getOpcode() == BO_AddAssign ||
         Node->getOpcode() == BO_SubAssign) &&
        LHS->getType()->isPointerType() && RHS->getType()->isIntegerType()) {
      RHS = CreateMultipliedInt(RHS, GetAddStep(LHS->getType()));
      RHS = CreateCastedInt(RHS, ASTCtx.getSizeType());
    }

    DFGNode *Result = new (Graph) DFGBinaryOpNode(Graph, Op, LHS, RHS,
                                                  Node->getType());
    return VisitAssignment(Target, Result, LHSExpr, Node);
  }
}

ExprValue DFGBuilder::VisitVirtualConditionalOperator(WConditionalOperator *Node) {
  WExpr *CondExpr = Node->getCond();
  WExpr *LHSExpr = Node->getLHS();
  WExpr *RHSExpr = Node->getRHS();
  ExprValue CondValue = GetExprValueOf(CondExpr);
  ExprValue LHSValue = GetExprValueOf(LHSExpr);
  ExprValue RHSValue = GetExprValueOf(RHSExpr);
  if (Node->getType()->isVectorType()) {
    llvm_unreachable("not implemented yet");

  } else {
    DFGNode *Cond = GetSingleValueOf(CondValue, CondExpr);
    DFGNode *LHS = GetSingleValueOf(LHSValue, LHSExpr);
    DFGNode *RHS = GetSingleValueOf(RHSValue, RHSExpr);
    DFGNode *Result = new (Graph) DFGTernaryOpNode(
        Graph, DFGTernaryOpNode::DTO_Conditional, Cond, LHS, RHS,
        Node->getType());
    return ExprValue(Result);
  }
}

ExprValue DFGBuilder::VisitInitListExpr(WInitListExpr *Node) {
  SmallVector<DFGNode*, 16> Fields;
  for (unsigned Index = 0, NumInits = Node->getNumInits();
       Index != NumInits; ++Index) {
    WExpr *Init = Node->getInit(Index);
    assert(Init->getType()->isScalarType() && "not implemented yet");
    Fields.push_back(GetSingleValueOf(VisitStmt(Init), Init));
  }
  return ExprValue(Fields);
}

ExprValue DFGBuilder::VisitDesignatedInitExpr(WDesignatedInitExpr *Node) {
  llvm_unreachable("not implemented yet");
}

ExprValue DFGBuilder::VisitImplicitValueInitExpr(WImplicitValueInitExpr *Node) {
  llvm_unreachable("not implemented yet");
}

ExprValue DFGBuilder::VisitParenListExpr(WParenListExpr *Node) {
  llvm_unreachable("not implemented yet");
}

ExprValue DFGBuilder::VisitExtVectorElementExpr(WExtVectorElementExpr *Node) {
  if (IndexedVarDeclRef SSAVars = Node->getIndexedUseDecl()) {
    return VisitIndexedVarDeclRef(SSAVars, Node->getIndexedDefDecl(),
                                  Node->getType());
  } else if (WVarDecl *MemVar = Node->getVarDecl()) {
    return ExprValue(LValue(MemVar));
  } else {
    SmallVector<unsigned, 16> Elts;
    Node->getEncodedElementAccess(Elts);
    assert(!Elts.empty());
    ExprValue BaseValue = VisitStmt(Node->getBase());
    if (Elts.size() == 1) {
      return GetElementOf(BaseValue, Elts[0], Node->getType());
    } else {
      assert(!BaseValue.hasLValue() && "not implemented yet");
      SmallVector<DFGNode*, 16> ResultRV;
      for (unsigned Index = 0, NumElts = Elts.size();
           Index != NumElts; ++Index) {
        assert(Elts[Index] < BaseValue.size());
        ResultRV.push_back(BaseValue[Elts[Index]]);
      }
      return ExprValue(ResultRV);
    }
  }
}

ExprValue DFGBuilder::VisitCXXBoolLiteralExpr(WCXXBoolLiteralExpr *Node) {
  return ExprValue(CreateIntConst((Node->getValue() ? 1 : 0), Node->getType()));
}

ExprValue DFGBuilder::VisitBlockExpr(WBlockExpr *Node) {
  llvm_unreachable("not implemented yet");
}

ExprValue DFGBuilder::VisitAsTypeExpr(WAsTypeExpr *Node) {
  QualType SrcTy = Node->getSrcExpr()->getType();
  QualType DestTy = Node->getType();
  assert(ASTCtx.getTypeSize(SrcTy) == ASTCtx.getTypeSize(DestTy));
  ExprValue SubValue = VisitStmt(Node->getSrcExpr());
  DFGNode *Operand = GetOrMakeSingleValueOf(SubValue, Node->getSrcExpr());
  DFGNode *Result = new (Graph) DFGUnaryOpNode(
      Graph, DFGUnaryOpNode::DUO_ReinterpretCast, Operand, DestTy);
  if (DestTy->isVectorType()) {
    SmallVector<DFGNode*, 16> RVList;
    SplitVector(Result, RVList);
    return ExprValue(RVList);
  } else {
    return ExprValue(Result);
  }
}

ExprValue DFGBuilder::VisitPhiFunction(WPhiFunction *Node) {
  return ExprValue();
}

ExprValue DFGBuilder::VisitWorkItemFunction(WWorkItemFunction *Node) {
  IndexedVarDecl *V;
  switch (Node->getFunctionKind()) {
    case WWorkItemFunction::WIF_get_global_size:
      V = VVars.getIndexedGlobalSize(Node->getArg());
      break;
    case WWorkItemFunction::WIF_get_global_id:
      V = VVars.getIndexedGlobalID(Node->getArg());
      break;
    case WWorkItemFunction::WIF_get_local_size:
      V = VVars.getIndexedLocalSize(Node->getArg());
      break;
    case WWorkItemFunction::WIF_get_local_id:
      V = VVars.getIndexedLocalID(Node->getArg());
      break;
    case WWorkItemFunction::WIF_get_num_groups:
      V = VVars.getIndexedNumWorkGroups(Node->getArg());
      break;
    case WWorkItemFunction::WIF_get_group_id:
      V = VVars.getIndexedWorkGroupID(Node->getArg());
      break;
    default:
      llvm_unreachable("invalid work-item function");
  }
  return ExprValue(GetRValueOf(V));
}

ExprValue DFGBuilder::VisitBaseMuFunction(WBaseMuFunction *Node) {
  llvm_unreachable("not supported");
}

ExprValue DFGBuilder::VisitAuxiliaryMuFunction(WAuxiliaryMuFunction *Node) {
  llvm_unreachable("not supported");
}

} // anonymous namespace

// DataflowGraph

bool DataflowGraph::hasBarrier() const {
  for (DFGNodeListTy::const_iterator I = Nodes.begin(), E = Nodes.end();
       I != E; ++I) {
    if (isa<DFGBarrierNode>(*I)) {
      return true;
    }
  }
  return false;
}

void DataflowGraph::addNode(DFGNode *Node) {
  Nodes.push_back(Node, BVCtx);
  HasTopologicalOrder = false;
  if (DFGSourceNode *source = dyn_cast<DFGSourceNode>(Node)) {
    assert(Source == NULL);
    Source = source;
  }
  if (DFGSinkNode *sink = dyn_cast<DFGSinkNode>(Node)) {
    assert(Sink == NULL);
    Sink = sink;
  }
}

void DataflowGraph::addLiveIn(IndexedVarDecl *Var) {
  LiveIns.push_back(Var, BVCtx);
}

void DataflowGraph::addLiveOut(IndexedVarDecl *Var) {
  LiveOuts.push_back(Var, BVCtx);
}

void DataflowGraph::addCompound(DFGCompoundNode *Compound) {
  Compounds.push_back(Compound, BVCtx);
  for (DFGCompoundNode::iterator I = Compound->begin(), E = Compound->end();
       I != E; ++I) {
    assert(!NodesInAnyCompound.count(*I));
    NodesInAnyCompound.insert(*I);
  }
}

bool DataflowGraph::isNodeInAnyCompound(DFGNode *Node) const {
  return NodesInAnyCompound.count(Node);
}

DFGCompoundNode *DataflowGraph::getCompoundOf(DFGNode *Node) const {
  for (CompoundListTy::const_iterator I = Compounds.begin(),
                                      E = Compounds.end();
       I != E; ++I) {
    if ((*I)->count(Node)) {
      return *I;
    }
  }
  assert(!isNodeInAnyCompound(Node));
  return NULL;
}

unsigned DataflowGraph::getNumPreds() const {
  assert(EntryBlock != NULL);
  return EntryBlock->pred_size();
}

unsigned DataflowGraph::getNumSuccs() const {
  assert(ExitBlock != NULL);
  return ExitBlock->real_succ_size();
}

DataflowGraph *DataflowGraph::getSucc(const ControlDataflowGraph *cdfg,
                                      unsigned Index) const {
  assert(Index < ExitBlock->real_succ_size() && "index out of bound");
  DataflowGraph *Succ = cdfg->getDFG(*(ExitBlock->real_succ_begin() + Index));
  assert(Succ != NULL && "DFG not found");
  return Succ;
}

void DataflowGraph::MakeTopologicalOrder() {
  if (HasTopologicalOrder) {
    return;
  }
  Nodes.clear();

  typedef OrderedDenseMap<DFGNode*, unsigned> RefCountMapTy;
  RefCountMapTy RefCount;
  RefCountMapTy CompoundRefCount;

  RefCount[Source] = 0;
  while (!RefCount.empty()) {
    DFGNode *AvailableNode = NULL;
    for (RefCountMapTy::const_iterator I = RefCount.begin(), E = RefCount.end();
         I != E; ++I) {
      DFGNode *Node = I->first;
      if (I->second == 0) {
        if (DFGNode *Compound = getCompoundOf(Node)) {
          if (!CompoundRefCount.count(Compound) ||
              CompoundRefCount[Compound] > 0) {
            continue;
          }
        }
        AvailableNode = Node;
        RefCount.erase(Node);
        break;
      }
    }
    assert(AvailableNode != NULL);

    Nodes.push_back(AvailableNode, BVCtx);
    for (DFGNode::succ_iterator S = AvailableNode->succ_begin(),
                                SEnd = AvailableNode->succ_end();
         S != SEnd; ++S) {
      DFGNode *Succ = *S;
      if (RefCount.count(Succ)) {
        assert(RefCount[Succ] > 0);
        RefCount[Succ]--;
      } else {
        assert(Succ->pred_size() > 0);
        RefCount[Succ] = Succ->pred_size() - 1;
      }
      if (DFGNode *Compound = getCompoundOf(Succ)) {
        if (Compound != getCompoundOf(AvailableNode)) {
          if (CompoundRefCount.count(Compound)) {
            assert(CompoundRefCount[Compound] > 0);
            CompoundRefCount[Compound]--;
          } else {
            assert(Compound->pred_size() > 0);
            CompoundRefCount[Compound] = Compound->pred_size() - 1;
          }
        }
      }
    }
  }

  HasTopologicalOrder = true;
}

void DataflowGraph::PruneUnusedNodes() {
  if (Nodes.empty()) {
    return;
  }
  for (reverse_iterator I = t_rbegin(), E = t_rend(); I != E; ++I) {
    DFGNode *Node = *I;
    if (!isa<DFGSinkNode>(Node) && Node->succ_size() == 0) {
      if (isa<DFGScatterNode>(Node)) {
        continue;
      }
      if (getCompoundOf(Node)) {
        continue;
      }
      for (DFGNode::pred_iterator P = Node->pred_begin(),
                                  PEnd = Node->pred_end();
           P != PEnd; ++P) {
        (*P)->removeSuccessor(Node, this);
      }
      HasTopologicalOrder = false;
    }
  }
  MakeTopologicalOrder();
}

void DataflowGraph::ReloadAllCompoundNeighbors() {
  for (CompoundListTy::iterator I = Compounds.begin(), E = Compounds.end();
       I != E; ++I) {
    (*I)->ReloadNeighbors(this);
  }
}

// ControlDataflowGraph

ControlDataflowGraph::ControlDataflowGraph(WCFG *cfg_, const ASTContext &ASTCtx)
  : cfg(cfg_), DataflowGraphMap(cfg_->size(), NULL), Entry(NULL), Exit(NULL),
    VVars(ASTCtx) {}

DataflowGraph *ControlDataflowGraph::getDFG(WCFGBlock *Block) const {
  return DataflowGraphMap[Block->getBlockID()];
}

void ControlDataflowGraph::setDFG(WCFGBlock *Block, DataflowGraph *DFG) {
  DataflowGraphMap[Block->getBlockID()] = DFG;
}

void ControlDataflowGraph::ConstructOnBlock(
    WCFGBlock *Block, const ASTContext &ASTCtx, const SnuCLOptions &SnuCLOpts,
    const AliasSet &AS) {
  if (getDFG(Block))
    return;

  DFGBuilder builder(ASTCtx, SnuCLOpts, AS, VVars);
  DataflowGraph *dfg = builder.build(Block);
  setDFG(Block, dfg);

  WCFGBlock *ExitBlock = dfg->getExitBlock();
  assert(ExitBlock != NULL);
  for (WCFGBlock::succ_iterator S = ExitBlock->real_succ_begin(),
                                SEnd = ExitBlock->real_succ_end();
       S != SEnd; ++S) {
    ConstructOnBlock(*S, ASTCtx, SnuCLOpts, AS);
  }

  if (dfg->getEntryBlock() == cfg->getEntry()) {
    assert(Entry == NULL);
    Entry = dfg;
  }
  if (dfg->getExitBlock() == cfg->getExit()) {
    assert(Exit == NULL);
    Exit = dfg;
  }
}

ControlDataflowGraph *ControlDataflowGraph::MakeCDFG(
    WCFG *cfg, const ASTContext &ASTCtx, const SnuCLOptions &SnuCLOpts,
    const AliasSet &AS) {
  ControlDataflowGraph *cdfg = new ControlDataflowGraph(cfg, ASTCtx);
  cdfg->ConstructOnBlock(cfg->getEntry(), ASTCtx, SnuCLOpts, AS);
  // Make sure that all basic blocks from the entry to the exit are translated
  assert(cdfg->Entry != NULL && cdfg->Exit != NULL);
  return cdfg;
}

} // naemspace snu

} // namespace clang
