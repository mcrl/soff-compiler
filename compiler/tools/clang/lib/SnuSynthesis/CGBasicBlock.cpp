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

#include "CGBasicBlock.h"
#include "CGCommon.h"
#include "CGLockSubsystem.h"
#include "CGMemorySubsystem.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/Type.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuSupport/OrderedDenseADT.h"
#include "clang/SnuSynthesis/CodeGenerator.h"
#include "clang/SnuSynthesis/DataflowGraph.h"
#include "clang/SnuSynthesis/Verilog.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringExtras.h"
#include <algorithm>
#include <string>

namespace clang {

namespace snu {

namespace Synthesis {

/*
 * DPOperation: A fixed-latency (compound) operation
 */

DPOperation::DPOperation(CodeGenContext &ctx, ModuleBuilder *parent)
  : Ctx(ctx), Parent(parent), Latency(0) {
  Enable = Parent->CreateTempWire("op_enable");
}

void DPOperation::addNode(DFGNode *Node) {
  VerilogSignal *Result = NULL;
  if (DFGUnaryOpNode *UO = dyn_cast<DFGUnaryOpNode>(Node)) {
    Result = CreateUnaryOpResult(UO);
  } else if (DFGBinaryOpNode *BO = dyn_cast<DFGBinaryOpNode>(Node)) {
    Result = CreateBinaryOpResult(BO);
  } else if (DFGTernaryOpNode *TO = dyn_cast<DFGTernaryOpNode>(Node)) {
    Result = CreateTernaryOpResult(TO);
  } else if (DFGRangeOpNode *RO = dyn_cast<DFGRangeOpNode>(Node)) {
    Result = CreateRangeOpResult(RO);
  } else if (DFGVariableRangeOpNode *VRO = dyn_cast<DFGVariableRangeOpNode>(Node)) {
    Result = CreateVariableRangeOpResult(VRO);
  } else if (DFGConcatOpNode *CO = dyn_cast<DFGConcatOpNode>(Node)) {
    Result = CreateConcatOpResult(CO);
  } else if (DFGSubstituteOpNode *SO = dyn_cast<DFGSubstituteOpNode>(Node)) {
    Result = CreateSubstituteOpResult(SO);
  } else if (DFGShiftRegisterNode *Q = dyn_cast<DFGShiftRegisterNode>(Node)) {
    Result = CreateShiftRegisterResult(Q);
  } else {
    llvm_unreachable("invalid operation");
  }
  if (Result != NULL) {
    Intermediate[Node] = Result;
  }

  unsigned Tstart = 0;
  for (DFGNode::const_pred_iterator P = Node->pred_begin(),
                                    PEnd = Node->pred_end();
       P != PEnd; ++P) {
    if (Tend.count(*P)) {
      Tstart = std::max(Tstart, Tend[*P]);
    }
  }
  Tend[Node] = Tstart + Ctx.getLmax(Node);
}

void DPOperation::markAsOutput(DFGNode *Node) {
  assert(Intermediate.count(Node));
  Output[Node] = Intermediate[Node];
  assert(Tend.count(Node));
  Latency = std::max(Latency, Tend[Node]);
}

VerilogSignal *DPOperation::getSignalOf(DFGNode *Node) {
  if (Intermediate.count(Node)) {
    return Intermediate[Node];
  } else if (DFGConstNode *Const = dyn_cast<DFGConstNode>(Node)) {
    VerilogSignal *In = Parent->CreateTempWireFor(Node, "op_const");
    Parent->addAssignWire(In,
      new VerilogConst(VR_Hexa, Const->getZExtValue(), In->getVectorWidthValue())
    );
    Intermediate[Node] = In;
    Tend[Node] = 0;
    return In;
  } else {
    VerilogSignal *In = Parent->CreateTempWireFor(Node, "op_input");
    Input[Node] = Intermediate[Node] = In;
    Tend[Node] = 0;
    return In;
  }
}

VerilogSignal *DPOperation::CreateUnaryOpResult(DFGUnaryOpNode *UO) {
  VerilogSignal *Operand = getSignalOf(UO->getOperand());
  VerilogSignal *Result = NULL;

  switch (UO->getOpcode()) {
    case DFGUnaryOpNode::DUO_Not: {
      Result = Parent->CreateTempWireFor(UO, "op_result");
      assert(Operand->getVectorWidth() == Result->getVectorWidth());
      // assign Result = ~Operand;
      Parent->addAssignWire(Result, CreateNotOf(Operand));
      return Result;
    }
    case DFGUnaryOpNode::DUO_ReinterpretCast: {
      return Operand;
    }
    case DFGUnaryOpNode::DUO_IntCast: {
      Result = Parent->CreateTempWireFor(UO, "op_result");
      bool FromSigned = UO->getOperand()->getType()->isSignedIntegerType();
      Parent->addAssignWire(Result,
        ConvertWidthOf(Operand, Result->getVectorWidthValue(), FromSigned)
      );
      return Result;
    }
    case DFGUnaryOpNode::DUO_Abs: {
      if (UO->getOperand()->getType()->isUnsignedIntegerType()) {
        return Operand;
      }
      break;
    }
    default: break;
  }

  if (VerilogModuleInstance *Impl = Parent->CreateIPInstance(UO, "op_impl")) {
    Result = Parent->CreateTempWireFor(UO, "op_result");
    Impl->addArgument("clk", Parent->CreateClockRef());
    Impl->addArgument("rstn", Parent->CreateRstnRef());
    Impl->addArgument("enable", new VerilogSignalRef(Enable));
    Impl->addArgument("a", new VerilogSignalRef(Operand));
    Impl->addArgument("q", new VerilogSignalRef(Result));
  }
  return Result;
}

VerilogSignal *DPOperation::CreateBinaryOpResult(DFGBinaryOpNode *BO) {
  VerilogSignal *LHS = getSignalOf(BO->getLHS());
  VerilogSignal *RHS = getSignalOf(BO->getRHS());
  VerilogSignal *Result = NULL;

  switch (BO->getOpcode()) {
    case DFGBinaryOpNode::DBO_And: {
      Result = Parent->CreateTempWireFor(BO, "op_result");
      assert(LHS->getVectorWidth() == Result->getVectorWidth() &&
             RHS->getVectorWidth() == Result->getVectorWidth());
      // assign Result = LHS & RHS;
      Parent->addAssignWire(Result,
        new VerilogBinaryOperator(VBO_AND, new VerilogSignalRef(LHS),
                                  new VerilogSignalRef(RHS))
      );
      return Result;
    }
    case DFGBinaryOpNode::DBO_Xor: {
      Result = Parent->CreateTempWireFor(BO, "op_result");
      assert(LHS->getVectorWidth() == Result->getVectorWidth() &&
             RHS->getVectorWidth() == Result->getVectorWidth());
      // assign Result = LHS ^ RHS;
      Parent->addAssignWire(Result,
        new VerilogBinaryOperator(VBO_XOR, new VerilogSignalRef(LHS),
                                  new VerilogSignalRef(RHS))
      );
      return Result;
    }
    case DFGBinaryOpNode::DBO_Or: {
      Result = Parent->CreateTempWireFor(BO, "op_result");
      assert(LHS->getVectorWidth() == Result->getVectorWidth() &&
             RHS->getVectorWidth() == Result->getVectorWidth());
      // assign Result = LHS | RHS;
      Parent->addAssignWire(Result,
        new VerilogBinaryOperator(VBO_OR, new VerilogSignalRef(LHS),
                                  new VerilogSignalRef(RHS))
      );
      return Result;
    }
    default: break;
  }

  if (VerilogModuleInstance *Impl = Parent->CreateIPInstance(BO, "op_impl")) {
    Result = Parent->CreateTempWireFor(BO, "op_result");
    Impl->addArgument("clk", Parent->CreateClockRef());
    Impl->addArgument("rstn", Parent->CreateRstnRef());
    Impl->addArgument("enable", new VerilogSignalRef(Enable));
    Impl->addArgument("a", new VerilogSignalRef(LHS));
    Impl->addArgument("b", new VerilogSignalRef(RHS));
    Impl->addArgument("q", new VerilogSignalRef(Result));
  }
  return Result;
}

VerilogSignal *DPOperation::CreateTernaryOpResult(DFGTernaryOpNode *TO) {
  VerilogSignal *Operand0 = getSignalOf(TO->getOperand0());
  VerilogSignal *Operand1 = getSignalOf(TO->getOperand1());
  VerilogSignal *Operand2 = getSignalOf(TO->getOperand2());
  VerilogSignal *Result = NULL;

  switch (TO->getOpcode()) {
    case DFGTernaryOpNode::DTO_Conditional: {
      Result = Parent->CreateTempRegFor(TO, "op_result");
      assert(Operand1->getVectorWidth() == Result->getVectorWidth() &&
             Operand2->getVectorWidth() == Result->getVectorWidth());
      // Result <= Operand0 ? Operand1 : Operand2;
      Parent->addAssignRegOnEnable(Enable, Result,
        new VerilogConditionalOperator(new VerilogSignalRef(Operand0),
                                       new VerilogSignalRef(Operand1),
                                       new VerilogSignalRef(Operand2))
      );
      return Result;
    }
    default: break;
  }

  if (VerilogModuleInstance *Impl = Parent->CreateIPInstance(TO, "op_impl")) {
    Result = Parent->CreateTempWireFor(TO, "op_result");
    Impl->addArgument("clk", Parent->CreateClockRef());
    Impl->addArgument("rstn", Parent->CreateRstnRef());
    Impl->addArgument("enable", new VerilogSignalRef(Enable));
    Impl->addArgument("a", new VerilogSignalRef(Operand0));
    Impl->addArgument("b", new VerilogSignalRef(Operand1));
    Impl->addArgument("c", new VerilogSignalRef(Operand2));
    Impl->addArgument("q", new VerilogSignalRef(Result));
  }
  return Result;
}

VerilogSignal *DPOperation::CreateRangeOpResult(DFGRangeOpNode *RO) {
  VerilogSignal *Operand = getSignalOf(RO->getOperand());
  VerilogSignal *Result = Parent->CreateTempWireFor(RO, "op_result");
  Parent->addAssignWire(Result,
    new VerilogSignalRef(Operand, VerilogParamConst(RO->getUpperIndex()),
                         VerilogParamConst(RO->getLowerIndex()))
  );
  return Result;
}

VerilogSignal *DPOperation::CreateVariableRangeOpResult(
    DFGVariableRangeOpNode *VRO) {
  VerilogSignal *Value = getSignalOf(VRO->getValue());
  VerilogSignal *Offset = getSignalOf(VRO->getOffset());
  VerilogSignal *Result = Parent->CreateTempRegFor(VRO, "op_result");
  Parent->addAssignRegOnEnable(Enable, Result,
    new VerilogPartSelect(Value, new VerilogSignalRef(Offset),
                          VerilogParamConst(VRO->getWidth()))
  );
  return Result;
}

VerilogSignal *DPOperation::CreateConcatOpResult(DFGConcatOpNode *CO) {
  VerilogSignal *Result = Parent->CreateTempWireFor(CO, "op_result");
  SmallVector<VerilogExpr*, 16> OperandExprs;
  for (int Index = CO->getNumOperands() - 1; Index >= 0; --Index) {
    VerilogSignal *Operand = getSignalOf(CO->getOperand(Index));
    OperandExprs.push_back(new VerilogSignalRef(Operand));
  }
  Parent->addAssignWire(Result, new VerilogConcat(OperandExprs));
  return Result;
}

VerilogSignal *DPOperation::CreateSubstituteOpResult(DFGSubstituteOpNode *SO) {
  VerilogSignal *Value = getSignalOf(SO->getValue());
  VerilogSignal *Offset = getSignalOf(SO->getOffset());
  VerilogSignal *NewValue = getSignalOf(SO->getNewValue());
  VerilogSignal *Result = Parent->CreateTempRegFor(SO, "op_result");
  unsigned EntireWidth = Value->getVectorWidthValue();
  unsigned SubWidth = SO->getWidth();
  assert(NewValue->getVectorWidthValue() == SubWidth);
  assert(SubWidth < EntireWidth);
  unsigned ComplWidth = EntireWidth - SubWidth;
  std::string ComplZero = std::string(ComplWidth, '0');
  std::string SubOne = std::string(SubWidth, '1');
  // Mask = ~(EntireWidth'b00...0011...11 << Offset)
  VerilogExpr *Mask = CreateNotOf(CreateLShlOf(
      new VerilogConst(VR_Binary, ComplZero + SubOne, EntireWidth),
      Offset));
  // Update = ({ComplWidth'b00...00, NewValue} << Offset)
  VerilogExpr *Update = CreateLShlOf(
      new VerilogConcat(new VerilogConst(VR_Binary, ComplZero, ComplWidth),
                        new VerilogSignalRef(NewValue)),
      Offset);
  // Result <= (Value & Mask) | Update;
  Parent->addAssignRegOnEnable(Enable, Result,
    CreateOrOf(CreateAndOf(Value, Mask), Update)
  );
  return Result;
}

VerilogSignal *DPOperation::CreateShiftRegisterResult(
    DFGShiftRegisterNode *SR) {
  VerilogSignal *Operand = getSignalOf(SR->getPredecessor());
  VerilogSignal *Result = Operand;
  for (unsigned i = 0; i < SR->getSize(); ++i) {
    Operand = Result;
    Result = Parent->CreateTempRegFor(SR, "op_shift");
    Parent->addAssignRegOnEnable(Enable, Result, Operand);
  }
  return Result;
}

/*
 * DPHandshakingUnit: A handshaking unit
 */

DPHandshakingUnit::DPHandshakingUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                                     DPOperation *Op)
  : Ctx(ctx), Parent(parent), Mem(NULL), Lock(NULL), IsEntry(false),
    IsExit(false), IsFinalized(false) {
  ValidIn = Parent->CreateTempWire("hs_valid_in");
  ValidOut = Parent->CreateTempWire("hs_valid_out");
  WaitIn = Parent->CreateTempWire("hs_wait_in");
  WaitOut = Parent->CreateTempWire("hs_wait_out");
  ValidBuffer = Parent->CreateTempReg("hs_valid_buffer");
  MakeFixedLatencyBufferedUnit(Op->getLatency());

  // assign enable = ~wait_in;
  Parent->addAssignWire(Op->enable(), CreateNotOf(WaitIn));

  for (DPOperation::port_iterator I = Op->input_begin(),
                                  IEnd = Op->input_end();
       I != IEnd; ++I) {
    assert(!Input.count(I->first));
    Input[I->first] = I->second;
  }
  for (DPOperation::port_iterator O = Op->output_begin(),
                                  OEnd = Op->output_end();
       O != OEnd; ++O) {
    CreateTempWireOutput(O->first);
    AssignBufferedOutput(O->first, O->second);
  }
}

DPHandshakingUnit::DPHandshakingUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                                     DFGQueueNode *Q)
  : Ctx(ctx), Parent(parent), Mem(NULL), Lock(NULL), IsFinalized(false) {
  ValidIn = Parent->CreateTempWire("hs_valid_in");
  ValidOut = Parent->CreateTempWire("hs_valid_out");
  WaitIn = Parent->CreateTempWire("hs_wait_in");
  WaitOut = Parent->CreateTempWire("hs_wait_out");
  ValidBuffer = NULL;

  if (Q->getType().isNull()) {
    Parent->InsertDatalessFIFOQueue(Q->getSize(), ValidIn, WaitIn, ValidOut, WaitOut);

  } else {
    VerilogSignal *In = GetOrCreateTempWireInput(Q->getPredecessor());
    VerilogSignal *Out = CreateTempWireOutput(Q);
    assert(In->getVectorWidth() == Out->getVectorWidth());
    Parent->InsertFIFOQueue("hs_queue", Out->getVectorWidth(), Q->getSize(), In,
                            ValidIn, WaitIn, Out, ValidOut, WaitOut);
  }
}

DPHandshakingUnit::DPHandshakingUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                                     DFGLoadNode *Load)
  : Ctx(ctx), Parent(parent), Mem(NULL), Lock(NULL), IsEntry(false),
    IsExit(false), IsFinalized(false) {
  ValidIn = Parent->CreateTempWire("hs_valid_in");
  ValidOut = Parent->CreateTempWire("hs_valid_out");
  WaitIn = Parent->CreateTempWire("hs_wait_in");
  WaitOut = Parent->CreateTempWire("hs_wait_out");
  ValidBuffer = NULL;

  Mem = new HAMMReadOnlyInterface(Parent, Parent->AcquireNameWithPrefix("dp_mem"),
                                  Ctx.getPhysicalAddressWidthFor(Load),
                                  Ctx.getTypeSize(Load));

  Parent->addAssignWire(Mem->address(), CreateMemoryAddress(Load));
  Parent->addAssignWire(Mem->read(), ValidIn);
  VerilogSignal *Out = CreateTempWireOutput(Load);
  Parent->addAssignWire(Out, Mem->readdata());
  Parent->addAssignWire(ValidOut, Mem->readdatavalid());
  Parent->addAssignWire(WaitIn, Mem->waitrequest());
  Parent->addAssignWire(Mem->waitresponse(), WaitOut);

  Mem->AnnotateAddressSpace(Load->getAddressSpace());
  Mem->AnnotateAliasGroup(Load->getAliasGroup());
}

DPHandshakingUnit::DPHandshakingUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                                     DFGStoreNode *Store)
  : Ctx(ctx), Parent(parent), Mem(NULL), Lock(NULL), IsEntry(false),
    IsExit(false), IsFinalized(false) {
  ValidIn = Parent->CreateTempWire("hs_valid_in");
  ValidOut = Parent->CreateTempWire("hs_valid_out");
  WaitIn = Parent->CreateTempWire("hs_wait_in");
  WaitOut = Parent->CreateTempWire("hs_wait_out");
  ValidBuffer = NULL;

  unsigned StoreWidth = (unsigned)Ctx.getTypeSize(Store->getAccessType());
  Mem = new AMMWriteOnlyInterface(Parent, Parent->AcquireNameWithPrefix("dp_mem"),
                                  Ctx.getPhysicalAddressWidthFor(Store),
                                  StoreWidth);
  VerilogSignal *Valid = Parent->CreateTempWire("hs_store_valid");
  VerilogSignal *Wait = Parent->CreateTempWire("hs_store_wait");

  Parent->addAssignWire(Mem->address(), CreateMemoryAddress(Store));
  VerilogSignal *InValue = GetOrCreateTempWireInput(Store->getValue());
  Parent->addAssignWire(Mem->writedata(), InValue);
  Parent->addAssignWire(Mem->byteenable(),
    new VerilogConst(VR_Binary, std::string(StoreWidth / 8, '1'), StoreWidth / 8)
  );
  Parent->InsertGatekeeper(ValidIn, WaitIn, Mem->write(), Mem->waitrequest(),
                           CreateNotOf(Wait));

  Parent->addAssignWire(Valid,
    CreateAndOf(Mem->write(), CreateNotOf(Mem->waitrequest()))
  );
  Parent->InsertDatalessFIFOQueue(Ctx.getLmax(Store) + 4, Valid, Wait, ValidOut, WaitOut);

  Mem->AnnotateAddressSpace(Store->getAddressSpace());
  Mem->AnnotateAliasGroup(Store->getAliasGroup());
}

DPHandshakingUnit::DPHandshakingUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                                     DFGAtomicNode *Atomic)
  : Ctx(ctx), Parent(parent), Mem(NULL), Lock(NULL), IsEntry(false),
    IsExit(false), IsFinalized(false) {
  ValidIn = Parent->CreateTempWire("hs_valid_in");
  ValidOut = Parent->CreateTempWire("hs_valid_out");
  WaitIn = Parent->CreateTempWire("hs_wait_in");
  WaitOut = Parent->CreateTempWire("hs_wait_out");
  ValidBuffer = NULL;

  unsigned AccessWidth = (unsigned)Ctx.getTypeSize(Atomic);
  assert((AccessWidth & (AccessWidth - 1)) == 0);

  Mem = new HAMMInterface(Parent, Parent->AcquireNameWithPrefix("dp_mem"),
                          Ctx.getPhysicalAddressWidthFor(Atomic),
                          Ctx.getTypeSize(Atomic));
  Mem->AnnotateAddressSpace(Atomic->getAddressSpace());
  Mem->AnnotateAliasGroup(Atomic->getAliasGroup());
  Mem->AnnotateAsAtomic();

  Lock = new BMLInterface(Parent, Parent->AcquireNameWithPrefix("dp_lock"),
                          LOCK_WIDTH);
  Lock->setGroupForMemoryLock(Atomic->getAddressSpace(),
                              Atomic->getAliasGroup());

  VerilogModuleInstance *Impl = NULL;
  if (DFGNullaryAtomicNode *NA = dyn_cast<DFGNullaryAtomicNode>(Atomic)) {
    Impl = Parent->CreateIPInstance(NA, "op_impl");
  } else if (DFGUnaryAtomicNode *UA = dyn_cast<DFGUnaryAtomicNode>(Atomic)) {
    Impl = Parent->CreateIPInstance(UA, "op_impl");
  } else if (DFGBinaryAtomicNode *BA = dyn_cast<DFGBinaryAtomicNode>(Atomic)) {
    Impl = Parent->CreateIPInstance(BA, "op_impl");
  }
  assert(Impl != NULL);
  Impl->addParamArgument("ADDR_WIDTH", VerilogParamConst(Mem->getAddressWidth()));
  Impl->addParamArgument("DATA_WIDTH", VerilogParamConst(Mem->getDataWidth()));
  Impl->addArgument("clk", Parent->CreateClockRef());
  Impl->addArgument("rstn", Parent->CreateRstnRef());
  Impl->addArgument("addr", CreateMemoryAddress(Atomic));
  if (DFGUnaryAtomicNode *UA = dyn_cast<DFGUnaryAtomicNode>(Atomic)) {
    VerilogSignal *InOperand = GetOrCreateTempWireInput(UA->getOperand());
    assert(AccessWidth == InOperand->getVectorWidthValue());
    Impl->addArgument("a", new VerilogSignalRef(InOperand));
  } else if (DFGBinaryAtomicNode *BA = dyn_cast<DFGBinaryAtomicNode>(Atomic)) {
    VerilogSignal *InLHS = GetOrCreateTempWireInput(BA->getLHS());
    VerilogSignal *InRHS = GetOrCreateTempWireInput(BA->getRHS());
    assert(AccessWidth == InLHS->getVectorWidthValue() &&
           AccessWidth == InRHS->getVectorWidthValue());
    Impl->addArgument("a", new VerilogSignalRef(InLHS));
    Impl->addArgument("b", new VerilogSignalRef(InRHS));
  }
  VerilogSignal *Out = CreateTempWireOutput(Atomic);
  assert(AccessWidth == Out->getVectorWidthValue());
  Impl->addArgument("q", new VerilogSignalRef(Out));
  Impl->addArgument("valid_in", new VerilogSignalRef(ValidIn));
  Impl->addArgument("wait_in", new VerilogSignalRef(WaitIn));
  Impl->addArgument("valid_out", new VerilogSignalRef(ValidOut));
  Impl->addArgument("wait_out", new VerilogSignalRef(WaitOut));
  Mem->ConnectToModuleInstanceAsMaster(Impl, "avm");
  Lock->ConnectToModuleInstance(Impl, "bym");
}

DPHandshakingUnit::DPHandshakingUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                                     DFGBarrierNode *Barrier)
  : Ctx(ctx), Parent(parent), Mem(NULL), Lock(NULL), IsEntry(false),
    IsExit(false), IsFinalized(false) {
  ValidIn = Parent->CreateTempWire("hs_valid_in");
  ValidOut = Parent->CreateTempWire("hs_valid_out");
  WaitIn = Parent->CreateTempWire("hs_wait_in");
  WaitOut = Parent->CreateTempWire("hs_wait_out");
  ValidBuffer = NULL;

  VerilogSignal *Valid = Parent->CreateTempWire("hs_barrier_valid");
  VerilogSignal *Wait = Parent->CreateTempWire("hs_barrier_wait");

  VerilogSignal *WorkGroupSize = Parent->getSignal("flat_local_size");
  assert(WorkGroupSize != NULL);

  bool First = true;
  for (DFGNode::succ_iterator S = Barrier->succ_begin(),
                              SEnd = Barrier->succ_end();
       S != SEnd; ++S) {
    DFGScatterNode *Succ = dyn_cast<DFGScatterNode>(*S);
    if (Succ == NULL) continue;
    DFGNode *Pred = Barrier->getPredOf(Succ);
    VerilogSignal *In = GetOrCreateTempWireInput(Pred);
    VerilogSignal *Out = CreateTempWireOutput(Succ);
    assert(In->getVectorWidth() == Out->getVectorWidth());
    Parent->InsertFIFOQueue("hs_barrier_queue", Out->getVectorWidth(),
                            (1 << Ctx.getPortLIDWidth()) * 2, In, ValidIn,
                            (First ? WaitIn : NULL), Out,
                            (First ? Valid : NULL), Wait);
    First = false;
  }

  unsigned CounterWidth = Ctx.getPortLIDWidth() + 2;

  VerilogExpr *Inc = CreateAndOf(ValidIn, CreateNotOf(WaitIn));
  VerilogExpr *Dec = CreateAndOf(Valid, CreateNotOf(Wait));
  VerilogSignal *Count = Parent->CreateCounter("hs_barrier_count", CounterWidth,
                                               Inc, Dec);

  VerilogExpr *WorkGroupArrived = CreateLAndOf(
      CreateNotEqualOf(Count, CreateDecimalZero()),
      new VerilogBinaryOperator(VBO_GE, new VerilogSignalRef(Count),
                                new VerilogSignalRef(WorkGroupSize)));
  VerilogSignal *Permit = Parent->CreateTempReg("hs_barrier_permit",
                                                CounterWidth);
  Parent->addAssignRegStmt(Permit,
    // if (hs_barrier_permit == 0)
    new VerilogIf(CreateEqualOf(Permit, CreateDecimalZero()),
      // begin
      (new VerilogCompound())->addStmt(
        // if (hs_barrier_count != 0 && hs_barrier_count >= work_group_size)
        new VerilogIf(WorkGroupArrived,
          // begin
          (new VerilogCompound())->addStmt(
            // hs_barrier_permit <= work_group_size
            CreateAssignOf(Permit, WorkGroupSize)
          )
          // end
        )
      ),
      // end
      // else begin
      (new VerilogCompound())->addStmt(
        // if (valid_out & ~wait_out)
        new VerilogIf(CreateAndOf(ValidOut, CreateNotOf(WaitOut)),
          // begin
          (new VerilogCompound())->addStmt(
            // hs_barrier_permit <= hs_barrier_permit - 1
            CreateAssignOf(Permit, CreateSubOf(Permit, CreateDecimalOne()))
          )
          // end
        )
      )
      // end
    )
  );
  Parent->InsertGatekeeper(Valid, Wait, ValidOut, WaitOut,
                           CreateNotEqualOf(Permit, CreateDecimalZero()));
}

DPHandshakingUnit::DPHandshakingUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                                     DFGSourceNode *Source,
                                     const std::string &BBName)
  : Ctx(ctx), Parent(parent), Mem(NULL), Lock(NULL), IsEntry(true),
    IsExit(false), IsFinalized(false) {
  ValidIn = Parent->CreateWire(SP_None, BBName + "_valid_in");
  ValidOut = Parent->CreateTempWire("bb_entry_valid_out");
  WaitIn = Parent->CreateWire(SP_None, BBName + "_wait_in");
  WaitOut = Parent->CreateTempWire("bb_entry_wait_out");
  ValidBuffer = Parent->CreateTempReg("hs_valid_buffer");
  MakeFixedLatencyBufferedUnit(1);

  for (DFGNode::succ_iterator S = Source->succ_begin(),
                              SEnd = Source->succ_end();
       S != SEnd; ++S) {
    DFGScatterNode *LiveIn = dyn_cast<DFGScatterNode>(*S);
    if (LiveIn == NULL) continue;

    std::string LiveInName = BBName + "_live_in_" +
                             NormalizedName(Source->getVariableOf(LiveIn));

    VerilogSignal *In = Parent->CreateTempWireFor(LiveIn, LiveInName);
    assert(!Input.count(LiveIn));
    Input[LiveIn] = In;

    VerilogSignal *Out = Parent->CreateTempRegFor(LiveIn, "hs_value");
    Parent->addAssignRegOnEnable(CreateNotOf(WaitIn), Out, In);

    CreateTempWireOutput(LiveIn);
    AssignBufferedOutput(LiveIn, Out);
  }
}

DPHandshakingUnit::DPHandshakingUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                                     DFGSinkNode *Sink,
                                     const std::string &BBName)
  : Ctx(ctx), Parent(parent), Mem(NULL), Lock(NULL), IsEntry(false),
    IsExit(true), IsFinalized(false) {
  ValidIn = Parent->CreateTempWire("bb_exit_valid_in");
  ValidOut = Parent->CreateWire(SP_None, BBName + "_valid_out");
  WaitIn = Parent->CreateTempWire("bb_exit_wait_in");
  WaitOut = Parent->CreateWire(SP_None, BBName + "_wait_out");
  ValidBuffer = Parent->CreateTempReg("hs_valid_buffer");
  MakeFixedLatencyBufferedUnit(1);

  for (unsigned Index = 0, NumLiveOuts = Sink->getNumLiveOuts();
       Index != NumLiveOuts; ++Index) {
    DFGNode *LiveOut = Sink->getLiveOut(Index);
    // Do not produce the same value twice
    if (Output.count(LiveOut)) {
      continue;
    }

    std::string LiveOutName;
    if (IndexedVarDecl *LiveOutDecl = Sink->getLiveOutDecl(Index)) {
      LiveOutName = BBName + "_live_out_" + NormalizedName(LiveOutDecl);
    } else {
      LiveOutName = BBName + "_cond_out";
    }

    VerilogSignal *In = GetOrCreateTempWireInput(LiveOut);

    VerilogSignal *Out = Parent->CreateTempRegFor(LiveOut, "hs_value");
    Parent->addAssignRegOnEnable(CreateNotOf(WaitIn), Out, In);

    assert(!Output.count(LiveOut));
    Output[LiveOut] = Parent->CreateTempWireFor(LiveOut, LiveOutName);
    AssignBufferedOutput(LiveOut, Out);
  }
}

void DPHandshakingUnit::addPredecessor(DPHandshakingUnit *Pred) {
  assert(!IsFinalized);
  if (!std::count(pred_begin(), pred_end(), Pred)) {
    Preds.push_back(Pred);
    assert(WaitIn != NULL);
    std::string PerPredWaitInName = WaitIn->getNameAsString() + "_to";
    PerPredWaitIn[Pred] = Parent->CreateTempWire(PerPredWaitInName);
  }
}

void DPHandshakingUnit::addSuccessor(DPHandshakingUnit *Succ) {
  assert(!IsFinalized);
  if (!std::count(succ_begin(), succ_end(), Succ)) {
    Succs.push_back(Succ);
    assert(ValidOut != NULL);
    std::string PerSuccValidOutName = ValidOut->getNameAsString() + "_to";
    PerSuccValidOut[Succ] = Parent->CreateTempWire(PerSuccValidOutName);
  }
}

void DPHandshakingUnit::Finalize() {
  assert(!IsFinalized);
  IsFinalized = true;

  if (!Preds.empty()) {
    for (port_iterator I = input_begin(), IEnd = input_end(); I != IEnd; ++I) {
      DFGNode *InNode = I->first;
      VerilogSignal *In = I->second;
      Parent->addAssignWire(In, FindPredecessorOutput(InNode));
    }
  }

  if (Preds.empty()) {
    if (!IsEntry) {
      // assign valid_in = 1'b1;
      Parent->addAssignWire(ValidIn, CreateOne());
    }

  } else if (Preds.size() == 1) {
    assert(ValidIn != NULL && WaitIn != NULL);
    DPHandshakingUnit *Pred = Preds[0];
    // assign valid_in = valid_out1;
    Parent->addAssignWire(ValidIn, Pred->valid_out_of(this));
    // assign wait_in1 = wait_in;
    Parent->addAssignWire(wait_in_of(Pred), WaitIn);

  } else {
    assert(ValidIn != NULL && WaitIn != NULL);

    VerilogExpr *ValidInExpr = NULL;
    for (const_pred_iterator P = pred_begin(), PEnd = pred_end();
         P != PEnd; ++P) {
      DPHandshakingUnit *Pred = *P;
      // assign valid_in = valid_out1 & valid_out2 & ...;
      VerilogSignal *ValidOutX = Pred->valid_out_of(this);
      if (ValidInExpr == NULL) {
        ValidInExpr = new VerilogSignalRef(ValidOutX);
      } else {
        ValidInExpr = CreateAndOf(ValidInExpr, ValidOutX);
      }
      // assign wait_in1 = wait_in | (~valid_in & valid_out1);
      Parent->addAssignWire(wait_in_of(Pred),
        CreateOrOf(WaitIn, CreateAndOf(CreateNotOf(ValidIn), ValidOutX))
      );
    }
    Parent->addAssignWire(ValidIn, ValidInExpr);
  }

  if (Succs.empty()) {
    if (!IsExit) {
      // assign wait_out = 1'b0;
      Parent->addAssignWire(WaitOut, CreateZero());
    }

  } else if (Succs.size() == 1) {
    assert(ValidOut != NULL && WaitOut != NULL);
    DPHandshakingUnit *Succ = Succs[0];
    // assign wait_out = wait_in1;
    Parent->addAssignWire(WaitOut, Succ->wait_in_of(this));
    // assign valid_out1 = valid_out;
    Parent->addAssignWire(valid_out_of(Succ), ValidOut);

  } else {
    assert(ValidOut != NULL && WaitOut != NULL);

    VerilogExpr *WaitOutExpr = NULL;
    for (const_succ_iterator S = succ_begin(), SEnd = succ_end();
         S != SEnd; ++S) {
      DPHandshakingUnit *Succ = *S;
      // assign wait_out = (valid_out1 & wait_in1) | (valid_out2 & wait_in2) | ...;
      VerilogSignal *ValidInX = valid_out_of(Succ);
      VerilogSignal *WaitInX = Succ->wait_in_of(this);
      if (WaitOutExpr == NULL) {
        WaitOutExpr = CreateAndOf(ValidInX, WaitInX);
      } else {
        WaitOutExpr = CreateOrOf(WaitOutExpr, CreateAndOf(ValidInX, WaitInX));
      }
      /* assign valid_out1 = valid_out & (~block1);
       * always @(posedge clk) begin
       *   block1 <= valid_out & wait_out & (~wait_in1 | block1);
       * end
       */
      VerilogSignal *BlockX = Parent->CreateTempReg("hs_block_from");
      Parent->addAssignReg(BlockX,
        CreateAndOf(CreateAndOf(ValidOut, WaitOut),
                    CreateOrOf(CreateNotOf(WaitInX), BlockX))
      );
      Parent->addAssignWire(valid_out_of(Succ),
        CreateAndOf(ValidOut, CreateNotOf(BlockX))
      );
    }
    Parent->addAssignWire(WaitOut, WaitOutExpr);
  }
}

VerilogSignal *DPHandshakingUnit::GetOrCreateTempWireInput(DFGNode *Node) {
  if (DFGConstNode *Const = dyn_cast<DFGConstNode>(Node)) {
    VerilogSignal *In = Parent->CreateTempWireFor(Node, "hs_const");
    Parent->addAssignWire(In,
      new VerilogConst(VR_Hexa, Const->getZExtValue(), In->getVectorWidthValue())
    );
    return In;
  }
  if (!Input.count(Node)) {
    Input[Node] = Parent->CreateTempWireFor(Node, "hs_in");
  }
  return Input[Node];
}

VerilogSignal *DPHandshakingUnit::CreateTempWireOutput(DFGNode *Node) {
  assert(!Output.count(Node));
  Output[Node] = Parent->CreateTempWireFor(Node, "hs_out");
  return Output[Node];
}

VerilogSignal *DPHandshakingUnit::FindPredecessorOutput(DFGNode *Node) {
  VerilogSignal *Result = NULL;
  for (const_pred_iterator P = pred_begin(), PEnd = pred_end();
       P != PEnd; ++P) {
    DPHandshakingUnit *Pred = *P;
    if (VerilogSignal *O = Pred->getOutput(Node)) {
      assert(Result == NULL && "output conflict");
      Result = O;
    }
  }
  assert(Result != NULL && "no matching output");
  return Result;
}

VerilogSignal *DPHandshakingUnit::FindSuccessorInput(DFGNode *Node) {
  VerilogSignal *Result = NULL;
  for (const_succ_iterator S = succ_begin(), SEnd = succ_end();
       S != SEnd; ++S) {
    DPHandshakingUnit *Succ = *S;
    if (VerilogSignal *I = Succ->getInput(Node)) {
      assert(Result == NULL && "input conflict");
      Result = I;
    }
  }
  assert(Result != NULL && "no matching input");
  return Result;
}

VerilogExpr *DPHandshakingUnit::CreateMemoryAddress(DFGMemoryAccessNode *Access) {
  VerilogSignal *InAddr = GetOrCreateTempWireInput(Access->getAddress());
  unsigned VAWidth = Ctx.getVirtualAddressWidthFor(Access);
  VerilogExpr *VA = ConvertWidthOf(InAddr, VAWidth);
  VerilogExpr *PA = VA;
  if (Access->getAddressSpace() == LangAS::opencl_local) {
    unsigned PAWidth = Ctx.getPhysicalAddressWidthFor(Access);
    if (PAWidth > VAWidth) {
      VerilogSignal *GID = GetOrCreateTempWireInput(Access->getAdditionalPredecessor());
      PA = new VerilogConcat(ConvertWidthOf(GID, PAWidth - VAWidth), VA);
    }
  }
  return PA;
}

void DPHandshakingUnit::MakeFixedLatencyBufferedUnit(unsigned Latency) {
  assert(ValidIn != NULL && ValidOut != NULL && WaitIn != NULL &&
         WaitOut != NULL && ValidBuffer != NULL);

  /* always @(posedge clk) begin
   *   if (~wait_in) begin
   *     valid_1 <= valid_in;
   *     valid_2 <= valid_1;
   *     ...
   *     valid_l <= valid_{l-1};
   *   end
   *
   *   if (wait_out & ~valid_buffer) begin
   *     valid_buffer <= valid_l;
   *   end
   *   else if (~wait_out) begin
   *     valid_buffer <= 1'b0;
   *   end
   * end
   */
  VerilogSignal *ValidX = ValidIn;
  for (unsigned i = 0; i < Latency; ++i) {
    VerilogSignal *NewValidX = Parent->CreateTempReg("hs_valid");
    Parent->addAssignRegOnEnable(CreateNotOf(WaitIn), NewValidX, ValidX);
    ValidX = NewValidX;
  }

  Parent->addAssignRegStmt(ValidBuffer,
    // if (wait_out & ~valid_buffer)
    new VerilogIf(CreateAndOf(WaitOut, CreateNotOf(ValidBuffer)),
      // begin
      (new VerilogCompound())->addStmt(
        // valid_buffer <= valid_l;
        CreateAssignOf(ValidBuffer, ValidX)
      ),
      // end
      // else if (~wait_out)
      new VerilogIf(CreateNotOf(WaitOut),
        // begin
        (new VerilogCompound())->addStmt(
          // valid_buffer <= 1'b0;
          CreateAssignOf(ValidBuffer, CreateZero())
        )
        // end
      )
    )
  );

  /* assign valid_out = valid_buffer | valid_l;
   * assign wait_in = valid_buffer & valid_l;
   */
  Parent->addAssignWire(ValidOut, CreateOrOf(ValidBuffer, ValidX));
  Parent->addAssignWire(WaitIn, CreateAndOf(ValidBuffer, ValidX));
}

void DPHandshakingUnit::AssignBufferedOutput(DFGNode *Node,
                                             VerilogSignal *Value) {
  assert(ValidBuffer != NULL);
  VerilogSignal *Buffer = Parent->CreateTempRegFor(Node, "hs_buffer");
  Parent->addAssignRegStmt(Buffer,
    // if (wait_out & ~valid_buffer)
    new VerilogIf(CreateAndOf(WaitOut, CreateNotOf(ValidBuffer)),
      // begin
      (new VerilogCompound())->addStmt(
        // buffer <= value;
        CreateAssignOf(Buffer, Value)
      )
      // end
    )
  );
  // assign out = valid_buffer ? buffer : value;
  assert(Output.count(Node));
  Parent->addAssignWire(Output[Node],
    CreateConditionalOf(ValidBuffer, Buffer, Value)
  );
}

std::string DPHandshakingUnit::NormalizedName(IndexedVarDecl *Var) {
  assert(Var != NULL);
  std::string name = Var->getDecl()->getName().str() + "_" +
                     llvm::utostr_32(Var->getIndex());
  size_t pos = 0;
  while ((pos = name.find(".", pos)) != std::string::npos) {
    name.replace(pos, 1, "_");
    pos += 1;
  }
  return name;
}

/*
 * DPBasicBlock: A basic block unit (= a set of handshaking unit)
 */

DPBasicBlockUnit::DPBasicBlockUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                                   DataflowGraph *graph)
  : Ctx(ctx), Parent(parent), Graph(graph), OutputCond(NULL) {
  Name = "bb" + llvm::utostr_32(Graph->getEntryBlock()->getBlockID());

  for (DataflowGraph::iterator I = Graph->begin(), E = Graph->end();
       I != E; ++I) {
    if (DFGPlatformConstNode *Const = dyn_cast<DFGPlatformConstNode>(*I)) {
      llvm_unreachable("not implemented yet");
    }
    if (DFGAddrOfNode *AddrOf = dyn_cast<DFGAddrOfNode>(*I)) {
      AddrOf->Resolve(Ctx.getAddressOf(AddrOf->getVariable()));
    }
  }

  DFGSourceNode *Source = Graph->getSource();
  DFGSinkNode *Sink = Graph->getSink();

  Entry = new DPHandshakingUnit(Ctx, Parent, Source, Name);
  Exit = new DPHandshakingUnit(Ctx, Parent, Sink, Name);
  Units.insert(Entry);
  Units.insert(Exit);

  ValidIn = Entry->valid_in();
  ValidOut = Exit->valid_out();
  WaitIn = Entry->wait_in();
  WaitOut = Exit->wait_out();
  for (DFGNode::succ_iterator S = Source->succ_begin(),
                              SEnd = Source->succ_end();
       S != SEnd; ++S) {
    DFGScatterNode *LiveIn = dyn_cast<DFGScatterNode>(*S);
    if (LiveIn == NULL) continue;
    IndexedVarDecl *Var = Source->getVariableOf(LiveIn);
    VerilogSignal *Value = Entry->getInput(LiveIn);
    assert(Value != NULL);
    assert(!Input.count(Var));
    Input[Var] = Value;
  }
  for (unsigned Index = 0, NumLiveOutVars = Sink->getNumLiveOutVariables();
       Index != NumLiveOutVars; ++Index) {
    DFGNode *LiveOut = Sink->getLiveOutVariable(Index);
    IndexedVarDecl *Var = Sink->getLiveOutVariableDecl(Index);
    VerilogSignal *Value = Exit->getOutput(LiveOut);
    assert(Value != NULL);
    assert(!Output.count(Var));
    Output[Var] = Value;
  }
  if (Sink->hasLiveOutCondition()) {
    DFGNode *LiveOut = Sink->getLiveOutCondition();
    VerilogSignal *Value = Exit->getOutput(LiveOut);
    assert(Value != NULL);
    assert(OutputCond == NULL);
    OutputCond = Value;
  }

  for (DataflowGraph::iterator I = Graph->t_begin(), E = Graph->t_end();
       I != E; ++I) {
    DFGNode *Node = *I;
    DPHandshakingUnit *Impl;
    if (NodeImpl.count(Node)) {
      Impl = NodeImpl[Node];
    } else if (DFGCompoundNode *Compound = Graph->getCompoundOf(Node)) {
      Impl = InstantiateNode(Compound);
      for (DFGCompoundNode::iterator N = Compound->begin(),
                                     NEnd = Compound->end();
           N != NEnd; ++N) {
        NodeImpl[*N] = Impl;
      }
    } else {
      Impl = InstantiateNode(Node);
      NodeImpl[Node] = Impl;
    }

    if (Impl == NULL) {
      continue;
    }

    if (!Units.count(Impl)) {
      Units.insert(Impl);
      if (AMMInterfaceBase *Mem = Impl->getMemoryInterface()) {
        Mems.push_back(Mem);
      }
      if (BMLInterface *Lock = Impl->getLockInterface()) {
        Locks.push_back(Lock);
      }
    }
    for (DFGNode::const_pred_iterator P = Node->pred_begin(),
                                      PEnd = Node->pred_end();
         P != PEnd; ++P) {
      DFGNode *Pred = *P;
      if (isa<DFGConstNode>(Pred)) {
        continue;
      }
      assert(NodeImpl.count(Pred));
      if (NodeImpl[Pred] != Impl) {
        NodeImpl[Pred]->addSuccessor(Impl);
        Impl->addPredecessor(NodeImpl[Pred]);
      }
    }
  }
  assert(Exit->num_preds() > 0);

  for (OrderedDenseSet<DPHandshakingUnit*>::const_iterator I = Units.begin(),
                                                           E = Units.end();
       I != E; ++I) {
    (*I)->Finalize();
  }
}

DPHandshakingUnit *DPBasicBlockUnit::InstantiateNode(DFGNode *Node) {
  switch (Node->getClass()) {
    case DFGNode::DFGSourceClass:
      return Entry;
    case DFGNode::DFGSinkClass:
      return Exit;

    case DFGNode::DFGIntConstClass:
    case DFGNode::DFGFloatConstClass:
    case DFGNode::DFGUndefinedConstClass:
    case DFGNode::DFGPlatformConstClass:
    case DFGNode::DFGAddrOfClass:
      return NULL; // Dummy

    case DFGNode::DFGUnaryOpClass:
    case DFGNode::DFGBinaryOpClass:
    case DFGNode::DFGTernaryOpClass:
    case DFGNode::DFGRangeOpClass:
    case DFGNode::DFGVariableRangeOpClass:
    case DFGNode::DFGConcatOpClass:
    case DFGNode::DFGSubstituteOpClass:
    case DFGNode::DFGShiftRegisterClass: {
      DPOperation *Op = new DPOperation(Ctx, Parent);
      Op->addNode(Node);
      Op->markAsOutput(Node);
      return new DPHandshakingUnit(Ctx, Parent, Op);
    }

    case DFGNode::DFGLoadClass:
      return new DPHandshakingUnit(Ctx, Parent,
                                   static_cast<DFGLoadNode*>(Node));
    case DFGNode::DFGStoreClass:
      return new DPHandshakingUnit(Ctx, Parent,
                                   static_cast<DFGStoreNode*>(Node));
    case DFGNode::DFGNullaryAtomicClass:
    case DFGNode::DFGUnaryAtomicClass:
    case DFGNode::DFGBinaryAtomicClass:
      return new DPHandshakingUnit(Ctx, Parent,
                                   static_cast<DFGAtomicNode*>(Node));

    case DFGNode::DFGFunctionCallClass: {
      llvm_unreachable("not implemented yet");
      return NULL;
    }

    case DFGNode::DFGQueueClass:
      return new DPHandshakingUnit(Ctx, Parent,
                                   static_cast<DFGQueueNode*>(Node));

    case DFGNode::DFGBarrierClass:
      return new DPHandshakingUnit(Ctx, Parent,
                                   static_cast<DFGBarrierNode*>(Node));

    case DFGNode::DFGScatterClass: {
      DFGScatterNode *Scatter = static_cast<DFGScatterNode*>(Node);
      assert(NodeImpl.count(Scatter->getOrigin()));
      return NodeImpl[Scatter->getOrigin()];
    }

    case DFGNode::DFGCompoundClass: {
      return InstantiateCompound(static_cast<DFGCompoundNode*>(Node));
    }

    default:
      llvm_unreachable("invalid node class");
      return NULL;
  }
}

DPHandshakingUnit *DPBasicBlockUnit::InstantiateCompound(
    DFGCompoundNode *Compound) {
  DPOperation *Op = new DPOperation(Ctx, Parent);
  // Traverse nodes in the compound node in a topological order
  for (DataflowGraph::iterator I = Graph->t_begin(), E = Graph->t_end();
       I != E; ++I) {
    DFGNode *Node = *I;
    if (Compound->count(Node)) {
      Op->addNode(Node);
    }
  }
  for (DataflowGraph::iterator I = Graph->t_begin(), E = Graph->t_end();
       I != E; ++I) {
    DFGNode *Node = *I;
    if (!Compound->count(Node)) {
      for (DFGNode::const_pred_iterator P = Node->pred_begin(),
                                        PEnd = Node->pred_end();
           P != PEnd; ++P) {
        if (Compound->count(*P)) {
          Op->markAsOutput(*P);
        }
      }
    }
  }
  return new DPHandshakingUnit(Ctx, Parent, Op);
}

} // namespace Synthesis

} // namespace snu

} // namespace clang
