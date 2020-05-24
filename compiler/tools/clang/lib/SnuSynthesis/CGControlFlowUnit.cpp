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

#include "CGControlFlowUnit.h"
#include "CGBasicBlock.h"
#include "CGCommon.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/Type.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuSupport/OrderedDenseADT.h"
#include "clang/SnuSynthesis/CodeGenerator.h"
#include "clang/SnuSynthesis/ControlFlowConstraint.h"
#include "clang/SnuSynthesis/DataflowGraph.h"
#include "clang/SnuSynthesis/StructuralAnalysis.h"
#include "clang/SnuSynthesis/Verilog.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include <string>

namespace clang {

namespace snu {

namespace Synthesis {

DPControlFlowUnit::DPControlFlowUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                                     ControlTreeNode *Node)
  : Ctx(ctx), Parent(parent), CFConstraint(Node->getControlFlowConstraint()),
    ValidIn(NULL), WaitIn(NULL) {
  NumOutputPorts = (Node->getNumSuccs() > 0 ? Node->getNumSuccs() : 1);
  ValidOut = new VerilogSignal*[NumOutputPorts];
  WaitOut = new VerilogSignal*[NumOutputPorts];
  Output = new PortMapTy[NumOutputPorts];
  for (unsigned Index = 0; Index != NumOutputPorts; ++Index) {
    ValidOut[Index] = NULL;
    WaitOut[Index] = NULL;
  }

  switch (Node->getClass()) {
    case ControlTreeNode::CTBasicBlockClass:
      InstantiateBasicBlock(static_cast<CTBasicBlock*>(Node));
      break;

    case ControlTreeNode::CTSequentialClass:
      InstantiateSequential(static_cast<CTSequential*>(Node));
      break;

    case ControlTreeNode::CTIfThenClass:
      InstantiateIfThen(static_cast<CTIfThen*>(Node));
      break;

    case ControlTreeNode::CTIfThenElseClass:
      InstantiateIfThenElse(static_cast<CTIfThenElse*>(Node));
      break;

    case ControlTreeNode::CTAcyclicClass:
      InstantiateAcyclic(static_cast<CTAcyclic*>(Node));
      break;

    case ControlTreeNode::CTSelfLoopClass:
      InstantiateSelfLoop(static_cast<CTSelfLoop*>(Node));
      break;

    case ControlTreeNode::CTWhileLoopClass:
      InstantiateWhileLoop(static_cast<CTWhileLoop*>(Node));
      break;

    case ControlTreeNode::CTNaturalLoopClass:
      InstantiateNaturalLoop(static_cast<CTNaturalLoop*>(Node));
      break;

    default:
      llvm_unreachable("impossible case");
  }

  assert(ValidIn != NULL && WaitIn != NULL);
  for (unsigned Index = 0; Index < NumOutputPorts; ++Index) {
    assert(ValidOut[Index] != NULL && WaitOut[Index] != NULL);
  }

  if (CFConstraint.hasSingleWorkGroupConstraint()) {
    EnforceSingleWorkGroupConstraint();
  }
  if (CFConstraint.hasWorkItemLimitConstraint()) {
    EnforceWorkItemLimit();
  }
}

void DPControlFlowUnit::InstantiateBasicBlock(CTBasicBlock *BB) {
  DPBasicBlockUnit *BBUnit = new DPBasicBlockUnit(Ctx, Parent, BB->getDFG());

  Input.insert(BBUnit->input_begin(), BBUnit->input_end());
  ValidIn = BBUnit->valid_in();
  WaitIn = BBUnit->wait_in();

  if (NumOutputPorts == 1) {
    Output[0].insert(BBUnit->output_begin(), BBUnit->output_end());
    ValidOut[0] = BBUnit->valid_out();
    WaitOut[0] = BBUnit->wait_out();
  } else {
    assert(NumOutputPorts == 2);
    assert(BBUnit->hasOutputCond());
    Output[0].insert(BBUnit->output_begin(), BBUnit->output_end());
    Output[1].insert(BBUnit->output_begin(), BBUnit->output_end());
    ValidOut[0] = Parent->CreateTempWire("cf_valid_out");
    ValidOut[1] = Parent->CreateTempWire("cf_valid_out");
    WaitOut[0] = Parent->CreateTempWire("cf_wait_out");
    WaitOut[1] = Parent->CreateTempWire("cf_wait_out");
    Parent->addAssignWire(ValidOut[0],
      CreateLAndOf(BBUnit->valid_out(), BBUnit->getOutputCond())
    );
    Parent->addAssignWire(ValidOut[1],
      CreateLAndOf(BBUnit->valid_out(), CreateLNotOf(BBUnit->getOutputCond()))
    );
    Parent->addAssignWire(BBUnit->wait_out(),
      CreateConditionalOf(BBUnit->getOutputCond(), WaitOut[0], WaitOut[1])
    );
  }

  Mems.append(BBUnit->mem_begin(), BBUnit->mem_end());
  Locks.append(BBUnit->lock_begin(), BBUnit->lock_end());
}

void DPControlFlowUnit::InstantiateSequential(CTSequential *Seq) {
  DPControlFlowUnit *First = new DPControlFlowUnit(Ctx, Parent,
                                                   Seq->getFirst());
  DPControlFlowUnit *Second = new DPControlFlowUnit(Ctx, Parent,
                                                    Seq->getSecond());

  assert(First->getNumOutputPorts() == 1 &&
         Second->getNumOutputPorts() == NumOutputPorts);

  InheritOneToOne(First->input_portmap(), First->valid_in(), First->wait_in(),
                  Input, ValidIn, WaitIn);
  ConnectOneToOne(First->output_portmap(0), First->valid_out(0),
                  First->wait_out(0), Second->input_portmap(),
                  Second->valid_in(), Second->wait_in());
  for (unsigned Index = 0; Index != NumOutputPorts; ++Index) {
    InheritOneToOne(Second->output_portmap(Index), Second->valid_out(Index),
                    Second->wait_out(Index), Output[Index], ValidOut[Index],
                    WaitOut[Index]);
  }

  Mems.append(First->mem_begin(), First->mem_end());
  Mems.append(Second->mem_begin(), Second->mem_end());
  Locks.append(First->lock_begin(), First->lock_end());
  Locks.append(Second->lock_begin(), Second->lock_end());
}

void DPControlFlowUnit::InstantiateIfThen(CTIfThen *IfThen) {
  DPControlFlowUnit *If = new DPControlFlowUnit(Ctx, Parent, IfThen->getIf());
  DPControlFlowUnit *Then = new DPControlFlowUnit(Ctx, Parent,
                                                  IfThen->getThen());

  assert(If->getNumOutputPorts() == 2 && Then->getNumOutputPorts() == 1 &&
         NumOutputPorts == 1);

  InheritOneToOne(If->input_portmap(), If->valid_in(), If->wait_in(), Input,
                  ValidIn, WaitIn);
  ConnectOneToOne(If->output_portmap(0), If->valid_out(0), If->wait_out(0),
                  Then->input_portmap(), Then->valid_in(), Then->wait_in());

  CreatePort(Output[0], IfThen->live_out_begin(0), IfThen->live_out_end(0));
  ValidOut[0] = Parent->CreateTempWire("cf_valid_out");
  WaitOut[0] = Parent->CreateTempWire("cf_wait_out");

  VerilogSignal *ThenValidOut = Then->valid_out(0);
  VerilogSignal *ThenWaitOut = Then->wait_out(0);
  VerilogSignal *ElseValidOut = If->valid_out(1);
  VerilogSignal *ElseWaitOut = If->wait_out(1);
  if (CFConstraint.hasWorkGroupOrderingConstraint()) {
    EnforceWorkGroupOrderingConstraint(Then->output_portmap(0), ThenValidOut,
                                       ThenWaitOut, If->output_portmap(1),
                                       ElseValidOut, ElseWaitOut);
  }
  ConnectTwoToOne(Then->output_portmap(0), ThenValidOut, ThenWaitOut,
                  If->output_portmap(1), ElseValidOut, ElseWaitOut, Output[0],
                  ValidOut[0], WaitOut[0], "then", "else");

  Mems.append(If->mem_begin(), If->mem_end());
  Mems.append(Then->mem_begin(), Then->mem_end());
  Locks.append(If->lock_begin(), If->lock_end());
  Locks.append(Then->lock_begin(), Then->lock_end());
}

void DPControlFlowUnit::InstantiateIfThenElse(CTIfThenElse *IfThenElse) {
  DPControlFlowUnit *If = new DPControlFlowUnit(Ctx, Parent,
                                                IfThenElse->getIf());
  DPControlFlowUnit *Then = new DPControlFlowUnit(Ctx, Parent,
                                                  IfThenElse->getThen());
  DPControlFlowUnit *Else = new DPControlFlowUnit(Ctx, Parent,
                                                  IfThenElse->getElse());

  assert(If->getNumOutputPorts() == 2 && Then->getNumOutputPorts() == 1 &&
         Else->getNumOutputPorts() == 1 && NumOutputPorts == 1);

  InheritOneToOne(If->input_portmap(), If->valid_in(), If->wait_in(), Input,
                  ValidIn, WaitIn);
  ConnectOneToOne(If->output_portmap(0), If->valid_out(0), If->wait_out(0),
                  Then->input_portmap(), Then->valid_in(), Then->wait_in());
  ConnectOneToOne(If->output_portmap(1), If->valid_out(1), If->wait_out(1),
                  Else->input_portmap(), Else->valid_in(), Else->wait_in());

  CreatePort(Output[0], IfThenElse->live_out_begin(0),
             IfThenElse->live_out_end(0));
  ValidOut[0] = Parent->CreateTempWire("cf_valid_out");
  WaitOut[0] = Parent->CreateTempWire("cf_wait_out");

  VerilogSignal *ThenValidOut = Then->valid_out(0);
  VerilogSignal *ThenWaitOut = Then->wait_out(0);
  VerilogSignal *ElseValidOut = Else->valid_out(0);
  VerilogSignal *ElseWaitOut = Else->wait_out(0);
  if (CFConstraint.hasWorkGroupOrderingConstraint()) {
    EnforceWorkGroupOrderingConstraint(Then->output_portmap(0), ThenValidOut,
                                       ThenWaitOut, Else->output_portmap(0),
                                       ElseValidOut, ElseWaitOut);
  }
  ConnectTwoToOne(Then->output_portmap(0), ThenValidOut, ThenWaitOut,
                  Else->output_portmap(0), ElseValidOut, ElseWaitOut, Output[0],
                  ValidOut[0], WaitOut[0], "then", "else");

  Mems.append(If->mem_begin(), If->mem_end());
  Mems.append(Then->mem_begin(), Then->mem_end());
  Mems.append(Else->mem_begin(), Else->mem_end());
  Locks.append(If->lock_begin(), If->lock_end());
  Locks.append(Then->lock_begin(), Then->lock_end());
  Locks.append(Else->lock_begin(), Else->lock_end());
}

void DPControlFlowUnit::InstantiateAcyclic(CTAcyclic *Acyclic) {
  llvm::DenseMap<ControlTreeNode*, DPControlFlowUnit*> NodeImpl;
  llvm::DenseMap<ControlTreeNode*, SmallVector<NToOneFromTy, 4> > PredsOfNode;
  llvm::DenseMap<unsigned, SmallVector<NToOneFromTy, 4> > PredsOfSucc;
  for (unsigned NI = 0, NumNodes = Acyclic->getNumNodes();
       NI != NumNodes; ++NI) {
    ControlTreeNode *Node = Acyclic->getNode(NI);
    DPControlFlowUnit *Impl = new DPControlFlowUnit(Ctx, Parent, Node);
    NodeImpl[Node] = Impl;
    Mems.append(Impl->mem_begin(), Impl->mem_end());
    Locks.append(Impl->lock_begin(), Impl->lock_end());
    if (Node == Acyclic->getEntry()) {
      InheritOneToOne(Impl->input_portmap(), Impl->valid_in(), Impl->wait_in(),
                      Input, ValidIn, WaitIn);
    }
    if (Node->isProgramExit()) {
      assert(Acyclic->getNumEdges(NI) == 0);
      assert(Impl->getNumOutputPorts() == 1 && NumOutputPorts == 1);
      InheritOneToOne(Impl->output_portmap(0), Impl->valid_out(0),
                      Impl->wait_out(0), Output[0], ValidOut[0], WaitOut[0]);
    }
    for (unsigned EI = 0, NumEdges = Acyclic->getNumEdges(NI);
         EI != NumEdges; ++EI) {
      NToOneFromTy From(Impl, EI);
      if (Acyclic->isInternalEdge(NI, EI)) {
        PredsOfNode[Acyclic->getInternalEdge(NI, EI)].push_back(From);
      } else {
        PredsOfSucc[Acyclic->getExternalEdge(NI, EI)].push_back(From);
      }
    }
  }
  for (unsigned NI = 0, NumNodes = Acyclic->getNumNodes();
       NI != NumNodes; ++NI) {
    ControlTreeNode *Node = Acyclic->getNode(NI);
    DPControlFlowUnit *Impl = NodeImpl[Node];
    if (PredsOfNode.count(Node)) {
      if (PredsOfNode[Node].size() == 1) {
        NToOneFromTy UniquePred = PredsOfNode[Node][0];
        ConnectOneToOne(UniquePred.output_portmap(), UniquePred.valid_out(),
                        UniquePred.wait_out(), Impl->input_portmap(),
                        Impl->valid_in(), Impl->wait_in());
      } else {
        ConnectNToOne(PredsOfNode[Node], Impl->input_portmap(),
                      Impl->valid_in(), Impl->wait_in());
      }
    }
  }
  for (unsigned SI = 0, NumSuccs = Acyclic->getNumSuccs();
       SI != NumSuccs; ++SI) {
    DataflowGraph *Succ = Acyclic->getSucc(SI);
    if (PredsOfSucc.count(SI)) {
      if (PredsOfSucc[SI].size() == 1) {
        NToOneFromTy UniquePred = PredsOfSucc[SI][0];
        InheritOneToOne(UniquePred.output_portmap(), UniquePred.valid_out(),
                        UniquePred.wait_out(), Output[SI], ValidOut[SI],
                        WaitOut[SI]);
      } else {
        CreatePort(Output[SI], Succ->live_in_begin(), Succ->live_in_end());
        ValidOut[SI] = Parent->CreateTempWire("cf_valid_out");
        WaitOut[SI] = Parent->CreateTempWire("cf_wait_out");
        ConnectNToOne(PredsOfSucc[SI], Output[SI], ValidOut[SI], WaitOut[SI]);
      }
    }
  }
}

void DPControlFlowUnit::InstantiateSelfLoop(CTSelfLoop *Loop) {
  DPControlFlowUnit *Body = new DPControlFlowUnit(Ctx, Parent, Loop->getBody());

  assert(Body->getNumOutputPorts() == 2 && NumOutputPorts == 1);

  CreatePort(Input, Loop->live_in_begin(), Loop->live_in_end());
  ValidIn = Parent->CreateTempWire("cf_valid_in");
  WaitIn = Parent->CreateTempWire("cf_wait_in");

  unsigned BackIndex = Loop->getLoopBackIndex();
  unsigned ExitIndex = Loop->getExitIndex();
  PortMapTy LBOut = Body->output_portmap(BackIndex);
  VerilogSignal *LBValidOut = Body->valid_out(BackIndex);
  VerilogSignal *LBWaitOut = Body->wait_out(BackIndex);
  if (CFConstraint.hasWorkItemLimitConstraint() &&
      CFConstraint.getWorkItemLimitIncrementQueueSize() > 0) {
    InsertWorkItemLimitIncrementQueue(LBOut, LBValidOut, LBWaitOut,
                                      Loop->getBody()->live_in_begin(),
                                      Loop->getBody()->live_in_end());
  }
  ConnectTwoToOne(LBOut, LBValidOut, LBWaitOut, Input, ValidIn, WaitIn,
                  Body->input_portmap(), Body->valid_in(), Body->wait_in(),
                  "back", "new");
  InheritOneToOne(Body->output_portmap(ExitIndex), Body->valid_out(ExitIndex),
                  Body->wait_out(ExitIndex), Output[0], ValidOut[0],
                  WaitOut[0]);

  Mems.append(Body->mem_begin(), Body->mem_end());
  Locks.append(Body->lock_begin(), Body->lock_end());
}

void DPControlFlowUnit::InstantiateWhileLoop(CTWhileLoop *Loop) {
  DPControlFlowUnit *Cond = new DPControlFlowUnit(Ctx, Parent, Loop->getCond());
  DPControlFlowUnit *Body = new DPControlFlowUnit(Ctx, Parent, Loop->getBody());

  assert(Cond->getNumOutputPorts() == 2 && Body->getNumOutputPorts() == 1 &&
         NumOutputPorts == 1);

  CreatePort(Input, Loop->live_in_begin(), Loop->live_in_end());
  ValidIn = Parent->CreateTempWire("cf_valid_in");
  WaitIn = Parent->CreateTempWire("cf_wait_in");

  unsigned BackIndex = Loop->getLoopBackIndex();
  unsigned ExitIndex = Loop->getExitIndex();
  PortMapTy LBOut = Body->output_portmap(0);
  VerilogSignal *LBValidOut = Body->valid_out(0);
  VerilogSignal *LBWaitOut = Body->wait_out(0);
  if (CFConstraint.hasWorkItemLimitConstraint() &&
      CFConstraint.getWorkItemLimitIncrementQueueSize() > 0) {
    InsertWorkItemLimitIncrementQueue(LBOut, LBValidOut, LBWaitOut,
                                      Loop->getCond()->live_in_begin(),
                                      Loop->getCond()->live_in_end());
  }
  ConnectTwoToOne(LBOut, LBValidOut, LBWaitOut, Input, ValidIn, WaitIn,
                  Cond->input_portmap(), Cond->valid_in(), Cond->wait_in(),
                  "back", "new");
  InheritOneToOne(Cond->output_portmap(ExitIndex), Cond->valid_out(ExitIndex),
                  Cond->wait_out(ExitIndex), Output[0], ValidOut[0],
                  WaitOut[0]);
  ConnectOneToOne(Cond->output_portmap(BackIndex), Cond->valid_out(BackIndex),
                  Cond->wait_out(BackIndex), Body->input_portmap(),
                  Body->valid_in(), Body->wait_in());

  Mems.append(Cond->mem_begin(), Cond->mem_end());
  Mems.append(Body->mem_begin(), Body->mem_end());
  Locks.append(Cond->lock_begin(), Cond->lock_end());
  Locks.append(Body->lock_begin(), Body->lock_end());
}

void DPControlFlowUnit::InstantiateNaturalLoop(CTNaturalLoop *Loop) {
  DPControlFlowUnit *Body = new DPControlFlowUnit(Ctx, Parent,
                                                  Loop->getBody());

  assert(Body->getNumOutputPorts() >= 2 &&
         Body->getNumOutputPorts() - 1 == NumOutputPorts);

  CreatePort(Input, Loop->live_in_begin(), Loop->live_in_end());
  ValidIn = Parent->CreateTempWire("cf_valid_in");
  WaitIn = Parent->CreateTempWire("cf_wait_in");

  unsigned BackIndex = Loop->getLoopBackIndex();
  PortMapTy LBOut = Body->output_portmap(BackIndex);
  VerilogSignal *LBValidOut = Body->valid_out(BackIndex);
  VerilogSignal *LBWaitOut = Body->wait_out(BackIndex);
  if (CFConstraint.hasWorkItemLimitConstraint() &&
      CFConstraint.getWorkItemLimitIncrementQueueSize() > 0) {
    InsertWorkItemLimitIncrementQueue(LBOut, LBValidOut, LBWaitOut,
                                      Loop->getBody()->live_in_begin(),
                                      Loop->getBody()->live_in_end());
  }
  ConnectTwoToOne(LBOut, LBValidOut, LBWaitOut, Input, ValidIn, WaitIn,
                  Body->input_portmap(), Body->valid_in(), Body->wait_in(),
                  "back", "new");
  for (unsigned Index = 0; Index != NumOutputPorts; ++Index) {
    unsigned BodyIndex = Index + (Index >= Loop->getLoopBackIndex());
    InheritOneToOne(Body->output_portmap(BodyIndex), Body->valid_out(BodyIndex),
                    Body->wait_out(BodyIndex), Output[Index], ValidOut[Index],
                    WaitOut[Index]);
  }

  Mems.append(Body->mem_begin(), Body->mem_end());
  Locks.append(Body->lock_begin(), Body->lock_end());
}

void DPControlFlowUnit::InheritOneToOne(PortMapTy &From,
                                        VerilogSignal *ValidFrom,
                                        VerilogSignal *WaitFrom,
                                        PortMapTy &To,
                                        VerilogSignal *&ValidTo,
                                        VerilogSignal *&WaitTo) {
  assert(ValidFrom != NULL && WaitFrom != NULL);
  assert(To.empty() && ValidTo == NULL && WaitTo == NULL);
  To.insert(From.begin(), From.end());
  ValidTo = ValidFrom;
  WaitTo = WaitFrom;
}

void DPControlFlowUnit::ConnectOneToOne(PortMapTy &From,
                                        VerilogSignal *ValidFrom,
                                        VerilogSignal *WaitFrom,
                                        PortMapTy &To,
                                        VerilogSignal *ValidTo,
                                        VerilogSignal *WaitTo) {
  assert(ValidFrom != NULL && WaitFrom != NULL);
  assert(ValidTo != NULL && WaitTo != NULL);
  AssignPort(To, From);
  Parent->addAssignWire(ValidTo, ValidFrom);
  Parent->addAssignWire(WaitFrom, WaitTo);
}

void DPControlFlowUnit::ConnectTwoToOne(PortMapTy &From0,
                                        VerilogSignal *ValidFrom0,
                                        VerilogSignal *WaitFrom0,
                                        PortMapTy &From1,
                                        VerilogSignal *ValidFrom1,
                                        VerilogSignal *WaitFrom1,
                                        PortMapTy &To,
                                        VerilogSignal *ValidTo,
                                        VerilogSignal *WaitTo,
                                        std::string From0Prefix,
                                        std::string From1Prefix) {
  assert(ValidFrom0 != NULL && WaitFrom0 != NULL);
  assert(ValidFrom1 != NULL && WaitFrom1 != NULL);
  assert(ValidTo != NULL && WaitTo != NULL);
  assert(From0Prefix != From1Prefix);

  VerilogSignal *Use0 = Parent->CreateTempWire("cf_use_" + From0Prefix);
  VerilogSignal *Use1 = Parent->CreateTempWire("cf_use_" + From1Prefix);
  VerilogSignal *Block0 = Parent->CreateTempReg("cf_block_" + From0Prefix);
  VerilogSignal *Block1 = Parent->CreateTempReg("cf_block_" + From1Prefix);
  /* assign use0 = valid_from0 & ~block0;
   * assign use1 = valid_from1 & ~block1;
   * assign to = (use0 ? from0 : from1);
   * assign valid_to = use0 | use1;
   * assign wait_from0 = wait_to | block0;
   * assign wait_from1 = wait_to | block1 | use0;
   * always @(posedge clk) begin
   *   block0 <= wait_to & ((~valid_from0 & valid_to) | block0);
   *   block1 <= wait_to & ((~valid_from1 & valid_to) | block1);
   * end
   */
  Parent->addAssignWire(Use0, CreateAndOf(ValidFrom0, CreateNotOf(Block0)));
  Parent->addAssignWire(Use1, CreateAndOf(ValidFrom1, CreateNotOf(Block1)));
  AssignPort(To, From0, From1, Use0);
  Parent->addAssignWire(ValidTo, CreateOrOf(Use0, Use1));
  Parent->addAssignWire(WaitFrom0, CreateOrOf(WaitTo, Block0));
  Parent->addAssignWire(WaitFrom1, CreateOrOf(CreateOrOf(WaitTo, Block1), Use0));
  Parent->addAssignReg(Block0,
    CreateAndOf(WaitTo,
                CreateOrOf(CreateAndOf(CreateNotOf(ValidFrom0), ValidTo), Block0))
  );
  Parent->addAssignReg(Block1,
    CreateAndOf(WaitTo,
                CreateOrOf(CreateAndOf(CreateNotOf(ValidFrom1), ValidTo), Block1))
  );
}

void DPControlFlowUnit::ConnectNToOne(ArrayRef<NToOneFromTy> From,
                                      PortMapTy& To, VerilogSignal *ValidTo,
                                      VerilogSignal *WaitTo) {
  assert(From.size() >= 2);
  SmallVector<VerilogSignal*, 16> Use;
  SmallVector<VerilogSignal*, 16> Block;
  for (unsigned X = 0; X != From.size(); X++) {
    Use.push_back(Parent->CreateTempWire("cf_use"));
    Block.push_back(Parent->CreateTempReg("cf_block"));
  }
  for (unsigned X = 0; X != From.size(); X++) {
    VerilogSignal *ValidFrom = From[X].valid_out();
    VerilogSignal *WaitFrom = From[X].wait_out();
    /* assign useX = validX & ~blockX;
     * assign waitX = wait_succ | blockX | use0 | use1 | ... | useX-1;
     * always @(posedge clk) begin
     *   blockX <= wait_succ & ((~validX & valid_succ) | blockX);
     * end
     */
    Parent->addAssignWire(Use[X], CreateAndOf(ValidFrom, CreateNotOf(Block[X])));
    VerilogExpr *WaitXExpr = CreateOrOf(WaitTo, Block[X]);
    for (unsigned Y = 0; Y < X; Y++) {
      WaitXExpr = CreateOrOf(WaitXExpr, Use[Y]);
    }
    Parent->addAssignWire(WaitFrom, WaitXExpr);
    Parent->addAssignReg(Block[X],
      CreateAndOf(WaitTo,
                  CreateOrOf(CreateAndOf(CreateNotOf(ValidFrom), ValidTo), Block[X]))
    );
  }
  /* assign to = (use0 ? from0 : (use1 ? from1 : ...));
   * assign valid_to = use0 | use1 | ...;
   */
  for (port_iterator I = To.begin(), E = To.end(); I != E; ++I) {
    IndexedVarDecl *Var = I->first;
    VerilogSignal *T = I->second;
    VerilogExpr *TExpr = NULL;
    for (int X = From.size() - 1; X >= 0; --X) {
      VerilogSignal *F = GetSignalInPort(From[X].output_portmap(), Var);
      if (TExpr == NULL) {
        TExpr = new VerilogSignalRef(F);
      } else {
        TExpr = CreateConditionalOf(Use[X], F, TExpr);
      }
    }
    Parent->addAssignWire(T, TExpr);
  }
  VerilogExpr *ValidToExpr = NULL;
  for (unsigned X = 0; X != From.size(); X++) {
    if (ValidToExpr == NULL) {
      ValidToExpr = new VerilogSignalRef(Use[X]);
    } else {
      ValidToExpr = CreateOrOf(ValidToExpr, Use[X]);
    }
  }
  Parent->addAssignWire(ValidTo, ValidToExpr);
}

void DPControlFlowUnit::CreatePort(
    PortMapTy &To, ControlTreeNode::const_live_var_iterator begin,
    ControlTreeNode::const_live_var_iterator end) {
  for (ControlTreeNode::const_live_var_iterator I = begin; I != end; ++I) {
    IndexedVarDecl *Var = *I;
    To[Var] = Parent->CreateTempWireFor(Var, "cf_live_var");
  }
}

VerilogSignal *DPControlFlowUnit::GetSignalInPort(PortMapTy &P,
                                                  IndexedVarDecl *Var) {
  if (Var->getIndex() == 0) {
    unsigned DummyWidth = Ctx.getTypeSize(Var);
    VerilogSignal *Dummy = Parent->CreateTempWire("cf_dummy", DummyWidth);
    Parent->addAssignWire(Dummy, CreateZero(DummyWidth));
    return Dummy;
  } else if (P.count(Var)) {
    return P[Var];
  }
  if (WPhiFunction *PF = dyn_cast<WPhiFunction>(Var->getDefinedStmt())) {
    IndexedVarDecl *AliasVar = NULL;
    for (unsigned Index = 0, NumArgs = PF->getNumArgs();
         Index != NumArgs; ++Index) {
      if (P.count(PF->getIndexedArgDecl(Index))) {
        assert(AliasVar == NULL || AliasVar == PF->getIndexedArgDecl(Index));
        AliasVar = PF->getIndexedArgDecl(Index);
      }
    }
    if (AliasVar) {
      if (AliasVar->getIndex() == 0) {
        unsigned DummyWidth = Ctx.getTypeSize(Var);
        VerilogSignal *Dummy = Parent->CreateTempWire("cf_dummy", DummyWidth);
        Parent->addAssignWire(Dummy, CreateZero(DummyWidth));
        return Dummy;
      } else if (P.count(AliasVar)) {
        return P[AliasVar];
      }
    }
  }
  llvm_unreachable("cannt find the variable in the port");
  return NULL;
}

void DPControlFlowUnit::AssignPort(PortMapTy &To, PortMapTy &From) {
  for (port_iterator I = To.begin(), E = To.end(); I != E; ++I) {
    IndexedVarDecl *Var = I->first;
    VerilogSignal *T = I->second;
    VerilogSignal *F = GetSignalInPort(From, Var);
    Parent->addAssignWire(T, F);
  }
}

void DPControlFlowUnit::AssignPort(PortMapTy &To, PortMapTy &From0,
                                   PortMapTy &From1, VerilogSignal *Use0) {
  for (port_iterator I = To.begin(), E = To.end(); I != E; ++I) {
    IndexedVarDecl *Var = I->first;
    VerilogSignal *T = I->second;
    VerilogSignal *F0 = GetSignalInPort(From0, Var);
    VerilogSignal *F1 = GetSignalInPort(From1, Var);
    Parent->addAssignWire(T, CreateConditionalOf(Use0, F0, F1));
  }
}

VerilogExpr *DPControlFlowUnit::GetWorkGroupConstraintKeyInPort(PortMapTy &Port) {
  assert(Port.count(Ctx.getFlatWorkGroupIDVar()));
  unsigned LSB = CFConstraint.getWorkGroupConstraintLSB();
  return new VerilogSignalRef(Port[Ctx.getFlatWorkGroupIDVar()],
                              VerilogParamConst(63), VerilogParamConst(LSB));
}

VerilogSignal *DPControlFlowUnit::CreateWorkItemCounter(unsigned Width) {
  VerilogSignal *Count = Parent->CreateTempReg("cf_count", Width);
  VerilogExpr *CountExpr = new VerilogSignalRef(Count);
  assert(ValidIn != NULL && WaitIn != NULL);
  CountExpr = CreateAddOf(CountExpr, CreateAndOf(ValidIn, CreateNotOf(WaitIn)));
  for (unsigned Index = 0; Index != NumOutputPorts; ++Index) {
    assert(ValidOut[Index] != NULL && WaitOut[Index] != NULL);
    CountExpr = CreateSubOf(CountExpr,
                            CreateAndOf(ValidOut[Index],
                                        CreateNotOf(WaitOut[Index])));
  }
  Parent->addAssignReg(Count, CountExpr);
  return Count;
}

void DPControlFlowUnit::EnforceSingleWorkGroupConstraint() {
  assert(CFConstraint.hasSingleWorkGroupConstraint());
  assert(ValidIn != NULL && WaitIn != NULL);

  unsigned LSB = CFConstraint.getWorkGroupConstraintLSB();
  VerilogSignal *Count = CreateWorkItemCounter(Ctx.getPortLIDWidth() + LSB + 1);
  VerilogSignal *Owner = Parent->CreateTempReg("cf_owner", 64 - LSB);
  VerilogExpr *InputWID = GetWorkGroupConstraintKeyInPort(Input);
  Parent->addAssignReg(Owner,
    CreateConditionalOf(CreateAndOf(ValidIn, CreateNotOf(WaitIn)),
                        InputWID, Owner)
  );
  VerilogSignal *OldValidIn = ValidIn;
  VerilogSignal *OldWaitIn = WaitIn;
  ValidIn = Parent->CreateTempWire("cf_valid_in");
  WaitIn = Parent->CreateTempWire("cf_wait_in");
  Parent->InsertGatekeeper(ValidIn, WaitIn, OldValidIn, OldWaitIn,
    CreateOrOf(CreateEqualOf(Owner, InputWID),
               CreateEqualOf(Count, CreateDecimalZero()))
  );
}

void DPControlFlowUnit::EnforceWorkGroupOrderingConstraint(
    PortMapTy &LeftPort, VerilogSignal *&ValidLeft, VerilogSignal *&WaitLeft,
    PortMapTy &RightPort, VerilogSignal *&ValidRight, VerilogSignal *&WaitRight) {
  assert(CFConstraint.hasWorkGroupOrderingConstraint());
  assert(ValidIn != NULL && WaitIn != NULL);

  unsigned LSB = CFConstraint.getWorkGroupConstraintLSB();
  VerilogSignal *Next = Parent->CreateTempWire("cf_next", 64 - LSB);
  VerilogSignal *NextValid = Parent->CreateTempWire("cf_next_valid");
  VerilogSignal *NextWait = Parent->CreateTempWire("cf_next_wait");
  VerilogSignal *QueueValidIn = Parent->CreateTempWire("cf_valid_in");
  VerilogSignal *QueueWaitIn = Parent->CreateTempWire("cf_wait_in");
  Parent->InsertFIFOQueue("cf_ordering_queue", Next->getVectorWidth(),
                          CFConstraint.getWorkGroupOrderingQueueSize() + 4,
                          GetWorkGroupConstraintKeyInPort(Input), QueueValidIn,
                          QueueWaitIn, Next, NextValid, NextWait);

  VerilogSignal *OldValidIn = ValidIn;
  VerilogSignal *OldWaitIn = WaitIn;
  ValidIn = Parent->CreateTempWire("cf_valid_in");
  WaitIn = Parent->CreateTempWire("cf_wait_in");
  Parent->InsertGatekeeper(ValidIn, WaitIn, OldValidIn, OldWaitIn,
                           CreateNotOf(QueueWaitIn));
  Parent->addAssignWire(QueueValidIn, CreateAndOf(ValidIn, CreateNotOf(WaitIn)));

  VerilogSignal *OldValidLeft = ValidLeft;
  VerilogSignal *OldWaitLeft = WaitLeft;
  VerilogSignal *OldValidRight = ValidRight;
  VerilogSignal *OldWaitRight = WaitRight;
  ValidLeft = Parent->CreateTempWire("cf_valid_out");
  WaitLeft = Parent->CreateTempWire("cf_wait_out");
  ValidRight = Parent->CreateTempWire("cf_valid_out");
  WaitRight = Parent->CreateTempWire("cf_wait_out");
  Parent->InsertGatekeeper(OldValidLeft, OldWaitLeft, ValidLeft, WaitLeft,
    CreateLAndOf(NextValid,
                 CreateEqualOf(GetWorkGroupConstraintKeyInPort(LeftPort), Next))
  );
  Parent->InsertGatekeeper(OldValidRight, OldWaitRight, ValidRight, WaitRight,
    CreateLAndOf(NextValid,
                 CreateEqualOf(GetWorkGroupConstraintKeyInPort(RightPort), Next))
  );

  Parent->addAssignWire(NextWait,
    CreateNotOf(CreateOrOf(CreateAndOf(ValidLeft, CreateNotOf(WaitLeft)),
                           CreateAndOf(ValidRight, CreateNotOf(WaitRight))))
  );
}

void DPControlFlowUnit::EnforceWorkItemLimit() {
  assert(CFConstraint.hasWorkItemLimitConstraint());
  assert(ValidIn != NULL && WaitIn != NULL);

  unsigned Limit = CFConstraint.getWorkItemLimit() +
                   CFConstraint.getWorkItemLimitIncrementQueueSize();
  VerilogSignal *Count = CreateWorkItemCounter(Log2(Limit) + 1);
  VerilogSignal *OldValidIn = ValidIn;
  VerilogSignal *OldWaitIn = WaitIn;
  ValidIn = Parent->CreateTempWire("cf_valid_in");
  WaitIn = Parent->CreateTempWire("cf_wait_in");
  Parent->InsertGatekeeper(ValidIn, WaitIn, OldValidIn, OldWaitIn,
    CreateNotEqualOf(Count, new VerilogConst(VR_Decimal, Limit))
  );
}

void DPControlFlowUnit::InsertWorkItemLimitIncrementQueue(
    PortMapTy &Port, VerilogSignal *&Valid, VerilogSignal *&Wait,
    ControlTreeNode::const_live_var_iterator out_begin,
    ControlTreeNode::const_live_var_iterator out_end) {
  assert(CFConstraint.hasWorkItemLimitConstraint());
  PortMapTy OldPort = Port;
  VerilogSignal *OldValid = Valid;
  VerilogSignal *OldWait = Wait;
  Port.clear();
  CreatePort(Port, out_begin, out_end);
  Valid = Parent->CreateTempWire("cf_valid_out");
  Wait = Parent->CreateTempWire("cf_wait_out");
  bool First = true;
  for (port_iterator I = Port.begin(), E = Port.end(); I != E; ++I) {
    IndexedVarDecl *Var = I->first;
    VerilogSignal *T = I->second;
    VerilogSignal *F = GetSignalInPort(OldPort, Var);
    assert(F->getVectorWidth() == T->getVectorWidth());
    Parent->InsertFIFOQueue("cf_queue", T->getVectorWidth(),
                            CFConstraint.getWorkItemLimitIncrementQueueSize(),
                            F, OldValid, (First ? OldWait : NULL), T,
                            (First ? Valid : NULL), Wait);
    First = false;
  }
}

} // namespace Synthesis

} // namespace snu

} // namespace clang
