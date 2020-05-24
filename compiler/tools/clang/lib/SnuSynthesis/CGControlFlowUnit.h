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

#ifndef LLVM_CLANG_SNU_SYNTHESIS_CGCONTROLFLOWUNIT_H
#define LLVM_CLANG_SNU_SYNTHESIS_CGCONTROLFLOWUNIT_H

#include "CGCommon.h"
#include "clang/AST/ASTContext.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuSupport/OrderedDenseADT.h"
#include "clang/SnuSynthesis/CodeGenerator.h"
#include "clang/SnuSynthesis/StructuralAnalysis.h"
#include "clang/SnuSynthesis/Verilog.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include <string>

namespace clang {

namespace snu {

namespace Synthesis {

class AMMInterfaceBase;
class BMLInterface;

/* DPControlFlowUnit: A control flow unit
 *
 * Input 1 -> [      ] -> Output 1
 * Input 2 -> [      ] -> Output 2
 * Input 3 -> [      ] -> Output 3
 * ValidIn -> [      ] -> ValidOut
 *  WaitIn <- [      ] <- WaitOut
 *            [      ]
 *            [      ] -> Output 1
 *            [      ] -> Output 2
 *            [ Unit ] -> Output 3
 *            [      ] -> ValidOut
 *            [      ] <- WaitOut
 *            [      ]
 *            [      ] <-> Mem 1
 *            [      ] <-> Mem 2
 *            [      ] <-> Mem 3
 *            [      ] <-> Lock 1
 *            [      ] <-> Lock 2
 */

class DPControlFlowUnit {
  CodeGenContext &Ctx;
  ModuleBuilder *Parent;
  const ControlFlowConstraint &CFConstraint;

  unsigned NumOutputPorts;

  VerilogSignal *ValidIn;
  VerilogSignal **ValidOut;
  VerilogSignal *WaitIn;
  VerilogSignal **WaitOut;

  typedef OrderedDenseMap<IndexedVarDecl*, VerilogSignal*> PortMapTy;
  PortMapTy Input;
  PortMapTy *Output;

  SmallVector<AMMInterfaceBase*, 64> Mems;
  SmallVector<BMLInterface*, 64> Locks;

public:
  DPControlFlowUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                    ControlTreeNode *Node);

  unsigned getNumOutputPorts() const { return NumOutputPorts; }

  VerilogSignal *valid_in() const { return ValidIn; }
  VerilogSignal *valid_out(unsigned Index) const {
    assert(ValidOut != NULL && Index < NumOutputPorts);
    return ValidOut[Index];
  }
  VerilogSignal *wait_in() const { return WaitIn; }
  VerilogSignal *wait_out(unsigned Index) const {
    assert(WaitOut != NULL && Index < NumOutputPorts);
    return WaitOut[Index];
  }

  typedef PortMapTy::const_iterator port_iterator;

  port_iterator input_begin() const { return Input.begin(); }
  port_iterator input_end() const { return Input.end(); }

  port_iterator output_begin(unsigned Index) const {
    assert(Output != NULL && Index < NumOutputPorts);
    return Output[Index].begin();
  }
  port_iterator output_end(unsigned Index) const {
    assert(Output != NULL && Index < NumOutputPorts);
    return Output[Index].end();
  }

  PortMapTy &input_portmap() { return Input; }
  const PortMapTy &input_portmap() const { return Input; }
  PortMapTy &output_portmap(unsigned Index) {
    assert(Output != NULL && Index < NumOutputPorts);
    return Output[Index];
  }
  const PortMapTy &output_portmap(unsigned Index) const {
    assert(Output != NULL && Index < NumOutputPorts);
    return Output[Index];
  }

  typedef SmallVectorImpl<AMMInterfaceBase*>::iterator mem_iterator;
  typedef SmallVectorImpl<AMMInterfaceBase*>::const_iterator const_mem_iterator;

  mem_iterator mem_begin() { return Mems.begin(); }
  mem_iterator mem_end() { return Mems.end(); }
  const_mem_iterator mem_begin() const { return Mems.begin(); }
  const_mem_iterator mem_end() const { return Mems.end(); }

  typedef SmallVectorImpl<BMLInterface*>::iterator lock_iterator;
  typedef SmallVectorImpl<BMLInterface*>::const_iterator const_lock_iterator;

  lock_iterator lock_begin() { return Locks.begin(); }
  lock_iterator lock_end() { return Locks.end(); }
  const_lock_iterator lock_begin() const { return Locks.begin(); }
  const_lock_iterator lock_end() const { return Locks.end(); }

private:
  void InstantiateBasicBlock(CTBasicBlock *BB);
  void InstantiateSequential(CTSequential *Seq);
  void InstantiateIfThen(CTIfThen *IfThen);
  void InstantiateIfThenElse(CTIfThenElse *IfThenElse);
  void InstantiateAcyclic(CTAcyclic *Acyclic);
  void InstantiateSelfLoop(CTSelfLoop *Loop);
  void InstantiateWhileLoop(CTWhileLoop *Loop);
  void InstantiateNaturalLoop(CTNaturalLoop *Loop);

  void InheritOneToOne(PortMapTy &From, VerilogSignal *ValidFrom,
                       VerilogSignal *WaitFrom, PortMapTy &To,
                       VerilogSignal *&ValidTo, VerilogSignal *&WaitTo);
  void ConnectOneToOne(PortMapTy &From, VerilogSignal *ValidFrom,
                       VerilogSignal *WaitFrom, PortMapTy &To,
                       VerilogSignal *ValidTo, VerilogSignal *WaitTo);
  void ConnectTwoToOne(PortMapTy &From0, VerilogSignal *ValidFrom0,
                       VerilogSignal *WaitFrom0, PortMapTy &From1,
                       VerilogSignal *ValidFrom1, VerilogSignal *WaitFrom1,
                       PortMapTy &To, VerilogSignal *ValidTo,
                       VerilogSignal *WaitTo, std::string From0Prefix,
                       std::string From1Prefix);

  struct NToOneFromTy {
    DPControlFlowUnit *Node;
    unsigned Edge;

    NToOneFromTy(DPControlFlowUnit *node, unsigned edge)
      : Node(node), Edge(edge) {}

    PortMapTy &output_portmap() const { return Node->output_portmap(Edge); }
    VerilogSignal *valid_out() const { return Node->valid_out(Edge); }
    VerilogSignal *wait_out() const { return Node->wait_out(Edge); }
  };

  void ConnectNToOne(ArrayRef<NToOneFromTy> From, PortMapTy &To,
                     VerilogSignal *ValidTo, VerilogSignal *WaitTo);

  void CreatePort(PortMapTy &To, ControlTreeNode::const_live_var_iterator begin,
                  ControlTreeNode::const_live_var_iterator end);
  VerilogSignal *GetSignalInPort(PortMapTy &P, IndexedVarDecl *Var);
  void AssignPort(PortMapTy &To, PortMapTy &From);
  void AssignPort(PortMapTy &To, PortMapTy &From0, PortMapTy &From1,
                  VerilogSignal *Use0);

  VerilogExpr *GetWorkGroupConstraintKeyInPort(PortMapTy &Port);
  VerilogSignal *CreateWorkItemCounter(unsigned Width);

  void EnforceSingleWorkGroupConstraint();
  void EnforceWorkGroupOrderingConstraint(PortMapTy &LeftPort,
                                          VerilogSignal *&ValidLeft,
                                          VerilogSignal *&WaitLeft,
                                          PortMapTy &RightPort,
                                          VerilogSignal *&ValidRight,
                                          VerilogSignal *&WaitRight);
  void EnforceWorkItemLimit();
  void InsertWorkItemLimitIncrementQueue(
    PortMapTy &Port, VerilogSignal *&Valid, VerilogSignal *&Wait,
    ControlTreeNode::const_live_var_iterator out_begin,
    ControlTreeNode::const_live_var_iterator out_end);

};

} // namespace Synthesis

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_SYNTHESIS_CGCONTROLFLOWUNIT_H
