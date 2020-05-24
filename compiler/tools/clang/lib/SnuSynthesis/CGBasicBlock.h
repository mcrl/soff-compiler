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

#ifndef LLVM_CLANG_SNU_SYNTHESIS_CGBASICBLOCK_H
#define LLVM_CLANG_SNU_SYNTHESIS_CGBASICBLOCK_H

#include "CGCommon.h"
#include "clang/AST/ASTContext.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuSupport/OrderedDenseADT.h"
#include "clang/SnuSynthesis/CodeGenerator.h"
#include "clang/SnuSynthesis/DataflowGraph.h"
#include "clang/SnuSynthesis/Verilog.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include <string>

namespace clang {

namespace snu {

namespace Synthesis {

class AMMInterfaceBase;
class BMLInterface;
class UnresolvedAddrOf;

/* DPOperation: A fixed-latency (compound) operation
 *
 *  Enable -> [                       ]
 * Input 1 -> [   Implementation of   ] -> Output 1
 * Input 2 -> [ one or more DFG nodes ] -> Output 2
 * Input 3 -> [                       ] -> Output 3
 *            <------- Latency ------->
 */

class DPOperation {
  CodeGenContext &Ctx;
  ModuleBuilder *Parent;

  VerilogSignal *Enable;

  typedef OrderedDenseMap<DFGNode*, VerilogSignal*> PortMapTy;
  PortMapTy Input;
  PortMapTy Output;
  llvm::DenseMap<DFGNode*, VerilogSignal*> Intermediate;

  unsigned Latency;
  llvm::DenseMap<DFGNode*, unsigned> Tend;

public:
  DPOperation(CodeGenContext &ctx, ModuleBuilder *parent);

  VerilogSignal *enable() const { return Enable; }

  typedef PortMapTy::const_iterator port_iterator;

  port_iterator input_begin() const { return Input.begin(); }
  port_iterator input_end() const { return Input.end(); }
  unsigned num_inputs() const { return Input.size(); }

  port_iterator output_begin() const { return Output.begin(); }
  port_iterator output_end() const { return Output.end(); }
  unsigned num_outputs() const { return Output.size(); }

  unsigned getLatency() const { return Latency; }

  void addNode(DFGNode *Node);
  void markAsOutput(DFGNode *Node);

private:
  VerilogSignal *getSignalOf(DFGNode *Node);

  VerilogSignal *CreateUnaryOpResult(DFGUnaryOpNode *UO);
  VerilogSignal *CreateBinaryOpResult(DFGBinaryOpNode *BO);
  VerilogSignal *CreateTernaryOpResult(DFGTernaryOpNode *TO);
  VerilogSignal *CreateRangeOpResult(DFGRangeOpNode *RO);
  VerilogSignal *CreateVariableRangeOpResult(DFGVariableRangeOpNode *VRO);
  VerilogSignal *CreateConcatOpResult(DFGConcatOpNode *CO);
  VerilogSignal *CreateSubstituteOpResult(DFGSubstituteOpNode *SO);
  VerilogSignal *CreateShiftRegisterResult(DFGShiftRegisterNode *SR);
};

/* DPHandshakingUnit: A handshaking unit
 *
 *                       Input 1 -> [      ] -> Output 1
 *                       Input 2 -> [      ] -> Output 2
 *                       Input 3 -> [      ] -> Output 3
 * (ValidOut) -> [  ] -> ValidIn -> [ Unit ] -> ValidOut -> [  ] -> ValidOut 1
 *   WaitIn 1 <- [  ] <-  WaitIn <- [      ] <- WaitOut  <- [  ] <- (WaitIn)
 *               [  ]               [      ]                [  ]
 * (ValidOut) -> [  ]               [      ] <-> Mem        [  ] -> ValidOut 2
 *   WaitIn 2 <- [  ]               [      ] <-> Lock       [  ] <- (WaitIn)
 *               [  ]                                       [  ]
 * (ValidOut) -> [  ]                                       [  ] -> ValidOut 3
 *   WaitIn 3 <- [  ]                                       [  ] <- (WaitIn)
 */

class DPHandshakingUnit {
  CodeGenContext &Ctx;
  ModuleBuilder *Parent;

  VerilogSignal *ValidIn;
  VerilogSignal *ValidOut;
  VerilogSignal *WaitIn;
  VerilogSignal *WaitOut;
  VerilogSignal *ValidBuffer;

  typedef OrderedDenseMap<DFGNode*, VerilogSignal*> PortMapTy;
  PortMapTy Input;
  PortMapTy Output;
  AMMInterfaceBase *Mem;
  BMLInterface *Lock;

  SmallVector<DPHandshakingUnit*, 4> Preds;
  SmallVector<DPHandshakingUnit*, 4> Succs;
  llvm::DenseMap<DPHandshakingUnit*, VerilogSignal*> PerPredWaitIn;
  llvm::DenseMap<DPHandshakingUnit*, VerilogSignal*> PerSuccValidOut;

  bool IsEntry;
  bool IsExit;
  bool IsFinalized;

public:
  // Fixed-latency (compound) operation
  DPHandshakingUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                    DPOperation *Op);
  // FIFO queue
  DPHandshakingUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                    DFGQueueNode *Q);
  // Load operation
  DPHandshakingUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                    DFGLoadNode *Load);
  // Store operation
  DPHandshakingUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                    DFGStoreNode *Store);
  // Atomic operation
  DPHandshakingUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                    DFGAtomicNode *Atomic);
  // Barrier
  DPHandshakingUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                    DFGBarrierNode *Barrier);
  // Source & Sink
  DPHandshakingUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                    DFGSourceNode *Source, const std::string &BBName);
  DPHandshakingUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                    DFGSinkNode *Sink, const std::string &BBName);

  VerilogSignal *valid_in() const { return ValidIn; }
  VerilogSignal *valid_out() const { return ValidOut; }
  VerilogSignal *wait_in() const { return WaitIn; }
  VerilogSignal *wait_out() const { return WaitOut; }

  typedef PortMapTy::const_iterator port_iterator;

  port_iterator input_begin() const { return Input.begin(); }
  port_iterator input_end() const { return Input.end(); }
  unsigned num_inputs() const { return Input.size(); }

  port_iterator output_begin() const { return Output.begin(); }
  port_iterator output_end() const { return Output.end(); }
  unsigned num_outputs() const { return Output.size(); }

  VerilogSignal *getInput(DFGNode *Node) const {
    if (Input.count(Node)) return Input.lookup(Node);
    return NULL;
  }
  VerilogSignal *getOutput(DFGNode *Node) const {
    if (Output.count(Node)) return Output.lookup(Node);
    return NULL;
  }
  AMMInterfaceBase *getMemoryInterface() const { return Mem; }
  BMLInterface *getLockInterface() const { return Lock; }

  typedef SmallVectorImpl<DPHandshakingUnit*>::iterator pred_iterator;
  typedef SmallVectorImpl<DPHandshakingUnit*>::const_iterator
      const_pred_iterator;

  pred_iterator pred_begin() { return Preds.begin(); }
  pred_iterator pred_end() { return Preds.end(); }
  const_pred_iterator pred_begin() const { return Preds.begin(); }
  const_pred_iterator pred_end() const { return Preds.end(); }
  unsigned num_preds() const { return Preds.size(); }

  typedef SmallVectorImpl<DPHandshakingUnit*>::iterator succ_iterator;
  typedef SmallVectorImpl<DPHandshakingUnit*>::const_iterator
      const_succ_iterator;

  succ_iterator succ_begin() { return Succs.begin(); }
  succ_iterator succ_end() { return Succs.end(); }
  const_succ_iterator succ_begin() const { return Succs.begin(); }
  const_succ_iterator succ_end() const { return Succs.end(); }
  unsigned num_succs() const { return Succs.size(); }

  VerilogSignal *wait_in_of(DPHandshakingUnit *Pred) const {
    assert(PerPredWaitIn.count(Pred));
    return PerPredWaitIn.lookup(Pred);
  }
  VerilogSignal *valid_out_of(DPHandshakingUnit *Succ) const {
    assert(PerSuccValidOut.count(Succ));
    return PerSuccValidOut.lookup(Succ);
  }

  void addPredecessor(DPHandshakingUnit *Pred);
  void addSuccessor(DPHandshakingUnit *Succ);

  void Finalize();

private:
  VerilogSignal *GetOrCreateTempWireInput(DFGNode *Node);
  VerilogSignal *CreateTempWireOutput(DFGNode *Node);
  VerilogSignal *FindPredecessorOutput(DFGNode *Node);
  VerilogSignal *FindSuccessorInput(DFGNode *Node);

  VerilogExpr *CreateMemoryAddress(DFGMemoryAccessNode *Access);

  void MakeFixedLatencyBufferedUnit(unsigned Latency);
  void AssignBufferedOutput(DFGNode *Node, VerilogSignal *Value);

  std::string NormalizedName(IndexedVarDecl *Var);
};

/* DPBasicBlockUnit: A basic block unit (= a set of handshaking units)
 *
 * Input 1 -> [      ] -> Output 1
 * Input 2 -> [      ] -> Output 2
 * Input 3 -> [      ] -> Output 3
 *            [      ] -> OutputCond
 * ValidIn -> [      ] -> ValidOut
 *  WaitIn <- [ Unit ] <- WaitOut
 *            [      ]
 *            [      ] <-> Mem 1
 *            [      ] <-> Mem 2
 *            [      ] <-> Mem 3
 *            [      ] <-> Lock 1
 *            [      ] <-> Lock 2
 */

class DPBasicBlockUnit {
  CodeGenContext &Ctx;
  ModuleBuilder *Parent;
  DataflowGraph *Graph;
  std::string Name;

  VerilogSignal *ValidIn;
  VerilogSignal *ValidOut;
  VerilogSignal *WaitIn;
  VerilogSignal *WaitOut;

  typedef OrderedDenseMap<IndexedVarDecl*, VerilogSignal*> PortMapTy;
  PortMapTy Input;
  PortMapTy Output;
  VerilogSignal *OutputCond;
  SmallVector<AMMInterfaceBase*, 4> Mems;
  SmallVector<BMLInterface*, 4> Locks;

  DPHandshakingUnit *Entry;
  DPHandshakingUnit *Exit;
  OrderedDenseSet<DPHandshakingUnit*> Units;
  llvm::DenseMap<DFGNode*, DPHandshakingUnit*> NodeImpl;

public:
  DPBasicBlockUnit(CodeGenContext &ctx, ModuleBuilder *parent,
                   DataflowGraph *graph);

  VerilogSignal *valid_in() const { return ValidIn; }
  VerilogSignal *valid_out() const { return ValidOut; }
  VerilogSignal *wait_in() const { return WaitIn; }
  VerilogSignal *wait_out() const { return WaitOut; }

  typedef PortMapTy::const_iterator port_iterator;

  port_iterator input_begin() const { return Input.begin(); }
  port_iterator input_end() const { return Input.end(); }
  unsigned num_inputs() const { return Input.size(); }

  port_iterator output_begin() const { return Output.begin(); }
  port_iterator output_end() const { return Output.end(); }
  unsigned num_outputs() const { return Output.size(); }

  VerilogSignal *getInput(IndexedVarDecl *Var) const {
    if (Input.count(Var)) return Input.lookup(Var);
    return NULL;
  }
  VerilogSignal *getOutput(IndexedVarDecl *Var) const {
    if (Output.count(Var)) return Output.lookup(Var);
    return NULL;
  }
  bool hasOutputCond() const { return OutputCond != NULL; }
  VerilogSignal *getOutputCond() const { return OutputCond; }

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
  DPHandshakingUnit *InstantiateNode(DFGNode *Node);
  DPHandshakingUnit *InstantiateCompound(DFGCompoundNode *Compound);
};

} // namespace Synthesis

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_SYNTHESIS_CGBASICBLOCK_H
