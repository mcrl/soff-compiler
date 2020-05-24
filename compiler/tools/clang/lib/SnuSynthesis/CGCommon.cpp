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

#include "CGCommon.h"
#include "CGMemorySubsystem.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/Decl.h"
#include "clang/AST/Type.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuFrontend/Options.h"
#include "clang/SnuSynthesis/ControlFlowConstraint.h"
#include "clang/SnuSynthesis/DataflowGraph.h"
#include "clang/SnuSynthesis/PlatformContext.h"
#include "clang/SnuSynthesis/StructuralAnalysis.h"
#include "clang/SnuSynthesis/Verilog.h"
#include "clang/SnuSynthesis/VirtualVariables.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/StringRef.h"
#include <algorithm>
#include <map>
#include <string>

namespace clang {

namespace snu {

namespace Synthesis {

VerilogExpr *ConvertWidthOf(VerilogSignal *Operand, unsigned ToWidth,
                            bool is_signed) {
  unsigned FromWidth = (unsigned)Operand->getVectorWidthValue();
  if (FromWidth == ToWidth) {
    return new VerilogSignalRef(Operand);
  } else if (FromWidth > ToWidth) {
    // Operand[ToWidth-1 : 0];
    return new VerilogSignalRef(Operand, VerilogParamConst(ToWidth - 1),
                                VerilogParamConst(0));
  } else {
    VerilogExpr *Padding;
    if (is_signed) {
      // n{Operand[FromWidth - 1]}
      VerilogExpr *MSB = new VerilogSignalRef(Operand,
                                              VerilogParamConst(FromWidth - 1),
                                              VerilogParamConst(FromWidth - 1));
      Padding = new VerilogReplication(ToWidth - FromWidth, MSB);
    } else {
      Padding = new VerilogConst(VR_Binary, 0, ToWidth - FromWidth);
    }
    return new VerilogConcat(Padding, new VerilogSignalRef(Operand));
  }
}

/*
 * CodeGenContext
 */

CodeGenContext::CodeGenContext(const ASTContext &AC, PlatformContext &PC,
                               const SnuCLOptions &Opts,
                               const ControlDataflowGraph *cdfg,
                               const ControlTreeNode *CT)
  : ASTCtx(AC), PCtx(PC), SnuCLOpts(Opts),
    VVars(cdfg->getVirtualVariablePool()),
    TopLevelConstraint(CT->getControlFlowConstraint()), LocalMemLayout(NULL) {
}

unsigned CodeGenContext::getTopLevelSingleWorkGroupGranularity() const {
  if (TopLevelConstraint.hasSingleWorkGroupConstraint()) {
    return TopLevelConstraint.getWorkGroupGranularity();
  } else {
    return 0;
  }
}

const LocalMemoryLayout &CodeGenContext::getLocalMemoryLayout() const {
  assert(LocalMemLayout != NULL);
  return *LocalMemLayout;
}

void CodeGenContext::RegisterLocalMemoryLayout(const LocalMemoryLayout *Layout) {
  assert(LocalMemLayout == NULL);
  LocalMemLayout = Layout;
}

uint64_t CodeGenContext::getAddressOf(WVarDecl *Var) const {
  assert(LocalMemLayout != NULL);
  return LocalMemLayout->getAddressOf(Var);
}

unsigned CodeGenContext::getVirtualAddressWidthFor(
    DFGMemoryAccessNode *Node) const {
  if (Node->getAddressSpace() == LangAS::opencl_global) {
    return getGlobalMemAddressWidth();
  } else if (Node->getAddressSpace() == LangAS::opencl_local) {
    assert(LocalMemLayout != NULL);
    return LocalMemLayout->getVirtualAddressWidthFor(Node);
  } else {
    llvm_unreachable("not implemented yet");
    return 0;
  }
}

unsigned CodeGenContext::getPhysicalAddressWidthFor(
    DFGMemoryAccessNode *Node) const {
  if (Node->getAddressSpace() == LangAS::opencl_global) {
    return getGlobalMemAddressWidth();
  } else if (Node->getAddressSpace() == LangAS::opencl_local) {
    assert(LocalMemLayout != NULL);
    return LocalMemLayout->getPhysicalAddressWidthFor(Node);
  } else {
    llvm_unreachable("not implemented yet");
    return 0;
  }
}

unsigned CodeGenContext::getDataWidthFor(DFGMemoryAccessNode *Node) const {
  if (Node->getAddressSpace() == LangAS::opencl_global) {
    return getGlobalMemDataWidth();
  } else if (Node->getAddressSpace() == LangAS::opencl_local) {
    assert(LocalMemLayout != NULL);
    return LocalMemLayout->getDataWidthFor(Node);
  } else {
    llvm_unreachable("not implemented yet");
    return 0;
  }
}

/*
 * ModuleBuilder
 */

ModuleBuilder::ModuleBuilder(StringRef name, CodeGenContext &ctx)
  : VerilogModule(name), Ctx(ctx) {
  Clock = CreateWire(SP_Input, "clk");
  Rstn = CreateWire(SP_Input, "rstn");

  OnClock = new VerilogCompound();
  addStmt(
    // always @(posedge clk) [OnClock]
    new VerilogAlways(VerilogEvent(EvK_Posedge, new VerilogSignalRef(Clock)),
      OnClock
    )
  );
}

void ModuleBuilder::addAssignWire(VerilogSignal *Target,
                                       VerilogExprPtrOrSignalPtr Value) {
  assert(Target->getKind() == SK_Wire);
  addStmt(CreateAssignOf(Target, Value));
}

void ModuleBuilder::addAssignReg(VerilogSignal *Target,
                                      VerilogExprPtrOrSignalPtr Value) {
  assert(Target->getKind() == SK_Reg);
  addAssignRegStmt(Target, CreateAssignOf(Target, Value));
}

void ModuleBuilder::addAssignRegOnEnable(VerilogExprPtrOrSignalPtr Enable,
                                              VerilogSignal *Target,
                                              VerilogExprPtrOrSignalPtr Value) {
  assert(Target->getKind() == SK_Reg);
  addAssignRegStmt(Target,
    // if (enable)
    new VerilogIf(Enable,
      // begin
      (new VerilogCompound())->addStmt(
        // Target <= Value;
        CreateAssignOf(Target, Value)
      )
      // end
    )
  );
}

void ModuleBuilder::addAssignRegStmt(VerilogSignal *Target,
                                          VerilogStmt *Stmt) {
  assert(Target->getKind() == SK_Reg);
  addStmtOnClock(
    // if (~rstn)
    new VerilogIf(CreateNotOf(CreateRstnRef()),
      // begin
      (new VerilogCompound())->addStmt(
        // Target <= n'b0;
        CreateAssignOf(Target, CreateZero(Target->getVectorWidthValue()))
      ),
      // end
      // else begin
      (new VerilogCompound())->addStmt(Stmt)
      // end
    )
  );
}

VerilogSignalRef *ModuleBuilder::CreateClockRef() {
  return new VerilogSignalRef(Clock);
}

VerilogSignalRef *ModuleBuilder::CreateRstnRef() {
  VerilogSignal *RstnX = rstn();
  for (unsigned i = 0; i < Ctx.getResetDelay(); ++i) {
    VerilogSignal *NewRstnX = CreateTempReg("rstn_x");
    switch (Ctx.getResetAttribute()) {
      case 0: break; // do nothing
      case 1: NewRstnX->addAttribute(SA_DontMerge); break;
      case 2: NewRstnX->addAttribute(SA_DontTouch); break;
      default: break;
    }
    addStmtOnClock(CreateAssignOf(NewRstnX, RstnX));
    RstnX = NewRstnX;
  }
  return new VerilogSignalRef(RstnX);
}

VerilogSignalRef *ModuleBuilder::CreateRstnRef(StringRef rstn_name) {
  VerilogSignal *RstnX = getSignal(rstn_name);
  for (unsigned i = 0; i < Ctx.getResetDelay(); ++i) {
    VerilogSignal *NewRstnX = CreateTempReg("rstn_x");
    addStmtOnClock(CreateAssignOf(NewRstnX, RstnX));
    RstnX = NewRstnX;
  }
  return new VerilogSignalRef(RstnX);
}

VerilogSignal *ModuleBuilder::CreateTempWire(StringRef namePrefix) {
  std::string name = AcquireNameWithPrefix(namePrefix);
  VerilogSignal *signal = new VerilogSignal(this, SK_Wire, SP_None, name);
  addSignal(signal);
  return signal;
}

VerilogSignal *ModuleBuilder::CreateTempWire(
    StringRef namePrefix, VerilogParamConstOrInt vectorWidth) {
  std::string name = AcquireNameWithPrefix(namePrefix);
  VerilogSignal *signal = new VerilogSignal(this, SK_Wire, SP_None, name,
                                            vectorWidth);
  addSignal(signal);
  return signal;
}

VerilogSignal *ModuleBuilder::CreateTempWire(
    StringRef namePrefix, VerilogParamConstOrInt vectorWidth,
    VerilogParamConstOrInt arrayWidth) {
  std::string name = AcquireNameWithPrefix(namePrefix);
  VerilogSignal *signal = new VerilogSignal(this, SK_Wire, SP_None, name,
                                            vectorWidth, arrayWidth);
  addSignal(signal);
  return signal;
}

VerilogSignal *ModuleBuilder::CreateTempReg(StringRef namePrefix) {
  std::string name = AcquireNameWithPrefix(namePrefix);
  VerilogSignal *signal = new VerilogSignal(this, SK_Reg, SP_None, name);
  addSignal(signal);
  return signal;
}

VerilogSignal *ModuleBuilder::CreateTempReg(
    StringRef namePrefix, VerilogParamConstOrInt vectorWidth) {
  std::string name = AcquireNameWithPrefix(namePrefix);
  VerilogSignal *signal = new VerilogSignal(this, SK_Reg, SP_None, name,
                                            vectorWidth);
  addSignal(signal);
  return signal;
}

VerilogSignal *ModuleBuilder::CreateTempReg(
    StringRef namePrefix, VerilogParamConstOrInt vectorWidth,
    VerilogParamConstOrInt arrayWidth) {
  std::string name = AcquireNameWithPrefix(namePrefix);
  VerilogSignal *signal = new VerilogSignal(this, SK_Reg, SP_None, name,
                                            vectorWidth, arrayWidth);
  addSignal(signal);
  return signal;
}

VerilogSignal *ModuleBuilder::CreateTempWireFor(IndexedVarDecl *Var,
                                                StringRef Prefix) {
  unsigned Width = Ctx.getTypeSize(Var);
  return CreateTempWire(Prefix, VerilogParamConst(Width));
}

VerilogSignal *ModuleBuilder::CreateTempWireFor(DFGNode *Node,
                                                StringRef Prefix) {
  assert(!Node->getType().isNull());
  unsigned Width = Ctx.getTypeSize(Node);
  return CreateTempWire(Prefix, VerilogParamConst(Width));
}

VerilogSignal *ModuleBuilder::CreateTempRegFor(IndexedVarDecl *Var,
                                               StringRef Prefix) {
  unsigned Width = Ctx.getTypeSize(Var);
  return CreateTempReg(Prefix, VerilogParamConst(Width));
}

VerilogSignal *ModuleBuilder::CreateTempRegFor(DFGNode *Node,
                                               StringRef Prefix) {
  assert(!Node->getType().isNull());
  unsigned Width = Ctx.getTypeSize(Node);
  return CreateTempReg(Prefix, VerilogParamConst(Width));
}

VerilogModuleInstance *ModuleBuilder::CreateTempModuleInstance(
    VerilogModule *target, StringRef namePrefix) {
  std::string name = AcquireNameWithPrefix(namePrefix);
  VerilogModuleInstance *inst = new VerilogModuleInstance(this, target, name);
  addModuleInstance(inst);
  return inst;
}

std::string ModuleBuilder::AcquireNameWithPrefix(StringRef prefix) {
  unsigned index;
  if (NamingCount.count(prefix)) {
    index = (++NamingCount[prefix]);
  } else {
    index = (NamingCount[prefix] = 0);
  }
  std::string name;
  name += prefix;
  name += "__";
  name += llvm::utostr_32(index);
  return name;
}

VerilogModuleInstance *ModuleBuilder::CreateIPInstance(StringRef IPName,
                                                       StringRef Prefix) {
  if (PlatformIPCore *IP = Ctx.getIP(IPName)) {
    return CreateTempModuleInstance(IP->getModule(), Prefix);
  }
  return NULL;
}

VerilogModuleInstance *ModuleBuilder::CreateIPInstance(DFGUnaryOpNode *Op,
                                                       StringRef Prefix) {
  if (PlatformIPCore *IP = Ctx.getIP(Op)) {
    return CreateTempModuleInstance(IP->getModule(), Prefix);
  }
  return NULL;
}

VerilogModuleInstance *ModuleBuilder::CreateIPInstance(DFGBinaryOpNode *Op,
                                                       StringRef Prefix) {
  if (PlatformIPCore *IP = Ctx.getIP(Op)) {
    return CreateTempModuleInstance(IP->getModule(), Prefix);
  }
  return NULL;
}

VerilogModuleInstance *ModuleBuilder::CreateIPInstance(DFGTernaryOpNode *Op,
                                                       StringRef Prefix) {
  if (PlatformIPCore *IP = Ctx.getIP(Op)) {
    return CreateTempModuleInstance(IP->getModule(), Prefix);
  }
  return NULL;
}

VerilogModuleInstance *ModuleBuilder::CreateIPInstance(DFGNullaryAtomicNode *Op,
                                                       StringRef Prefix) {
  if (PlatformIPCore *IP = Ctx.getIP(Op)) {
    return CreateTempModuleInstance(IP->getModule(), Prefix);
  }
  return NULL;
}

VerilogModuleInstance *ModuleBuilder::CreateIPInstance(DFGUnaryAtomicNode *Op,
                                                       StringRef Prefix) {
  if (PlatformIPCore *IP = Ctx.getIP(Op)) {
    return CreateTempModuleInstance(IP->getModule(), Prefix);
  }
  return NULL;
}

VerilogModuleInstance *ModuleBuilder::CreateIPInstance(DFGBinaryAtomicNode *Op,
                                                       StringRef Prefix) {
  if (PlatformIPCore *IP = Ctx.getIP(Op)) {
    return CreateTempModuleInstance(IP->getModule(), Prefix);
  }
  return NULL;
}

void ModuleBuilder::InsertGatekeeper(VerilogSignal *ValidFrom,
                                     VerilogSignal *WaitFrom,
                                     VerilogSignal *ValidTo,
                                     VerilogSignal *WaitTo,
                                     VerilogExprPtrOrSignalPtr Pass) {
  VerilogSignal *P = CreateTempWire("gatekeeper_pass");
  addAssignWire(P, Pass);
  addAssignWire(ValidTo, CreateAndOf(ValidFrom, P));
  addAssignWire(WaitFrom,
    CreateOrOf(WaitTo, CreateAndOf(ValidFrom, CreateNotOf(P)))
  );
}

VerilogSignal *ModuleBuilder::CreateCounter(
    StringRef Prefix, VerilogParamConstOrInt Width,
    VerilogExprPtrOrSignalPtr Increment, VerilogExprPtrOrSignalPtr Decrement) {
  VerilogSignal *Counter = CreateTempReg(Prefix, Width);
  VerilogSignal *Inc = CreateTempWire("counter_inc");
  VerilogSignal *Dec = CreateTempWire("counter_dec");
  addAssignWire(Inc, Increment);
  addAssignWire(Dec, Decrement);
  addAssignRegStmt(Counter,
    // if (inc & ~dec)
    new VerilogIf(CreateAndOf(Inc, CreateNotOf(Dec)),
      // begin
      (new VerilogCompound())->addStmt(
        // count <= count + 1
        CreateAssignOf(Counter, CreateAddOf(Counter, CreateDecimalOne()))
      ),
      // end
      // else if (~inc & dec)
      new VerilogIf(CreateAndOf(CreateNotOf(Inc), Dec),
        // begin
        (new VerilogCompound())->addStmt(
          // count <= count - 1
          CreateAssignOf(Counter, CreateSubOf(Counter, CreateDecimalOne()))
        )
        // end
      )
    )
  );
  return Counter;
}

VerilogSignal *ModuleBuilder::CreateCounter(
    StringRef Prefix, VerilogParamConstOrInt Width,
    ArrayRef<VerilogExprPtrOrSignalPtr> Increments,
    ArrayRef<VerilogExprPtrOrSignalPtr> Decrements) {
  assert(!Increments.empty() && !Decrements.empty());
  if (Increments.size() == 1 && Decrements.size() == 1) {
    return CreateCounter(Prefix, Width, Increments[0], Decrements[0]);
  }
  VerilogSignal *Counter = CreateTempReg(Prefix, Width);
  VerilogExpr *CounterExpr = new VerilogSignalRef(Counter);
  for (unsigned Index = 0, NumIncs = Increments.size();
       Index != NumIncs; ++Index) {
    CounterExpr = CreateAddOf(CounterExpr, Increments[Index]);
  }
  for (unsigned Index = 0, NumDecs = Decrements.size();
       Index != NumDecs; ++Index) {
    CounterExpr = CreateSubOf(CounterExpr, Decrements[Index]);
  }
  addAssignReg(Counter, CounterExpr);
  return Counter;
}

void ModuleBuilder::InsertFIFOQueue(StringRef Prefix,
                                    VerilogParamConstOrInt Width,
                                    unsigned MaxLatency,
                                    VerilogExprPtrOrSignalPtr In,
                                    VerilogExprPtrOrSignalPtr ValidIn,
                                    VerilogSignal *WaitIn,
                                    VerilogExprPtrOrSignalPtr Out,
                                    VerilogSignal *ValidOut,
                                    VerilogExprPtrOrSignalPtr WaitOut) {
  VerilogModuleInstance *Impl = CreateIPInstance("fifo_queue", Prefix);
  assert(Impl != NULL);
  Impl->addParamArgument("WIDTH", Width);
  Impl->addParamArgument("MAX_LATENCY", VerilogParamConst(MaxLatency));
  Impl->addArgument("clk", CreateClockRef());
  Impl->addArgument("rstn", CreateRstnRef());
  Impl->addArgument("in", In);
  Impl->addArgument("valid_in", ValidIn);
  if (WaitIn != NULL) {
    Impl->addArgument("wait_in", new VerilogSignalRef(WaitIn));
  }
  Impl->addArgument("out", Out);
  if (ValidOut != NULL) {
    Impl->addArgument("valid_out", new VerilogSignalRef(ValidOut));
  }
  Impl->addArgument("wait_out", WaitOut);
}

void ModuleBuilder::InsertDatalessFIFOQueue(unsigned MaxLatency,
                                            VerilogExprPtrOrSignalPtr ValidIn,
                                            VerilogSignal *WaitIn,
                                            VerilogSignal *ValidOut,
                                            VerilogExprPtrOrSignalPtr WaitOut) {
  if (WaitIn == NULL) {
    WaitIn = CreateTempWire("dataless_fifo_queue_wait_in");
  }
  if (ValidOut == NULL) {
    ValidOut = CreateTempWire("dataless_fifo_queue_valid_out");
  }
  VerilogExpr *Inc = CreateAndOf(ValidIn, CreateNotOf(WaitIn));
  VerilogExpr *Dec = CreateAndOf(ValidOut, CreateNotOf(WaitOut));
  VerilogSignal *Count = CreateCounter("dataless_fifo_queue_count",
                                       Log2(MaxLatency) + 2, Inc, Dec);
  // assign wait_in = count == LIMIT;
  // assign valid_out = count != 0;
  addAssignWire(WaitIn,
    CreateEqualOf(Count, new VerilogConst(VR_Decimal, MaxLatency))
  );
  addAssignWire(ValidOut, CreateNotEqualOf(Count, CreateDecimalZero()));
}

} // namespace Synthesis

} // namespace snu

} // namespace clang
