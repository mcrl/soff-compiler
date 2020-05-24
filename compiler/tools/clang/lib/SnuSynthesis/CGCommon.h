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

#ifndef LLVM_CLANG_SNU_SYNTHESIS_CGCOMMON_H
#define LLVM_CLANG_SNU_SYNTHESIS_CGCOMMON_H

#include "clang/AST/ASTContext.h"
#include "clang/AST/Decl.h"
#include "clang/AST/Type.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuFrontend/Options.h"
#include "clang/SnuSynthesis/ControlFlowConstraint.h"
#include "clang/SnuSynthesis/DataflowGraph.h"
#include "clang/SnuSynthesis/PlatformContext.h"
#include "clang/SnuSynthesis/StructuralAnalysis.h"
#include "clang/SnuSynthesis/Verilog.h"
#include "clang/SnuSynthesis/VirtualVariables.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/StringRef.h"
#include <map>
#include <string>

#define ARG_WIDTH 4096
#define PC_WIDTH 4096
#define LOCK_WIDTH 4

namespace clang {

namespace snu {

class IndexedVarDecl;
class WDeclContext;
class WVarDecl;

namespace Synthesis {

class LocalMemoryLayout;

inline unsigned Log2(unsigned x) {
  unsigned logx = 0;
  while (x > 1) {
    x >>= 1;
    logx++;
  }
  return logx;
}

// Constructors for commonly-used statements

inline VerilogExpr *CreateZero(unsigned Size = 1) {
  return new VerilogConst(VR_Binary, 0, Size);
}

inline VerilogExpr *CreateDecimalZero() {
  return new VerilogConst(VR_Decimal, 0);
}

inline VerilogExpr *CreateOne() {
  return new VerilogConst(VR_Binary, 1, 1);
}

inline VerilogExpr *CreateDecimalOne() {
  return new VerilogConst(VR_Decimal, 1);
}

inline VerilogSignalRef *CreateArrayElementOf(VerilogSignal *Array,
                                              unsigned Index) {
  return new VerilogSignalRef(Array, new VerilogConst(VR_Decimal, Index));
}

inline VerilogExpr *CreateNotOf(VerilogExprPtrOrSignalPtr Operand) {
  return new VerilogUnaryOperator(VUO_NOT, Operand);
}

inline VerilogExpr *CreateAndOf(VerilogExprPtrOrSignalPtr LHS,
                                VerilogExprPtrOrSignalPtr RHS) {
  return new VerilogBinaryOperator(VBO_AND, LHS, RHS);
}

inline VerilogExpr *CreateOrOf(VerilogExprPtrOrSignalPtr LHS,
                               VerilogExprPtrOrSignalPtr RHS) {
  return new VerilogBinaryOperator(VBO_OR, LHS, RHS);
}

inline VerilogExpr *CreateXorOf(VerilogExprPtrOrSignalPtr LHS,
                                VerilogExprPtrOrSignalPtr RHS) {
  return new VerilogBinaryOperator(VBO_XOR, LHS, RHS);
}

inline VerilogExpr *CreateLNotOf(VerilogExprPtrOrSignalPtr Operand) {
  return new VerilogUnaryOperator(VUO_LNOT, Operand);
}

inline VerilogExpr *CreateLAndOf(VerilogExprPtrOrSignalPtr LHS,
                                 VerilogExprPtrOrSignalPtr RHS) {
  return new VerilogBinaryOperator(VBO_LAND, LHS, RHS);
}

inline VerilogExpr *CreateLOrOf(VerilogExprPtrOrSignalPtr LHS,
                                VerilogExprPtrOrSignalPtr RHS) {
  return new VerilogBinaryOperator(VBO_LOR, LHS, RHS);
}

inline VerilogExpr *CreateEqualOf(VerilogExprPtrOrSignalPtr LHS,
                                  VerilogExprPtrOrSignalPtr RHS) {
  return new VerilogBinaryOperator(VBO_LEQ, LHS, RHS);
}

inline VerilogExpr *CreateNotEqualOf(VerilogExprPtrOrSignalPtr LHS,
                                     VerilogExprPtrOrSignalPtr RHS) {
  return new VerilogBinaryOperator(VBO_LNE, LHS, RHS);
}

inline VerilogExpr *CreateAddOf(VerilogExprPtrOrSignalPtr LHS,
                                VerilogExprPtrOrSignalPtr RHS) {
  return new VerilogBinaryOperator(VBO_ADD, LHS, RHS);
}

inline VerilogExpr *CreateSubOf(VerilogExprPtrOrSignalPtr LHS,
                                VerilogExprPtrOrSignalPtr RHS) {
  return new VerilogBinaryOperator(VBO_SUB, LHS, RHS);
}

inline VerilogExpr *CreateLShlOf(VerilogExprPtrOrSignalPtr LHS,
                                 VerilogExprPtrOrSignalPtr RHS) {
  return new VerilogBinaryOperator(VBO_LSHL, LHS, RHS);
}

inline VerilogExpr *CreateConditionalOf(VerilogExprPtrOrSignalPtr Cond,
                                        VerilogExprPtrOrSignalPtr LHS,
                                        VerilogExprPtrOrSignalPtr RHS) {
  return new VerilogConditionalOperator(Cond, LHS, RHS);
}

inline VerilogStmt *CreateAssignOf(VerilogSignal *LHS,
                                   VerilogExprPtrOrSignalPtr RHS) {
  return new VerilogAssign(new VerilogSignalRef(LHS), RHS);
}

inline VerilogStmt *CreateAssignOf(VerilogSignalRef *LHS,
                                   VerilogExprPtrOrSignalPtr RHS) {
  return new VerilogAssign(LHS, RHS);
}

VerilogExpr *ConvertWidthOf(VerilogSignal *Operand, unsigned ToWidth,
                            bool is_signed = false);

class CodeGenContext {
  const ASTContext &ASTCtx;
  PlatformContext &PCtx;
  const SnuCLOptions &SnuCLOpts;
  const VirtualVariablePool &VVars;
  const ControlFlowConstraint &TopLevelConstraint;
  const LocalMemoryLayout *LocalMemLayout;

public:
  CodeGenContext(const ASTContext &AC, PlatformContext &PC,
                 const SnuCLOptions &Opts, const ControlDataflowGraph *cdfg,
                 const ControlTreeNode *CT);

  uint64_t getTypeSize(QualType Ty) const {
    return ASTCtx.getTypeSize(Ty);
  }
  uint64_t getTypeSize(WVarDecl *Var) const {
    return ASTCtx.getTypeSize(Var->getType());
  }
  uint64_t getTypeSize(IndexedVarDecl *Var) const {
    return ASTCtx.getTypeSize(Var->getType());
  }
  uint64_t getTypeSize(DFGNode *Node) const {
    return ASTCtx.getTypeSize(Node->getType());
  }
  uint64_t getTypeSizeInChars(QualType Ty) const {
    return getTypeSize(Ty) / ASTCtx.getCharWidth();
  }
  uint64_t getTypeSizeInChars(WVarDecl *Var) const {
    return getTypeSize(Var) / ASTCtx.getCharWidth();
  }
  uint64_t getSubVarOffset(WSubVarDecl *SubVar) const {
    return SubVar->getOffset(ASTCtx);
  }

  unsigned getNumDatapathInstances() const {
    return SnuCLOpts.NumDatapathInstances;
  }
  SnuCLOptions::MemorySubsystemKind getMemorySubsystemKind() const {
    return SnuCLOpts.MemorySubsystem;
  }

  unsigned getResetDelay() const { return PCtx.getResetDelay(); }
  unsigned getResetAttribute() const { return PCtx.getResetAttribute(); }
  bool isGlobalMemUseAXI() const { return PCtx.isMemUseAXI(); }
  unsigned getGlobalMemNumPorts() const { return PCtx.getMemNumPorts(); }
  unsigned getGlobalMemAddressWidth() const { return PCtx.getMemAddressWidth(); }
  unsigned getGlobalMemDataWidth() const { return PCtx.getMemDataWidth(); }
  unsigned getGlobalMemBurstWidth() const { return PCtx.getMemBurstWidth(); }
  unsigned getPortGIDWidth() const { return PCtx.getPortGIDWidth(); }
  unsigned getPortLIDWidth() const { return PCtx.getPortLIDWidth(); }
  unsigned getPortNIDWidth() const { return 64; }
  unsigned getCapacityLocalMemChunk() const { return PCtx.getCapacityLocalMemChunk(); }

  PlatformIPCore *getIP(StringRef Name) const { return PCtx.getIP(Name); }
  PlatformIPCore *getIP(DFGUnaryOpNode *Op) const { return PCtx.getIP(Op); }
  PlatformIPCore *getIP(DFGBinaryOpNode *Op) const { return PCtx.getIP(Op); }
  PlatformIPCore *getIP(DFGTernaryOpNode *Op) const { return PCtx.getIP(Op); }
  PlatformIPCore *getIP(DFGNullaryAtomicNode *Op) const { return PCtx.getIP(Op); }
  PlatformIPCore *getIP(DFGUnaryAtomicNode *Op) const { return PCtx.getIP(Op); }
  PlatformIPCore *getIP(DFGBinaryAtomicNode *Op) const { return PCtx.getIP(Op); }
  unsigned getLmax(DFGNode *Node) const { return PCtx.getLmax(Node); }

  IndexedVarDecl *getGlobalIDVar(unsigned Index) const {
    return VVars.getIndexedGlobalID(Index);
  }
  IndexedVarDecl *getLocalIDVar(unsigned Index) const {
    return VVars.getIndexedLocalID(Index);
  }
  IndexedVarDecl *getWorkGroupIDVar(unsigned Index) const {
    return VVars.getIndexedWorkGroupID(Index);
  }
  IndexedVarDecl *getGlobalSizeVar(unsigned Index) const {
    return VVars.getIndexedGlobalSize(Index);
  }
  IndexedVarDecl *getLocalSizeVar(unsigned Index) const {
    return VVars.getIndexedLocalSize(Index);
  }
  IndexedVarDecl *getNumWorkGroupsVar(unsigned Index) const {
    return VVars.getIndexedNumWorkGroups(Index);
  }
  IndexedVarDecl *getFlatLocalIDVar() const {
    return VVars.getIndexedFlatLocalID();
  }
  IndexedVarDecl *getFlatWorkGroupIDVar() const {
    return VVars.getIndexedFlatWorkGroupID();
  }
  IndexedVarDecl *getFlatLocalSizeVar() const {
    return VVars.getIndexedFlatLocalSize();
  }

  unsigned getTopLevelSingleWorkGroupGranularity() const;

  const LocalMemoryLayout &getLocalMemoryLayout() const;
  void RegisterLocalMemoryLayout(const LocalMemoryLayout *Layout);

  uint64_t getAddressOf(WVarDecl *Var) const;
  unsigned getVirtualAddressWidthFor(DFGMemoryAccessNode *Node) const;
  unsigned getPhysicalAddressWidthFor(DFGMemoryAccessNode *Node) const;
  unsigned getDataWidthFor(DFGMemoryAccessNode *Node) const;
};

/*
 * ModuleBuilder
 *
 * module foo(clk, rstn, ...)
 *   [wire assignments]
 *   always @(posedge clk) begin
 *     if (~rstn) begin
 *       [register resets]
 *     end
 *     else begin
 *       [register assignments]
 *     end
 *   end
 * endmodule
 */

class ModuleBuilder : public VerilogModule {
  CodeGenContext &Ctx;
  VerilogSignal *Clock;
  VerilogSignal *Rstn;
  VerilogCompound *OnClock;
  std::map<std::string, unsigned> NamingCount;

public:
  ModuleBuilder(StringRef name, CodeGenContext &ctx);

  CodeGenContext &getCodeGenContext() const { return Ctx; }

  VerilogSignal *clock() const { return Clock; }
  VerilogSignal *rstn() const { return Rstn; }

  void addStmtOnClock(VerilogStmt *S) {
    OnClock->addStmt(S);
  }

  void addAssignWire(VerilogSignal *Target, VerilogExprPtrOrSignalPtr Value);
  void addAssignReg(VerilogSignal *Target, VerilogExprPtrOrSignalPtr Value);
  void addAssignRegOnEnable(VerilogExprPtrOrSignalPtr Enable,
                            VerilogSignal *Target,
                            VerilogExprPtrOrSignalPtr Value);
  void addAssignRegStmt(VerilogSignal *Target, VerilogStmt *Stmt);

  VerilogSignalRef *CreateClockRef();
  VerilogSignalRef *CreateRstnRef();
  VerilogSignalRef *CreateRstnRef(StringRef rstn_name);

  VerilogSignal *CreateWire(VerilogSignalPortKind portKind, StringRef name) {
    return VerilogModule::CreateWire(portKind, name);
  }
  VerilogSignal *CreateWire(VerilogSignalPortKind portKind, StringRef name,
                            VerilogParamConstOrInt vectorWidth) {
    return VerilogModule::CreateWire(portKind, name, vectorWidth);
  }
  VerilogSignal *CreateWire(VerilogSignalPortKind portKind, StringRef name,
                            VerilogParamConstOrInt vectorWidth,
                            VerilogParamConstOrInt arrayWidth) {
    return VerilogModule::CreateWire(portKind, name, vectorWidth, arrayWidth);
  }
  VerilogSignal *CreateReg(VerilogSignalPortKind portKind, StringRef name) {
    return VerilogModule::CreateReg(portKind, name);
  }
  VerilogSignal *CreateReg(VerilogSignalPortKind portKind, StringRef name,
                           VerilogParamConstOrInt vectorWidth) {
    return VerilogModule::CreateReg(portKind, name, vectorWidth);
  }
  VerilogSignal *CreateReg(VerilogSignalPortKind portKind, StringRef name,
                           VerilogParamConstOrInt vectorWidth,
                           VerilogParamConstOrInt arrayWidth) {
    return VerilogModule::CreateReg(portKind, name, vectorWidth, arrayWidth);
  }

  VerilogSignal *CreateTempWire(StringRef namePrefix);
  VerilogSignal *CreateTempWire(StringRef namePrefix,
                                VerilogParamConstOrInt vectorWidth);
  VerilogSignal *CreateTempWire(StringRef namePrefix,
                                VerilogParamConstOrInt vectorWidth,
                                VerilogParamConstOrInt arrayWidth);
  VerilogSignal *CreateTempReg(StringRef namePrefix);
  VerilogSignal *CreateTempReg(StringRef namePrefix,
                               VerilogParamConstOrInt vectorWidth);
  VerilogSignal *CreateTempReg(StringRef namePrefix,
                               VerilogParamConstOrInt vectorWidth,
                               VerilogParamConstOrInt arrayWidth);

  VerilogSignal *CreateTempWireFor(IndexedVarDecl *Var, StringRef namePrefix);
  VerilogSignal *CreateTempWireFor(DFGNode *Node, StringRef namePrefix);
  VerilogSignal *CreateTempRegFor(IndexedVarDecl *Var, StringRef namePrefix);
  VerilogSignal *CreateTempRegFor(DFGNode *Node, StringRef Prefix);

  VerilogSignalRef *CreateSignalRef(StringRef name) {
    return new VerilogSignalRef(getSignal(name));
  }

  VerilogModuleInstance *CreateTempModuleInstance(VerilogModule *target,
                                                  StringRef namePrefix);

  std::string AcquireNameWithPrefix(StringRef prefix);

  VerilogModuleInstance *CreateIPInstance(StringRef IPName, StringRef Prefix);
  VerilogModuleInstance *CreateIPInstance(DFGUnaryOpNode *Op, StringRef Prefix);
  VerilogModuleInstance *CreateIPInstance(DFGBinaryOpNode *Op,
                                          StringRef Prefix);
  VerilogModuleInstance *CreateIPInstance(DFGTernaryOpNode *Op,
                                          StringRef Prefix);
  VerilogModuleInstance *CreateIPInstance(DFGNullaryAtomicNode *Op,
                                          StringRef Prefix);
  VerilogModuleInstance *CreateIPInstance(DFGUnaryAtomicNode *Op,
                                          StringRef Prefix);
  VerilogModuleInstance *CreateIPInstance(DFGBinaryAtomicNode *Op,
                                          StringRef Prefix);

  // Commonly used stuffs
  void InsertGatekeeper(VerilogSignal *ValidFrom, VerilogSignal *WaitFrom,
                        VerilogSignal *ValidTo, VerilogSignal *WaitTo,
                        VerilogExprPtrOrSignalPtr Pass);
  VerilogSignal *CreateCounter(StringRef Prefix, VerilogParamConstOrInt Width,
                               VerilogExprPtrOrSignalPtr Increment,
                               VerilogExprPtrOrSignalPtr Decrement);
  VerilogSignal *CreateCounter(StringRef Prefix, VerilogParamConstOrInt Width,
                               ArrayRef<VerilogExprPtrOrSignalPtr> Increments,
                               ArrayRef<VerilogExprPtrOrSignalPtr> Decrements);
  void InsertFIFOQueue(StringRef Prefix, VerilogParamConstOrInt Width,
                       unsigned MaxLatency, VerilogExprPtrOrSignalPtr In,
                       VerilogExprPtrOrSignalPtr ValidIn, VerilogSignal *WaitIn,
                       VerilogExprPtrOrSignalPtr Out, VerilogSignal *ValidOut,
                       VerilogExprPtrOrSignalPtr WaitOut);
  void InsertDatalessFIFOQueue(unsigned MaxLatency,
                               VerilogExprPtrOrSignalPtr ValidIn,
                               VerilogSignal *WaitIn, VerilogSignal *ValidOut,
                               VerilogExprPtrOrSignalPtr WaitOut);

};

} // namespace Synthesis

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_SYNTHESIS_CGDATAPATH_H
