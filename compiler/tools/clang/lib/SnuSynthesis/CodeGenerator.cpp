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

#include "clang/SnuSynthesis/CodeGenerator.h"
#include "CGCommon.h"
#include "CGControlFlowUnit.h"
#include "CGLockSubsystem.h"
#include "CGMemorySubsystem.h"
#include "clang/AST/Decl.h"
#include "clang/AST/Type.h"
#include "clang/Basic/AddressSpaces.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuAST/WCFG.h"
#include "clang/SnuAnalysis/PointerAnalysis.h"
#include "clang/SnuFrontend/Options.h"
#include "clang/SnuSynthesis/DataflowGraph.h"
#include "clang/SnuSynthesis/PlatformContext.h"
#include "clang/SnuSynthesis/StructuralAnalysis.h"
#include "clang/SnuSynthesis/Verilog.h"
#include "clang/SnuSynthesis/VirtualVariables.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/StringRef.h"
#include <string>

namespace clang {

namespace snu {

namespace {

using namespace Synthesis;

class DatapathModule: public ModuleBuilder {
  CodeGenContext &Ctx;
  VerilogSignal *ValidIn;
  VerilogSignal *ValidOut;
  VerilogSignal *WaitIn;
  llvm::DenseMap<const IndexedVarDecl*, VerilogSignal*> WorkItemArg;
  VerilogSignal *KernelArg;
  unsigned KernelArgWidth;
  llvm::DenseMap<const WVarDecl*, unsigned> KernelArgOffset;

  SmallVector<AMMInterfaceBase*, 64> MemPorts;
  SmallVector<BMLInterface*, 64> LockPorts;

public:
  DatapathModule(CodeGenContext &ctx, FunctionDecl *KernelFunc,
                 ControlTreeNode *BodyTree, const WDeclContext &DeclCtx);

  unsigned getKernelArgWidth() const { return KernelArgWidth; }

  typedef SmallVectorImpl<AMMInterfaceBase*>::iterator mem_port_iterator;
  typedef SmallVectorImpl<AMMInterfaceBase*>::const_iterator
      const_mem_port_iterator;

  mem_port_iterator mem_port_begin() { return MemPorts.begin(); }
  mem_port_iterator mem_port_end() { return MemPorts.end(); }
  const_mem_port_iterator mem_port_begin() const { return MemPorts.begin(); }
  const_mem_port_iterator mem_port_end() const { return MemPorts.end(); }

  typedef SmallVectorImpl<BMLInterface*>::iterator lock_port_iterator;
  typedef SmallVectorImpl<BMLInterface*>::const_iterator
      const_lock_port_iterator;

  lock_port_iterator lock_port_begin() { return LockPorts.begin(); }
  lock_port_iterator lock_port_end() { return LockPorts.end(); }
  const_lock_port_iterator lock_port_begin() const { return LockPorts.begin(); }
  const_lock_port_iterator lock_port_end() const { return LockPorts.end(); }

private:
  void addWorkItemArgs();
  void addKernelArgs(FunctionDecl *KernelFunc, const WDeclContext &DeclCtx);

  void ConnectWorkItemArg(IndexedVarDecl *InVar, VerilogSignal *In);
  void ConnectKernelArg(IndexedVarDecl *InVar, VerilogSignal *In);
  void ConnectLocalMemoryPointerArg(IndexedVarDecl *InVar, VerilogSignal *In);
};

DatapathModule::DatapathModule(CodeGenContext &ctx, FunctionDecl *KernelFunc,
                               ControlTreeNode *BodyTree,
                               const WDeclContext &DeclCtx)
  : ModuleBuilder("datapath_" + KernelFunc->getNameAsString(), ctx), Ctx(ctx) {
  ValidIn = CreateWire(SP_Input, "valid_in");
  ValidOut = CreateWire(SP_Output, "valid_out");
  WaitIn = CreateWire(SP_Output, "wait_in");
  addWorkItemArgs();
  addKernelArgs(KernelFunc, DeclCtx);

  DPControlFlowUnit *Impl = new DPControlFlowUnit(Ctx, this, BodyTree);
  assert(Impl->getNumOutputPorts() == 1);
  addAssignWire(Impl->valid_in(), ValidIn);
  addAssignWire(WaitIn, Impl->wait_in());
  addAssignWire(ValidOut, Impl->valid_out(0));
  addAssignWire(Impl->wait_out(0), CreateZero());
  for (DPControlFlowUnit::port_iterator P = Impl->input_begin(),
                                        PEnd = Impl->input_end();
       P != PEnd; ++P) {
    IndexedVarDecl *InVar = P->first;
    VerilogSignal *In = P->second;
    if (InVar->isParameter()) {
      ConnectKernelArg(InVar, In);
    } else if (isa<WVirtualVarDecl>(InVar->getDecl())) {
      ConnectWorkItemArg(InVar, In);
    } else {
      addAssignWire(In, CreateDecimalZero());
    }
  }

  for (DPControlFlowUnit::mem_iterator M = Impl->mem_begin(),
                                       MEnd = Impl->mem_end();
       M != MEnd; ++M) {
    AMMInterfaceBase *Port = (*M)->Clone(this, AcquireNameWithPrefix("avm"),
                                         AMMInterfaceBase::AMMI_PK_MasterPort);
    (*M)->ConnectToSlave(Port);
    MemPorts.push_back(Port);
  }

  for (DPControlFlowUnit::lock_iterator L = Impl->lock_begin(),
                                        LEnd = Impl->lock_end();
       L != LEnd; ++L) {
    BMLInterface *Port = (*L)->Clone(this, AcquireNameWithPrefix("bym"),
                                     BMLInterface::BMLI_PK_MasterPort);
    (*L)->ConnectToSlave(Port);
    LockPorts.push_back(Port);
  }
}

void DatapathModule::addWorkItemArgs() {
  WorkItemArg[Ctx.getGlobalSizeVar(0)] = CreateWire(SP_Input, "global_size_0", Ctx.getPortGIDWidth());
  WorkItemArg[Ctx.getGlobalSizeVar(1)] = CreateWire(SP_Input, "global_size_1", Ctx.getPortGIDWidth());
  WorkItemArg[Ctx.getGlobalSizeVar(2)] = CreateWire(SP_Input, "global_size_2", Ctx.getPortGIDWidth());
  WorkItemArg[Ctx.getLocalSizeVar(0)] = CreateWire(SP_Input, "local_size_0", Ctx.getPortLIDWidth() + 1);
  WorkItemArg[Ctx.getLocalSizeVar(1)] = CreateWire(SP_Input, "local_size_1", Ctx.getPortLIDWidth() + 1);
  WorkItemArg[Ctx.getLocalSizeVar(2)] = CreateWire(SP_Input, "local_size_2", Ctx.getPortLIDWidth() + 1);
  WorkItemArg[Ctx.getNumWorkGroupsVar(0)] = CreateWire(SP_Input, "num_groups_0", Ctx.getPortGIDWidth());
  WorkItemArg[Ctx.getNumWorkGroupsVar(1)] = CreateWire(SP_Input, "num_groups_1", Ctx.getPortGIDWidth());
  WorkItemArg[Ctx.getNumWorkGroupsVar(2)] = CreateWire(SP_Input, "num_groups_2", Ctx.getPortGIDWidth());
  WorkItemArg[Ctx.getGlobalIDVar(0)] = CreateWire(SP_Input, "global_id_0", Ctx.getPortGIDWidth());
  WorkItemArg[Ctx.getGlobalIDVar(1)] = CreateWire(SP_Input, "global_id_1", Ctx.getPortGIDWidth());
  WorkItemArg[Ctx.getGlobalIDVar(2)] = CreateWire(SP_Input, "global_id_2", Ctx.getPortGIDWidth());
  WorkItemArg[Ctx.getLocalIDVar(0)] = CreateWire(SP_Input, "local_id_0", Ctx.getPortLIDWidth());
  WorkItemArg[Ctx.getLocalIDVar(1)] = CreateWire(SP_Input, "local_id_1", Ctx.getPortLIDWidth());
  WorkItemArg[Ctx.getLocalIDVar(2)] = CreateWire(SP_Input, "local_id_2", Ctx.getPortLIDWidth());
  WorkItemArg[Ctx.getWorkGroupIDVar(0)] = CreateWire(SP_Input, "group_id_0", Ctx.getPortGIDWidth());
  WorkItemArg[Ctx.getWorkGroupIDVar(1)] = CreateWire(SP_Input, "group_id_1", Ctx.getPortGIDWidth());
  WorkItemArg[Ctx.getWorkGroupIDVar(2)] = CreateWire(SP_Input, "group_id_2", Ctx.getPortGIDWidth());
  WorkItemArg[Ctx.getFlatLocalIDVar()] = CreateWire(SP_Input, "flat_local_id", Ctx.getPortLIDWidth());
  WorkItemArg[Ctx.getFlatWorkGroupIDVar()] = CreateWire(SP_Input, "flat_group_id", Ctx.getPortNIDWidth());
  WorkItemArg[Ctx.getFlatLocalSizeVar()] = CreateWire(SP_Input, "flat_local_size", Ctx.getPortLIDWidth() + 1);
}

void DatapathModule::addKernelArgs(FunctionDecl *KernelFunc,
                                   const WDeclContext &DeclCtx) {
  KernelArgWidth = 0;
  KernelArgOffset.clear();
  for (FunctionDecl::param_const_iterator P = KernelFunc->param_begin(),
                                          PEnd = KernelFunc->param_end();
       P != PEnd; ++P) {
    // Param can be null if *P does not used
    if (WVarDecl *Param = DeclCtx.Lookup(*P)) {
      KernelArgOffset[Param] = KernelArgWidth;
      if (Param->isCompound()) {
        for (unsigned Index = 0, NumSubVars = Param->getNumSubVars();
             Index != NumSubVars; ++Index) {
          WSubVarDecl *SubParam = Param->getSubVar(Index);
          KernelArgOffset[SubParam] = KernelArgWidth +
                                      Ctx.getSubVarOffset(SubParam);
        }
      }
    }
    KernelArgWidth += (unsigned)Ctx.getTypeSize((*P)->getType());
  }
  KernelArg = CreateWire(SP_Input, "kernel_arg", KernelArgWidth);
}

void DatapathModule::ConnectWorkItemArg(IndexedVarDecl *InVar,
                                        VerilogSignal *In) {
  assert(WorkItemArg.count(InVar));
  addAssignWire(In, WorkItemArg[InVar]);
}

void DatapathModule::ConnectKernelArg(IndexedVarDecl *InVar,
                                      VerilogSignal *In) {
  // Local memory pointers are initialized by the compiler
  QualType InTy = InVar->getType();
  if (InTy->isPointerType() &&
      InTy->getPointeeType().getAddressSpace() == LangAS::opencl_local) {
    ConnectLocalMemoryPointerArg(InVar, In);
    return;
  }

  assert(KernelArgOffset.count(InVar->getDecl()));
  unsigned Offset = KernelArgOffset[InVar->getDecl()];
  unsigned Width = (unsigned)Ctx.getTypeSize(InVar);
  assert(Width == In->getVectorWidthValue());
  assert(Offset + Width <= KernelArgWidth);
  addAssignWire(In,
    new VerilogSignalRef(KernelArg, VerilogParamConst(Offset + Width - 1),
                         VerilogParamConst(Offset))
  );
}

void DatapathModule::ConnectLocalMemoryPointerArg(IndexedVarDecl *InVar,
                                                  VerilogSignal *In) {
  addAssignWire(In,
    new VerilogConst(VR_Hexa, Ctx.getAddressOf(InVar->getDecl()),
                     In->getVectorWidthValue())
  );
}

class KernelModule: public ModuleBuilder {
  CodeGenContext &Ctx;
  DatapathModule *Datapath;
  const unsigned NumInstances;

  GlobalMemorySubsystem *GlobalMem;
  LockSubsystem *Lock;

public:
  KernelModule(CodeGenContext &ctx, FunctionDecl *KernelFunc,
               ControlTreeNode *BodyTree, DatapathModule *datapath,
               const AliasSet &Aliases);

private:
  void addOpenCLPorts();
  void addGlobalMemoryPorts();
  void addOpenCLSignals();

  void SynthesizeDispatchers();
  void SynthesizeWorkGroupDispatcher();
  void SynthesizeWorkItemDispatcher(unsigned Index);
  void SynthesizeDatapathInstance(unsigned Index);

  VerilogSignalRef *CreateSubArrayOf(VerilogSignal *Array, unsigned From,
                                     unsigned To);
};

KernelModule::KernelModule(CodeGenContext &ctx, FunctionDecl *KernelFunc,
                           ControlTreeNode *BodyTree, DatapathModule *datapath,
                           const AliasSet &Aliases)
  : ModuleBuilder("kernel_" + KernelFunc->getNameAsString(), ctx), Ctx(ctx),
    Datapath(datapath), NumInstances(Ctx.getNumDatapathInstances()) {
  GlobalMem = new GlobalMemorySubsystem(this);
  Lock = new LockSubsystem(this);

  CreateWire(SP_Input, "clk2x");
  addOpenCLPorts();
  addGlobalMemoryPorts();
  addOpenCLSignals();
  for (unsigned Index = 0; Index != NumInstances; ++Index) {
    SynthesizeDatapathInstance(Index);
  }

  VerilogSignal *PItems = CreateReg(SP_None, "processed_work_items", Ctx.getPortNIDWidth());
  VerilogExpr *PItemsExpr = new VerilogSignalRef(PItems);
  for (unsigned Index = 0; Index != NumInstances; ++Index) {
    PItemsExpr = CreateAddOf(PItemsExpr, CreateArrayElementOf(getSignal("dp_valid_out"), Index));
  }
  addAssignRegStmt(PItems,
    // if (opencl_complete)
    new VerilogIf(CreateSignalRef("opencl_complete"),
      // begin
      (new VerilogCompound())->addStmt(
        // processed_work_items <= 0;
        CreateAssignOf(PItems, CreateDecimalZero())
      ),
      // end
      // else begin
      (new VerilogCompound())->addStmt(
        // processed_work_items <= processed_work_items +
        //                         dp_valid_out[0] + dp_valid_out[1] + ...;
        CreateAssignOf(PItems, PItemsExpr)
      )
      // end
    )
  );
  addAssignWire(getSignal("opencl_complete"),
    CreateEqualOf(PItems, getSignal("opencl_num_work_items"))
  );

  GlobalMem->Finalize(getSignal("opencl_clean"), getSignal("opencl_cleaned"));
  Lock->Finalize();

  SynthesizeDispatchers();

  addAssignWire(getSignal("opencl_pc"), CreateZero(PC_WIDTH));
}

void KernelModule::addOpenCLPorts() {
  CreateWire(SP_Input, "opencl_rstn");
  CreateWire(SP_Input, "opencl_on");
  CreateWire(SP_Output, "opencl_complete");
  CreateWire(SP_Input, "opencl_clean");
  CreateWire(SP_Output, "opencl_cleaned");
  CreateWire(SP_Input, "opencl_global_size_0", Ctx.getPortGIDWidth());
  CreateWire(SP_Input, "opencl_global_size_1", Ctx.getPortGIDWidth());
  CreateWire(SP_Input, "opencl_global_size_2", Ctx.getPortGIDWidth());
  CreateWire(SP_Input, "opencl_local_size_0", Ctx.getPortLIDWidth() + 1);
  CreateWire(SP_Input, "opencl_local_size_1", Ctx.getPortLIDWidth() + 1);
  CreateWire(SP_Input, "opencl_local_size_2", Ctx.getPortLIDWidth() + 1);
  CreateWire(SP_Input, "opencl_num_groups_0", Ctx.getPortGIDWidth());
  CreateWire(SP_Input, "opencl_num_groups_1", Ctx.getPortGIDWidth());
  CreateWire(SP_Input, "opencl_num_groups_2", Ctx.getPortGIDWidth());
  CreateWire(SP_Input, "opencl_num_work_items", Ctx.getPortNIDWidth());
  CreateWire(SP_Input, "opencl_num_work_groups", Ctx.getPortNIDWidth());
  CreateWire(SP_Input, "opencl_work_group_size", Ctx.getPortLIDWidth() + 1);
  CreateWire(SP_Input, "opencl_arg", ARG_WIDTH);
  CreateWire(SP_Output, "opencl_pc", PC_WIDTH);
}

void KernelModule::addGlobalMemoryPorts() {
  for (unsigned Index = 0, NumPorts = Ctx.getGlobalMemNumPorts();
       Index != NumPorts; ++Index) {
    if (Ctx.isGlobalMemUseAXI()) {
      GlobalMem->addGlobalMemoryPort(new AXIInterface(
          this, "mem_" + llvm::utostr_32(Index), Ctx.getGlobalMemAddressWidth(),
          Ctx.getGlobalMemDataWidth(), AXIInterface::AXII_PK_MasterPort));
    } else {
      GlobalMem->addGlobalMemoryPort(new AMMInterface(
          this, "mem_" + llvm::utostr_32(Index), Ctx.getGlobalMemAddressWidth(),
          Ctx.getGlobalMemDataWidth(), Ctx.getGlobalMemBurstWidth(),
          AMMInterface::AMMI_PK_MasterPort));
    }
  }
}

void KernelModule::addOpenCLSignals() {
  CreateWire(SP_None, "opencl_global_id_0", Ctx.getPortGIDWidth(), NumInstances);
  CreateWire(SP_None, "opencl_global_id_1", Ctx.getPortGIDWidth(), NumInstances);
  CreateWire(SP_None, "opencl_global_id_2", Ctx.getPortGIDWidth(), NumInstances);
  CreateWire(SP_None, "opencl_local_id_0", Ctx.getPortLIDWidth(), NumInstances);
  CreateWire(SP_None, "opencl_local_id_1", Ctx.getPortLIDWidth(), NumInstances);
  CreateWire(SP_None, "opencl_local_id_2", Ctx.getPortLIDWidth(), NumInstances);
  CreateWire(SP_None, "opencl_group_id_0", Ctx.getPortGIDWidth(), NumInstances);
  CreateWire(SP_None, "opencl_group_id_1", Ctx.getPortGIDWidth(), NumInstances);
  CreateWire(SP_None, "opencl_group_id_2", Ctx.getPortGIDWidth(), NumInstances);
  CreateWire(SP_None, "opencl_global_offset_0", Ctx.getPortGIDWidth(), NumInstances);
  CreateWire(SP_None, "opencl_global_offset_1", Ctx.getPortGIDWidth(), NumInstances);
  CreateWire(SP_None, "opencl_global_offset_2", Ctx.getPortGIDWidth(), NumInstances);
  CreateWire(SP_None, "opencl_flat_group_id", Ctx.getPortNIDWidth(), NumInstances);
  CreateWire(SP_None, "opencl_flat_local_id", Ctx.getPortLIDWidth(), NumInstances);
  CreateWire(SP_None, "dp_valid_in", 1, NumInstances);
  CreateWire(SP_None, "dp_wait_in", 1, NumInstances);
  CreateWire(SP_None, "dp_valid_out", 1, NumInstances);
}

void KernelModule::SynthesizeDispatchers() {
  CreateWire(SP_None, "wgdisp_valid_out", 1, NumInstances);
  CreateWire(SP_None, "wgdisp_wait_out", 1, NumInstances);
  SynthesizeWorkGroupDispatcher();
  for (unsigned Index = 0; Index != NumInstances; ++Index) {
    SynthesizeWorkItemDispatcher(Index);
  }
}

void KernelModule::SynthesizeWorkGroupDispatcher() {
  VerilogModuleInstance *Impl = CreateIPInstance("work_group_dispatcher", "wgdisp_inst");
  Impl->addParamArgument("NUM_OUTPUT", VerilogParamConst(NumInstances));
  Impl->addParamArgument("GWIDTH", VerilogParamConst(Ctx.getPortGIDWidth()));
  Impl->addParamArgument("LWIDTH", VerilogParamConst(Ctx.getPortLIDWidth()));
  Impl->addArgument("clk", CreateClockRef());
  Impl->addArgument("rstn", CreateRstnRef("opencl_rstn"));
  Impl->addArgument("trigger", CreateSignalRef("opencl_on"));
  Impl->addArgument("num_groups_0", CreateSignalRef("opencl_num_groups_0"));
  Impl->addArgument("num_groups_1", CreateSignalRef("opencl_num_groups_1"));
  Impl->addArgument("num_groups_2", CreateSignalRef("opencl_num_groups_2"));
  Impl->addArgument("local_size_0", CreateSignalRef("opencl_local_size_0"));
  Impl->addArgument("local_size_1", CreateSignalRef("opencl_local_size_1"));
  Impl->addArgument("local_size_2", CreateSignalRef("opencl_local_size_2"));
  Impl->addArgument("group_id_0", CreateSignalRef("opencl_group_id_0"));
  Impl->addArgument("group_id_1", CreateSignalRef("opencl_group_id_1"));
  Impl->addArgument("group_id_2", CreateSignalRef("opencl_group_id_2"));
  Impl->addArgument("global_offset_0", CreateSignalRef("opencl_global_offset_0"));
  Impl->addArgument("global_offset_1", CreateSignalRef("opencl_global_offset_1"));
  Impl->addArgument("global_offset_2", CreateSignalRef("opencl_global_offset_2"));
  Impl->addArgument("flat_group_id", CreateSignalRef("opencl_flat_group_id"));
  Impl->addArgument("valid_out", CreateSignalRef("wgdisp_valid_out"));
  Impl->addArgument("wait_out", CreateSignalRef("wgdisp_wait_out"));
}

void KernelModule::SynthesizeWorkItemDispatcher(unsigned Index) {
  VerilogModuleInstance *Impl = CreateIPInstance("work_item_dispatcher", "widisp_inst");
  Impl->addParamArgument("GWIDTH", VerilogParamConst(Ctx.getPortGIDWidth()));
  Impl->addParamArgument("LWIDTH", VerilogParamConst(Ctx.getPortLIDWidth()));
  Impl->addArgument("clk", CreateClockRef());
  Impl->addArgument("rstn", CreateRstnRef("opencl_rstn"));
  Impl->addArgument("local_size_0", CreateSignalRef("opencl_local_size_0"));
  Impl->addArgument("local_size_1", CreateSignalRef("opencl_local_size_1"));
  Impl->addArgument("local_size_2", CreateSignalRef("opencl_local_size_2"));
  Impl->addArgument("global_offset_0", CreateArrayElementOf(getSignal("opencl_global_offset_0"), Index));
  Impl->addArgument("global_offset_1", CreateArrayElementOf(getSignal("opencl_global_offset_1"), Index));
  Impl->addArgument("global_offset_2", CreateArrayElementOf(getSignal("opencl_global_offset_2"), Index));
  Impl->addArgument("valid_in", CreateArrayElementOf(getSignal("wgdisp_valid_out"), Index));
  Impl->addArgument("wait_in", CreateArrayElementOf(getSignal("wgdisp_wait_out"), Index));
  Impl->addArgument("global_id_0", CreateArrayElementOf(getSignal("opencl_global_id_0"), Index));
  Impl->addArgument("global_id_1", CreateArrayElementOf(getSignal("opencl_global_id_1"), Index));
  Impl->addArgument("global_id_2", CreateArrayElementOf(getSignal("opencl_global_id_2"), Index));
  Impl->addArgument("local_id_0", CreateArrayElementOf(getSignal("opencl_local_id_0"), Index));
  Impl->addArgument("local_id_1", CreateArrayElementOf(getSignal("opencl_local_id_1"), Index));
  Impl->addArgument("local_id_2", CreateArrayElementOf(getSignal("opencl_local_id_2"), Index));
  Impl->addArgument("flat_local_id", CreateArrayElementOf(getSignal("opencl_flat_local_id"), Index));
  Impl->addArgument("valid_out", CreateArrayElementOf(getSignal("dp_valid_in"), Index));
  Impl->addArgument("wait_out", CreateArrayElementOf(getSignal("dp_wait_in"), Index));
}

void KernelModule::SynthesizeDatapathInstance(unsigned Index) {
  VerilogModuleInstance *Impl = CreateModuleInstance(
      Datapath, "datapath_inst_" + llvm::utostr_32(Index));
  Impl->addArgument("clk", CreateClockRef());
  Impl->addArgument("rstn", CreateSignalRef("opencl_rstn"));
  Impl->addArgument("global_size_0", CreateSignalRef("opencl_global_size_0"));
  Impl->addArgument("global_size_1", CreateSignalRef("opencl_global_size_1"));
  Impl->addArgument("global_size_2", CreateSignalRef("opencl_global_size_2"));
  Impl->addArgument("local_size_0", CreateSignalRef("opencl_local_size_0"));
  Impl->addArgument("local_size_1", CreateSignalRef("opencl_local_size_1"));
  Impl->addArgument("local_size_2", CreateSignalRef("opencl_local_size_2"));
  Impl->addArgument("num_groups_0", CreateSignalRef("opencl_num_groups_0"));
  Impl->addArgument("num_groups_1", CreateSignalRef("opencl_num_groups_1"));
  Impl->addArgument("num_groups_2", CreateSignalRef("opencl_num_groups_2"));
  Impl->addArgument("global_id_0", CreateArrayElementOf(getSignal("opencl_global_id_0"), Index));
  Impl->addArgument("global_id_1", CreateArrayElementOf(getSignal("opencl_global_id_1"), Index));
  Impl->addArgument("global_id_2", CreateArrayElementOf(getSignal("opencl_global_id_2"), Index));
  Impl->addArgument("local_id_0", CreateArrayElementOf(getSignal("opencl_local_id_0"), Index));
  Impl->addArgument("local_id_1", CreateArrayElementOf(getSignal("opencl_local_id_1"), Index));
  Impl->addArgument("local_id_2", CreateArrayElementOf(getSignal("opencl_local_id_2"), Index));
  Impl->addArgument("group_id_0", CreateArrayElementOf(getSignal("opencl_group_id_0"), Index));
  Impl->addArgument("group_id_1", CreateArrayElementOf(getSignal("opencl_group_id_1"), Index));
  Impl->addArgument("group_id_2", CreateArrayElementOf(getSignal("opencl_group_id_2"), Index));
  Impl->addArgument("flat_local_size", CreateSignalRef("opencl_work_group_size"));
  Impl->addArgument("flat_group_id", CreateArrayElementOf(getSignal("opencl_flat_group_id"), Index));
  Impl->addArgument("flat_local_id", CreateArrayElementOf(getSignal("opencl_flat_local_id"), Index));
  Impl->addArgument("kernel_arg",
    new VerilogSignalRef(getSignal("opencl_arg"),
                         VerilogParamConst(Datapath->getKernelArgWidth() - 1),
                         VerilogParamConst(0))
  );
  Impl->addArgument("valid_in", CreateArrayElementOf(getSignal("dp_valid_in"), Index));
  Impl->addArgument("wait_in", CreateArrayElementOf(getSignal("dp_wait_in"), Index));
  Impl->addArgument("valid_out", CreateArrayElementOf(getSignal("dp_valid_out"), Index));

  LocalMemorySubsystem *LocalMem = NULL;
  for (DatapathModule::const_mem_port_iterator M = Datapath->mem_port_begin(),
                                               MEnd = Datapath->mem_port_end();
       M != MEnd; ++M) {
    AMMInterfaceBase *Wire = (*M)->Clone(this, AcquireNameWithPrefix("dp_mem"));
    Wire->ConnectToModuleInstanceAsSlave(Impl, (*M)->getName());
    if (Wire->getAddressSpace() == LangAS::opencl_global) {
      GlobalMem->addInterface(Index, Wire);
    } else if (Wire->getAddressSpace() == LangAS::opencl_local) {
      if (LocalMem == NULL) {
        LocalMem = new LocalMemorySubsystem(this, Ctx.getLocalMemoryLayout());
      }
      LocalMem->addInterface(Wire);
    }
  }
  if (LocalMem != NULL) {
    LocalMem->Finalize();
  }

  for (DatapathModule::const_lock_port_iterator L = Datapath->lock_port_begin(),
                                                LEnd = Datapath->lock_port_end();
       L != LEnd; ++L) {
    BMLInterface *Wire = (*L)->Clone(this, AcquireNameWithPrefix("dp_lock"));
    Wire->ConnectToModuleInstance(Impl, (*L)->getName());
    Lock->addInterface(Wire);
  }
}

VerilogSignalRef *KernelModule::CreateSubArrayOf(VerilogSignal *Array,
                                                 unsigned From, unsigned To) {
  assert(From < To);
  assert(Array->isArray());
  VerilogSignal *SubArray = CreateTempWire("sub_" + Array->getNameAsString(), Array->getVectorWidth(), VerilogParamConst(To - From));
  for (unsigned Index = From; Index != To; ++Index) {
    addStmt(
      CreateAssignOf(CreateArrayElementOf(SubArray, Index - From),
                     CreateArrayElementOf(Array, Index))
    );
  }
  return new VerilogSignalRef(SubArray);
}

class OpenCLProgramModule: public VerilogModule {
  ArrayRef<VerilogModule*> Kernels;
  SmallVector<VerilogSignal*, 64> Inputs;
  SmallVector<VerilogSignal*, 64> Outputs;
  llvm::DenseMap<VerilogSignal*, VerilogExpr*> OutputExpr;

public:
  explicit OpenCLProgramModule(ArrayRef<VerilogModule*> K);

private:
  VerilogSignal *CreatePort(StringRef Name);
  void ConnectKernelOutput(VerilogModuleInstance *KernelInst,
                           VerilogSignal *Output, VerilogSignal *Select,
                           unsigned Index);
};

OpenCLProgramModule::OpenCLProgramModule(ArrayRef<VerilogModule*> K)
  : VerilogModule("kernel"), Kernels(K) {
  assert(!Kernels.empty());
  VerilogSignal *Clk = CreateWire(SP_Input, "clk");
  VerilogSignal *Rstn = CreateWire(SP_Input, "rstn");
  VerilogSignal *Select = CreateWire(SP_Input, "opencl_select",
                                     VerilogParamConst(8));
  VerilogSignal *On = CreateWire(SP_Input, "opencl_on");
  VerilogSignal *Clean = CreateWire(SP_Input, "opencl_clean");
  VerilogSignal *ReadDataValid = CreateWire(SP_Input, "mem_0_readdatavalid");
  for (VerilogModule::signal_iterator I = Kernels[0]->signal_begin(),
                                      E = Kernels[0]->signal_end();
       I != E; ++I) {
    VerilogSignal *S = *I;
    if (!S->isPort()) continue;
    CreatePort(S->getName());
  }
  for (unsigned Index = 0, NumKernels = Kernels.size();
       Index != NumKernels; ++Index) {
    VerilogModuleInstance *Inst = CreateModuleInstance(
        Kernels[Index], Kernels[Index]->getNameAsString() + "_inst");
    Inst->addArgument("clk", new VerilogSignalRef(Clk));
    Inst->addArgument("rstn", new VerilogSignalRef(Rstn));
    if (Kernels.size() == 1) {
      Inst->addArgument("opencl_on", new VerilogSignalRef(On));
      Inst->addArgument("opencl_clean", new VerilogSignalRef(Clean));
      Inst->addArgument("mem_0_readdatavalid", new VerilogSignalRef(ReadDataValid));
    } else {
      VerilogExpr *IndexExpr = new VerilogConst(VR_Binary, Index, 8);
      VerilogExpr *MaskExpr = CreateEqualOf(Select, IndexExpr);
      Inst->addArgument("opencl_on", CreateAndOf(On, MaskExpr));
      Inst->addArgument("opencl_clean", CreateAndOf(Clean, MaskExpr));
      Inst->addArgument("mem_0_readdatavalid", CreateAndOf(ReadDataValid, MaskExpr));
    }
    for (SmallVectorImpl<VerilogSignal*>::const_iterator I = Inputs.begin(),
                                                         IEnd = Inputs.end();
         I != IEnd; ++I) {
      Inst->addArgument((*I)->getName(), new VerilogSignalRef(*I));
    }
    for (SmallVectorImpl<VerilogSignal*>::const_iterator O = Outputs.begin(),
                                                         OEnd = Outputs.end();
         O != OEnd; ++O) {
      ConnectKernelOutput(Inst, *O, Select, Index);
    }
  }
  for (SmallVectorImpl<VerilogSignal*>::const_iterator O = Outputs.begin(),
                                                       OEnd = Outputs.end();
       O != OEnd; ++O) {
    assert(OutputExpr.count(*O));
    addStmt(CreateAssignOf(*O, OutputExpr[*O]));
  }
}

VerilogSignal *OpenCLProgramModule::CreatePort(StringRef Name) {
  if (hasSignal(Name)) {
    return NULL;
  }
  VerilogSignalPortKind PortKind = SP_None;
  VerilogParamConst VectorWidth;
  for (unsigned Index = 0, NumKernels = Kernels.size();
       Index != NumKernels; ++Index) {
    assert(Kernels[Index]->hasSignal(Name));
    VerilogSignal *S = Kernels[Index]->getSignal(Name);
    assert(S->isPort());
    assert(!S->isArray() && "not implemented yet");
    if (Index == 0) {
      PortKind = S->getPortKind();
      VectorWidth = S->getVectorWidth();
    } else {
      assert(PortKind == S->getPortKind());
      if (VectorWidth.isParametrized()) {
        assert(VectorWidth == S->getVectorWidth());
      } else {
        if (VectorWidth.getValue() < S->getVectorWidth().getValue()) {
          VectorWidth = S->getVectorWidth();
        }
      }
    }
  }
  VerilogSignal *Port = CreateWire(PortKind, Name, VectorWidth);
  if (PortKind == SP_Input) {
    Inputs.push_back(Port);
  } else {
    Outputs.push_back(Port);
  }
  return Port;
}

void OpenCLProgramModule::ConnectKernelOutput(VerilogModuleInstance *KernelInst,
                                              VerilogSignal *Output,
                                              VerilogSignal *Select,
                                              unsigned Index) {
  assert(!Output->isArray() && "not implemented yet");
  VerilogSignal *Output_Index = CreateWire(
      SP_None, Output->getNameAsString() + "_" + llvm::utostr_32(Index),
      Output->getVectorWidth());
  KernelInst->addArgument(Output->getName(), new VerilogSignalRef(Output_Index));
  if (OutputExpr.count(Output)) {
    VerilogExpr *IndexExpr = new VerilogConst(VR_Binary, Index, 8);
    OutputExpr[Output] = CreateConditionalOf(CreateEqualOf(Select, IndexExpr),
                                             Output_Index, OutputExpr[Output]);
  } else {
    OutputExpr[Output] = new VerilogSignalRef(Output_Index);
  }
}

} // anonymous namespace

OpenCLKernelArgMetadata::OpenCLKernelArgMetadata(ParmVarDecl *Param) {
  address_qualifier = 0x119E;
  access_qualifier = 0x11A3;
  type_name = "";
  type_qualifier = 0;
  name = Param->getName();
}

OpenCLKernelMetadata::OpenCLKernelMetadata(FunctionDecl *Func,
                                           PlatformContext &PCtx) {
  name = Func->getName();
  num_args = Func->getNumParams();
  attributes = "";
  work_group_size = (1 << PCtx.getPortLIDWidth());
  if (Func->hasAttr<ReqdWorkGroupSizeAttr>()) {
    ReqdWorkGroupSizeAttr *A = Func->getAttr<ReqdWorkGroupSizeAttr>();
    compile_work_group_size_0 = A->getXDim();
    compile_work_group_size_1 = A->getYDim();
    compile_work_group_size_2 = A->getZDim();
  } else {
    compile_work_group_size_0 = 0;
    compile_work_group_size_1 = 0;
    compile_work_group_size_2 = 0;
  }
  local_mem_size = PCtx.getCapacityLocalMemChunk();
  preferred_work_group_size_multiple = (1 << PCtx.getPortLIDWidth());
  private_mem_size = 0;
  for (unsigned Index = 0, NumParams = Func->getNumParams();
       Index != NumParams; ++Index) {
    args.push_back(OpenCLKernelArgMetadata(Func->getParamDecl(Index)));
  }
}

#define JSON_FIELD_I(key) "\"" #key "\": " << key << ""
#define JSON_FIELD_S(key) "\"" #key "\": \"" << key << "\""

void OpenCLKernelArgMetadata::print(raw_ostream &OS) const {
  OS << '{';
  OS << JSON_FIELD_I(address_qualifier) << ", ";
  OS << JSON_FIELD_I(access_qualifier) << ", ";
  OS << JSON_FIELD_S(type_name) << ", ";
  OS << JSON_FIELD_I(type_qualifier) << ", ";
  OS << JSON_FIELD_S(name);
  OS << '}';
}

void OpenCLKernelMetadata::print(raw_ostream &OS) const {
  OS << '{';
  OS << JSON_FIELD_S(name) << ", ";
  OS << JSON_FIELD_I(num_args) << ", ";
  OS << JSON_FIELD_S(attributes) << ", ";
  OS << JSON_FIELD_I(work_group_size) << ", ";
  OS << JSON_FIELD_I(compile_work_group_size_0) << ", ";
  OS << JSON_FIELD_I(compile_work_group_size_1) << ", ";
  OS << JSON_FIELD_I(compile_work_group_size_2) << ", ";
  OS << JSON_FIELD_I(local_mem_size) << ", ";
  OS << JSON_FIELD_I(preferred_work_group_size_multiple) << ", ";
  OS << JSON_FIELD_I(private_mem_size) << ", ";
  OS << "\"args\": [";
  for (SmallVectorImpl<OpenCLKernelArgMetadata>::const_iterator
           I = args.begin(), E = args.end();
       I != E; ++I) {
    if (I != args.begin()) OS << ", ";
    I->print(OS);
  }
  OS << ']';
  OS << '}';
}

#undef JSON_FIELD_I
#undef JSON_FIELD_S

void WFPCodeGenerator::HandleKernel(FunctionDecl *KernelFunc,
                                    ControlDataflowGraph *Body,
                                    ControlTreeNode *BodyTree,
                                    const WDeclContext &DeclCtx,
                                    const AliasSet &Aliases,
                                    PlatformContext &PCtx) {
  CodeGenContext Ctx(ASTCtx, PCtx, SnuCLOpts, Body, BodyTree);
  LocalMemoryLayout LMLayout(Ctx, Body, Aliases);
  Ctx.RegisterLocalMemoryLayout(&LMLayout);

  DatapathModule *Datapath = new DatapathModule(Ctx, KernelFunc, BodyTree, DeclCtx);
  KernelModule *Kernel = new KernelModule(Ctx, KernelFunc, BodyTree, Datapath, Aliases);

  KernelFuncs.push_back(KernelFunc);
  DatapathModules.push_back(Datapath);
  KernelModules.push_back(Kernel);
  KernelMetadatas.push_back(new OpenCLKernelMetadata(KernelFunc, PCtx));
}

VerilogFile *WFPCodeGenerator::ExportSingleKernelFile(FunctionDecl *KernelFunc) {
  for (unsigned Index = 0, NumKernels = KernelFuncs.size();
       Index != NumKernels; ++Index) {
    if (KernelFuncs[Index] == KernelFunc) {
      VerilogFile *File = new VerilogFile(KernelFunc->getNameAsString());
      File->addModule(DatapathModules[Index]);
      File->addModule(KernelModules[Index]);
      File->addModule(new OpenCLProgramModule(KernelModules[Index]));
      return File;
    }
  }
  llvm_unreachable("kernel not found");
}

VerilogFile *WFPCodeGenerator::ExportAllKernelFile() {
  VerilogFile *File = new VerilogFile("__all__");
  for (unsigned Index = 0, NumKernels = KernelFuncs.size();
       Index != NumKernels; ++Index) {
    File->addModule(DatapathModules[Index]);
    File->addModule(KernelModules[Index]);
  }
  File->addModule(new OpenCLProgramModule(KernelModules));
  return File;
}

void WFPCodeGenerator::PrintMetadata(raw_ostream &OS) const {
  OS << "[\n";
  for (unsigned Index = 0, NumKernels = KernelFuncs.size();
       Index != NumKernels; ++Index) {
    OS << "  ";
    KernelMetadatas[Index]->print(OS);
    if (Index + 1 != NumKernels) {
      OS << ',';
    }
    OS << '\n';
  }
  OS << "]\n";
}

} // namespace snu

} // namespace clang
