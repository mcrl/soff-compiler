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

#ifndef LLVM_CLANG_SNU_SYNTHESIS_PLATFORMCONTEXT_H
#define LLVM_CLANG_SNU_SYNTHESIS_PLATFORMCONTEXT_H

#include "clang/AST/ASTContext.h"
#include "clang/AST/Decl.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuSynthesis/DataflowGraph.h"
#include "clang/SnuSynthesis/Verilog.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/StringRef.h"
#include <map>
#include <string>

namespace clang {

namespace snu {

class PlatformVariables {
  std::map<std::string, unsigned> Var;

public:
  explicit PlatformVariables(StringRef SetupFile);

  unsigned GetVar(StringRef key) const;
  unsigned GetVar(StringRef group, StringRef sub) const;

private:
  bool LoadTargetVars(StringRef TargetFile);
};

class PlatformIPCore {
public:
  enum LatencyKind {
   LK_Undefined,
   LK_Fixed,
   LK_Variable,
   LK_FixedSingle
  };

private:
  VerilogModule *Module;
  unsigned Lmax;
  LatencyKind Lkind;

public:
  explicit PlatformIPCore(VerilogModule *module);
  PlatformIPCore(VerilogModule *module, unsigned lmax, LatencyKind lkind);

  VerilogModule *getModule() const { return Module; }
  StringRef getName() const { return Module->getName(); }
  unsigned getLmax() const;
  LatencyKind getLkind() const { return Lkind; }
  bool isVariableLatency() const { return Lkind == LK_Variable; }
  unsigned getNstall() const;

  static LatencyKind InterpretAsLkind(unsigned v);
  static bool IsVariableLatency(LatencyKind Lkind) {
    return Lkind == LK_Variable;
  }
};

class PlatformIPCollection {
  const ASTContext &ASTCtx;
  const PlatformVariables &PVars;

  // key strings are allocated in PlatformIPCore objects
  std::map<StringRef, PlatformIPCore*> IP;

public:
  PlatformIPCollection(const ASTContext &C, const PlatformVariables &V);

  PlatformIPCore *getIP(StringRef Name);
  PlatformIPCore *getIP(DFGUnaryOpNode *Op);
  PlatformIPCore *getIP(DFGBinaryOpNode *Op);
  PlatformIPCore *getIP(DFGTernaryOpNode *Op);
  PlatformIPCore *getIP(DFGNullaryAtomicNode *Op);
  PlatformIPCore *getIP(DFGUnaryAtomicNode *Op);
  PlatformIPCore *getIP(DFGBinaryAtomicNode *Op);

private:
  void AddUnaryOpIP(StringRef name, unsigned aWidth, unsigned qWidth);
  void AddBinaryOpIP(StringRef name, unsigned aWidth, unsigned bWidth,
                     unsigned qWidth);
  void AddTernaryOpIP(StringRef name, unsigned aWidth, unsigned bWidth,
                      unsigned cWidth, unsigned qWidth);
  void AddNullaryAtomicIP(StringRef name, unsigned width);
  void AddUnaryAtomicIP(StringRef name, unsigned width);
  void AddBinaryAtomicIP(StringRef name, unsigned width);

  std::string GetTypeCode(QualType Ty);
  std::string GetAddressSpaceCode(unsigned AS);
};

class PlatformContext {
  const ASTContext &ASTCtx;
  const PlatformVariables &PVars;
  PlatformIPCollection &PIPs;
  FunctionDecl *Kernel;

  unsigned LIDWidth;
  llvm::DenseMap<DFGNode*, unsigned> LmaxCache;
  llvm::DenseMap<DFGNode*, bool> LvarCache;
  llvm::DenseMap<DFGNode*, unsigned> NstallCache;

public:
  PlatformContext(const ASTContext &C, const PlatformVariables &PV,
                  PlatformIPCollection &PI, FunctionDecl *K);

  unsigned getResetDelay() const { return PVars.GetVar("reset", "delay"); }
  unsigned getResetAttribute() const { return PVars.GetVar("reset", "attribute"); }
  bool isMemUseAXI() const { return PVars.GetVar("mem", "use_axi"); }
  unsigned getMemNumPorts() const { return PVars.GetVar("mem", "num_ports"); }
  unsigned getMemAddressWidth() const { return PVars.GetVar("mem", "address_width"); }
  unsigned getMemDataWidth() const { return PVars.GetVar("mem", "data_width"); }
  unsigned getMemBurstWidth() const { return PVars.GetVar("mem", "burst_width"); }
  unsigned getPortGIDWidth() const { return PVars.GetVar("port", "gid_width"); }
  unsigned getPortLIDWidth() const { return LIDWidth; }
  unsigned getCapacityLocalMemChunk() const {
    return PVars.GetVar("capacity", "local_mem_chunk");
  }

  PlatformIPCore *getIP(StringRef Name) { return PIPs.getIP(Name); }
  PlatformIPCore *getIP(DFGUnaryOpNode *Op) { return PIPs.getIP(Op); }
  PlatformIPCore *getIP(DFGBinaryOpNode *Op) { return PIPs.getIP(Op); }
  PlatformIPCore *getIP(DFGTernaryOpNode *Op) { return PIPs.getIP(Op); }
  PlatformIPCore *getIP(DFGNullaryAtomicNode *Op) { return PIPs.getIP(Op); }
  PlatformIPCore *getIP(DFGUnaryAtomicNode *Op) { return PIPs.getIP(Op); }
  PlatformIPCore *getIP(DFGBinaryAtomicNode *Op) { return PIPs.getIP(Op); }

  unsigned getLmax(DFGNode *Node);
  bool isVariableLatency(DFGNode *Node);
  unsigned getNstall(DFGNode *Node);

private:
  unsigned ComputeLmax(DFGNode *Node);
  bool ComputeIsVariableLatency(DFGNode *Node);
  unsigned ComputeNstall(DFGNode *Node);
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_SYNTHESIS_PLATFORMCONTEXT_H
