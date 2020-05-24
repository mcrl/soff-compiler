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

#ifndef LLVM_CLANG_SNU_SYNTHESIS_CODEGENERATOR_H
#define LLVM_CLANG_SNU_SYNTHESIS_CODEGENERATOR_H

#include "clang/AST/ASTContext.h"
#include "clang/AST/Decl.h"
#include "clang/Basic/LLVM.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include <string>

namespace clang {

namespace snu {

class AliasSet;
class ControlDataflowGraph;
class ControlTreeNode;
class PlatformContext;
class SnuCLOptions;
class VerilogFile;
class VerilogModule;
class WDeclContext;

class OpenCLKernelArgMetadata {
  unsigned address_qualifier;
  unsigned access_qualifier;
  std::string type_name;
  unsigned type_qualifier;
  std::string name;

public:
  explicit OpenCLKernelArgMetadata(ParmVarDecl *Param);

  void print(raw_ostream &OS) const;
};

class OpenCLKernelMetadata {
  std::string name;
  unsigned num_args;
  std::string attributes;
  size_t work_group_size;
  size_t compile_work_group_size_0;
  size_t compile_work_group_size_1;
  size_t compile_work_group_size_2;
  uint64_t local_mem_size;
  size_t preferred_work_group_size_multiple;
  uint64_t private_mem_size;
  SmallVector<OpenCLKernelArgMetadata, 64> args;

public:
  OpenCLKernelMetadata(FunctionDecl *Func, PlatformContext &PCtx);

  void print(raw_ostream &OS) const;
};

class WFPCodeGenerator {
  const ASTContext &ASTCtx;
  const SnuCLOptions &SnuCLOpts;

  SmallVector<FunctionDecl*, 16> KernelFuncs;
  SmallVector<VerilogModule*, 16> DatapathModules;
  SmallVector<VerilogModule*, 16> KernelModules;
  SmallVector<OpenCLKernelMetadata*, 16> KernelMetadatas;

public:
  WFPCodeGenerator(const ASTContext &AC, const SnuCLOptions &Opts)
    : ASTCtx(AC), SnuCLOpts(Opts) {}

  void HandleKernel(FunctionDecl *KernelFunc, ControlDataflowGraph *Body,
                    ControlTreeNode *BodyTree, const WDeclContext &DeclCtx,
                    const AliasSet &Aliases, PlatformContext &PCtx);

  VerilogFile *ExportSingleKernelFile(FunctionDecl *KernelFunc);
  VerilogFile *ExportAllKernelFile();
  void PrintMetadata(raw_ostream &OS) const;
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_SYNTHESIS_CODEGENERATOR_H
