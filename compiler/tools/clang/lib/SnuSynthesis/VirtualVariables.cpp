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

#include "clang/SnuSynthesis/VirtualVariables.h"
#include "clang/AST/ASTContext.h"
#include "clang/SnuAST/WAST.h"

namespace clang {

namespace snu {

VirtualVariablePool::VirtualVariablePool(const ASTContext &Ctx) {
  GlobalID[0] = new (Ctx) WVirtualVarDecl("gid0", Ctx.getSizeType(), Ctx);
  GlobalID[1] = new (Ctx) WVirtualVarDecl("gid1", Ctx.getSizeType(), Ctx);
  GlobalID[2] = new (Ctx) WVirtualVarDecl("gid2", Ctx.getSizeType(), Ctx);
  LocalID[0] = new (Ctx) WVirtualVarDecl("lid0", Ctx.getSizeType(), Ctx);
  LocalID[1] = new (Ctx) WVirtualVarDecl("lid1", Ctx.getSizeType(), Ctx);
  LocalID[2] = new (Ctx) WVirtualVarDecl("lid2", Ctx.getSizeType(), Ctx);
  WorkGroupID[0] = new (Ctx) WVirtualVarDecl("wid0", Ctx.getSizeType(), Ctx);
  WorkGroupID[1] = new (Ctx) WVirtualVarDecl("wid1", Ctx.getSizeType(), Ctx);
  WorkGroupID[2] = new (Ctx) WVirtualVarDecl("wid2", Ctx.getSizeType(), Ctx);
  GlobalSize[0] = new (Ctx) WVirtualVarDecl("G0", Ctx.getSizeType(), Ctx);
  GlobalSize[1] = new (Ctx) WVirtualVarDecl("G1", Ctx.getSizeType(), Ctx);
  GlobalSize[2] = new (Ctx) WVirtualVarDecl("G2", Ctx.getSizeType(), Ctx);
  LocalSize[0] = new (Ctx) WVirtualVarDecl("L0", Ctx.getSizeType(), Ctx);
  LocalSize[1] = new (Ctx) WVirtualVarDecl("L1", Ctx.getSizeType(), Ctx);
  LocalSize[2] = new (Ctx) WVirtualVarDecl("L2", Ctx.getSizeType(), Ctx);
  NumWorkGroups[0] = new (Ctx) WVirtualVarDecl("W0", Ctx.getSizeType(), Ctx);
  NumWorkGroups[1] = new (Ctx) WVirtualVarDecl("W1", Ctx.getSizeType(), Ctx);
  NumWorkGroups[2] = new (Ctx) WVirtualVarDecl("W2", Ctx.getSizeType(), Ctx);
  FlatGlobalID = new (Ctx) WVirtualVarDecl("gid", Ctx.getSizeType(), Ctx);
  FlatLocalID = new (Ctx) WVirtualVarDecl("lid", Ctx.getSizeType(), Ctx);
  FlatWorkGroupID = new (Ctx) WVirtualVarDecl("wid", Ctx.getSizeType(), Ctx);
  FlatGlobalSize = new (Ctx) WVirtualVarDecl("G", Ctx.getSizeType(), Ctx);
  FlatLocalSize = new (Ctx) WVirtualVarDecl("L", Ctx.getSizeType(), Ctx);
  FlatNumWorkGroups = new (Ctx) WVirtualVarDecl("W", Ctx.getSizeType(), Ctx);

  IndexedGlobalID[0] = new (Ctx) IndexedVarDecl(GlobalID[0], 1);
  IndexedGlobalID[1] = new (Ctx) IndexedVarDecl(GlobalID[1], 1);
  IndexedGlobalID[2] = new (Ctx) IndexedVarDecl(GlobalID[2], 1);
  IndexedLocalID[0] = new (Ctx) IndexedVarDecl(LocalID[0], 1);
  IndexedLocalID[1] = new (Ctx) IndexedVarDecl(LocalID[1], 1);
  IndexedLocalID[2] = new (Ctx) IndexedVarDecl(LocalID[2], 1);
  IndexedWorkGroupID[0] = new (Ctx) IndexedVarDecl(WorkGroupID[0], 1);
  IndexedWorkGroupID[1] = new (Ctx) IndexedVarDecl(WorkGroupID[1], 1);
  IndexedWorkGroupID[2] = new (Ctx) IndexedVarDecl(WorkGroupID[2], 1);
  IndexedGlobalSize[0] = new (Ctx) IndexedVarDecl(GlobalSize[0], 1);
  IndexedGlobalSize[1] = new (Ctx) IndexedVarDecl(GlobalSize[1], 1);
  IndexedGlobalSize[2] = new (Ctx) IndexedVarDecl(GlobalSize[2], 1);
  IndexedLocalSize[0] = new (Ctx) IndexedVarDecl(LocalSize[0], 1);
  IndexedLocalSize[1] = new (Ctx) IndexedVarDecl(LocalSize[1], 1);
  IndexedLocalSize[2] = new (Ctx) IndexedVarDecl(LocalSize[2], 1);
  IndexedNumWorkGroups[0] = new (Ctx) IndexedVarDecl(NumWorkGroups[0], 1);
  IndexedNumWorkGroups[1] = new (Ctx) IndexedVarDecl(NumWorkGroups[1], 1);
  IndexedNumWorkGroups[2] = new (Ctx) IndexedVarDecl(NumWorkGroups[2], 1);
  IndexedFlatGlobalID = new (Ctx) IndexedVarDecl(FlatGlobalID, 1);
  IndexedFlatLocalID = new (Ctx) IndexedVarDecl(FlatLocalID, 1);
  IndexedFlatWorkGroupID = new (Ctx) IndexedVarDecl(FlatWorkGroupID, 1);
  IndexedFlatGlobalSize = new (Ctx) IndexedVarDecl(FlatGlobalSize, 1);
  IndexedFlatLocalSize = new (Ctx) IndexedVarDecl(FlatLocalSize, 1);
  IndexedFlatNumWorkGroups = new (Ctx) IndexedVarDecl(FlatNumWorkGroups, 1);
}

} // namespace snu

} // namespace clang
