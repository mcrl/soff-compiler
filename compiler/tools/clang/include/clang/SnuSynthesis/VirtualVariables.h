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

#ifndef LLVM_CLANG_SNU_ANALYSIS_VIRTUALVARIABLES_H
#define LLVM_CLANG_SNU_ANALYSIS_VIRTUALVARIABLES_H

#include "clang/AST/ASTContext.h"
#include "clang/SnuAST/WAST.h"

namespace clang {

namespace snu {

class VirtualVariablePool {
  WVirtualVarDecl *GlobalID[3];
  WVirtualVarDecl *LocalID[3];
  WVirtualVarDecl *WorkGroupID[3];
  WVirtualVarDecl *GlobalSize[3];
  WVirtualVarDecl *LocalSize[3];
  WVirtualVarDecl *NumWorkGroups[3];

  WVirtualVarDecl *FlatGlobalID;
  WVirtualVarDecl *FlatLocalID;
  WVirtualVarDecl *FlatWorkGroupID;
  WVirtualVarDecl *FlatGlobalSize;
  WVirtualVarDecl *FlatLocalSize;
  WVirtualVarDecl *FlatNumWorkGroups;

  IndexedVarDecl *IndexedGlobalID[3];
  IndexedVarDecl *IndexedLocalID[3];
  IndexedVarDecl *IndexedWorkGroupID[3];
  IndexedVarDecl *IndexedGlobalSize[3];
  IndexedVarDecl *IndexedLocalSize[3];
  IndexedVarDecl *IndexedNumWorkGroups[3];

  IndexedVarDecl *IndexedFlatGlobalID;
  IndexedVarDecl *IndexedFlatLocalID;
  IndexedVarDecl *IndexedFlatWorkGroupID;
  IndexedVarDecl *IndexedFlatGlobalSize;
  IndexedVarDecl *IndexedFlatLocalSize;
  IndexedVarDecl *IndexedFlatNumWorkGroups;

public:
  explicit VirtualVariablePool(const ASTContext &Ctx);

  WVirtualVarDecl *getGlobalID(unsigned Index) const {
    assert(Index < 3);
    return GlobalID[Index];
  }
  WVirtualVarDecl *getLocalID(unsigned Index) const {
    assert(Index < 3);
    return LocalID[Index];
  }
  WVirtualVarDecl *getWorkGroupID(unsigned Index) const {
    assert(Index < 3);
    return WorkGroupID[Index];
  }
  WVirtualVarDecl *getGlobalSize(unsigned Index) const {
    assert(Index < 3);
    return GlobalSize[Index];
  }
  WVirtualVarDecl *getLocalSize(unsigned Index) const {
    assert(Index < 3);
    return LocalSize[Index];
  }
  WVirtualVarDecl *getNumWorkGroups(unsigned Index) const {
    assert(Index < 3);
    return NumWorkGroups[Index];
  }

  WVirtualVarDecl *getFlatGlobalID() const {
    return FlatGlobalID;
  }
  WVirtualVarDecl *getFlatLocalID() const {
    return FlatLocalID;
  }
  WVirtualVarDecl *getFlatWorkGroupID() const {
    return FlatWorkGroupID;
  }
  WVirtualVarDecl *getFlatGlobalSize() const {
    return FlatGlobalSize;
  }
  WVirtualVarDecl *getFlatLocalSize() const {
    return FlatLocalSize;
  }
  WVirtualVarDecl *getFlatNumWorkGroups() const {
    return FlatNumWorkGroups;
  }

  IndexedVarDecl *getIndexedGlobalID(unsigned Index) const {
    assert(Index < 3);
    return IndexedGlobalID[Index];
  }
  IndexedVarDecl *getIndexedLocalID(unsigned Index) const {
    assert(Index < 3);
    return IndexedLocalID[Index];
  }
  IndexedVarDecl *getIndexedWorkGroupID(unsigned Index) const {
    assert(Index < 3);
    return IndexedWorkGroupID[Index];
  }
  IndexedVarDecl *getIndexedGlobalSize(unsigned Index) const {
    assert(Index < 3);
    return IndexedGlobalSize[Index];
  }
  IndexedVarDecl *getIndexedLocalSize(unsigned Index) const {
    assert(Index < 3);
    return IndexedLocalSize[Index];
  }
  IndexedVarDecl *getIndexedNumWorkGroups(unsigned Index) const {
    assert(Index < 3);
    return IndexedNumWorkGroups[Index];
  }

  IndexedVarDecl *getIndexedFlatGlobalID() const {
    return IndexedFlatGlobalID;
  }
  IndexedVarDecl *getIndexedFlatLocalID() const {
    return IndexedFlatLocalID;
  }
  IndexedVarDecl *getIndexedFlatWorkGroupID() const {
    return IndexedFlatWorkGroupID;
  }
  IndexedVarDecl *getIndexedFlatGlobalSize() const {
    return IndexedFlatGlobalSize;
  }
  IndexedVarDecl *getIndexedFlatLocalSize() const {
    return IndexedFlatLocalSize;
  }
  IndexedVarDecl *getIndexedFlatNumWorkGroups() const {
    return IndexedFlatNumWorkGroups;
  }
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_ANALYSIS_VIRTUALVARIABLES_H
