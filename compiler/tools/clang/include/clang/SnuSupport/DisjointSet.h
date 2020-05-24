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

#ifndef LLVM_CLANG_SNU_SUPPORT_DISJOINTSET_H
#define LLVM_CLANG_SNU_SUPPORT_DISJOINTSET_H

#include "clang/Basic/LLVM.h"
#include "llvm/ADT/DenseMap.h"
#include <vector>

namespace clang {

namespace snu {

template<typename ValueT>
class DisjointSet {
  struct TreeElement {
    ValueT value;
    unsigned parent;
    unsigned rank;
  };

  typedef std::vector<TreeElement> TreeTy;
  typedef llvm::DenseMap<ValueT, unsigned> TreeIndexMapTy;

  TreeTy Tree;
  TreeIndexMapTy TreeIndex;

  unsigned find_set(unsigned index) {
    unsigned root = index;
    while (Tree[root].parent != root) {
      root = Tree[root].parent;
    }
    if (Tree[index].parent != root) {
      // path compression
      unsigned p = index;
      while (p != root) {
        unsigned next_p = Tree[p].parent;
        Tree[p].parent = root;
        p = next_p;
      }
    }
    return root;
  }

  unsigned lookup(const ValueT &V) {
    typename TreeIndexMapTy::const_iterator I = TreeIndex.find(V);
    assert(I != TreeIndex.end());
    return find_set(I->second);
  }

public:
  typedef ValueT SubsetTy;

  DisjointSet() {}
  ~DisjointSet() {}

  bool exist(const ValueT &V) const {
    return TreeIndex.count(V);
  }

  const SubsetTy &operator[](const ValueT &V) {
    unsigned index = lookup(V);
    return Tree[index].value;
  }

  bool is_same_set(const ValueT &X, const ValueT &Y) {
    unsigned x = lookup(X);
    unsigned y = lookup(Y);
    return (x == y);
  }

  unsigned num_values() const {
    return Tree.size();
  }

  unsigned num_sets() const {
    unsigned count = 0;
    for (unsigned index = 0, size = Tree.size();
         index != size; ++index) {
      if (Tree[index].parent == index) {
        count++;
      }
    }
    return count;
  }

  SubsetTy create_set(const ValueT &V) {
    TreeElement elem;
    elem.value = V;
    elem.parent = (unsigned)Tree.size();
    elem.rank = 0;
    Tree.push_back(elem);
    TreeIndex[V] = elem.parent;
    return elem.value;
  }

  SubsetTy union_set(const ValueT &X, const ValueT &Y) {
    unsigned x = lookup(X);
    unsigned y = lookup(Y);
    if (x == y) {
      return Tree[x].value;
    } else if (Tree[x].rank > Tree[y].rank) {
      Tree[y].parent = x;
      return Tree[x].value;
    } else {
      Tree[x].parent = y;
      if (Tree[x].rank == Tree[y].rank) {
        Tree[y].rank++;
      }
      return Tree[y].value;
    }
  }

  void union_all() {
    Tree[0].parent = 0;
    Tree[0].rank = 1;
    for (unsigned index = 1, size = Tree.size();
         index != size; ++index) {
      Tree[index].parent = 0;
      Tree[index].rank = 0;
    }
  }

  void clear() {
    Tree.clear();
    TreeIndex.clear();
  }
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_SUPPORT_DISJOINTSET_H
