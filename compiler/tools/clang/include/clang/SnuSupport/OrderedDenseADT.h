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

#ifndef LLVM_CLANG_SNU_SUPPORT_ORDEREDDENSEADT_H
#define LLVM_CLANG_SNU_SUPPORT_ORDEREDDENSEADT_H

#include "clang/Basic/LLVM.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/DenseSet.h"
#include <algorithm>
#include <vector>

namespace clang {

namespace snu {

template<typename ValueT>
class OrderedDenseSet {
  typedef typename llvm::DenseSet<ValueT> StorageTy;
  typedef typename std::vector<ValueT> OrderVectorTy;
  StorageTy Storage;
  OrderVectorTy Order;

public:
  bool empty() const { return Storage.empty(); }
  unsigned size() const { return Storage.size(); }
  bool count(const ValueT &V) const { return Storage.count(V); }

  void clear() {
    Storage.clear();
    Order.clear();
  }

  void insert(const ValueT &V) {
    if (!Storage.count(V)) {
      Storage.insert(V);
      Order.push_back(V);
    }
  }

  void erase(const ValueT &V) {
    if (Storage.count(V)) {
      Storage.erase(V);
      Order.erase(std::find(Order.begin(), Order.end(), V));
    }
  }

  typedef typename OrderVectorTy::const_iterator const_iterator;
  const_iterator begin() const { return Order.begin(); }
  const_iterator end() const { return Order.end(); }
};

template<typename KeyT, typename ValueT>
class OrderedDenseMap {
  typedef typename llvm::DenseMap<KeyT, ValueT> StorageTy;
  typedef typename std::vector<KeyT> OrderVectorTy;
  StorageTy Storage;
  OrderVectorTy Order;

public:
  bool empty() const { return Storage.empty(); }
  unsigned size() const { return Storage.size(); }
  bool count(const KeyT &V) const { return Storage.count(V); }
  ValueT lookup(const KeyT &V) const { return Storage.lookup(V); }

  void clear() {
    Storage.clear();
    Order.clear();
  }

  void insert(const std::pair<KeyT, ValueT> &KV) {
    if (!Storage.count(KV.first)) {
      Order.push_back(KV.first);
    }
    Storage.insert(KV);
  }

  template<typename InputIt>
  void insert(InputIt I, InputIt E) {
    for (; I != E; ++I) {
      insert(*I);
    }
  }

  void erase(const KeyT &Key) {
    if (Storage.count(Key)) {
      Order.erase(std::find(Order.begin(), Order.end(), Key));
      Storage.erase(Key);
    }
  }

  ValueT &operator[](const KeyT &Key) {
    if (!Storage.count(Key)) {
      Order.push_back(Key);
    }
    return Storage[Key];
  }

  class ConstIterator {
    typedef std::pair<KeyT, ValueT> Bucket;
    typedef typename OrderVectorTy::const_iterator OrderIterator;

    OrderIterator I;
    const StorageTy &Storage;

  public:
    ConstIterator(OrderIterator i, const StorageTy &storage)
      : I(i), Storage(storage) {}

    const Bucket &operator*() const {
      return *(Storage.find(*I));
    }
    const Bucket *operator->() const {
      return Storage.find(*I).operator->();
    }

    bool operator==(const ConstIterator &RHS) const {
      return I == RHS.I && &Storage == &(RHS.Storage);
    }
    bool operator!=(const ConstIterator &RHS) const {
      return I != RHS.I || &Storage != &(RHS.Storage);
    }

    ConstIterator &operator++() {
      ++I;
      return *this;
    }
    ConstIterator operator++(int) {
      ConstIterator Ret = *this;
      ++*this;
      return Ret;
    }
  };

  typedef ConstIterator const_iterator;
  const_iterator begin() const { return ConstIterator(Order.begin(), Storage); }
  const_iterator end() const { return ConstIterator(Order.end(), Storage); }
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_SUPPORT_ORDEREDDENSEADT_H
