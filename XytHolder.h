#ifndef DISJOINT_CBSH_XYTHOLDER_H
#define DISJOINT_CBSH_XYTHOLDER_H

#include <algorithm>
#include <list>
#include <tuple>
#include <exception>
#include <iostream>

template <class T>
class XytHolder {
public:
    XytHolder(int xy_size) : xy_size(xy_size) {
        data = new std::list<std::tuple<int,T>>*[xy_size];
        for (int i = 0; i < xy_size; ++i) {
            data[i] = nullptr;
        }
    }

    // Shallow copy
    XytHolder(const XytHolder& other) {
        xy_size = other.xy_size;
        data = new std::list<std::tuple<int,T>>*[other.xy_size];
        for (int i = 0; i < xy_size; ++i) {
            data[i] = nullptr;
        }
        for (int i = 0; i < other.xy_size ; ++i) {
            if (other.data[i] != nullptr) {
                for (auto pair: *other.data[i]) {
                    set(i, pair[0], pair[1]);
                }
            }
        }
    }

    std::tuple<bool, T> get(int location_id, int t) {  // FIXME: Yes, passing by copy
        // Linear lookup
        if (data[location_id] == nullptr)
            return std::make_tuple(false, nullptr);
        for (auto it = data[location_id]->begin(); it != data[location_id]->end() ; ++it)  {
            auto [it_t, it_n] = *it;
            if (it_t == t) {
                return std::make_tuple(true, it_n);
            }
            else if (it_t > t) {
                return std::make_tuple(false, nullptr);
            }
        }
        return std::make_tuple(false, nullptr);
    }

    void set(int location_id, int t, T value) {  // FIXME: Yes, passing by copy
        ++count;
        // Linear insertion
        if (data[location_id] == nullptr)
            data[location_id] = new std::list<std::tuple<int,T>>();
        for (auto it = data[location_id]->begin(); it != data[location_id]->end() ; ++it)  {
            auto [it_t, it_n] = *it;
            if (it_t == t) {
                std::cout << "Unexpected re-insertion of item to XytHolder!" << std::endl;
                std::abort();
            }
            else if (it_t > t) {
                data[location_id]->insert(it, std::make_tuple(t, value));  // inserts before the iterator
                return;
            }
        }
        data[location_id]->push_back(std::make_tuple(t, value));
    }

    size_t size() { return count; }

    ~XytHolder() {
        for (int i = 0; i < xy_size; ++i) {
            delete data[i];
        }
        delete[] data;
    }

    void clear() {
        for (int i = 0; i < xy_size; ++i) {
            delete data[i];
        }
        delete[] data;
        data = new std::list<std::tuple<int,T>>[xy_size];
        for (int i = 0; i < xy_size; ++i) {
            data[i] = nullptr;
        }
    }

    int xy_size;
    int count = 0;
    std::list<std::tuple<int,T>>** data;
};


#endif //DISJOINT_CBSH_XYTHOLDER_H
