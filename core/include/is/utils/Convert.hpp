/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 * Copyright (C) 2020 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef _IS_UTILS_CONVERT_HPP_
#define _IS_UTILS_CONVERT_HPP_

#include <is/core/Message.hpp>

#include <algorithm>
#include <mutex>
#include <type_traits>
#include <vector>

#include <boost/array.hpp>

namespace eprosima {
namespace is {
namespace utils {

//==============================================================================
// Helper: get a typed value from a DynamicData field.
// These are the building blocks used by the Convert<> specializations below.
// The DynamicData (shared_ptr) + MemberId pair replaces the old
// ReadableDynamicDataRef / WritableDynamicDataRef pattern.

namespace detail {

// Typed getters
inline void get_value(const xtypes::DynamicData& d, xtypes::MemberId id, bool& v)
{ d->get_boolean_value(v, id); }
inline void get_value(const xtypes::DynamicData& d, xtypes::MemberId id, char& v)
{ int8_t tmp; d->get_int8_value(tmp, id); v = static_cast<char>(tmp); }
inline void get_value(const xtypes::DynamicData& d, xtypes::MemberId id, int8_t& v)
{ d->get_int8_value(v, id); }
inline void get_value(const xtypes::DynamicData& d, xtypes::MemberId id, uint8_t& v)
{ d->get_uint8_value(v, id); }
inline void get_value(const xtypes::DynamicData& d, xtypes::MemberId id, int16_t& v)
{ d->get_int16_value(v, id); }
inline void get_value(const xtypes::DynamicData& d, xtypes::MemberId id, uint16_t& v)
{ d->get_uint16_value(v, id); }
inline void get_value(const xtypes::DynamicData& d, xtypes::MemberId id, int32_t& v)
{ d->get_int32_value(v, id); }
inline void get_value(const xtypes::DynamicData& d, xtypes::MemberId id, uint32_t& v)
{ d->get_uint32_value(v, id); }
inline void get_value(const xtypes::DynamicData& d, xtypes::MemberId id, int64_t& v)
{ d->get_int64_value(v, id); }
inline void get_value(const xtypes::DynamicData& d, xtypes::MemberId id, uint64_t& v)
{ d->get_uint64_value(v, id); }
inline void get_value(const xtypes::DynamicData& d, xtypes::MemberId id, float& v)
{ d->get_float32_value(v, id); }
inline void get_value(const xtypes::DynamicData& d, xtypes::MemberId id, double& v)
{ d->get_float64_value(v, id); }
inline void get_value(const xtypes::DynamicData& d, xtypes::MemberId id, long double& v)
{ d->get_float128_value(v, id); }
inline void get_value(const xtypes::DynamicData& d, xtypes::MemberId id, std::string& v)
{ d->get_string_value(v, id); }
inline void get_value(const xtypes::DynamicData& d, xtypes::MemberId id, std::basic_string<char16_t>& v)
{
    std::wstring tmp;
    d->get_wstring_value(tmp, id);
    v.assign(tmp.begin(), tmp.end());
}

// Typed setters
inline void set_value(xtypes::DynamicData& d, xtypes::MemberId id, bool v)
{ d->set_boolean_value(id, v); }
inline void set_value(xtypes::DynamicData& d, xtypes::MemberId id, char v)
{ d->set_int8_value(id, static_cast<int8_t>(v)); }
inline void set_value(xtypes::DynamicData& d, xtypes::MemberId id, int8_t v)
{ d->set_int8_value(id, v); }
inline void set_value(xtypes::DynamicData& d, xtypes::MemberId id, uint8_t v)
{ d->set_uint8_value(id, v); }
inline void set_value(xtypes::DynamicData& d, xtypes::MemberId id, int16_t v)
{ d->set_int16_value(id, v); }
inline void set_value(xtypes::DynamicData& d, xtypes::MemberId id, uint16_t v)
{ d->set_uint16_value(id, v); }
inline void set_value(xtypes::DynamicData& d, xtypes::MemberId id, int32_t v)
{ d->set_int32_value(id, v); }
inline void set_value(xtypes::DynamicData& d, xtypes::MemberId id, uint32_t v)
{ d->set_uint32_value(id, v); }
inline void set_value(xtypes::DynamicData& d, xtypes::MemberId id, int64_t v)
{ d->set_int64_value(id, v); }
inline void set_value(xtypes::DynamicData& d, xtypes::MemberId id, uint64_t v)
{ d->set_uint64_value(id, v); }
inline void set_value(xtypes::DynamicData& d, xtypes::MemberId id, float v)
{ d->set_float32_value(id, v); }
inline void set_value(xtypes::DynamicData& d, xtypes::MemberId id, double v)
{ d->set_float64_value(id, v); }
inline void set_value(xtypes::DynamicData& d, xtypes::MemberId id, long double v)
{ d->set_float128_value(id, v); }
inline void set_value(xtypes::DynamicData& d, xtypes::MemberId id, const std::string& v)
{ d->set_string_value(id, v); }
inline void set_value(xtypes::DynamicData& d, xtypes::MemberId id, const std::basic_string<char16_t>& v)
{
    std::wstring tmp(v.begin(), v.end());
    d->set_wstring_value(id, tmp);
}

} // namespace detail

//==============================================================================
/**
 * @brief A utility to help with converting data between DynamicData fields and
 *        middleware-specific data structures.
 *
 *        This struct works as-is on primitive types (arithmetic types and strings).
 *        For compound class types a template specialization should be created.
 *
 * ### New calling convention (Fast-DDS XTypes API)
 *
 *   // Read a field from DynamicData into a native value:
 *   Convert<int32_t>::from_xtype_field(parent_data, member_id, native_value);
 *
 *   // Write a native value into a DynamicData field:
 *   Convert<int32_t>::to_xtype_field(native_value, parent_data, member_id);
 *
 * The old single-argument ReadableDynamicDataRef / WritableDynamicDataRef pattern
 * is replaced by passing the parent DynamicData (shared_ptr) together with the
 * MemberId for that field.
 */
template<typename Type>
struct Convert
{
    using native_type = Type;

    static constexpr bool type_is_primitive =
            std::is_arithmetic<Type>::value
            || std::is_same<std::string, Type>::value
            || std::is_same<std::basic_string<char16_t>, Type>::value;

    /**
     * @brief Read a field value from DynamicData into a native type.
     *
     * @param[in]  from  Parent DynamicData containing the field.
     * @param[in]  id    MemberId of the field within @p from.
     * @param[out] to    Destination native value.
     */
    static void from_xtype_field(
            const xtypes::DynamicData& from,
            xtypes::MemberId id,
            native_type& to)
    {
        detail::get_value(from, id, to);
    }

    /**
     * @brief Write a native value into a DynamicData field.
     *
     * @param[in]     from  Source native value.
     * @param[in,out] to    Parent DynamicData that will receive the value.
     * @param[in]     id    MemberId of the field within @p to.
     */
    static void to_xtype_field(
            const native_type& from,
            xtypes::DynamicData& to,
            xtypes::MemberId id)
    {
        static_assert(type_is_primitive,
                "The is::utils::Convert struct should be specialized for non-primitive types");
        detail::set_value(to, id, from);
    }
};

//==============================================================================
/**
 * @brief Specialization that handles the char/uint8 mismatch produced by rosidl:
 *        rosidl parses 'char' as signed from .msg files but as unsigned from IDL.
 */
struct CharConvert
{
    using native_type = char;
    static constexpr bool type_is_primitive = true;

    static void from_xtype_field(
            const xtypes::DynamicData& from,
            xtypes::MemberId id,
            native_type& to)
    {
        // Check whether the stored type is uint8 or int8/char
        fastdds::dds::DynamicTypeMember::_ref_type member;
        fastdds::dds::MemberDescriptor::_ref_type desc =
            fastdds::dds::traits<fastdds::dds::MemberDescriptor>::make_shared();
        if (from->type()->get_member(member, id) ==
                eprosima::fastdds::dds::RETCODE_OK)
        {
            member->get_descriptor(desc);
            if (desc->type()->get_kind() == fastdds::dds::TK_UINT8)
            {
                uint8_t tmp;
                from->get_uint8_value(tmp, id);
                to = static_cast<char>(tmp);
                return;
            }
        }
        int8_t tmp;
        from->get_int8_value(tmp, id);
        to = static_cast<char>(tmp);
    }

    static void to_xtype_field(
            const native_type& from,
            xtypes::DynamicData& to,
            xtypes::MemberId id)
    {
        fastdds::dds::DynamicTypeMember::_ref_type member;
        fastdds::dds::MemberDescriptor::_ref_type desc =
            fastdds::dds::traits<fastdds::dds::MemberDescriptor>::make_shared();
        if (to->type()->get_member(member, id) ==
                eprosima::fastdds::dds::RETCODE_OK)
        {
            member->get_descriptor(desc);
            if (desc->type()->get_kind() == fastdds::dds::TK_UINT8)
            {
                to->set_uint8_value(id, static_cast<uint8_t>(from));
                return;
            }
        }
        to->set_int8_value(id, static_cast<int8_t>(from));
    }
};

template<>
struct Convert<char> : CharConvert { };

//==============================================================================
/**
 * @brief Helper for compound (struct/message) types.
 *
 * To create a specialization:
 *
 * @code{.cpp}
 * template<>
 * struct Convert<native::middleware::type>
 *     : MessageConvert<
 *         native::middleware::type,
 *         &native::middleware::convert_from_xtype_fnc,
 *         &native::middleware::convert_to_xtype_fnc
 *     > { };
 * @endcode
 *
 * Where the conversion functions have signatures:
 *   void convert_from(const xtypes::DynamicData& from, MemberId id, Type& to);
 *   void convert_to  (const Type& from, xtypes::DynamicData& to, MemberId id);
 */
template<
    typename Type,
    void (* _from_xtype)(const xtypes::DynamicData& from, xtypes::MemberId id, Type& to),
    void (* _to_xtype)  (const Type& from, xtypes::DynamicData& to, xtypes::MemberId id)>
struct MessageConvert
{
    using native_type = Type;
    static constexpr bool type_is_primitive = false;

    static void from_xtype_field(
            const xtypes::DynamicData& from,
            xtypes::MemberId id,
            native_type& to)
    {
        (*_from_xtype)(from, id, to);
    }

    static void to_xtype_field(
            const native_type& from,
            xtypes::DynamicData& to,
            xtypes::MemberId id)
    {
        (*_to_xtype)(from, to, id);
    }
};

//==============================================================================
/**
 * @brief Helper for resizable unbounded container types (e.g. std::vector).
 */
template<
    typename ElementType,
    template <typename, typename> class NativeType,
    typename Allocator,
    std::size_t UpperBound,
    std::enable_if_t<std::is_base_of<std::vector<ElementType, Allocator>,
                                      NativeType<ElementType, Allocator>>::value,
    bool> = true>
struct ResizableUnboundedContainerConvert
{
    using native_type = NativeType<ElementType, Allocator>;
    static constexpr bool type_is_primitive = Convert<ElementType>::type_is_primitive;

    static void from_xtype_field(
            const xtypes::DynamicData& from,
            xtypes::MemberId seq_id,
            native_type& to)
    {
        // Loan the sequence DynamicData
        auto seq_data = from->loan_value(seq_id);
        const std::size_t N = std::min(
            static_cast<std::size_t>(seq_data->get_item_count()), UpperBound);
        to.resize(N);
        for (std::size_t i = 0; i < N; ++i)
        {
            xtypes::MemberId elem_id = seq_data->get_member_id_at_index(
                static_cast<uint32_t>(i));
            Convert<ElementType>::from_xtype_field(seq_data, elem_id, to[i]);
        }
        from->return_loaned_value(seq_data);
    }

    static void to_xtype_field(
            const native_type& from,
            xtypes::DynamicData& to,
            xtypes::MemberId seq_id)
    {
        auto seq_data = to->loan_value(seq_id);
        const std::size_t N = std::min(from.size(), UpperBound);
        seq_data->clear_all_values();
        for (std::size_t i = 0; i < N; ++i)
        {
            xtypes::MemberId elem_id = static_cast<xtypes::MemberId>(i);
            Convert<ElementType>::to_xtype_field(from[i], seq_data, elem_id);
        }
        to->return_loaned_value(seq_data);
    }
};

template<typename ElementType, typename Allocator>
struct Convert<std::vector<ElementType, Allocator>>
    : ResizableUnboundedContainerConvert<
        ElementType,
        std::vector,
        Allocator,
        std::numeric_limits<typename std::vector<ElementType, Allocator>::size_type>::max()>
{
};

//==============================================================================
/**
 * @brief Helper for resizable bounded container types.
 */
template<
    typename ElementType,
    template <typename, std::size_t, typename> class NativeType,
    typename Allocator,
    std::size_t UpperBound>
struct ResizableBoundedContainerConvert
{
    using native_type = NativeType<ElementType, UpperBound, Allocator>;
    static constexpr bool type_is_primitive = Convert<ElementType>::type_is_primitive;

    static void from_xtype_field(
            const xtypes::DynamicData& from,
            xtypes::MemberId seq_id,
            native_type& to)
    {
        auto seq_data = from->loan_value(seq_id);
        const std::size_t N = std::min(
            static_cast<std::size_t>(seq_data->get_item_count()), UpperBound);
        to.resize(N);
        for (std::size_t i = 0; i < N; ++i)
        {
            xtypes::MemberId elem_id = seq_data->get_member_id_at_index(
                static_cast<uint32_t>(i));
            Convert<ElementType>::from_xtype_field(seq_data, elem_id, to[i]);
        }
        from->return_loaned_value(seq_data);
    }

    static void to_xtype_field(
            const native_type& from,
            xtypes::DynamicData& to,
            xtypes::MemberId seq_id)
    {
        auto seq_data = to->loan_value(seq_id);
        const std::size_t N = std::min(from.size(), UpperBound);
        seq_data->clear_all_values();
        for (std::size_t i = 0; i < N; ++i)
        {
            xtypes::MemberId elem_id = static_cast<xtypes::MemberId>(i);
            Convert<ElementType>::to_xtype_field(from[i], seq_data, elem_id);
        }
        to->return_loaned_value(seq_data);
    }
};

template<typename ElementType, std::size_t N, typename Allocator,
        template<typename, std::size_t, typename> class VectorImpl>
struct Convert<VectorImpl<ElementType, N, Allocator>>
    : ResizableBoundedContainerConvert<ElementType, VectorImpl, Allocator, N>
{
};

//==============================================================================
/**
 * @brief Helper for fixed-size array types (std::array, boost::array).
 */
template<
    typename ElementType,
    template <typename, std::size_t> class NativeType,
    std::size_t UpperBound,
    std::enable_if_t<
        std::is_base_of<std::array<ElementType, UpperBound>,  NativeType<ElementType, UpperBound>>::value
     || std::is_base_of<boost::array<ElementType, UpperBound>, NativeType<ElementType, UpperBound>>::value,
    bool> = true>
struct NonResizableContainerConvert
{
    using native_type = NativeType<ElementType, UpperBound>;
    static constexpr bool type_is_primitive = Convert<ElementType>::type_is_primitive;

    static void from_xtype_field(
            const xtypes::DynamicData& from,
            xtypes::MemberId arr_id,
            native_type& to)
    {
        auto arr_data = from->loan_value(arr_id);
        const std::size_t N = std::min(
            static_cast<std::size_t>(arr_data->get_item_count()), UpperBound);
        for (std::size_t i = 0; i < N; ++i)
        {
            xtypes::MemberId elem_id = arr_data->get_member_id_at_index(
                static_cast<uint32_t>(i));
            Convert<ElementType>::from_xtype_field(arr_data, elem_id, to[i]);
        }
        from->return_loaned_value(arr_data);
    }

    static void to_xtype_field(
            const native_type& from,
            xtypes::DynamicData& to,
            xtypes::MemberId arr_id)
    {
        auto arr_data = to->loan_value(arr_id);
        const std::size_t N = std::min(from.size(), UpperBound);
        for (std::size_t i = 0; i < N; ++i)
        {
            xtypes::MemberId elem_id = arr_data->get_member_id_at_index(
                static_cast<uint32_t>(i));
            Convert<ElementType>::to_xtype_field(from[i], arr_data, elem_id);
        }
        to->return_loaned_value(arr_data);
    }
};

template<template <typename, std::size_t> class Array, typename ElementType, std::size_t N>
struct Convert<Array<ElementType, N>>
    : NonResizableContainerConvert<ElementType, Array, N>
{
};

//==============================================================================
/**
 * @brief A thread-safe repository for resources to avoid unnecessary allocations.
 */
template<typename Resource, Resource(* initializerT)()>
class ResourcePool
{
public:

    ResourcePool(
            const std::size_t initial_depth = 1)
    {
        _queue.reserve(initial_depth);
        for (std::size_t i = 0; i < initial_depth; ++i)
        {
            _queue.emplace_back((_initializer)());
        }
    }

    void setInitializer(
            std::function<Resource()> initializer)
    {
        _initializer = std::move(initializer);
    }

    Resource pop()
    {
        if (_queue.empty())
        {
            return (_initializer)();
        }

        std::unique_lock<std::mutex> lock(_mutex);
        Resource r = std::move(_queue.back());
        _queue.pop_back();
        return r;
    }

    void recycle(
            Resource&& r)
    {
        std::unique_lock<std::mutex> lock(_mutex);
        _queue.emplace_back(std::move(r));
    }

private:

    std::vector<Resource> _queue;
    std::mutex _mutex;
    std::function<Resource()> _initializer = initializerT;
};

//==============================================================================
template<typename Resource>
using UniqueResourcePool =
        ResourcePool<std::unique_ptr<Resource>, &std::make_unique<Resource>>;

template<typename Resource>
std::unique_ptr<Resource> initialize_unique_null()
{
    return nullptr;
}

template<typename Resource>
using SharedResourcePool =
        ResourcePool<std::shared_ptr<Resource>, &std::make_shared<Resource>>;

template<typename Resource>
std::shared_ptr<Resource> initialize_shared_null()
{
    return nullptr;
}

} //  namespace utils
} //  namespace is
} //  namespace eprosima

#endif //  _IS_UTILS_CONVERT_HPP_
