/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef _IS_CORE_MESSAGE_HPP_
#define _IS_CORE_MESSAGE_HPP_

// Fast-DDS XTypes API (replaces the old eprosima::xtypes API from src/xtypes/)
#include <fastdds/dds/xtypes/dynamic_types/DynamicData.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicType.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicTypeMember.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicTypeBuilderFactory.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicDataFactory.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicTypeBuilder.hpp>
#include <fastdds/dds/xtypes/dynamic_types/MemberDescriptor.hpp>
#include <fastdds/dds/xtypes/dynamic_types/TypeDescriptor.hpp>
#include <fastdds/dds/xtypes/dynamic_types/Types.hpp>
#include <fastdds/dds/xtypes/utils.hpp>
#include <fastdds/dds/core/Types.hpp>

// Bring Fast-DDS xtypes types into eprosima::xtypes for compatibility with
// existing Integration Service code that refers to eprosima::xtypes::*.
namespace eprosima {
namespace xtypes {

// Core data types - both are shared_ptr wrappers in the new API
using DynamicData     = fastdds::dds::DynamicData::_ref_type;
using DynamicType     = fastdds::dds::DynamicType::_ref_type;
using DynamicTypeBuilder = fastdds::dds::DynamicTypeBuilder::_ref_type;

// In the old API, DynamicType::Ptr was std::shared_ptr<DynamicType>.
// In the new API, DynamicType IS already the shared_ptr (_ref_type).
// Code that used DynamicType::Ptr should now use xtypes::DynamicType directly.

// Member/type descriptor types
using MemberDescriptor   = fastdds::dds::MemberDescriptor::_ref_type;
using TypeDescriptor     = fastdds::dds::TypeDescriptor::_ref_type;
using DynamicTypeMember  = fastdds::dds::DynamicTypeMember::_ref_type;

// Member ID and type kind
using MemberId  = fastdds::dds::MemberId;
using TypeKind  = fastdds::dds::TypeKind;
using ObjectName = fastdds::dds::ObjectName;

// Factory singletons (convenience re-exports)
using DynamicTypeBuilderFactory = fastdds::dds::DynamicTypeBuilderFactory;
using DynamicDataFactory        = fastdds::dds::DynamicDataFactory;

// In the old API, ReadableDynamicDataRef / WritableDynamicDataRef were lightweight
// references to a specific field inside a DynamicData.  In the new API there is no
// such sub-reference type — callers obtain a MemberId from the parent DynamicData
// and use typed get/set methods.  For code that still needs to pass "a field ref"
// around, use DynamicData (the shared_ptr) together with a MemberId.
// The aliases below let old-style declarations compile while the bodies are updated.
using ReadableDynamicDataRef = fastdds::dds::DynamicData::_ref_type;
using WritableDynamicDataRef = fastdds::dds::DynamicData::_ref_type;

// TypeKind constants re-exported under the old nested-namespace style used by
// conversion.cpp and FieldToString.cpp:
//   was  xtypes::TypeKind::INT_32_TYPE
//   now  xtypes::TypeKind::INT_32_TYPE  (still works via this alias enum)
namespace TypeKindConstants {
    constexpr fastdds::dds::TypeKind BOOLEAN_TYPE   = fastdds::dds::TK_BOOLEAN;
    constexpr fastdds::dds::TypeKind CHAR_8_TYPE    = fastdds::dds::TK_CHAR8;
    constexpr fastdds::dds::TypeKind INT_8_TYPE     = fastdds::dds::TK_INT8;
    constexpr fastdds::dds::TypeKind UINT_8_TYPE    = fastdds::dds::TK_UINT8;
    constexpr fastdds::dds::TypeKind INT_16_TYPE    = fastdds::dds::TK_INT16;
    constexpr fastdds::dds::TypeKind UINT_16_TYPE   = fastdds::dds::TK_UINT16;
    constexpr fastdds::dds::TypeKind INT_32_TYPE    = fastdds::dds::TK_INT32;
    constexpr fastdds::dds::TypeKind UINT_32_TYPE   = fastdds::dds::TK_UINT32;
    constexpr fastdds::dds::TypeKind INT_64_TYPE    = fastdds::dds::TK_INT64;
    constexpr fastdds::dds::TypeKind UINT_64_TYPE   = fastdds::dds::TK_UINT64;
    constexpr fastdds::dds::TypeKind FLOAT_32_TYPE  = fastdds::dds::TK_FLOAT32;
    constexpr fastdds::dds::TypeKind FLOAT_64_TYPE  = fastdds::dds::TK_FLOAT64;
    constexpr fastdds::dds::TypeKind FLOAT_128_TYPE = fastdds::dds::TK_FLOAT128;
    constexpr fastdds::dds::TypeKind STRING_TYPE    = fastdds::dds::TK_STRING8;
    constexpr fastdds::dds::TypeKind WSTRING_TYPE   = fastdds::dds::TK_STRING16;
    constexpr fastdds::dds::TypeKind ARRAY_TYPE     = fastdds::dds::TK_ARRAY;
    constexpr fastdds::dds::TypeKind SEQUENCE_TYPE  = fastdds::dds::TK_SEQUENCE;
    constexpr fastdds::dds::TypeKind STRUCTURE_TYPE = fastdds::dds::TK_STRUCTURE;
    constexpr fastdds::dds::TypeKind UNION_TYPE     = fastdds::dds::TK_UNION;
    constexpr fastdds::dds::TypeKind ENUM_TYPE      = fastdds::dds::TK_ENUM;
    constexpr fastdds::dds::TypeKind MAP_TYPE       = fastdds::dds::TK_MAP;
    constexpr fastdds::dds::TypeKind NONE_TYPE      = fastdds::dds::TK_NONE;
} // namespace TypeKindConstants

// Helper: build a DynamicType for a primitive C++ type.
// Replaces the old  xtypes::primitive_type<T>()  pattern.
template<typename T>
inline DynamicType primitive_type();

template<> inline DynamicType primitive_type<bool>()
{ return DynamicTypeBuilderFactory::get_instance()->get_primitive_type(fastdds::dds::TK_BOOLEAN); }
template<> inline DynamicType primitive_type<char>()
{ return DynamicTypeBuilderFactory::get_instance()->get_primitive_type(fastdds::dds::TK_CHAR8); }
template<> inline DynamicType primitive_type<int8_t>()
{ return DynamicTypeBuilderFactory::get_instance()->get_primitive_type(fastdds::dds::TK_INT8); }
template<> inline DynamicType primitive_type<uint8_t>()
{ return DynamicTypeBuilderFactory::get_instance()->get_primitive_type(fastdds::dds::TK_UINT8); }
template<> inline DynamicType primitive_type<int16_t>()
{ return DynamicTypeBuilderFactory::get_instance()->get_primitive_type(fastdds::dds::TK_INT16); }
template<> inline DynamicType primitive_type<uint16_t>()
{ return DynamicTypeBuilderFactory::get_instance()->get_primitive_type(fastdds::dds::TK_UINT16); }
template<> inline DynamicType primitive_type<int32_t>()
{ return DynamicTypeBuilderFactory::get_instance()->get_primitive_type(fastdds::dds::TK_INT32); }
template<> inline DynamicType primitive_type<uint32_t>()
{ return DynamicTypeBuilderFactory::get_instance()->get_primitive_type(fastdds::dds::TK_UINT32); }
template<> inline DynamicType primitive_type<int64_t>()
{ return DynamicTypeBuilderFactory::get_instance()->get_primitive_type(fastdds::dds::TK_INT64); }
template<> inline DynamicType primitive_type<uint64_t>()
{ return DynamicTypeBuilderFactory::get_instance()->get_primitive_type(fastdds::dds::TK_UINT64); }
template<> inline DynamicType primitive_type<float>()
{ return DynamicTypeBuilderFactory::get_instance()->get_primitive_type(fastdds::dds::TK_FLOAT32); }
template<> inline DynamicType primitive_type<double>()
{ return DynamicTypeBuilderFactory::get_instance()->get_primitive_type(fastdds::dds::TK_FLOAT64); }
template<> inline DynamicType primitive_type<long double>()
{ return DynamicTypeBuilderFactory::get_instance()->get_primitive_type(fastdds::dds::TK_FLOAT128); }

// Helper: create a DynamicType for a string
inline DynamicType string_type(uint32_t bound = fastdds::dds::LENGTH_UNLIMITED)
{ return DynamicTypeBuilderFactory::get_instance()->create_string_type(bound)->build(); }

// Helper: create a sequence DynamicType wrapping element_type
inline DynamicType sequence_type(const DynamicType& element_type,
                                  uint32_t bound = fastdds::dds::LENGTH_UNLIMITED)
{
    return DynamicTypeBuilderFactory::get_instance()
        ->create_sequence_type(element_type, bound)->build();
}

// Helper: build a named struct DynamicType from a lambda that populates the builder.
//   auto my_type = make_struct_type("Msg", [](DynamicTypeBuilder& b) {
//       add_member(b, "field", primitive_type<int32_t>());
//   });
inline void add_member(const fastdds::dds::DynamicTypeBuilder::_ref_type& builder,
                        const std::string& name,
                        const DynamicType& type)
{
    fastdds::dds::MemberDescriptor::_ref_type md =
        fastdds::dds::traits<fastdds::dds::MemberDescriptor>::make_shared();
    md->name(name);
    md->type(type);
    builder->add_member(md);
}

} //  namespace xtypes
} //  namespace eprosima

#endif //  _IS_CORE_MESSAGE_HPP_
