/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <is/json-xtypes/conversion.hpp>
#include <limits>
#include <iostream>

namespace eprosima {
namespace is {
namespace json_xtypes {

// ---------------------------------------------------------------------------
// Helpers for special float values

template<typename T>
static T get_json_float(Json::const_reference json_node)
{
    if (json_node.is_number())
    {
        return json_node.get<T>();
    }
    const std::string s = json_node.get<std::string>();
    if (s == "inf")  return  std::numeric_limits<T>::infinity();
    if (s == "-inf") return -std::numeric_limits<T>::infinity();
    if (s == "nan")  return  std::numeric_limits<T>::quiet_NaN();
    if (s == "-nan") return -std::numeric_limits<T>::quiet_NaN();

    std::ostringstream err;
    err << "get_json_float: unexpected value '" << s << "'";
    throw UnsupportedType(err.str());
}

template<typename T>
static Json float_to_json(T value)
{
    if (std::isnormal(value))
    {
        return value;
    }
    return std::to_string(value);
}

// ---------------------------------------------------------------------------
// xtypes_to_json — recursive helper

static void xtypes_to_json_impl(
        const xtypes::DynamicData& data,
        const xtypes::DynamicType& type,
        Json& out)
{
    using namespace fastdds::dds;

    const TypeKind kind = type->get_kind();

    switch (kind)
    {
        case TK_STRUCTURE:
        {
            const uint32_t count = type->get_member_count();
            for (uint32_t i = 0; i < count; ++i)
            {
                DynamicTypeMember::_ref_type member;
                type->get_member_by_index(member, i);

                MemberDescriptor::_ref_type desc = traits<MemberDescriptor>::make_shared();
                member->get_descriptor(desc);

                const std::string name = desc->name().to_string();
                const MemberId id = member->get_id();
                const TypeKind member_kind = desc->type()->get_kind();

                if (member_kind == TK_STRUCTURE || member_kind == TK_SEQUENCE || member_kind == TK_ARRAY)
                {
                    auto nested = data->loan_value(id);
                    Json child;
                    xtypes_to_json_impl(nested, desc->type(), child);
                    data->return_loaned_value(nested);
                    out[name] = std::move(child);
                }
                else
                {
                    Json val;
                    xtypes_to_json_impl(data, desc->type(), val);
                    // We need to read from the parent using the member id
                    // Re-enter with a single-value helper
                    switch (member_kind)
                    {
                        case TK_BOOLEAN: { bool v; data->get_boolean_value(v, id); out[name] = v; break; }
                        case TK_CHAR8:   { char v; data->get_char8_value(v, id); out[name] = v; break; }
                        case TK_INT8:    { int8_t v; data->get_int8_value(v, id); out[name] = v; break; }
                        case TK_UINT8:   { uint8_t v; data->get_uint8_value(v, id); out[name] = v; break; }
                        case TK_INT16:   { int16_t v; data->get_int16_value(v, id); out[name] = v; break; }
                        case TK_UINT16:  { uint16_t v; data->get_uint16_value(v, id); out[name] = v; break; }
                        case TK_INT32:   { int32_t v; data->get_int32_value(v, id); out[name] = v; break; }
                        case TK_UINT32:  { uint32_t v; data->get_uint32_value(v, id); out[name] = v; break; }
                        case TK_INT64:   { int64_t v; data->get_int64_value(v, id); out[name] = v; break; }
                        case TK_UINT64:  { uint64_t v; data->get_uint64_value(v, id); out[name] = v; break; }
                        case TK_FLOAT32: { float v; data->get_float32_value(v, id); out[name] = float_to_json(v); break; }
                        case TK_FLOAT64: { double v; data->get_float64_value(v, id); out[name] = float_to_json(v); break; }
                        case TK_FLOAT128:{ long double v; data->get_float128_value(v, id); out[name] = float_to_json(static_cast<double>(v)); break; }
                        case TK_STRING8: { std::string v; data->get_string_value(v, id); out[name] = v; break; }
                        default: throw UnsupportedType(std::string("struct member kind ") + std::to_string(static_cast<int>(member_kind)));
                    }
                }
            }
            break;
        }

        case TK_SEQUENCE:
        case TK_ARRAY:
        {
            // data is already the sequence/array DynamicData
            const uint32_t count = data->get_item_count();
            out = Json::array();

            // Get element type via type descriptor
            TypeDescriptor::_ref_type td = traits<TypeDescriptor>::make_shared();
            type->get_descriptor(td);
            const DynamicType::_ref_type elem_type = td->element_type();
            const TypeKind elem_kind = elem_type->get_kind();

            for (uint32_t i = 0; i < count; ++i)
            {
                MemberId elem_id = data->get_member_id_at_index(i);
                if (elem_kind == TK_STRUCTURE || elem_kind == TK_SEQUENCE || elem_kind == TK_ARRAY)
                {
                    auto nested = data->loan_value(elem_id);
                    Json child;
                    xtypes_to_json_impl(nested, elem_type, child);
                    data->return_loaned_value(nested);
                    out.push_back(std::move(child));
                }
                else
                {
                    switch (elem_kind)
                    {
                        case TK_BOOLEAN: { bool v; data->get_boolean_value(v, elem_id); out.push_back(v); break; }
                        case TK_CHAR8:   { char v; data->get_char8_value(v, elem_id); out.push_back(v); break; }
                        case TK_INT8:    { int8_t v; data->get_int8_value(v, elem_id); out.push_back(v); break; }
                        case TK_UINT8:   { uint8_t v; data->get_uint8_value(v, elem_id); out.push_back(v); break; }
                        case TK_INT16:   { int16_t v; data->get_int16_value(v, elem_id); out.push_back(v); break; }
                        case TK_UINT16:  { uint16_t v; data->get_uint16_value(v, elem_id); out.push_back(v); break; }
                        case TK_INT32:   { int32_t v; data->get_int32_value(v, elem_id); out.push_back(v); break; }
                        case TK_UINT32:  { uint32_t v; data->get_uint32_value(v, elem_id); out.push_back(v); break; }
                        case TK_INT64:   { int64_t v; data->get_int64_value(v, elem_id); out.push_back(v); break; }
                        case TK_UINT64:  { uint64_t v; data->get_uint64_value(v, elem_id); out.push_back(v); break; }
                        case TK_FLOAT32: { float v; data->get_float32_value(v, elem_id); out.push_back(float_to_json(v)); break; }
                        case TK_FLOAT64: { double v; data->get_float64_value(v, elem_id); out.push_back(float_to_json(v)); break; }
                        case TK_FLOAT128:{ long double v; data->get_float128_value(v, elem_id); out.push_back(float_to_json(static_cast<double>(v))); break; }
                        case TK_STRING8: { std::string v; data->get_string_value(v, elem_id); out.push_back(v); break; }
                        default: throw UnsupportedType(std::string("sequence element kind ") + std::to_string(static_cast<int>(elem_kind)));
                    }
                }
            }
            break;
        }

        default:
            throw UnsupportedType(std::string("top-level kind ") + std::to_string(static_cast<int>(kind)));
    }
}

// ---------------------------------------------------------------------------
// json_to_xtypes — recursive helper

static void json_to_xtypes_impl(
        const Json& json,
        const xtypes::DynamicData& data,
        const xtypes::DynamicType& type)
{
    using namespace fastdds::dds;

    const TypeKind kind = type->get_kind();

    switch (kind)
    {
        case TK_STRUCTURE:
        {
            const uint32_t count = type->get_member_count();
            for (uint32_t i = 0; i < count; ++i)
            {
                DynamicTypeMember::_ref_type member;
                type->get_member_by_index(member, i);

                MemberDescriptor::_ref_type desc = traits<MemberDescriptor>::make_shared();
                member->get_descriptor(desc);

                const std::string name = desc->name().to_string();
                const MemberId id = member->get_id();
                const TypeKind member_kind = desc->type()->get_kind();

                if (json.find(name) == json.end())
                {
                    std::cerr << "[json-xtypes] json_to_xtypes: missing member '" << name << "'" << std::endl;
                    continue;
                }
                const Json& jval = json[name];

                if (member_kind == TK_STRUCTURE || member_kind == TK_SEQUENCE || member_kind == TK_ARRAY)
                {
                    auto nested = data->loan_value(id);
                    json_to_xtypes_impl(jval, nested, desc->type());
                    data->return_loaned_value(nested);
                }
                else
                {
                    switch (member_kind)
                    {
                        case TK_BOOLEAN: data->set_boolean_value(id, jval.get<bool>()); break;
                        case TK_CHAR8:   data->set_char8_value(id, jval.get<char>()); break;
                        case TK_INT8:    data->set_int8_value(id, jval.get<int8_t>()); break;
                        case TK_UINT8:   data->set_uint8_value(id, jval.get<uint8_t>()); break;
                        case TK_INT16:   data->set_int16_value(id, jval.get<int16_t>()); break;
                        case TK_UINT16:  data->set_uint16_value(id, jval.get<uint16_t>()); break;
                        case TK_INT32:   data->set_int32_value(id, jval.get<int32_t>()); break;
                        case TK_UINT32:  data->set_uint32_value(id, jval.get<uint32_t>()); break;
                        case TK_INT64:   data->set_int64_value(id, jval.get<int64_t>()); break;
                        case TK_UINT64:  data->set_uint64_value(id, jval.get<uint64_t>()); break;
                        case TK_FLOAT32: data->set_float32_value(id, get_json_float<float>(jval)); break;
                        case TK_FLOAT64: data->set_float64_value(id, get_json_float<double>(jval)); break;
                        case TK_FLOAT128:data->set_float128_value(id, get_json_float<long double>(jval)); break;
                        case TK_STRING8: data->set_string_value(id, jval.get<std::string>()); break;
                        default: throw UnsupportedType(std::string("struct member kind ") + std::to_string(static_cast<int>(member_kind)));
                    }
                }
            }
            break;
        }

        case TK_SEQUENCE:
        {
            // data is the sequence DynamicData; json should be an array.
            // Fast-DDS 3.x has no insert_sequence_data().
            // - For complex elements use set_complex_value(i, elem) which auto-resizes.
            // - For primitive elements collect into a typed vector and use set_XXX_values().
            TypeDescriptor::_ref_type td = traits<TypeDescriptor>::make_shared();
            type->get_descriptor(td);
            const DynamicType::_ref_type elem_type = td->element_type();
            const TypeKind elem_kind = elem_type->get_kind();

            data->clear_all_values();
            if (elem_kind == TK_STRUCTURE || elem_kind == TK_SEQUENCE || elem_kind == TK_ARRAY)
            {
                uint32_t i = 0;
                for (const auto& jelem : json)
                {
                    auto elem_data = DynamicDataFactory::get_instance()->create_data(elem_type);
                    json_to_xtypes_impl(jelem, elem_data, elem_type);
                    data->set_complex_value(i++, elem_data);
                }
            }
            else
            {
                switch (elem_kind)
                {
                    case TK_BOOLEAN: { BooleanSeq v; for (const auto& e : json) v.push_back(e.get<bool>()); data->set_boolean_values(MEMBER_ID_INVALID, v); break; }
                    case TK_CHAR8:   { CharSeq v;    for (const auto& e : json) v.push_back(e.get<char>()); data->set_char8_values(MEMBER_ID_INVALID, v); break; }
                    case TK_INT8:    { Int8Seq v;    for (const auto& e : json) v.push_back(e.get<int8_t>()); data->set_int8_values(MEMBER_ID_INVALID, v); break; }
                    case TK_UINT8:   { UInt8Seq v;   for (const auto& e : json) v.push_back(e.get<uint8_t>()); data->set_uint8_values(MEMBER_ID_INVALID, v); break; }
                    case TK_INT16:   { Int16Seq v;   for (const auto& e : json) v.push_back(e.get<int16_t>()); data->set_int16_values(MEMBER_ID_INVALID, v); break; }
                    case TK_UINT16:  { UInt16Seq v;  for (const auto& e : json) v.push_back(e.get<uint16_t>()); data->set_uint16_values(MEMBER_ID_INVALID, v); break; }
                    case TK_INT32:   { Int32Seq v;   for (const auto& e : json) v.push_back(e.get<int32_t>()); data->set_int32_values(MEMBER_ID_INVALID, v); break; }
                    case TK_UINT32:  { UInt32Seq v;  for (const auto& e : json) v.push_back(e.get<uint32_t>()); data->set_uint32_values(MEMBER_ID_INVALID, v); break; }
                    case TK_INT64:   { Int64Seq v;   for (const auto& e : json) v.push_back(e.get<int64_t>()); data->set_int64_values(MEMBER_ID_INVALID, v); break; }
                    case TK_UINT64:  { UInt64Seq v;  for (const auto& e : json) v.push_back(e.get<uint64_t>()); data->set_uint64_values(MEMBER_ID_INVALID, v); break; }
                    case TK_FLOAT32: { Float32Seq v; for (const auto& e : json) v.push_back(get_json_float<float>(e)); data->set_float32_values(MEMBER_ID_INVALID, v); break; }
                    case TK_FLOAT64: { Float64Seq v; for (const auto& e : json) v.push_back(get_json_float<double>(e)); data->set_float64_values(MEMBER_ID_INVALID, v); break; }
                    case TK_FLOAT128:{ Float128Seq v;for (const auto& e : json) v.push_back(get_json_float<long double>(e)); data->set_float128_values(MEMBER_ID_INVALID, v); break; }
                    case TK_STRING8: { StringSeq v;  for (const auto& e : json) v.push_back(e.get<std::string>()); data->set_string_values(MEMBER_ID_INVALID, v); break; }
                    default: throw UnsupportedType(std::string("sequence element kind ") + std::to_string(static_cast<int>(elem_kind)));
                }
            }
            break;
        }

        case TK_ARRAY:
        {
            TypeDescriptor::_ref_type td = traits<TypeDescriptor>::make_shared();
            type->get_descriptor(td);
            const DynamicType::_ref_type elem_type = td->element_type();
            const TypeKind elem_kind = elem_type->get_kind();

            uint32_t i = 0;
            for (const auto& jelem : json)
            {
                MemberId elem_id = data->get_member_id_at_index(i++);
                if (elem_kind == TK_STRUCTURE || elem_kind == TK_SEQUENCE || elem_kind == TK_ARRAY)
                {
                    auto nested = data->loan_value(elem_id);
                    json_to_xtypes_impl(jelem, nested, elem_type);
                    data->return_loaned_value(nested);
                }
                else
                {
                    switch (elem_kind)
                    {
                        case TK_BOOLEAN: data->set_boolean_value(elem_id, jelem.get<bool>()); break;
                        case TK_CHAR8:   data->set_char8_value(elem_id, jelem.get<char>()); break;
                        case TK_INT8:    data->set_int8_value(elem_id, jelem.get<int8_t>()); break;
                        case TK_UINT8:   data->set_uint8_value(elem_id, jelem.get<uint8_t>()); break;
                        case TK_INT16:   data->set_int16_value(elem_id, jelem.get<int16_t>()); break;
                        case TK_UINT16:  data->set_uint16_value(elem_id, jelem.get<uint16_t>()); break;
                        case TK_INT32:   data->set_int32_value(elem_id, jelem.get<int32_t>()); break;
                        case TK_UINT32:  data->set_uint32_value(elem_id, jelem.get<uint32_t>()); break;
                        case TK_INT64:   data->set_int64_value(elem_id, jelem.get<int64_t>()); break;
                        case TK_UINT64:  data->set_uint64_value(elem_id, jelem.get<uint64_t>()); break;
                        case TK_FLOAT32: data->set_float32_value(elem_id, get_json_float<float>(jelem)); break;
                        case TK_FLOAT64: data->set_float64_value(elem_id, get_json_float<double>(jelem)); break;
                        case TK_FLOAT128:data->set_float128_value(elem_id, get_json_float<long double>(jelem)); break;
                        case TK_STRING8: data->set_string_value(elem_id, jelem.get<std::string>()); break;
                        default: throw UnsupportedType(std::string("array element kind ") + std::to_string(static_cast<int>(elem_kind)));
                    }
                }
            }
            break;
        }

        default:
            throw UnsupportedType(std::string("top-level kind ") + std::to_string(static_cast<int>(kind)));
    }
}

// ---------------------------------------------------------------------------
// Public API

Json convert(
        const xtypes::DynamicData& input,
        const std::string submember)
{
    Json json_message;
    Json& target = submember.empty() ? json_message : json_message[submember];
    xtypes_to_json_impl(input, input->type(), target);
    return json_message;
}

xtypes::DynamicData convert(
        const xtypes::DynamicType& type,
        const Json& input,
        const std::string submember)
{
    xtypes::DynamicData data =
        xtypes::DynamicDataFactory::get_instance()->create_data(type);

    const Json& source = submember.empty() ? input : input.at(submember);
    json_to_xtypes_impl(source, data, type);
    return data;
}

} //  namespace json_xtypes
} //  namespace is
} //  namespace eprosima
