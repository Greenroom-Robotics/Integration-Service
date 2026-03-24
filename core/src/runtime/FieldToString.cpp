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

#include <is/core/runtime/FieldToString.hpp>

#include <string>
#include <unordered_map>
#include <functional>

namespace eprosima {
namespace is {
namespace core {

class FieldToString::Implementation
{
public:

    static Implementation& instance()
    {
        static Implementation instance;
        return instance;
    }

    const std::string to_string(
            const xtypes::DynamicData& parent,
            const std::string& field_name,
            const std::string& details)
    {
        // Get the MemberId for this field
        xtypes::MemberId id = parent->get_member_id_by_name(field_name);
        if (id == fastdds::dds::MEMBER_ID_INVALID)
        {
            throw UnknownFieldToStringCast("<unknown — member not found>", field_name, details);
        }

        // Determine the TypeKind of this member
        fastdds::dds::DynamicTypeMember::_ref_type member;
        if (parent->type()->get_member_by_name(member, field_name) !=
                fastdds::dds::RETCODE_OK)
        {
            throw UnknownFieldToStringCast("<unknown — type lookup failed>", field_name, details);
        }

        fastdds::dds::MemberDescriptor::_ref_type desc =
            fastdds::dds::traits<fastdds::dds::MemberDescriptor>::make_shared();
        member->get_descriptor(desc);
        fastdds::dds::TypeKind kind = desc->type()->get_kind();

        _logger << utils::Logger::Level::DEBUG
                << "Trying to convert TypeKind " << static_cast<int>(kind)
                << " for field '" << field_name << "' to string" << std::endl;

        const auto it = _conversions.find(kind);
        if (it != _conversions.end())
        {
            return it->second(parent, id);
        }

        // Special case: string type (TK_STRING8) — not in the primitive map but
        // handled separately because we can just retrieve it directly.
        if (kind == fastdds::dds::TK_STRING8)
        {
            std::string val;
            parent->get_string_value(val, id);
            return val;
        }

        _logger << utils::Logger::Level::ERROR
                << "Failed to convert TypeKind " << static_cast<int>(kind)
                << " for field '" << field_name << "' to string" << std::endl;

        throw UnknownFieldToStringCast(std::to_string(static_cast<int>(kind)),
                field_name, details);
    }

private:

    using ConversionFunc = std::function<std::string(
                        const xtypes::DynamicData&, xtypes::MemberId)>;
    using ConversionMap = std::unordered_map<fastdds::dds::TypeKind, ConversionFunc>;

    Implementation()
        : _logger("is::core::FieldToString")
    {
        using namespace fastdds::dds;

        _conversions[TK_BOOLEAN] =
                [](const DynamicData::_ref_type& d, MemberId id) -> std::string {
                    bool v; d->get_boolean_value(v, id);
                    return std::to_string(v);
                };
        _conversions[TK_CHAR8] =
                [](const DynamicData::_ref_type& d, MemberId id) -> std::string {
                    char v; d->get_char8_value(v, id);
                    return std::string(1, v);
                };
        _conversions[TK_INT8] =
                [](const DynamicData::_ref_type& d, MemberId id) -> std::string {
                    int8_t v; d->get_int8_value(v, id);
                    return std::to_string(v);
                };
        _conversions[TK_UINT8] =
                [](const DynamicData::_ref_type& d, MemberId id) -> std::string {
                    uint8_t v; d->get_uint8_value(v, id);
                    return std::to_string(v);
                };
        _conversions[TK_INT16] =
                [](const DynamicData::_ref_type& d, MemberId id) -> std::string {
                    int16_t v; d->get_int16_value(v, id);
                    return std::to_string(v);
                };
        _conversions[TK_UINT16] =
                [](const DynamicData::_ref_type& d, MemberId id) -> std::string {
                    uint16_t v; d->get_uint16_value(v, id);
                    return std::to_string(v);
                };
        _conversions[TK_INT32] =
                [](const DynamicData::_ref_type& d, MemberId id) -> std::string {
                    int32_t v; d->get_int32_value(v, id);
                    return std::to_string(v);
                };
        _conversions[TK_UINT32] =
                [](const DynamicData::_ref_type& d, MemberId id) -> std::string {
                    uint32_t v; d->get_uint32_value(v, id);
                    return std::to_string(v);
                };
        _conversions[TK_INT64] =
                [](const DynamicData::_ref_type& d, MemberId id) -> std::string {
                    int64_t v; d->get_int64_value(v, id);
                    return std::to_string(v);
                };
        _conversions[TK_UINT64] =
                [](const DynamicData::_ref_type& d, MemberId id) -> std::string {
                    uint64_t v; d->get_uint64_value(v, id);
                    return std::to_string(v);
                };
        _conversions[TK_FLOAT32] =
                [](const DynamicData::_ref_type& d, MemberId id) -> std::string {
                    float v; d->get_float32_value(v, id);
                    return std::to_string(v);
                };
        _conversions[TK_FLOAT64] =
                [](const DynamicData::_ref_type& d, MemberId id) -> std::string {
                    double v; d->get_float64_value(v, id);
                    return std::to_string(v);
                };
        _conversions[TK_FLOAT128] =
                [](const DynamicData::_ref_type& d, MemberId id) -> std::string {
                    long double v; d->get_float128_value(v, id);
                    return std::to_string(static_cast<double>(v));
                };
        _conversions[TK_STRING8] =
                [](const DynamicData::_ref_type& d, MemberId id) -> std::string {
                    std::string v; d->get_string_value(v, id);
                    return v;
                };
    }

    Implementation(const Implementation&) = delete;
    Implementation(Implementation&&) = delete;
    ~Implementation() = default;

    ConversionMap _conversions;
    utils::Logger _logger;
};

//==============================================================================
FieldToString::FieldToString(
        const std::string& usage_details)
    : _pimpl(Implementation::instance())
    , _details(usage_details)
{
}

//==============================================================================
FieldToString::FieldToString(
        const FieldToString& other)
    : _pimpl(Implementation::instance())
    , _details(other._details)
{
}

//==============================================================================
FieldToString::FieldToString(
        FieldToString&& other)
    : _pimpl(Implementation::instance())
    , _details(std::move(other._details))
{
}

//==============================================================================
const std::string FieldToString::to_string(
        const eprosima::xtypes::DynamicData& parent,
        const std::string& field_name) const
{
    return _pimpl.to_string(parent, field_name, _details);
}

//==============================================================================
const std::string& FieldToString::details() const
{
    return _details;
}

//==============================================================================
std::string& FieldToString::details()
{
    return _details;
}

//==============================================================================
UnknownFieldToStringCast::UnknownFieldToStringCast(
        const std::string& type,
        const std::string& field_name,
        const std::string& details)
    : std::runtime_error(
        std::string()
        + "ERROR: Unable to cast type '" + type + "' of field '" + field_name
        + "' to a string. Details: " + details)
    , _type(type)
    , _field_name(field_name)
{
}

//==============================================================================
const std::string& UnknownFieldToStringCast::type() const
{
    return _type;
}

//==============================================================================
const std::string& UnknownFieldToStringCast::field_name() const
{
    return _field_name;
}

} //  namespace core
} //  namespace is
} //  namespace eprosima
