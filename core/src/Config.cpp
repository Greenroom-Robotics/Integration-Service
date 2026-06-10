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

#include <is/core/Config.hpp>
#include <is/systemhandle/SystemHandle.hpp>

#include <algorithm>
#include <iostream>

namespace eprosima {
namespace is {
namespace core {
namespace internal {
namespace {

//==============================================================================
bool scalar_or_list_node_to_set(
        const YAML::Node& node,
        std::set<std::string>& values,
        const std::string& field,
        const std::string& route_type)
{
    bool valid = true;

    if (node.IsSequence())
    {
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
        {
            values.insert(it->as<std::string>());
        }
    }
    else if (node.IsScalar())
    {
        values.insert(node.as<std::string>());
    }
    else
    {
        valid = false;
    }

    if (!valid)
    {
        Config::logger << utils::Logger::Level::ERROR
                       << "config-file '" << field << "' entry in " << route_type
                       << " route must point to a string or list of at least one string"
                       << std::endl;
        valid = false;
    }

    return valid;
}

//==============================================================================
std::unique_ptr<TopicRoute> parse_topic_route(
        const YAML::Node& node)
{
    auto route = std::make_unique<TopicRoute>();
    bool valid = true;

    valid &= scalar_or_list_node_to_set(
        node["from"], route->from, "from", "topic");

    valid &= scalar_or_list_node_to_set(
        node["to"], route->to, "to", "topic");

    std::ostringstream from_list;
    if (node["from"].IsSequence())
    {
        from_list << "[ ";

        for (const auto& from : node["from"])
        {
            from_list << from.as<std::string>() << " ";
        }

        from_list << "]";
    }
    else
    {
        from_list << node["from"].as<std::string>();
    }

    std::ostringstream to_list;
    if (node["to"].IsSequence())
    {
        to_list << "[ ";

        for (const auto& to : node["to"])
        {
            to_list << to.as<std::string>() << " ";
        }

        to_list << "]";
    }
    else
    {
        to_list << node["to"].as<std::string>();
    }

    if (!valid)
    {
        Config::logger << utils::Logger::Level::ERROR
                       << "Topic route { from: '" << from_list.str()
                       << "', to: '" << to_list.str()
                       << "' } is invalid." << std::endl;
        return nullptr;
    }

    Config::logger << utils::Logger::Level::DEBUG
                   << "Topic route { from: '" << from_list.str()
                   << "', to: '" << to_list.str()
                   << "' } is correct." << std::endl;

    return route;
}

//==============================================================================
std::unique_ptr<ServiceRoute> parse_service_route(
        const YAML::Node& node)
{
    bool valid = true;
    auto route = std::make_unique<ServiceRoute>();

    const YAML::Node& server = node["server"];
    if (!server)
    {
        Config::logger << utils::Logger::Level::ERROR
                       << "Config-file service route must contain a 'server' entry!"
                       << std::endl;
        valid = false;
    }

    if (!server.IsScalar() || server.as<std::string>().empty())
    {
        Config::logger << utils::Logger::Level::ERROR
                       << "Config-file service route 'server' entry must be a scalar "
                       << "string! Only one middleware can be the server on a route!"
                       << std::endl;
        valid = false;
    }
    else
    {
        route->server = server.as<std::string>();
    }

    valid &= scalar_or_list_node_to_set(
        node["clients"], route->clients, "clients", "service");

    std::ostringstream client_list;
    if (node["clients"].IsSequence())
    {
        client_list << "[ ";

        for (const auto& client : node["clients"])
        {
            client_list << client.as<std::string>() << " ";
        }

        client_list << "]";
    }
    else
    {
        client_list << node["clients"].as<std::string>();
    }

    if (!valid)
    {
        Config::logger << utils::Logger::Level::ERROR
                       << "Service route { server: '" << node["server"].as<std::string>()
                       << "', clients: '" << client_list.str() << "' is invalid." << std::endl;

        return nullptr;
    }

    Config::logger << utils::Logger::Level::DEBUG
                   << "Service route { server: '" << node["server"].as<std::string>()
                   << "', clients: '" << client_list.str() << "' is correct." << std::endl;

    return route;
}

//==============================================================================
bool add_types(
        const YAML::Node& node,
        const std::string& filename,
        [[maybe_unused]] std::map<std::string, eprosima::xtypes::DynamicType>& types)
{
    if (!node["types"])
    {
        return true;
    }

    if (!node["types"]["idls"])
    {
        Config::logger << utils::Logger::Level::ERROR
                       << "The config file '" << filename
                       << "' has a 'types' entry which doesn't contains an 'idls' entry."
                       << std::endl;
        return false;
    }

    if (!node["types"]["idls"].IsSequence())
    {
        Config::logger << utils::Logger::Level::ERROR
                       << "The config file '" << filename << "' has an 'idls' entry but "
                       << "it's not a YAML sequence." << std::endl;
        return false;
    }

    std::vector<std::string> include_paths;
    if (node["types"]["paths"])
    {
        for (auto& path : node["types"]["paths"])
        {
            Config::logger << utils::Logger::Level::DEBUG
                           << "Adding path '" << path.as<std::string>()
                           << "' to the IDL file database." << std::endl;

            include_paths.push_back(path.as<std::string>());
        }
    }

    uint16_t idl_index = 1;
    for (auto& entry: node["types"]["idls"])
    {
        (void)entry;
        Config::logger << utils::Logger::Level::ERROR
                       << "Inline IDL type definitions in YAML config (idls entry "
                       << idl_index << " in '" << filename
                       << "') are not supported with the Fast-DDS XTypes API. "
                       << "Types must be registered via mix files." << std::endl;
        return false;
    }

    return true;
}

//==============================================================================
bool add_named_route(
        const std::string& name,
        const YAML::Node& node,
        std::map<std::string, TopicRoute>& topic_routes,
        std::map<std::string, ServiceRoute>& service_routes)
{
    bool inserted = false;
    if (node["from"] && node["to"])
    {
        const auto route = parse_topic_route(node);
        if (!route)
        {
            Config::logger << utils::Logger::Level::ERROR
                           << "Failed to parse the route named '"
                           << name << "'" << std::endl;
            return false;
        }

        Config::logger << utils::Logger::Level::DEBUG
                       << "Add topic route '" << name << "'." << std::endl;

        inserted = topic_routes.insert(std::make_pair(name, *route)).second;
    }
    else if (node["server"] && node["clients"])
    {
        const auto route = parse_service_route(node);
        if (!route)
        {
            Config::logger << utils::Logger::Level::ERROR
                           << "Failed to parse the route named '"
                           << name << "'" << std::endl;

            return false;
        }

        Config::logger << utils::Logger::Level::DEBUG
                       << "Add service route '" << name << "'." << std::endl;

        inserted = service_routes.insert(std::make_pair(name, *route)).second;
    }
    else
    {
        Config::logger << utils::Logger::Level::ERROR
                       << "Cannot recognize route named '" << name << "' because it "
                       << "does not match a topic route scheme {from: X, to: Y} "
                       << "or a service route scheme {server: X, clients: Y}" << std::endl;
        return false;
    }

    if (!inserted)
    {
        Config::logger << utils::Logger::Level::WARN
                       << "Duplicate route named '" << name
                       << "'!" << std::endl;
    }

    return inserted;
}

//==============================================================================
void set_middleware_config(
        std::map<std::string, YAML::Node>& middleware_configs,
        const TopicRoute& route,
        const YAML::Node& node)
{
    for (const std::string& from : route.from)
    {
        if (node[from])
        {
            middleware_configs[from] = node[from];
        }
        else
        {
            middleware_configs[from] = node;
        }
    }
    for (const std::string& to : route.to)
    {
        if (node[to])
        {
            middleware_configs[to] = node[to];
        }
        else
        {
            middleware_configs[to] = node;
        }
    }
}

//==============================================================================
void set_middleware_config(
        std::map<std::string, YAML::Node>& middleware_configs,
        const ServiceRoute& route,
        const YAML::Node& node)
{
    // TODO (@jamoralp): types, route, and remap should be ignored here?
    middleware_configs[route.server] = node;

    for (const std::string& client : route.clients)
    {
        middleware_configs[client] = node;
    }
}

//==============================================================================
template<typename ConfigType, typename RouteType>
bool add_topic_or_service_config(
        const std::string& channel_type,
        const std::string& name,
        const YAML::Node& node,
        const std::map<std::string, RouteType>& predefined_routes,
        std::map<std::string, ConfigType>& config_map,
        std::function<void(ConfigType&, std::string&&)> set_type,
        std::function<void(ConfigType&, std::string&&)> set_reply_type,
        std::function<void(ConfigType&, const RouteType&)> set_route,
        std::function<std::unique_ptr<RouteType>(const YAML::Node&)> parse_route)
{
    bool valid = true;
    ConfigType config;

    const YAML::Node& type = node["type"];
    if (!type)
    {
        if (channel_type == "service")
        {
            const YAML::Node& request_type = node["request_type"];
            const YAML::Node& reply_type = node["reply_type"];

            if (!request_type && !reply_type)
            {
                Config::logger << utils::Logger::Level::ERROR
                               << "The " << channel_type << " configuration '" << name
                               << "' is missing its 'type', or 'request_type' "
                               << " and 'reply_type' fields!" << std::endl;
                valid = false;
            }
            else
            {
                std::string request_type_str = request_type.as<std::string>();
                set_type(config, std::move(request_type_str));
                std::string reply_type_str = reply_type.as<std::string>();
                set_reply_type(config, std::move(reply_type_str));
            }
        }
        else
        {
            Config::logger << utils::Logger::Level::ERROR
                           << "The " << channel_type << " configuration '" << name
                           << "' is missing its 'type' field!" << std::endl;
            valid = false;
        }
    }
    else
    {
        std::string type_str = type.as<std::string>();
        set_type(config, std::move(type_str));
    }

    const YAML::Node& route = node["route"];
    if (!route)
    {
        Config::logger << utils::Logger::Level::ERROR
                       << "The " << channel_type << " configuration '" << name
                       << "' is missing its 'route' field!" << std::endl;
        valid = false;
    }
    else
    {
        if (route.IsScalar())
        {
            const std::string& route_name = route.as<std::string>();
            const auto it = predefined_routes.find(route_name);
            if (it == predefined_routes.end())
            {
                Config::logger << utils::Logger::Level::ERROR
                               << "The " << channel_type << " configuration '" << name
                               << "' requested a route named '" << route_name << "' but a "
                               << "route with that name was never defined in the 'routes' "
                               << "dictionary!" << std::endl;
                valid = false;
            }
            else
            {
                set_route(config, it->second);
            }
        }
        else if (route.IsMap())
        {
            const auto route_config = parse_route(route);
            if (!route_config)
            {
                Config::logger << utils::Logger::Level::ERROR
                               << "Failed to parse the route configuration for the "
                               << channel_type << " '" << name << "'!" << std::endl;
                valid = false;
            }
            else
            {
                set_route(config, *route_config);
            }
        }
    }

    const YAML::Node& remap = node["remap"];
    if (remap)
    {
        if (!remap.IsMap())
        {
            Config::logger << utils::Logger::Level::ERROR
                           << "A 'remap' field was given for the " << channel_type
                           << " configuration '" << name
                           << "', but it is not a dictionary!" << std::endl;
            valid = false;
        }
        else
        {
            for (YAML::const_iterator it = remap.begin(); it != remap.end(); ++it)
            {
                TopicInfo& remap_info = config.remap[it->first.as<std::string>()];
                if (it->second["topic"])
                {
                    remap_info.name = it->second["topic"].as<std::string>();
                }
                if (it->second["type"])
                {
                    remap_info.type = it->second["type"].as<std::string>();
                }
                if (it->second["request_type"])
                {
                    remap_info.type = it->second["request_type"].as<std::string>();
                }
                if (it->second["reply_type"])
                {
                    remap_info.reply_type = it->second["reply_type"].as<std::string>();
                }
            }
        }
    }

    /**
     *  TODO(@jamoralp): Come up with a yaml scene for specifying middleware
     * configurations per topic/service and then fill in the middleware_configs map here.
     */
    // Proposal
    if (valid)
    {
        set_middleware_config(config.middleware_configs, config.route, node);
    }
    // End of proposal

    if (valid)
    {
        if (!config_map.insert(std::make_pair(name, config)).second)
        {
            // If configuration specified twice, we will stick with the first one
            Config::logger << utils::Logger::Level::WARN
                           << channel_type << " configuration '" << name
                           << "' was specified twice!" << std::endl;
        }
    }

    return valid;
}

//==============================================================================
bool add_topic_config(
        const std::string& name,
        const YAML::Node& node,
        const std::map<std::string, TopicRoute>& topic_routes,
        std::map<std::string, TopicConfig>& topic_configs)
{
    return add_topic_or_service_config<TopicConfig, TopicRoute>(
        "topic", name, node, topic_routes, topic_configs,
        [=](TopicConfig& config, std::string&& type)
        {
            Config::logger << utils::Logger::Level::DEBUG
                           << "Set type '" << type << "' for topic '"
                           << name << "'." << std::endl;

            config.message_type = std::move(type);
        },
        [=](TopicConfig&, std::string&&)
        {
        },
        [=](TopicConfig& config, const TopicRoute& route)
        {
            Config::logger << utils::Logger::Level::DEBUG;
            Config::logger << "Set route '{ from: ";

            for (const auto& _from : route.from)
            {
                Config::logger << _from << " ";
            }

            Config::logger << ", to: ";
            for (const auto& _to : route.to)
            {
                Config::logger << _to << " ";
            }

            Config::logger << "}' for topic '" << name << "'." << std::endl;

            config.route = route;
        },
        [](const YAML::Node& route)
        {
            return parse_topic_route(route);
        });
}

//==============================================================================
bool add_service_config(
        const std::string& name,
        const YAML::Node& node,
        const std::map<std::string, ServiceRoute>& service_routes,
        std::map<std::string, ServiceConfig>& service_configs)
{
    return add_topic_or_service_config<ServiceConfig, ServiceRoute>(
        "service", name, node, service_routes, service_configs,
        [=](ServiceConfig& config, std::string&& type)
        {
            Config::logger << utils::Logger::Level::DEBUG
                           << "Set request type '" << type
                           << "' for service '" << name
                           << "'." << std::endl;

            config.request_type = std::move(type);
        },
        [=](ServiceConfig& config, std::string&& type)
        {
            Config::logger << utils::Logger::Level::DEBUG
                           << "Set reply type '" << type
                           << "' for service '" << name
                           << "'." << std::endl;

            config.reply_type = std::move(type);
        },
        [=](ServiceConfig& config, const ServiceRoute& route)
        {
            Config::logger << utils::Logger::Level::DEBUG;
            Config::logger << "Set route '{ server: " << route.server;

            Config::logger << ", clients: ";
            for (const auto& _client : route.clients)
            {
                Config::logger << _client << " ";
            }

            Config::logger << "}' for service '" << name << "'." << std::endl;

            config.route = route;
        },
        [](const YAML::Node& route)
        {
            return parse_service_route(route);
        });
}

//==============================================================================
using ReadDictEntry =
        std::function<bool (const std::string& key, const YAML::Node& value)>;

bool read_dictionary(
        const YAML::Node& config_node,
        const std::string& field_name,
        const std::string& filename,
        const ReadDictEntry& read_fcn)
{
    const YAML::Node& field = config_node[field_name];
    if (field)
    {
        if (!field.IsMap())
        {
            Config::logger << utils::Logger::Level::ERROR
                           << "The config file '" << filename << "' has a '"
                           << field_name
                           << "' field, but it is not pointing to a dictionary of "
                // We use this to make the field name singular
                           << field_name.substr(0, field_name.size() - 1)
                           << "configurations!" << std::endl;
            return false;
        }

        bool configs_okay = true;
        for (YAML::const_iterator it = field.begin(); it != field.end(); ++it)
        {
            configs_okay &= read_fcn(it->first.as<std::string>(), it->second);
        }

        if (!configs_okay)
        {
            Config::logger << utils::Logger::Level::ERROR
                           << "Error found in an entry of the '" << field_name
                           << "' dictionary of the config-file '" << filename << "'"
                           << std::endl;
            return false;
        }
    }

    return true;
}

//==============================================================================
YAML::Node config_or_empty_node(
        const std::string& key,
        const std::map<std::string, YAML::Node>& map)
{
    const auto it = map.find(key);
    if (it == map.end())
    {
        return YAML::Node();
    }

    return it->second;
}

//==============================================================================
TopicInfo remap_if_needed(
        const std::string& middleware,
        const std::map<std::string, TopicInfo>& remap,
        const TopicInfo& original)
{
    const auto it = remap.find(middleware);
    if (it == remap.end())
    {
        return original;
    }

    TopicInfo config = original;
    if (!it->second.name.empty())
    {
        config.name = it->second.name;
    }
    if (!it->second.type.empty())
    {
        config.type = it->second.type;
    }
    if (!it->second.reply_type.empty())
    {
        config.reply_type = it->second.reply_type;
    }
    return config;
}

} //  anonymous namespace

//==============================================================================
utils::Logger Config::logger("is::core::Config");

//==============================================================================
Config::Config(
        const YAML::Node& node,
        const std::string& filename)
{
    if (node && !node.IsNull())
    {
        parse(node, filename);
    }
}

//==============================================================================
Config Config::from_file(
        const std::string& file)
{
    YAML::Node config_node;
    try
    {
        config_node = YAML::LoadFile(file);
    }
    catch (const std::exception& e)
    {
        logger << utils::Logger::Level::ERROR
               << "Could not parse the config-file named '" << file
               << "': " << e.what() << std::endl;
        return Config();
    }

    if (!config_node)
    {
        logger << utils::Logger::Level::ERROR
               << "Could not parse the config-file '" << file
               << "'" << std::endl;
        return Config();
    }

    return Config(config_node, file);
}

//==============================================================================
bool Config::parse(
        const YAML::Node& config_node,
        const std::string& file)
{
    _okay = false;

    /**
     * YAML formatting checks for Integration Service configuration.
     */
    if (!config_node.IsMap())
    {
        logger << utils::Logger::Level::ERROR
               << "The config-file '" << file << "' needs to be a map "
               << "containing 'systems', and possibly 'routes', 'topics', "
               << "and/or 'services'!" << std::endl;
        return false;
    }

    const YAML::Node& systems = config_node["systems"];
    if (!systems || !systems.IsMap())
    {
        logger << utils::Logger::Level::ERROR
               << "The config-file '" << file << "' is missing a "
               << "dictionary for 'systems'! You must specify at least two "
               << "systems in your config-file." << std::endl;
        return false;
    }

    /**
     * Iterates through systems and retrieves their 'type' (ros2, ros1, dds, websocket...)
     * and the `types-from` attribute, if applicable.
     */
    for (YAML::const_iterator it = systems.begin(); it != systems.end(); ++it)
    {
        const std::string middleware_alias = it->first.as<std::string>();

        const YAML::Node& config = it->second;

        const YAML::Node& type_node = config["type"];
        const std::string middleware = type_node ?
                type_node.as<std::string>() : middleware_alias;

        const YAML::Node& types_from_node = config["types-from"];
        std::vector<std::string> types_from;

        if (types_from_node)
        {
            if (types_from_node.IsSequence())
            {
                for (const YAML::Node& types : types_from_node)
                {
                    types_from.push_back(types.as<std::string>());
                }
            }
            else
            {
                types_from.push_back(types_from_node.as<std::string>());
            }
        }

        _m_middlewares.insert(
            std::make_pair(
                middleware_alias, MiddlewareConfig{middleware, types_from, config, config_node["types"]}));
    }

    if (_m_middlewares.size() < 2)
    {
        logger << utils::Logger::Level::ERROR
               << "The config-file '" << file << "' does not specify enough "
               << "middlewares '" << _m_middlewares.size() << "'! The "
               << "minimum is 2." << std::endl;
        return false;
    }

    /**
     * Retrieves types from the `types` section and adds them to the _m_types database.
     */
    if (!add_types(config_node, file, _m_types))
    {
        return false;
    }

    /**
     * Retrieves routes from the `routes` section and adds them to the _m_topic_routes
     * or the _m_service_routes database accordingly.
     */
    auto read_route =
            [&](const std::string& key, const YAML::Node& node) -> bool
            {
                return add_named_route(key, node, _m_topic_routes, _m_service_routes);
            };

    if (!read_dictionary(config_node, "routes", file, read_route))
    {
        return false;
    }

    /**
     * Retrieves topics from the `topics` section and adds them to the _m_topic_configs database.
     */
    auto read_topic =
            [&](const std::string& key, const YAML::Node& node) -> bool
            {
                return add_topic_config(key, node, _m_topic_routes, _m_topic_configs);
            };

    if (!read_dictionary(config_node, "topics", file, read_topic))
    {
        return false;
    }

    /**
     * Retrieves services from the `services` section and adds them to the _m_service_configs database.
     */
    auto read_service =
            [&](const std::string& key, const YAML::Node& node) -> bool
            {
                return add_service_config(key, node, _m_service_routes, _m_service_configs);
            };

    if (!read_dictionary(config_node, "services", file, read_service))
    {
        return false;
    }

    /**
     * Checks topics configuration.
     */
    for (const auto& [topic_name, topic_config] : _m_topic_configs)
    {
        /**
         * Checks that the route associated to the topic is correct, in terms of
         * the middlewares it connects being present in the `systems` section.
         *
         * The type will be added to the RequiredTypes map only if no remapping
         * attributes are being set for this middleware.
         */
        const std::set<std::string>& middlewares = topic_config.route.all();

        for (const std::string& mw : middlewares)
        {
            if (_m_middlewares.find(mw) == _m_middlewares.end())
            {
                logger << utils::Logger::Level::ERROR
                       << "Unrecognized system '" << mw << "' requested for topic '"
                       << topic_name << "'." << std::endl;
                return false;
            }

            auto it_mw_remap = topic_config.remap.find(mw);
            if (it_mw_remap == topic_config.remap.end() || it_mw_remap->second.type.empty())
            {
                _m_required_types[mw].messages.insert(topic_config.message_type);
            }
        }

        /**
         * Also, checks that the remapping attributes are correctly defined, that is,
         * remapping can only be done to one of the systems specified in the route.
         */
        for (auto&& [mw_name, topic_info] : topic_config.remap)
        {
            if (_m_middlewares.find(mw_name) == _m_middlewares.end())
            {
                logger << utils::Logger::Level::ERROR
                       << "Unrecognized system '" << mw_name
                       << "' requested for remapping topic "
                       << "'" << topic_name << "'" << std::endl;
                return false;
            }

            if (!topic_info.type.empty())
            {
                _m_required_types[mw_name].messages.insert(topic_info.type);
            }
        }
    }

    /**
     * Check services configuration.
     */
    for (const auto& [service_name, service_config] : _m_service_configs)
    {
        /**
         * Checks that the route associated to the service is correct, in terms of
         * the middlewares it connects being present in the `systems` section.
         *
         * The type will be added to the RequiredTypes map only if no remapping
         * attributes are being set for this middleware.
         */
        const std::set<std::string>& middlewares = service_config.route.all();
        for (const std::string& mw : middlewares)
        {
            if (_m_middlewares.find(mw) == _m_middlewares.end())
            {
                logger << utils::Logger::Level::ERROR
                       << "Unrecognized system '" << mw << "' requested for service "
                       << "'" << service_name << "'" << std::endl;
                return false;
            }

            auto it_mw_remap = service_config.remap.find(mw);
            if ((it_mw_remap == service_config.remap.end() || it_mw_remap->second.type == "") &&
                    service_config.request_type != "")
            {
                _m_required_types[mw].services.insert(service_config.request_type);
            }
            if ((it_mw_remap == service_config.remap.end() || it_mw_remap->second.reply_type == "") &&
                    service_config.reply_type != "")
            {
                _m_required_types[mw].services.insert(service_config.reply_type);
            }
        }

        /**
         * Also, checks that the remapping attributes are correctly defined, that is,
         * remapping can only be done to one of the systems specified in the route.
         */
        for (auto&& [mw_name, service_info] : service_config.remap)
        {
            if (_m_middlewares.find(mw_name) == _m_middlewares.end())
            {
                logger << utils::Logger::Level::ERROR
                       << "Unrecognized system '" << mw_name << "' requested for remapping service "
                       << "'" << service_name << "'" << std::endl;
                return false;
            }
            _m_required_types[mw_name].services.insert(service_info.type);
            if (!service_info.reply_type.empty())
            {
                _m_required_types[mw_name].services.insert(service_info.reply_type);
            }
        }
    }

    /**
     * Checks for defined but unused middlewares. Also, checks that at least two are being used.
     */
    std::size_t active_mw_count = 0;
    for (const auto& [mw_name, mw_config] : _m_middlewares)
    {
        const auto it = _m_required_types.find(mw_name);
        if (it == _m_required_types.end())
        {
            logger << utils::Logger::Level::WARN
                   << "The middleware '" << mw_name << "' of type '"
                   << mw_config.type << "' is not being used in any topics "
                   << "or services" << std::endl;
        }
        else
        {
            ++active_mw_count;
        }
    }

    if (active_mw_count < 2)
    {
        logger << utils::Logger::Level::ERROR
               << "Fewer than 2 middlewares (" << active_mw_count << ") are "
               << "being used in this current configuration! This means that "
               << "running Integration Service will not be useful." << std::endl;
        return false;
    }

    _okay = true;
    return true;
}

//==============================================================================
bool Config::okay() const
{
    return _okay;
}

//==============================================================================
Config::operator bool() const
{
    return _okay;
}

//==============================================================================
bool Config::load_middlewares(
        is::internal::SystemHandleInfoMap& info_map) const
{
    /**
     * Sorts middlewares according to their dependencies in the `types-from` department.
     * Middlewares specified in a `types-from` tag must be configured first.
     */
    using Entry = std::map<std::string, MiddlewareConfig>::value_type;

    std::list<Entry> middlewares(_m_middlewares.begin(), _m_middlewares.end());

    middlewares.sort(
        [](const Entry& a, const Entry& b) -> bool
        {
            auto& from = b.second.types_from;
            return std::find(from.begin(), from.end(), a.first) != from.end();
        });

    /**
     * Iterates through the sorted middlewares list, to load them in the appropriate order.
     */
    for (const auto& [mw_name, mw_config] : middlewares)
    {
        const auto& ref_mw_name = mw_name;
        const std::string& middleware_type = mw_config.type;

        logger << utils::Logger::Level::DEBUG
               << "Config::load_middlewares: looking for middleware '" << mw_name
               << "' with type '" << middleware_type << "'" << std::endl;

        const Search search(mw_config.type);

        /**
         * Looks for the middleware's SystemHandle dynamic library.
         */
        std::vector<std::string> checked_paths;
        const std::string path = search.find_middleware_mix(&checked_paths);

        if (path.empty())
        {
            logger << utils::Logger::Level::ERROR
                   << "Unable to find .mix file for middleware '" << middleware_type << "'. "
                   << "The following locations were checked unsuccessfully: \n";

            for (const std::string& checked_path : checked_paths)
            {
                logger << "\n\t- " << checked_path;
            }

            logger << "\nTry adding your middleware's install path to IS_PREFIX_PATH "
                   << "or IS_" << Search::to_env_format(middleware_type) << "_PREFIX_PATH "
                   << "environment variables." << std::endl;

            return false;
        }

        if (!Mix::from_file(path).load())
        {
            logger << utils::Logger::Level::ERROR
                   << "Unable to load the dynamic libraries present in the .mix file '"
                   << path << "'." << std::endl;

            return false;
        }

        /**
         * After loading the mix file, the middleware's SystemHandle library should be
         * loaded, and it should be possible to find the middleware info in the
         * internal Register.
         */
        is::internal::SystemHandleInfo info = is::internal::Register::get(middleware_type);

        if (!info || !info.handle->preprocess_types(mw_config.types_node))
        {
            return false;
        }

        /**
         * Now, it iterates the middleware required types map.
         * For each middleware, it checks which types it needs, and places them into
         * the SystemHandleInfo structure.
         */
        const auto requirements = _m_required_types.find(mw_name);

        if (requirements != _m_required_types.end())
        {
            /**
             * Adds topics message types into the type registry, avoiding to insert duplicates.
             */
            for (const std::string& required_type : requirements->second.messages)
            {
                auto type_it = _m_types.find(required_type);

                if (type_it != _m_types.end())
                {
                    info.types.emplace(*type_it);
                }
            }

            /**
             * Adds service types into the type registry, avoiding to insert duplicates.
             */
            for (const std::string& required_type : requirements->second.services)
            {
                auto type_it = _m_types.find(required_type);

                if (type_it != _m_types.end())
                {
                    info.types.emplace(*type_it);
                }
            }

            /**
             * Checks here the `types-from` attribute for this middleware.
             * If it exists, it will contain a list of the middlewares it wants to
             * import the types from.
             *
             * Check that this middleware already exists in the
             * is::internal::SystemHandleInfoMap, and iterate over its types to copy them into the
             * target middleware, that is, `mw_name`.
             */
            if (!mw_config.types_from.empty())
            {
                for (const std::string& mw_from : mw_config.types_from)
                {
                    const auto it = info_map.find(mw_from);
                    if (it == info_map.end())
                    {
                        logger << utils::Logger::Level::ERROR
                               << "'types-from' references to a non-existent middleware: '"
                               << mw_from << "'. Maybe it has not been registered yet?"
                               << std::endl;

                        return false;
                    }

                    for (auto&& it_type : info_map.at(mw_from).types)
                    {
                        // Preserve the source middleware's registry key (the
                        // canonical 'package/msg/Type' name used by the config and
                        // RequiredTypes). The Fast-DDS DynamicType's get_name() is
                        // the IDL-qualified form ('package::msg::Type'), which no
                        // longer matches those keys, so also index by it for
                        // lookups that go through the type name.
                        info.types.emplace(it_type.first, it_type.second);
                        info.types.emplace(std::string(it_type.second->get_name()), it_type.second);
                    }
                }

                /**
                 * Now that we have added types from the `types-from` tag, the
                 * SystemHandleInfo struct for this middleware (mw_name) should be complete.
                 *
                 * Therefore, iterating through its required_types and checking that every
                 * type exists in the SystemHandleInfo::TypeRegistry should be ok. Otherwise,
                 * it warns and returns false.
                 */
                for (const std::string& required_type : requirements->second.messages)
                {
                    if (!info.types.count(required_type))
                    {
                        logger << utils::Logger::Level::ERROR
                               << "The middleware '" << mw_name
                               << "' must satisfy the required topic type '"
                               << required_type << "', but it does not seem to be "
                               << "available neither in its type registry or inherited "
                               << "from its 'types-from' reference middlewares" << std::endl;

                        return false;
                    }
                }

                for (const std::string& required_type : requirements->second.services)
                {
                    if (!info.types.count(required_type))
                    {
                        logger << utils::Logger::Level::ERROR
                               << "The middleware '" << mw_name
                               << "' must satisfy the required service type '"
                               << required_type << "', but it does not seem to be "
                               << "available neither in its type registry or inherited "
                               << "from its 'types-from' reference middlewares" << std::endl;

                        return false;
                    }
                }
            }
        }
        else
        {
            // Check if another middleware relies on types builtin or dynamically loaded by the middleware but not
            // explicit in the configuration file
            if ( middlewares.end() ==
                    std::find_if(middlewares.begin(), middlewares.end(), [&ref_mw_name](const Entry& mw)
                    {
                        auto& from = mw.second.types_from;
                        return ref_mw_name != mw.first && std::find(from.begin(), from.end(), ref_mw_name) != from.end();
                    }))
            {
                logger << utils::Logger::Level::ERROR
                       << "The middleware '" << mw_name
                       << "' has no types associated" << std::endl;
                return false;
            }
        }

        /**
         * Finally, now that the SystemHandleInfo struct is filled with all its types, it
         * calls to the SystemHandle::configure override function for the selected middleware.
         */
        if ( info.handle->configure(
                    requirements->second, mw_config.config_node, info.types))
        {
            // If the middleware was correctly configured, it inserts it within the info_map.
            info_map.insert(std::make_pair(mw_name, std::move(info)));
        }
        else
        {
            logger << utils::Logger::Level::ERROR
                   << "The middleware '" << mw_name
                   << "' configuration step failed" << std::endl;
            return false;
        }
    }

    return true;
}

//==============================================================================
bool Config::configure_topics(
        const is::internal::SystemHandleInfoMap& info_map,
        SubscriptionCallbacks& subscription_callbacks) const
{
    bool valid = true;

    /**
     * Iterates through the topics section of the provided configuration.
     */
    for (const auto& [topic_name, topic_config] : _m_topic_configs)
    {
        /**
         * First, it checks topic compatibility in terms of the registered types
         * in the source and destination endpoints.
         */
        if (!check_topic_compatibility(info_map, topic_name, topic_config))
        {
            valid = false;
            continue;
        }

        /**
         * Helper struct to store an Integration Service publisher
         * and its published DynamicType.
         */
        struct PublisherData
        {
            PublisherData(
                    std::shared_ptr<TopicPublisher> m_publisher,
                    const eprosima::xtypes::DynamicType& m_type)
                : publisher(m_publisher)
                , type(m_type)
            {
            }

            std::shared_ptr<TopicPublisher> publisher;
            const eprosima::xtypes::DynamicType& type;
        };

        std::vector<PublisherData> publishers;
        publishers.reserve(topic_config.route.to.size());

        for (const std::string& to : topic_config.route.to)
        {
            /**
             * The `to` endpoint within the route tells the related system that
             * its SystemHandle must produce a publisher, so that the final application
             * can subscribe to it and receive the information as described in the route
             * data flow. Therefore, this middleware must have publishing capabilities,
             * that is, its SystemHandleInfo::TopicPublisherSystem pointer must not be NULL.
             */
            const auto it_to = info_map.find(to);
            if (it_to == info_map.end() || !it_to->second.topic_publisher)
            {
                logger << utils::Logger::Level::ERROR
                       << "Could not find topic publishing capabilities for system "
                       << "named '" << to << "', requested for topic '"
                       << topic_name << "'." << std::endl;

                valid = false;
                continue;
            }

            /**
             * Does remapping and type resolution, if applicable.
             */
            TopicInfo topic_info = remap_if_needed(
                to, topic_config.remap, TopicInfo(topic_name, topic_config.message_type));

            xtypes::DynamicType pub_type = resolve_type(
                it_to->second.types, topic_info.type);

            /**
             * Advertises the TopicPublisher using the TopicPublisherSystem provided
             * by the "to" middleware's SystemHandle.
             */
            std::shared_ptr<TopicPublisher> publisher =
                    it_to->second.topic_publisher->advertise(topic_info.name,
                            (topic_info.type.find(".") == std::string::npos
                            ? pub_type
                            : _m_types.at(topic_info.type.substr(0, topic_info.type.find(".")))),
                            config_or_empty_node(to, topic_config.middleware_configs));

            if (!publisher)
            {
                logger << utils::Logger::Level::ERROR
                       << "The system '" << to << "' failed to produce a publisher "
                       << "for the topic '" << topic_name << "' and message type '"
                       << topic_config.message_type << "'." << std::endl;

                valid = false;
            }
            else
            {
                logger << utils::Logger::Level::INFO
                       << "[" << to << " SystemHandle] Produced a publisher "
                       << "for the topic '" << topic_name << "', with message type '"
                       << topic_config.message_type << "'." << std::endl;

                publishers.emplace_back(PublisherData(publisher, pub_type));
            }
        }

        /**
         * For each `from` attribute in the route, the corresponding SystemHandle
         * must produce a subscriber that fetches the data from the user's source
         * application and convert it to the common language representation, that is,
         * `eprosima::xtypes::DynamicData`.
         * Then, this subscriber callback will take care of publishing the data
         * in each one of the TopicPublishers defined in the `to` middleware list.
         */
        for (const std::string& from : topic_config.route.from)
        {
            /**
             * First, it checks the subscribing capabilities of the middleware's SystemHandle.
             */
            const auto it_from = info_map.find(from);
            if (it_from == info_map.end() || !it_from->second.topic_subscriber)
            {
                logger << utils::Logger::Level::ERROR
                       << "Could not find topic subscribing capabilities for system "
                       << "named '" << from << "', requested for topic '"
                       << topic_name << "'." << std::endl;
                valid = false;
                continue;
            }

            /**
             * Does remapping and type resolution, if applicable.
             */
            TopicInfo topic_info = remap_if_needed(
                from, topic_config.remap, TopicInfo(topic_name, topic_config.message_type));

            xtypes::DynamicType sub_type = resolve_type(
                it_from->second.types, topic_info.type);

            /**
             * Helper struct to store an Integration Service publisher
             * and its published DynamicType.
             */
            struct Publication
            {
                Publication(
                        const PublisherData& publisher_data,
                        const eprosima::xtypes::DynamicType& /*sub_type*/)
                    : publisher(publisher_data.publisher)
                    , type(publisher_data.type)
                {
                }

                std::shared_ptr<TopicPublisher> publisher;
                eprosima::xtypes::DynamicType type;
            };

            std::vector<Publication> publications;
            publications.reserve(publishers.size());

            for (const auto& pub : publishers)
            {
                publications.emplace_back(Publication(pub, sub_type));
            }

            /**
             * Defines the Integration Service SubscriptionCallback lambda that will
             * iterate over all the publishers created from the `to` field and
             * publish the data received through this subscriber over them.
             * This is the core of the `from/to` route communication process.
             */

            std::unique_ptr<TopicSubscriberSystem::SubscriptionCallback> unique_callback = nullptr;
            TopicSubscriberSystem* topic_subscriber_system = it_from->second.topic_subscriber;

            unique_callback.reset(new TopicSubscriberSystem::SubscriptionCallback(
                        [=](const eprosima::xtypes::DynamicData& message,
                        void* filter_handle)
                        {
                            if (topic_subscriber_system->is_internal_message(filter_handle))
                            {
                                return;
                            }

                            for (const Publication& publication : publications)
                            {
                                publication.publisher->publish(message);
                            }
                        }));

            bool subscribed = it_from->second.topic_subscriber->subscribe(
                topic_info.name,
                (topic_info.type.find(".") == std::string::npos
                ? sub_type
                : _m_types.at(topic_info.type.substr(0, topic_info.type.find(".")))),
                unique_callback.get(),
                config_or_empty_node(from, topic_config.middleware_configs));

            subscription_callbacks.emplace_back(std::move(unique_callback));

            if (subscribed)
            {
                logger << utils::Logger::Level::INFO
                       << "[" << from << " SystemHandle] Subscribed "
                       << "to topic '" << topic_name << "', with message type '"
                       << topic_config.message_type << "'." << std::endl;
            }
            else
            {
                logger << utils::Logger::Level::ERROR
                       << "[" << from << " SystemHandle] Failed to subscribe "
                       << "to topic '" << topic_name << "', with message type '"
                       << topic_config.message_type << "'." << std::endl;
            }

            valid &= subscribed;
        }
    }

    return valid;
}

//==============================================================================
bool Config::configure_services(
        const is::internal::SystemHandleInfoMap& info_map,
        RequestCallbacks& request_callbacks) const
{
    bool valid = true;

    /**
     * Iterates through the services section of the provided configuration.
     */
    for (const auto& [service_name, service_config] : _m_service_configs)
    {
        /**
         * First, it checks service compatibility in terms of the registered types
         * in the source and destination endpoints, both for request and reply types.
         */
        if (!check_service_compatibility(info_map, service_name, service_config))
        {
            valid = false;
            continue;
        }

        const std::string& server = service_config.route.server;
        const auto it_server = info_map.find(server);

        if (it_server == info_map.end() || !it_server->second.service_provider)
        {
            logger << utils::Logger::Level::ERROR
                   << "Could not find service providing capabilities for system "
                   << "named '" << server << "', requested for service '"
                   << service_name << "'." << std::endl;

            valid = false;
            continue;
        }

        /**
         * Does remapping and type resolution, if applicable.
         */
        ServiceInfo server_info = remap_if_needed(
            server, service_config.remap,
            ServiceInfo(service_name, service_config.request_type, service_config.reply_type));

        xtypes::DynamicType server_type = resolve_type(
            it_server->second.types, server_info.type);

        /**
         * Creates the ServiceProvider instance, differenciating the case of the service having a reply type, or not.
         */
        std::shared_ptr<ServiceProvider> provider = nullptr;

        if (!service_config.reply_type.empty())
        {
            xtypes::DynamicType server_reply_type = resolve_type(
                it_server->second.types, server_info.reply_type);

            provider =
                    it_server->second.service_provider->create_service_proxy(
                server_info.name,
                (server_info.type.find(".") == std::string::npos
                ? server_type
                : _m_types.at(server_info.type.substr(0, server_info.type.find(".")))),
                (server_info.reply_type.find(".") == std::string::npos
                ? server_reply_type
                : _m_types.at(server_info.reply_type.substr(0, server_info.reply_type.find(".")))),
                config_or_empty_node(server, service_config.middleware_configs));
        }
        else
        {
            logger << utils::Logger::Level::DEBUG
                   << "[" << server << " SystemHandle] The requested service server for the service '"
                   << service_name << "' does not have a reply type" << std::endl;

            provider =
                    it_server->second.service_provider->create_service_proxy(
                server_info.name,
                (server_info.type.find(".") == std::string::npos
                ? server_type
                : _m_types.at(server_info.type.substr(0, server_info.type.find(".")))),
                config_or_empty_node(server, service_config.middleware_configs));
        }

        if (!provider)
        {
            logger << utils::Logger::Level::ERROR
                   << "The system '" << server << "' failed to create a service provider "
                   << "for the service '" << service_name << "', with request type '"
                   << service_config.request_type << "'";

            if (!service_config.reply_type.empty())
            {
                logger << " and reply type '" << service_config.reply_type << "'";
            }
            logger << "." << std::endl;

            return false;
        }
        else
        {
            logger << utils::Logger::Level::INFO
                   << " [" << server << " SystemHandle] Produced a service provider "
                   << "for the service '" << service_name << "', with request type '"
                   << service_config.request_type << "'";

            if (!service_config.reply_type.empty())
            {
                logger << " and reply type '" << service_config.reply_type << "'";
            }
            logger << "." << std::endl;
        }

        /**
         * Defines the Integration Service RequestCallback lambda that will
         * be called each time the user client application makes a request.
         *
         * The specific middleware's SystemHandle implementation of the ServiceClient proxy
         * should internally create a "server" of the specific middleware, to receive the
         * request from the user application. This service uses the RequestCallback lambda to
         * use the ServiceProvider created before, each time a request comes from the user (which
         * defines a destination middleware client internally) to actually call the service
         * on the user server application.
         *
         * The `call service` method signature passes as input argument a reference to the
         * ServiceClient that made the request, which will call `receive_response` to send
         * the response back to the user's client application.
         */
        for (const std::string& client : service_config.route.clients)
        {
            /**
             * First, it checks the middleware's SystemHandle capabilities for creating service clients.
             */
            const auto it_client = info_map.find(client);
            if (it_client == info_map.end() || !it_client->second.service_client)
            {
                logger << utils::Logger::Level::ERROR
                       << "Could not find service client capabilities for system "
                       << "named '" << client << "', requested for service '"
                       << service_name << "'." << std::endl;

                valid = false;
                continue;
            }

            /**
             * Does remapping and type resolution, if applicable.
             */
            ServiceInfo client_info = remap_if_needed(
                client, service_config.remap,
                ServiceInfo(service_name, service_config.request_type, service_config.reply_type));

            xtypes::DynamicType client_type = resolve_type(
                it_client->second.types, client_info.type);

            /**
             * Defines the RequestCallback that will perform the corresponding call to the service.
             */
            std::unique_ptr<ServiceClientSystem::RequestCallback> unique_callback = nullptr;
            unique_callback.reset(new ServiceClientSystem::RequestCallback(
                        [=](
                            const eprosima::xtypes::DynamicData& request,
                            ServiceClient& service_client,
                            const std::shared_ptr<void>& call_handle)
                        {
                            provider->call_service(request, service_client, call_handle);
                        }));

            /**
             * Finally, creates the service client proxy, differentiating between the cases of
             * having a request_type + a reply_type, or only an unique type defined for the service.
             */
            bool created_client_proxy;

            if (client_info.reply_type.empty())
            {
                logger << utils::Logger::Level::DEBUG
                       << "[" << client << " SystemHandle] The requested service client for the service '"
                       << service_name << "' does not have a reply type" << std::endl;

                created_client_proxy = it_client->second.service_client->create_client_proxy(
                    client_info.name,
                    (client_info.type.find(".") == std::string::npos
                    ? client_type
                    : _m_types.at(client_info.type.substr(0, client_info.type.find(".")))),
                    unique_callback.get(),
                    config_or_empty_node(client, service_config.middleware_configs));
            }
            else
            {
                xtypes::DynamicType client_reply_type = resolve_type(
                    it_client->second.types, client_info.reply_type);

                created_client_proxy = it_client->second.service_client->create_client_proxy(
                    client_info.name,
                    (client_info.type.find(".") == std::string::npos
                    ? client_type
                    : _m_types.at(client_info.type.substr(0, client_info.type.find(".")))),
                    (client_info.reply_type.find(".") == std::string::npos
                    ? client_reply_type
                    : _m_types.at(client_info.reply_type.substr(0, client_info.reply_type.find(".")))),
                    unique_callback.get(),
                    config_or_empty_node(client, service_config.middleware_configs));
            }

            request_callbacks.emplace_back(std::move(unique_callback));

            if (created_client_proxy)
            {
                logger << utils::Logger::Level::INFO
                       << "[" << client << " SystemHandle] Produced a service client "
                       << "for the service '" << service_name << "', with request type '"
                       << service_config.request_type << "'";

                if (!service_config.reply_type.empty())
                {
                    logger << " and reply type '" << service_config.reply_type << "'";
                }
                logger << "." << std::endl;
            }
            else
            {
                logger << utils::Logger::Level::ERROR
                       << "The system '" << client << "' failed to create a service client "
                       << "for the service '" << service_name << "', with request type '"
                       << service_config.request_type << "'";

                if (!service_config.reply_type.empty())
                {
                    logger << " and reply type '" << service_config.reply_type << "'";
                }
                logger << "." << std::endl;
            }

            valid &= created_client_proxy;
        }
    }

    return valid;
}

//==============================================================================
bool Config::check_topic_compatibility(
        const is::internal::SystemHandleInfoMap& info_map,
        const std::string& topic_name,
        const TopicConfig& config) const
{
    bool valid = true;

    for (const std::string& from : config.route.from)
    {
        const auto it_from = info_map.find(from);

        TopicInfo topic_info_from = remap_if_needed(from, config.remap, TopicInfo(topic_name, config.message_type));
        xtypes::DynamicType from_type = resolve_type(it_from->second.types, topic_info_from.type);
        (void)from_type;

        for (const std::string& to : config.route.to)
        {
            const auto it_to = info_map.find(to);
            (void)it_to;

            TopicInfo topic_info_to = remap_if_needed(to, config.remap, TopicInfo(topic_name, config.message_type));
            (void)topic_info_to;

            // Type compatibility checking is not available in the Fast-DDS XTypes API.
            // Compatibility is enforced at the DDS layer by the underlying middleware.
        }
    }

    return valid;
}

//==============================================================================
bool Config::check_service_compatibility(
        const is::internal::SystemHandleInfoMap& info_map,
        const std::string& service_name,
        const ServiceConfig& config) const
{
    bool valid = true;

    for (const std::string& client : config.route.clients)
    {
        const auto it_client = info_map.find(client);

        ServiceInfo client_info = remap_if_needed(client, config.remap,
                        ServiceInfo(service_name, config.request_type, config.reply_type));
        xtypes::DynamicType client_type =
                resolve_type(it_client->second.types, client_info.type);
        (void)client_type;

        const auto it_server = info_map.find(config.route.server);

        ServiceInfo server_info = remap_if_needed(config.route.server, config.remap,
                        ServiceInfo(service_name, config.request_type, config.reply_type));
        xtypes::DynamicType server_type =
                resolve_type(it_server->second.types, server_info.type);
        (void)server_type;

        // Type compatibility checking is not available in the Fast-DDS XTypes API.
        // Compatibility is enforced at the DDS layer by the underlying middleware.
    }

    return valid;
}

xtypes::DynamicType Config::resolve_type(
        const TypeRegistry& types,
        const std::string& path) const
{
    if (path.find(".") == std::string::npos)
    {
        return types.find(path)->second;
    }

    std::string path_aux = path;
    std::string type_name = path_aux.substr(0, path_aux.find("."));
    xtypes::DynamicType current = _m_types.at(type_name);

    while (path_aux.find(".") != std::string::npos)
    {
        path_aux = path_aux.substr(path_aux.find(".") + 1);
        std::string member = path_aux.substr(0, path_aux.find("."));

        fastdds::dds::DynamicTypeMember::_ref_type member_obj;
        if (current->get_member_by_name(member_obj, member) != fastdds::dds::RETCODE_OK)
        {
            break;
        }

        fastdds::dds::MemberDescriptor::_ref_type desc =
            fastdds::dds::traits<fastdds::dds::MemberDescriptor>::make_shared();
        member_obj->get_descriptor(desc);
        current = desc->type();
    }

    return current;
}

} //  namespace internal
} //  namespace core
} //  namespace is
} //  namespace eprosima
