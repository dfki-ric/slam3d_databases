#pragma once

#include <string>
// dont define the U macro in the http client, it conflicts with the eigen U macro use _XPLATSTR() instead
// #define _TURN_OFF_PLATFORM_STRING
// #include <cpprest/json.h>
#include <slam3d/core/Types.hpp>

#include <neo4j-client.h>

#include "Neo4jValue.hpp"
#include "Neo4jParamaterSet.hpp"

namespace slam3d {

// class MeasurementStorage;

class Neo4jConversion {
 public:
    static std::string eigenMatrixToString(const Eigen::MatrixXd& mat);
    static Eigen::MatrixXd eigenMatrixFromString(const std::string & string);

    // static void constraintToJson(Constraint::Ptr constraint, web::json::value* json);
    // static Constraint::Ptr jsonToConstraint(web::json::value& json);

    // static EdgeObject edgeObjectFromJson(web::json::value& json);
    // static VertexObject vertexObjectFromJson(web::json::value& json);

    // bolt, libneo4j conversions


    // static VertexObjectList vertexObjectList(neo4j_result_stream_t *results);
    static VertexObject vertexObject(const neo4j_result_t *result);
    static MetaData vertexMeasurementData(const neo4j_result_t *result);



    // static EdgeObjectList edgeObjectList(neo4j_result_stream_t *results);
    static EdgeObject edgeObject(const neo4j_result_t *result);

    static Constraint::Ptr constraint(const neo4j_result_t *result);
    static bool constraint(const Constraint::Ptr, neo4j_result_t *result);

    static void constraintToParameters(Constraint::Ptr constraint, const std::string& setname, ParamaterSet* set);

    static ParamaterSet createParamaterSet(const MetaData& v);

    static ParamaterSet createParamaterSet(const VertexObject& v);

 private:
    static void parseVertexMeasurementData(MetaData* vmd, Neo4jValue& properties);


};

}  // namespace slam3d

