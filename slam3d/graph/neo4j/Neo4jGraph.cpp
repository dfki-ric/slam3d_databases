// workaround for:
//https://svn.boost.org/trac/boost/ticket/10382
// #define BOOST_NO_CXX11_DEFAULTED_FUNCTIONS


#include "Neo4jGraph.hpp"

// #include <boost/format.hpp>
#include <slam3d/core/Solver.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

#include <fstream>

#include "Neo4jConnection.hpp"
#include "Neo4jParamaterSet.hpp"
#include "Neo4jValue.hpp"

using namespace slam3d;

Neo4jGraph::Neo4jGraph(Logger* log, MeasurementStorage* storage, const Neo4jConnection::ServerConfig &graphserver) : Graph(log, storage)
{
    neo4j = std::make_shared<Neo4jConnection>(graphserver);

    VertexObjectList existingVertices = getVertices();
    IdType maxindex = 0;
    for (const auto& vertex : existingVertices) {
        if (vertex.index > maxindex) {
            maxindex = vertex.index;
        }
    }
    init(maxindex+1);
}

Neo4jGraph::~Neo4jGraph()
{
}

void Neo4jGraph::init(const size_t &indexer_start)
{
	// call parent init
	Graph::init(indexer_start);

	if (indexer_start == 0) {
		// insert a dummy node as a source of unary edges
		VertexObject vo;
		vo.index = mIndexer.getNext();
		vo.fixed = true;
		vo.correctedPose = Transform::Identity();
		vo.measurementUuid = boost::uuids::nil_uuid();
		vo.label = "origin";
		vo.typeName = "void";
		addVertex(vo);
	}
}

void Neo4jGraph::clear()
{
    // printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
    std::string request = "match (n) detach delete n";
    neo4j->runQuery(request, [&](neo4j_result_t *element){});
}

const EdgeObjectList Neo4jGraph::getEdges(const StringSet& sensors)  const
{
    std::string request = "MATCH (a:Vertex)-[r]->(b:Vertex) WHERE r.inverted=false AND r.source <> r.target";

    if(sensors.size())
    {
      request += " AND r.sensor IN [";
      for(const std::string& sensor : sensors)
      {
        request += "\"" + sensor + "\",";
      }
      request.back() = ']';
    }
    request += " RETURN r";

    EdgeObjectList objectList;
    neo4j->runQuery(request, [&](neo4j_result_t *element){
        objectList.push_back(Neo4jConversion::edgeObject(element));
    });
	return objectList;
}

void Neo4jGraph::addVertex(const VertexObject& v) {
    std::string request = "CREATE (n:Vertex $props)";

    ParamaterSet params = Neo4jConversion::createParamaterSet(v);

    neo4j->runQuery(request, [&](neo4j_result_t *element){}, params.get());

    // printf("%s:%i %li\n", __PRETTY_FUNCTION__, __LINE__, v.subMeasurements.size());

    // VertexObject has additional measurements
    if (v.subMeasurements.size()) {
        for (const auto &measurement : v.subMeasurements) {
            request = "CREATE (n:VertexMeasurement $props)";
            ParamaterSet subparams = Neo4jConversion::createParamaterSet(measurement);
            neo4j->runQuery(request, [&](neo4j_result_t *element){}, subparams.get());
        }
    
        // add a reference edge to find submeasurements via edge and not index
        request = "MATCH (a:Vertex), (b:VertexMeasurement) WHERE a.index="+std::to_string(v.index)+" AND b.index="+std::to_string(v.index) \
        + " CREATE (a)-[r:subMeasurement]->(b)";

        neo4j->runQuery(request, [&](neo4j_result_t *element){});

    }
    // add position as extra statement (point property cannot be directly set via json props, there it is added as string)
    request = "MATCH (n:Vertex) WHERE n.index="+std::to_string(v.index)+" SET n.location = point({"
                             +   "x: " + std::to_string(v.correctedPose.translation().x())
                             + ", y: " + std::to_string(v.correctedPose.translation().y())
                             + ", z: " + std::to_string(v.correctedPose.translation().z())
                             + "})";

    std::cout << request << std::endl;

    neo4j->runQuery(request, [&](neo4j_result_t *element){});
}

void Neo4jGraph::addEdge(const EdgeObject& e) {
    addEdge(e, true);
}

void Neo4jGraph::addEdge(const EdgeObject& e, bool addInverse) {
    std::string constrainttypename = e.constraint->getTypeName();
    std::replace(constrainttypename.begin(), constrainttypename.end(), '(', '_');
    std::replace(constrainttypename.begin(), constrainttypename.end(), ')', '_');

    std::string request = "MATCH (a:Vertex), (b:Vertex) WHERE a.index="+std::to_string(e.source)+" AND b.index="+std::to_string(e.target) \
        + " CREATE (a)-[r:" + constrainttypename + " $props]->(b) RETURN type(r)";

    ParamaterSet params;
    params.addParameterSet("props");
    params.addParameterToSet("props", "label", e.label);
    params.addParameterToSet("props", "source", e.source);
    params.addParameterToSet("props", "target", e.target);
    params.addParameterToSet("props", "sensor", e.constraint->getSensorName());
    params.addParameterToSet("props", "timestamp_tv_sec", e.constraint->getTimestamp().tv_sec);
    params.addParameterToSet("props", "timestamp_tv_usec", e.constraint->getTimestamp().tv_usec);
    params.addBoolParameterToSet("props", "inverted", false);
    params.addParameterToSet("props", "constraint_type", e.constraint->getType());
    Neo4jConversion::constraintToParameters(e.constraint, "props", &params);

    neo4j->runQuery(request, [&](neo4j_result_t *element){}, params.get());


    if (addInverse) {
        request = "MATCH (a:Vertex), (b:Vertex) WHERE a.index="+std::to_string(e.target)+" AND b.index="+std::to_string(e.source) \
                + " CREATE (a)-[r:" + constrainttypename + " $props]->(b) RETURN type(r)";

        ParamaterSet inverse_params;
        inverse_params.addParameterSet("props");
        inverse_params.addParameterToSet("props", "label", e.label);
        inverse_params.addParameterToSet("props", "source", e.target);
        inverse_params.addParameterToSet("props", "target", e.source);
        inverse_params.addParameterToSet("props", "sensor", e.constraint->getSensorName());
        inverse_params.addParameterToSet("props", "timestamp_tv_sec", e.constraint->getTimestamp().tv_sec);
        inverse_params.addParameterToSet("props", "timestamp_tv_usec", e.constraint->getTimestamp().tv_usec);
        inverse_params.addBoolParameterToSet("props", "inverted", true);
        inverse_params.addParameterToSet("props", "constraint_type", e.constraint->getType());
        Neo4jConversion::constraintToParameters(e.constraint, "props", &inverse_params);
        
        neo4j->runQuery(request, [&](neo4j_result_t *element){}, inverse_params.get());
    }
}

void Neo4jGraph::removeEdge(IdType source, IdType target, const std::string& sensor) {
    std::string request = "MATCH (a:Vertex)-[r]-(b:Vertex) WHERE a.index="+std::to_string(source)+" AND b.index="+std::to_string(target)+" AND r.sensor=\""+sensor+"\" DELETE r";
    slam3d::VertexObjectList vertexobjlist;
    neo4j->runQuery(request, [&](neo4j_result_t *element){});
}


const StringSet Neo4jGraph::getVertexSensors() const {
    std::string request = "MATCH (a:Vertex) RETURN DISTINCT a.sensorName";
    StringSet result;
    neo4j->runQuery(request, [&](neo4j_result_t *element){
        Neo4jValue val(element);
        result.insert(val.as_string());
    });
    return result;
}

const StringSet Neo4jGraph::getEdgeSensors() const {
    std::string request = "MATCH ()-[r]->() RETURN DISTINCT r.sensor";
    StringSet result;
    neo4j->runQuery(request, [&](neo4j_result_t *element){
        Neo4jValue val(element);
        result.insert(val.as_string());
    });
    return result;
}


const VertexObjectList Neo4jGraph::getVertices(const StringSet& sensors) const {
    std::string request = "MATCH (a:Vertex)";
    if(sensors.size())
    {
      request += " WHERE a.sensorName IN [";
      for(const std::string& sensor : sensors)
      {
        request += "\"" + sensor + "\",";
      }
      request.back() = ']';
    }
    
    request += " RETURN a ORDER BY a.index";
    slam3d::VertexObjectList vertexobjlist;
    neo4j->runQuery(request, [&](neo4j_result_t *element){
        VertexObject vo = Neo4jConversion::vertexObject(element);
        vertexobjlist.push_back(vo);
    });
    addSubMeasurementsToVertexObjectList(&vertexobjlist);
    return vertexobjlist;
}

const VertexObjectList Neo4jGraph::getVerticesByType(const std::string& type) const {
    std::string request = "MATCH (a:Vertex) WHERE a.typeName='"+type+"' RETURN a ORDER BY a.index";
    slam3d::VertexObjectList vertexobjlist;
    neo4j->runQuery(request, [&](neo4j_result_t *element){
        vertexobjlist.push_back(Neo4jConversion::vertexObject(element));
    });
    addSubMeasurementsToVertexObjectList(&vertexobjlist);
    return vertexobjlist;
}

const VertexObjectList Neo4jGraph::getNearbyVertices(const Transform &location, float radius, const StringSet& sensors) const {
    std::string request;

    if (sensors.empty()) {
        request = "MATCH (a:Vertex) WHERE point.distance(point({x:"+std::to_string(location.translation().x())+", y:"+std::to_string(location.translation().y())+", z:"+std::to_string(location.translation().z())+"}), a.location) < "+std::to_string(radius)+" RETURN a ORDER BY a.index";
    } else {
        // construct set query
        std::string sensorsstr = "[";
        for (const auto& sensor: sensors){
            sensorsstr += "\"" + sensor + "\",";
        }
        sensorsstr.back() = ']';
        request = "MATCH (a:Vertex) WHERE point.distance(point({x:"+std::to_string(location.translation().x())+", y:"+std::to_string(location.translation().y())+", z:"+std::to_string(location.translation().z())+"}), a.location) < "+std::to_string(radius)+" AND a.sensorName IN "+sensorsstr+" RETURN a ORDER BY a.index";
    }

    slam3d::VertexObjectList vertexobjlist;
    neo4j->runQuery(request, [&](neo4j_result_t *element){
        vertexobjlist.push_back(Neo4jConversion::vertexObject(element));
    });
    addSubMeasurementsToVertexObjectList(&vertexobjlist);
    return vertexobjlist;
}

const VertexObject Neo4jGraph::getVertex(IdType id)  const {
    std::string request = "MATCH (n:Vertex) WHERE n.index="+std::to_string(id)+" RETURN n";
    slam3d::VertexObject vertexobj;
    neo4j->runQuery(request, [&](neo4j_result_t *element){
        vertexobj = Neo4jConversion::vertexObject(element);
    });
    addSubMeasurementsToVertexObject(&vertexobj);
    return vertexobj;
}

const VertexObject Neo4jGraph::getVertex(boost::uuids::uuid id) const {
    std::string uuid = boost::lexical_cast<std::string>(id);
    std::string request = "MATCH (n:Vertex) WHERE n.measurementUuid = "+uuid+" RETURN n AS node";
    slam3d::VertexObject vertexobj;
    neo4j->runQuery(request, [&](neo4j_result_t *element){
        vertexobj = Neo4jConversion::vertexObject(element);
    });
    addSubMeasurementsToVertexObject(&vertexobj);
    return vertexobj;
}

void Neo4jGraph::setVertex(IdType id, const VertexObject& v) {
    std::string request = "MATCH (n:Vertex) WHERE n.index="+std::to_string(v.index)+" SET n = $props";

    ParamaterSet params = Neo4jConversion::createParamaterSet(v);

    neo4j->runQuery(request, [&](neo4j_result_t *element){}, params.get());

    if (v.subMeasurements.size()) {
        for (const auto &measurement : v.subMeasurements) {
            request = "MATCH (n:VertexMeasurement) WHERE n.index="+std::to_string(v.index)+" SET n = $props";
            params = Neo4jConversion::createParamaterSet(v);
            neo4j->runQuery(request, [&](neo4j_result_t *element){}, params.get());
        }
    
        // add a reference edge to find submeasurements via edge and not index
        // TODO check if duplicates are created

        request = "MATCH (a:Vertex), (b:VertexMeasurement) WHERE a.index="+std::to_string(v.index)+" AND b.index="+std::to_string(v.index) \
        + " CREATE (a)-[r:subMeasurement]->(b)";
        neo4j->runQuery(request, [&](neo4j_result_t *element){});
    }




}

const EdgeObject Neo4jGraph::getEdge(IdType source, IdType target, const std::string& sensor) const {
    std::string request = "MATCH (a:Vertex)-[r]->(b:Vertex) WHERE a.index="+std::to_string(source)+" AND b.index="+std::to_string(target)+" AND r.sensor='"+sensor+"' RETURN r";
    EdgeObject object;
    size_t found = neo4j->runQuery(request, [&](neo4j_result_t *element) {
        object = Neo4jConversion::edgeObject(element);
    });
    if (found == 0) {
        throw InvalidEdge(source, target);
    }
    return object;
}

const EdgeObjectList Neo4jGraph::getOutEdges(IdType source) const {

    std::string request = "MATCH (a:Vertex)-[r]->() WHERE a.index="+std::to_string(source)+" RETURN r";
    
    EdgeObjectList objectList;
    neo4j->runQuery(request, [&](neo4j_result_t *element){
        objectList.push_back(Neo4jConversion::edgeObject(element));
    });
    return objectList;
}

const EdgeObjectList Neo4jGraph::getConnectingEdges(const VertexObjectList& vertices) const {
    // sort ids into set
	std::set<int> v_ids;

	for(VertexObjectList::const_iterator v = vertices.begin(); v != vertices.end(); v++) {
		v_ids.insert(v->index);
	}
    //generate query list:
    std::string list = "[";
    for (const auto& id: v_ids) {
        list += std::to_string(id) + ",";
    }
    //replace last , with ]
    list.back() = ']';

    std::string request = "MATCH (a)-[r]->(b) where a.index IN "+list+" AND b.index IN "+list+" RETURN r";

    EdgeObjectList objectList;
    neo4j->runQuery(request, [&](neo4j_result_t *element){
        objectList.push_back(Neo4jConversion::edgeObject(element));
    });
    return objectList;
}

void Neo4jGraph::writeGraphToFile(const std::string& name)
{
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
}

const VertexObjectList Neo4jGraph::getVerticesInRange(IdType source_id, unsigned range) const
{
    // std::string request = "match (v1:Vertex)--{1,"+std::to_string(range)+"}(v2:Vertex) where v1.index="+std::to_string(source_id)+" return v2 as node ORDER BY v2.index";
    std::string request = "match (v1:Vertex)-[r:SE_3_]-{1,"+std::to_string(range)+"}(v2:Vertex) WHERE NONE(re in r where type(re)=\"Tentative\") AND v1.index="+std::to_string(source_id)+" return DISTINCT v2 as node ORDER BY v2.index";


    slam3d::VertexObjectList vertexobjlist;
    neo4j->runQuery(request, [&](neo4j_result_t *element){
        vertexobjlist.push_back(Neo4jConversion::vertexObject(element));
    });
    addSubMeasurementsToVertexObjectList(&vertexobjlist);
    return vertexobjlist;
}

float Neo4jGraph::calculateGraphDistance(IdType source_id, IdType target_id) const {
    // std::string request = "MATCH (a:Vertex), (b:Vertex), p=shortestPath((a)-[:*]->(b)) WHERE a.index="+std::to_string(source_id)+" AND b.index="+std::to_string(target_id)+" RETURN LENGTH(p)";
    std::string request = "MATCH (a:Vertex), (b:Vertex), p=shortestPath((a)-[r:SE_3_*]->(b)) WHERE NONE(re in r where type(re)=\"Tentative\") AND a.index="+std::to_string(source_id)+" AND b.index="+std::to_string(target_id)+" RETURN LENGTH(p)";
    float result = -1;
    
    neo4j->runQuery(request, [&](neo4j_result_t *element){
        Neo4jValue val (element);
        result = val.as_integer();
    });
    if (result < 0) {
        throw InvalidVertex(source_id);
    }
    return result;
}


void Neo4jGraph::setCorrectedPose(IdType id, const Transform& pose)
{
    std::string request = "MATCH (n:Vertex) WHERE n.index="+std::to_string(id)+" SET n.location = point({"
                            +   "x: " + std::to_string(pose.translation().x())
                            + ", y: " + std::to_string(pose.translation().y())
                            + ", z: " + std::to_string(pose.translation().z())
                            + "}) ,"
                            + " n.correctedPose = \"" + Neo4jConversion::eigenMatrixToString(pose.matrix()) + "\"";
    
    neo4j->runQuery(request, [&](neo4j_result_t *element){});
}

void Neo4jGraph::addSubMeasurementsToVertexObjectList(slam3d::VertexObjectList* vertexobjlist) const {
    // check if additional measurements are par of the VO, the vectro is resized by Neo4jConversion::vertexObject, as a notification, but it is not filled
    for(auto& vertexobj : *vertexobjlist) {
        addSubMeasurementsToVertexObject(&vertexobj);
    }
}

void Neo4jGraph::addSubMeasurementsToVertexObject(slam3d::VertexObject* vertexobj) const {
    // check if additional measurements are par of the VO, the vectro is resized by Neo4jConversion::vertexObject, as a notification, but it is not filled
    if (vertexobj->subMeasurements.size()) {
        // request by edges
        std::string request = "match (v1:Vertex)-[r:subMeasurement]-(v2:VertexMeasurement) where v1.index="+std::to_string(vertexobj->index)+" return v2";

        size_t index = 0;
        neo4j->runQuery(request, [&](neo4j_result_t *element){
            vertexobj->subMeasurements[index] = Neo4jConversion::vertexMeasurementData(element);
            ++index;
        });
    }
}


